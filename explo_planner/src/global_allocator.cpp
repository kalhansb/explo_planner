#include "explo_planner/global_allocator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "explo_planner/cell_world.hpp"

namespace explo_planner {

namespace {

/// Millimetres. The quantum has to be coarse enough that two robots'
/// independently accumulated route costs land in the same bucket despite
/// float rounding, and fine enough that genuinely different tours do not tie:
/// cells are 10 m, so a millimetre is four orders of magnitude below anything
/// the allocator is trying to distinguish.
constexpr double kMmPerM = 1000.0;

inline uint32_t robotBit(int id) {
  return (id >= 0 && id < 32) ? (1u << id) : 0u;
}

/// FNV-1a 32-bit. A second local copy of the construction in cell_world.cpp,
/// kept local for the reason stated there: the two hash different things, and
/// a shared helper invites folding them into one value, which would stop a
/// reader being told WHICH half disagrees.
///
/// The two copies do NOT have to stay byte-compatible with each other. Nothing
/// compares an alloc_hash to a sharedHash; the only comparison is between two
/// PROCESSES running the same binary, where both copies are identical by
/// construction.
///
/// NOTHING PINS A LITERAL, and this said "test_global_allocator pins alloc_hash
/// to a literal" until 2026-09-18. There is not one hash constant in that file
/// or any other. What the AllocHash group actually pins is RELATIONAL — equal
/// problems digest equal, an unformed problem digests 0, and each of six
/// channels shared_hash is blind to moves the digest — and every one of those
/// statements holds under ANY injective-enough hash. Swap FNV-1a for a
/// different multiplier here and the whole suite still passes.
///
/// That gap is smaller than it sounds and is not worth a literal. The value is
/// compared only within one binary, so a silent change costs nothing live; it
/// costs only the offline join of alloc_hash columns ACROSS binary generations,
/// which no analysis does (they join two robots of one campaign). And a literal
/// would have to be re-baselined on every legitimate change to the digest input
/// — plus it would ride on route costs that reach the digest through a double
/// and an llround, so it would be pinning the optimiser's floating-point as
/// well as the hash. Relational tests were the right call. Just do not read
/// this paragraph as saying the construction itself is guarded.
struct Fnv1a {
  uint32_t h = 2166136261u;
  void byte(uint8_t b) { h ^= b; h *= 16777619u; }
  void i64(int64_t v) {
    for (int i = 0; i < 8; ++i) byte(static_cast<uint8_t>((v >> (i * 8)) & 0xff));
  }
};

/// Shorthand for the public routeCostMm below, kept so the solve loops read
/// the way they always did. One implementation, two names — not two
/// implementations.
inline long long routeCost(const CellWorld& w, int start,
                           const std::vector<int>& tour) {
  return GlobalAllocator::routeCostMm(w, start, tour);
}

} // namespace

bool shouldDemoteStaleFocus(int skips, int k, CellStatus status) {
  // k < 1 would demote a cell on the first tick it produced no candidate,
  // which is every cell on the tick it is first assigned. The node clamps the
  // param, and this refuses the same case independently: two guards, because
  // the one that matters is whichever the next caller forgets.
  if (k < 1) return false;
  if (skips < k) return false;
  // Only a cell still CLAIMING to be worth exploring. If the re-measure
  // promoted it, the counter was chasing a cell that had already been cleared
  // and there is nothing to write off.
  return status == CellStatus::EXPLORING ||
         status == CellStatus::EXPLORING_BY_OTHERS;
}

int Allocation::focusFor(int self_id,
                         const std::vector<AllocRobot>& robots) const {
  for (size_t i = 0; i < robots.size() && i < tours.size(); ++i) {
    if (robots[i].id != self_id) continue;
    return tours[i].empty() ? -1 : tours[i].front();
  }
  return -1;
}

long long GlobalAllocator::costMm(const CellWorld& world, int a, int b) {
  if (a == b) return 0;
  if (!world.grid().valid(a) || !world.grid().valid(b)) return 0;
  double d = world.distance(a, b);
  // Negative is the graph's "unreachable" sentinel, not a distance. See the
  // header: falling back keeps the cell in the problem.
  if (!(d >= 0.0)) d = world.centroidDistance(a, b);
  if (!std::isfinite(d) || d < 0.0) return 0;
  return static_cast<long long>(std::llround(d * kMmPerM));
}

long long GlobalAllocator::routeCostMm(const CellWorld& world, int start,
                                       const std::vector<int>& tour) {
  long long c = 0;
  int prev = start;
  for (int id : tour) {
    c += costMm(world, prev, id);
    prev = id;
  }
  return c;
}

Allocation GlobalAllocator::solve(const CellWorld& world,
                                  const std::vector<AllocRobot>& robots_in,
                                  const Config& cfg) {
  Allocation out;
  out.tours.assign(robots_in.size(), {});
  out.costs_mm.assign(robots_in.size(), 0);

  if (!world.configured()) {
    // The one path that leaves alloc_hash/edge_hash at 0. There was no problem
    // to digest, so 0 here means "no problem was formed", never "the problems
    // matched". A reader comparing two robots must skip refused solves, not
    // treat equal zeros as agreement.
    out.refused = "cell world is not configured";
    return out;
  }

  // --- candidate set ------------------------------------------------------
  // Scanned BEFORE the vehicle filter, not because the solve needs it here but
  // because the R3 digest below has to cover the whole problem and has to be
  // written on every return path that formed one. The refusal ORDER is
  // unchanged: the `cand.empty()` and max_candidates returns stay where they
  // always were, after the vehicle refusal, so which reason a caller sees is
  // exactly what it was.
  std::vector<int> cand;
  for (int id = 0; id < world.size(); ++id) {
    const CellStatus s = world.status(id);
    if (s == CellStatus::EXPLORING || s == CellStatus::EXPLORING_BY_OTHERS)
      cand.push_back(id);
  }

  // --- R3 / §3.6: digest of the PROBLEM ------------------------------------
  // Assembled here, once, from the same values the solve is about to use, and
  // written before any refusal — a refused solve still had a problem, and
  // "these two robots refused for different reasons" is only interpretable if
  // you can first establish they were refusing the same thing.
  //
  // Ordering is imposed explicitly at every level, because the input vector's
  // order is the caller's and must not reach a value two processes compare:
  // vehicles by id, candidates ascending (the scan above already produces
  // them that way, and it is asserted rather than assumed by construction
  // since `world.size()` walks ids in order).
  //
  // The vehicle list hashed here is `robots_in` ENTIRE — including robots the
  // filter below is about to drop as finished or unlocatable. That is
  // deliberate: "my peer is finished" versus "my peer is still working" is a
  // disagreement about the problem, and it is one of the two channels §3.6
  // says nothing could see. Folding in only the survivors would hide exactly
  // the case that matters.
  //
  // TWO OF Config's THREE FIELDS ARE PART OF THE PROBLEM (2026-09-18). All
  // three were excluded before, deliberately and with a test asserting it
  // (AllocHash.SolverConfigIsNotPartOfTheProblem), on the grounds that config
  // is a property of the SOLVER and a mismatch is a deployment fault the run
  // params already record. That reasoning is exactly right for one field and
  // wrong for the other two, so the split is now drawn where the distinction
  // actually falls rather than around the whole struct:
  //
  //   comms_mask     IN. It restricts which cells a disconnected robot may be
  //                  assigned AT ALL, so it changes the feasible set — that is
  //                  the problem, not an approach to it. And this is not a
  //                  hypothetical misconfiguration: the flag exists so that the
  //                  gap between a masked and an unmasked solve can be measured
  //                  (see Config::comms_mask — "the cost gap ... IS the
  //                  reconnection value P4 gates on"). Those two solves differ
  //                  in NOTHING ELSE, so with comms_mask excluded the one
  //                  comparison the flag was added to support is precisely the
  //                  one where the digest declares both sides identical.
  //
  //   max_candidates IN. It decides refused versus solved. A refused solve
  //                  carries its digest on purpose, so that "these two robots
  //                  refused for different reasons" is interpretable — but with
  //                  the threshold excluded, a robot that refused and a robot
  //                  that solved the same candidate set under a laxer cap agree
  //                  on the key, and the digest reports them as having faced
  //                  the same thing when the cap is the entire difference.
  //
  //   polish_passes  OUT, and the old rationale survives intact here. 2-opt
  //                  passes change the tours and nothing else: same vehicles,
  //                  same candidates, same feasible set, same refusal. Two
  //                  robots differing only in polish_passes ARE solving the
  //                  same problem and getting different answers to it, which is
  //                  the one thing the digest is supposed to be able to say.
  //                  Folding it in would convert that finding into a silent
  //                  "different problem" and lose it.
  //
  // Latent today either way: one launch supplies every robot in a cell, so all
  // three are equal across the fleet by construction. That is a property of the
  // CALLER, and it is not what the digest claims to depend on.
  //
  // Folded in LAST, after the world's edge hash, so the contribution order of
  // everything already being logged is untouched. Values are not comparable
  // across this change, which is the standing rule anyway: never join across
  // generations.
  {
    Fnv1a f;
    std::vector<const AllocRobot*> by_id;
    by_id.reserve(robots_in.size());
    for (const AllocRobot& r : robots_in) by_id.push_back(&r);
    std::stable_sort(by_id.begin(), by_id.end(),
                     [](const AllocRobot* a, const AllocRobot* b) {
                       return a->id < b->id;
                     });
    f.i64(static_cast<int64_t>(by_id.size()));
    for (const AllocRobot* r : by_id) {
      f.i64(r->id);
      f.i64(r->cell);
      f.byte(r->in_comms ? 1u : 0u);
      f.byte(r->finished ? 1u : 0u);
      // IN, and not optional: it decides which vehicles the solve is given, so
      // two robots disagreeing about a peer's mode are solving different
      // problems and the digest has to say so. Leaving it out would let them
      // agree on the key while allocating over different vehicle sets, which
      // is the exact failure the split above exists to make visible.
      f.byte(r->off_frontier ? 1u : 0u);
    }
    f.i64(static_cast<int64_t>(cand.size()));
    for (int id : cand) f.i64(id);
    const uint32_t eh = world.edgeHash();
    f.i64(static_cast<int64_t>(eh));
    f.byte(cfg.comms_mask ? 1u : 0u);
    f.i64(static_cast<int64_t>(cfg.max_candidates));
    // cfg.polish_passes is NOT folded in. See the split above; this omission is
    // load-bearing and has a test.
    out.edge_hash  = eh;
    out.alloc_hash = f.h;
  }

  // --- vehicle set --------------------------------------------------------
  // Sorted by id so the caller's vector order cannot reach the result, and
  // carrying the caller's index so the output stays index-aligned with the
  // input the caller passed. A robot that has left the frontier is removed
  // outright and its cells return to the pool (§3.4) -- leaving it in with an
  // empty tour would let the makespan balance keep reserving work for a robot
  // that has stopped. `off_frontier` is the superset test and `finished`
  // implies it, so the pair below is one condition written as two for the
  // reader; see AllocRobot for why they are separate fields.
  struct Veh { int idx; int id; int cell; bool in_comms; };
  std::vector<Veh> veh;
  for (size_t i = 0; i < robots_in.size(); ++i) {
    const AllocRobot& r = robots_in[i];
    if (r.finished || r.off_frontier) continue;
    // An unlocatable robot is dropped rather than defaulted to some cell: every
    // cost involving it would be fiction, and a fiction that changes the
    // makespan changes the OTHER robot's tour too. Dropped, it simply gets no
    // focus cell and falls back to unrestricted planning, which is the
    // documented degradation.
    if (!world.grid().valid(r.cell)) continue;
    if (r.id < 0 || r.id >= 32) continue;   // outside the known_by mask width
    veh.push_back({static_cast<int>(i), r.id, r.cell, r.in_comms});
  }
  std::sort(veh.begin(), veh.end(),
            [](const Veh& a, const Veh& b) { return a.id < b.id; });

  if (veh.empty()) {
    out.refused = "no locatable robot still on the frontier to allocate to";
    return out;
  }

  // --- candidate set: scanned above, adjudicated here ----------------------
  if (cand.empty()) return out;            // solved; nothing to do
  if (static_cast<int>(cand.size()) > cfg.max_candidates) {
    out.refused = "too many candidate cells (" +
                  std::to_string(cand.size()) + " > " +
                  std::to_string(cfg.max_candidates) +
                  "): refusing rather than truncating, because a truncated "
                  "candidate set is one two robots would cut differently";
    return out;
  }

  // masked[v][c]: vehicle v may not take candidate index c.
  const size_t nv = veh.size(), nc = cand.size();
  std::vector<uint8_t> masked(nv * nc, 0);
  if (cfg.comms_mask) {
    for (size_t v = 0; v < nv; ++v) {
      if (veh[v].in_comms) continue;
      const uint32_t bit = robotBit(veh[v].id);
      for (size_t c = 0; c < nc; ++c)
        if (!(world.cell(cand[c]).known_by & bit)) masked[v * nc + c] = 1;
    }
  }

  // --- greedy makespan-balanced insertion ---------------------------------
  // Each round: over every (unassigned cell, vehicle, insertion position),
  // find the one whose insertion leaves the SMALLEST resulting makespan, and
  // commit it. Ties break on (cell id, robot id, position) -- a total order
  // over values both robots agree on -- and the scan visits them in exactly
  // that order under a strict <, so the first minimum found is the tie-break
  // winner without a separate comparison.
  std::vector<uint8_t> taken(nc, 0);
  std::vector<std::vector<int>> tour(nv);
  std::vector<long long> cost(nv, 0);

  for (size_t placed = 0; placed < nc; ++placed) {
    long long best_makespan = std::numeric_limits<long long>::max();
    size_t best_c = 0, best_v = 0, best_pos = 0;
    bool found = false;

    for (size_t c = 0; c < nc; ++c) {
      if (taken[c]) continue;
      for (size_t v = 0; v < nv; ++v) {
        if (masked[v * nc + c]) continue;
        // Makespan contributed by every OTHER vehicle is fixed this round.
        long long others = 0;
        for (size_t k = 0; k < nv; ++k)
          if (k != v) others = std::max(others, cost[k]);

        for (size_t pos = 0; pos <= tour[v].size(); ++pos) {
          std::vector<int> trial = tour[v];
          trial.insert(trial.begin() + static_cast<long>(pos), cand[c]);
          const long long rc = routeCost(world, veh[v].cell, trial);
          const long long ms = std::max(others, rc);
          if (ms < best_makespan) {
            best_makespan = ms;
            best_c = c; best_v = v; best_pos = pos;
            found = true;
          }
        }
      }
    }

    // Nothing legal left: every remaining candidate is masked away from every
    // vehicle. Report them rather than looping.
    if (!found) break;

    taken[best_c] = 1;
    tour[best_v].insert(tour[best_v].begin() + static_cast<long>(best_pos),
                        cand[best_c]);
    cost[best_v] = routeCost(world, veh[best_v].cell, tour[best_v]);
  }

  for (size_t c = 0; c < nc; ++c)
    if (!taken[c]) out.unassigned.push_back(cand[c]);

  // --- capped 2-opt polish ------------------------------------------------
  // FIRST improving move in (i, j) order, not the best one: both are
  // deterministic, but "first" is also cheap to reason about and cannot be
  // perturbed by a tie between two equally good moves.
  for (size_t v = 0; v < nv; ++v) {
    for (int pass = 0; pass < cfg.polish_passes; ++pass) {
      bool improved = false;
      const size_t n = tour[v].size();
      for (size_t i = 0; i + 1 < n && !improved; ++i) {
        for (size_t j = i + 1; j < n && !improved; ++j) {
          std::vector<int> trial = tour[v];
          std::reverse(trial.begin() + static_cast<long>(i),
                       trial.begin() + static_cast<long>(j) + 1);
          const long long rc = routeCost(world, veh[v].cell, trial);
          if (rc < cost[v]) {
            tour[v].swap(trial);
            cost[v] = rc;
            improved = true;
          }
        }
      }
      if (!improved) break;
    }
  }

  for (size_t v = 0; v < nv; ++v) {
    out.tours[static_cast<size_t>(veh[v].idx)] = tour[v];
    out.costs_mm[static_cast<size_t>(veh[v].idx)] = cost[v];
  }
  return out;
}

} // namespace explo_planner
