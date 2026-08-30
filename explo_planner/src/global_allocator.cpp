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

/// Route cost of `tour` starting from `start`, in mm. Open route: the robots
/// are not coming back, so there is no closing leg. Adding one would inflate
/// every cost by a term that depends only on the tour's last cell, which biases
/// the makespan balance toward tours that happen to end near their origin —
/// a preference nothing in the mission wants.
long long routeCost(const CellWorld& w, int start,
                    const std::vector<int>& tour) {
  long long c = 0;
  int prev = start;
  for (int id : tour) {
    c += GlobalAllocator::costMm(w, prev, id);
    prev = id;
  }
  return c;
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

Allocation GlobalAllocator::solve(const CellWorld& world,
                                  const std::vector<AllocRobot>& robots_in,
                                  const Config& cfg) {
  Allocation out;
  out.tours.assign(robots_in.size(), {});
  out.costs_mm.assign(robots_in.size(), 0);

  if (!world.configured()) {
    out.refused = "cell world is not configured";
    return out;
  }

  // --- vehicle set --------------------------------------------------------
  // Sorted by id so the caller's vector order cannot reach the result, and
  // carrying the caller's index so the output stays index-aligned with the
  // input the caller passed. A finished robot is removed outright and its
  // cells return to the pool (§3.4) -- leaving it in with an empty tour would
  // let the makespan balance keep reserving work for a robot that has stopped.
  struct Veh { int idx; int id; int cell; bool in_comms; };
  std::vector<Veh> veh;
  for (size_t i = 0; i < robots_in.size(); ++i) {
    const AllocRobot& r = robots_in[i];
    if (r.finished) continue;
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
    out.refused = "no locatable, unfinished robot to allocate to";
    return out;
  }

  // --- candidate set ------------------------------------------------------
  std::vector<int> cand;
  for (int id = 0; id < world.size(); ++id) {
    const CellStatus s = world.status(id);
    if (s == CellStatus::EXPLORING || s == CellStatus::EXPLORING_BY_OTHERS)
      cand.push_back(id);
  }
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
