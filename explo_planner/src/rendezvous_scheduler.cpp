// Moved comments: doc/rendezvous_scheduler_notes.md
#include "explo_planner/rendezvous_scheduler.hpp"

#include <algorithm>
#include <limits>

namespace explo_planner {

namespace {

/// One vehicle as this file needs it: the caller's index (to reach its tour),
/// the fleet id (the walk order), and the cell it starts from.
struct Veh {
  size_t idx;
  int    id;
  int    cell;
};

/// The same vehicle filter GlobalAllocator::solve applies, duplicated on
/// purpose: if the allocator's filter changes, this one must change with it.
/// The tests pin the agreement. (notes: rzv-vehicle-filter-duplicated)
std::vector<Veh> vehicles(const CellWorld& world,
                          const std::vector<AllocRobot>& robots) {
  std::vector<Veh> v;
  for (size_t i = 0; i < robots.size(); ++i) {
    const AllocRobot& r = robots[i];
    if (r.finished || r.off_frontier) continue;
    if (!world.grid().valid(r.cell)) continue;
    if (r.id < 0 || r.id >= 32) continue;
    v.push_back({i, r.id, r.cell});
  }
  // Fleet-id order, so the caller's vector order cannot reach the result.
  std::sort(v.begin(), v.end(),
            [](const Veh& a, const Veh& b) { return a.id < b.id; });
  return v;
}

/// What forcing one robot's tour through `c` costs it, and when it gets there.
struct Detour {
  long long cost_mm  = 0;   ///< route cost of the tour with `c` in it
  long long reach_mm = 0;   ///< prefix cost from the start cell to `c`
};

/// Best insertion of c into tour from start: least resulting route cost, ties
/// to the lowest position (strict <). A cell already on the tour is not
/// re-inserted; its reach is the prefix to its first occurrence.
/// (notes: rzv-detour-best-insertion)
Detour detour(const CellWorld& w, int start, const std::vector<int>& tour,
              int c) {
  const auto hit = std::find(tour.begin(), tour.end(), c);
  if (hit != tour.end()) {
    const size_t upto = static_cast<size_t>(hit - tour.begin()) + 1;
    Detour d;
    d.cost_mm  = GlobalAllocator::routeCostMm(w, start, tour);
    d.reach_mm = GlobalAllocator::routeCostMm(
        w, start, std::vector<int>(tour.begin(), tour.begin() +
                                                     static_cast<long>(upto)));
    return d;
  }

  Detour best;
  best.cost_mm = std::numeric_limits<long long>::max();
  for (size_t pos = 0; pos <= tour.size(); ++pos) {
    std::vector<int> trial = tour;
    trial.insert(trial.begin() + static_cast<long>(pos), c);
    const long long rc = GlobalAllocator::routeCostMm(w, start, trial);
    if (rc < best.cost_mm) {
      best.cost_mm = rc;
      best.reach_mm = GlobalAllocator::routeCostMm(
          w, start,
          std::vector<int>(trial.begin(),
                           trial.begin() + static_cast<long>(pos) + 1));
    }
  }
  return best;
}

}  // namespace

long long RendezvousScheduler::travelMs(long long mm, long long speed_mm_s) {
  if (speed_mm_s <= 0) return -1;
  if (mm <= 0) return 0;
  return (mm * 1000) / speed_mm_s;
}

RendezvousPlan RendezvousScheduler::solve(const CellWorld& world,
                                          const std::vector<AllocRobot>& robots,
                                          const Allocation& alloc,
                                          int floor_cell,
                                          long long mission_elapsed_ms,
                                          const Config& cfg) {
  RendezvousPlan out;

  if (!world.configured()) {
    out.refused = "cell world is not configured";
    return out;
  }
  if (cfg.speed_mm_s <= 0) {
    out.refused = "speed_mm_s is not positive: every arrival time would be "
                  "infinite and there is no safe value to substitute";
    return out;
  }
  if (alloc.tours.size() != robots.size()) {
    // Index alignment is the contract that lets a tour be attributed to a
    // starting cell without searching by id. Refuse loudly rather than
    // attribute tours to the wrong robots, which would produce a plan that
    // looks entirely reasonable and that neither robot can keep.
    out.refused = "allocation is not index-aligned with the vehicle set";
    return out;
  }

  const std::vector<Veh> veh = vehicles(world, robots);
  if (veh.empty()) {
    out.refused = "no locatable robot still on the frontier to meet";
    return out;
  }

  // --- base makespan ------------------------------------------------------
  // Recomputed from the tours, not read from alloc.costs_mm (a test pins that
  // they agree), so the penalty is a difference of two values computed here.
  // (notes: rzv-base-makespan-recomputed)
  std::vector<long long> base(veh.size(), 0);
  long long base_makespan = 0;
  for (size_t v = 0; v < veh.size(); ++v) {
    base[v] = GlobalAllocator::routeCostMm(world, veh[v].cell,
                                           alloc.tours[veh[v].idx]);
    base_makespan = std::max(base_makespan, base[v]);
  }

  // --- candidate set ------------------------------------------------------
  // The union of the solved tours, plus the floor. Sorted-unique so the argmin
  // walk visits cells in ascending id and the first strict minimum IS the
  // tie-break winner, with no separate comparison to keep in step.
  std::vector<int> cand;
  for (const Veh& v : veh) {
    const std::vector<int>& t = alloc.tours[v.idx];
    cand.insert(cand.end(), t.begin(), t.end());
  }
  std::sort(cand.begin(), cand.end());
  cand.erase(std::unique(cand.begin(), cand.end()), cand.end());

  const bool have_floor = world.grid().valid(floor_cell);

  // Admissible: reachable by every robot on the roadmap and not in cfg.exclude.
  // Presence in each peer's cell world is not checked: cell ids are geometry
  // and the grid config hash is verified on the wire.
  // (notes: rzv-admissibility)
  std::vector<int> admissible;
  for (int c : cand) {
    if (have_floor && c == floor_cell) continue;   // added unconditionally below
    if (std::find(cfg.exclude.begin(), cfg.exclude.end(), c) !=
        cfg.exclude.end()) {
      ++out.rejected_excluded;
      continue;
    }
    bool reachable = true;
    for (const Veh& v : veh) {
      if (v.cell == c) continue;
      // A negative world.distance is the unreachable sentinel. Tested directly,
      // not through costMm, whose centroid-distance fallback is wrong for
      // promising to stand in a cell at a given minute.
      // (notes: rzv-unreachable-sentinel)
      if (!(world.distance(v.cell, c) >= 0.0)) { reachable = false; break; }
    }
    if (!reachable) { ++out.rejected_unreachable; continue; }
    admissible.push_back(c);
  }
  if (have_floor) {
    // The floor is admitted unconditionally — not excluded, not reachability-
    // filtered. Filtering it out could empty the candidate set, and an empty
    // set is the deadlock the no-show rule exists to prevent.
    //
    // floor_cell is the centroid of the team's own vehicle cells and carries no
    // comms-range guarantee. (notes: rzv-floor-not-midpoint)
    admissible.push_back(floor_cell);
    std::sort(admissible.begin(), admissible.end());
  }
  out.candidates = static_cast<int>(admissible.size());

  if (admissible.empty()) {
    // Name the cause: an empty cand (no tours) and a cand filtered to nothing
    // are different failures with opposite remedies. Reachability is an AND
    // over the team, so larger fleets shrink the admissible set.
    // (notes: rzv-name-empty-candidates-cause)
    if (cand.empty()) {
      out.refused = "no admissible meeting cell: the snapshot carries no "
                    "tours to meet on";
    } else {
      out.refused =
          "no admissible meeting cell: all " + std::to_string(cand.size()) +
          " tour cell(s) were rejected (" +
          std::to_string(out.rejected_unreachable) +
          " unreachable for at least one robot, " +
          std::to_string(out.rejected_excluded) + " excluded)";
    }
    return out;
  }

  // --- argmin over the makespan penalty -----------------------------------
  long long best_penalty = std::numeric_limits<long long>::max();
  long long best_reach_mm = 0;
  int best_cell = -1;
  for (int c : admissible) {
    long long makespan = 0, slowest_reach = 0;
    for (size_t v = 0; v < veh.size(); ++v) {
      const Detour d = detour(world, veh[v].cell, alloc.tours[veh[v].idx], c);
      makespan = std::max(makespan, d.cost_mm);
      slowest_reach = std::max(slowest_reach, d.reach_mm);
    }
    const long long penalty = makespan - base_makespan;
    if (penalty < best_penalty) {          // strict: ascending id breaks ties
      best_penalty = penalty;
      best_reach_mm = slowest_reach;
      best_cell = c;
    }
  }

  out.cell             = best_cell;
  out.penalty_mm       = best_penalty;
  out.floor_won        = have_floor && best_cell == floor_cell;
  out.tour_interval_ms = travelMs(best_reach_mm, cfg.speed_mm_s);

  // --- when ---------------------------------------------------------------
  // The tour term: the SLOWEST robot's arrival under its own tour. It is what
  // the objective optimised and it remains the first of the two lower bounds.
  long long interval = out.tour_interval_ms;

  // The interval is also the gap between occurrences, so it must cover the
  // furthest robot's DIRECT drive (not along-tour), marked up by
  // depart_safety_milli; appointmentLeadMs uses the same constant.
  // (notes: rzv-reachability-floor)
  long long direct_mm = 0;
  for (const Veh& v : veh) {
    direct_mm =
        std::max(direct_mm, GlobalAllocator::costMm(world, v.cell, best_cell));
  }
  const long long drive_ms = travelMs(direct_mm, cfg.speed_mm_s);
  const long long floor_ms =
      std::max((drive_ms * std::max(0, cfg.depart_safety_milli)) / 1000,
               std::max<long long>(0, cfg.min_interval_ms));
  // What the objective alone asked for, kept so the two flags below can be
  // derived from the OUTCOME instead of from which branch happened to run.
  const long long unbounded = interval;
  if (floor_ms > interval) interval = floor_ms;

  // Findability cap: keep the interval at or under max_interval_ms so the last
  // robot to set off still finds its peers, but never below floor_ms; when they
  // conflict the floor wins. (notes: rzv-findability-cap)
  if (cfg.max_interval_ms > 0 && interval > cfg.max_interval_ms) {
    interval = std::max(cfg.max_interval_ms, floor_ms);
  }

  // Flags come from the three bounds, not the branch taken: capped means the
  // cap is below the tour term; floored means the floor exceeds min(tour, cap).
  // Both true is the broken findability inequality. (notes: rzv-bound-flags)
  const long long ask =
      cfg.max_interval_ms > 0 ? std::min(unbounded, cfg.max_interval_ms)
                              : unbounded;
  out.capped  = cfg.max_interval_ms > 0 && cfg.max_interval_ms < unbounded;
  out.floored = floor_ms > ask;

  out.interval_ms = interval;
  out.t_meet_ms   = mission_elapsed_ms + interval;
  return out;
}

RendezvousHandshake::Adopt RendezvousHandshake::adopt(
    bool peer_valid, bool peer_provisional,
    bool held_valid, bool held_provisional,
    bool peer_equals_held, bool held_reagree_due) {
  // Nothing on offer. Silence is not a withdrawal — the caller's record of what
  // this peer last said is untouched — so there is simply no decision to take.
  if (!peer_valid) return Adopt::kIgnore;

  // First contact with the proposal. Every robot in the run reaches this line
  // exactly once, and the triple it takes here is the one it drives to unless
  // the single upgrade below fires.
  if (!held_valid) return Adopt::kTake;

  // The one upgrade (held provisional, offer not) is tested before the equality
  // check on purpose: the same triple with a real choice behind it still
  // upgrades. Taking it clears this robot's flag.
  // (notes: rzv-handshake-upgrade)
  if (held_provisional && !peer_provisional) return Adopt::kUpgrade;

  // Already holding it. The overwhelmingly common case: the proposer
  // re-publishes its triple on every heartbeat for the whole run.
  if (peer_equals_held) return Adopt::kIgnore;

  // Tested after the equality check on purpose: the reagree flag survives
  // re-publications of the old triple and is spent only on a differing one. A
  // provisional replacement is not taken (the proposer's guard).
  // (notes: rzv-handshake-reagree)
  if (held_reagree_due && !peer_provisional) return Adopt::kReagree;

  // Anything else is a conflict (restarted proposer, fleet disagreeing on robot
  // 0, two campaigns on one bus): the caller keeps the triple it holds, which
  // its peers have echoed, and reports it loudly.
  // (notes: rzv-handshake-conflict)
  return Adopt::kConflict;
}

}  // namespace explo_planner
