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

/// The same vehicle filter GlobalAllocator::solve applies, deliberately
/// duplicated rather than shared: the filter is part of the ALLOCATION
/// problem's definition, and this file's contract is "the same set the
/// allocation was solved over". If the allocator's filter ever changes, this
/// one has to change with it, and a compile-time coupling would hide that by
/// silently agreeing. The tests pin the agreement instead.
std::vector<Veh> vehicles(const CellWorld& world,
                          const std::vector<AllocRobot>& robots) {
  std::vector<Veh> v;
  for (size_t i = 0; i < robots.size(); ++i) {
    const AllocRobot& r = robots[i];
    if (r.finished) continue;
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

/// Best insertion of `c` into `tour` driven from `start`.
///
/// "Best" = smallest resulting ROUTE COST, matching §3.5's definition of the
/// penalty; ties take the lowest position under a strict `<`, so the scan order
/// is the tie-break and there is nothing else to agree on.
///
/// A cell already on the tour is not re-inserted: the robot is going there
/// anyway, the detour is zero, and its arrival is simply the prefix to the
/// first occurrence. That asymmetry is the objective's whole point — the
/// cheapest meeting is one somebody was already driving to.
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

bool RendezvousScheduler::shouldDepart(long long t_meet_ms,
                                       long long mission_elapsed_ms,
                                       long long travel_ms,
                                       const Config& cfg) {
  if (t_meet_ms < 0) return false;          // no appointment to keep
  if (travel_ms < 0) return false;          // no usable travel estimate
  const long long remaining = t_meet_ms - mission_elapsed_ms;
  const long long need =
      (travel_ms * std::max(0, cfg.depart_safety_milli)) / 1000 +
      std::max<long long>(0, cfg.depart_margin_ms);
  return remaining <= need;
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
    out.refused = "no locatable, unfinished robot to meet";
    return out;
  }

  // --- base makespan ------------------------------------------------------
  // Recomputed from the tours rather than read out of alloc.costs_mm. The two
  // agree (a test pins it), but recomputing keeps this function well-defined
  // for a hand-built Allocation and, more importantly, makes the penalty a
  // difference of two values produced HERE — so it cannot be quietly turned
  // into a difference between a polished cost and an unpolished one.
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

  // Admissibility. A candidate every robot can reach on the roadmap, and that
  // the caller has not written off. Note what is NOT checked: presence in each
  // peer's cell world. Cell ids are geometry and the fleet's grid config hash
  // is verified on the wire, so a cell that exists here exists everywhere —
  // there is no analogue of mTARE's missing-cell case, which it handles by
  // calling exit(1) (§3.5.1, edge 4).
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
      // Negative is the graph's unreachable sentinel. Tested directly rather
      // than through costMm, which deliberately FALLS BACK to centroid
      // distance: that fallback is right for ranking a cell somebody will
      // eventually clear, and wrong for promising to stand in it at a
      // particular minute.
      if (!(world.distance(v.cell, c) >= 0.0)) { reachable = false; break; }
    }
    if (!reachable) { ++out.rejected_unreachable; continue; }
    admissible.push_back(c);
  }
  if (have_floor) {
    // The floor is admitted unconditionally — not excluded, not reachability-
    // filtered. It is the midpoint of two poses that were in radio contact, so
    // it is at most half a comms range from each robot, and it is the
    // destination the shipped implementation already drives to. Filtering it
    // out could empty the candidate set, and an empty set is the deadlock the
    // no-show rule exists to prevent.
    admissible.push_back(floor_cell);
    std::sort(admissible.begin(), admissible.end());
  }
  out.candidates = static_cast<int>(admissible.size());

  if (admissible.empty()) {
    out.refused = "no admissible meeting cell: the tours are empty and the "
                  "last-contact midpoint is outside the ROI";
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

  out.cell        = best_cell;
  out.penalty_mm  = best_penalty;
  out.floor_won   = have_floor && best_cell == floor_cell;
  out.interval_ms = travelMs(best_reach_mm, cfg.speed_mm_s);

  // --- when ---------------------------------------------------------------
  // t_meet = the SLOWEST robot's arrival under its own tour. Not a formula
  // with constants to tune — no min_interval, no farthest/(2*v), no clamp —
  // because the tours already determine it. mTARE's equivalent is a hand-tuned
  // interval whose own two defaults disagree with each other and with the
  // hardcoded value actually used (§3.5.1).
  long long interval = out.interval_ms;
  if (cfg.max_interval_ms > 0 && interval > cfg.max_interval_ms) {
    // The divergence cap: meet sooner because the maps are separating faster
    // than the tours imply. It is symmetric in the two robots, so both ends
    // cap to the same value; the cost is that the far robot arrives late by
    // the difference, which the wait cap absorbs and `capped` advertises.
    interval = cfg.max_interval_ms;
    out.capped = true;
  }
  out.t_meet_ms = mission_elapsed_ms + interval;
  return out;
}

}  // namespace explo_planner
