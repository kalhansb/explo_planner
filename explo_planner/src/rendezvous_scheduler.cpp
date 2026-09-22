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
    // filtered. Filtering it out could empty the candidate set, and an empty
    // set is the deadlock the no-show rule exists to prevent.
    //
    // NOT THE MIDPOINT (stale through generation 19; corrected 2026-09-18).
    // This comment used to justify the unconditional admission by saying the
    // floor "is the midpoint of two poses that were in radio contact, so it is
    // at most half a comms range from each robot". That construction was
    // deleted along with every other midpoint drive; what the node passes today
    // is the centroid of the TEAM'S OWN VEHICLE CELLS (see `floor_cell` on
    // solve), which carries no comms-range guarantee at all. The admission is
    // still unconditional, but now for the deadlock reason alone — which is the
    // reason that was always doing the work.
    admissible.push_back(floor_cell);
    std::sort(admissible.begin(), admissible.end());
  }
  out.candidates = static_cast<int>(admissible.size());

  if (admissible.empty()) {
    // NAME THE CAUSE. This message used to assert "the tours are empty and the
    // last-contact midpoint is outside the ROI" unconditionally, which is two
    // claims it is not entitled to make. An empty `cand` and a `cand` filtered
    // down to nothing are different failures with opposite remedies, and the
    // ts4 smoke spent a full N=4 rung on the message insisting on the first
    // while the allocator had been publishing tours since t+1.0 s.
    //
    // The distinction matters most at N>=3, because reachability below is an
    // AND over the whole team: every extra robot can only shrink the admissible
    // set. At N=3 the proposer already rejected 12 of 16 candidates. A fleet
    // that scales past the point where some robot can route to no shared tour
    // cell at all does not degrade — it loses the arm outright, silently.
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

  // THE REACHABILITY FLOOR. The interval is also the gap between occurrences,
  // so it has to be at least as long as it takes the furthest robot to get
  // there — otherwise a robot that keeps occurrence k is still driving when
  // k+1 happens, and the schedule is one it can never be on time for.
  //
  // DIRECT distance, not along-tour. The tour prefix was the right quantity for
  // the objective (it measures how nearly free the meeting is) and is the wrong
  // one here: a robot keeping an appointment ABANDONS its tour and drives
  // straight to the cell. The two differ by however much of the tour sat
  // between the robot and the near-miss, which is exactly the slack the
  // objective was maximising.
  //
  // It carries `depart_safety_milli` because a floor built on the bare travel
  // time would be one nothing downstream considers achievable — the estimate is
  // a graph distance at a nominal speed and the real drive is neither. A note
  // here called this the multiplier's only reader because the departure test
  // was gone (generation 19); the test came back on 2026-09-19 and reads the
  // same constant, deliberately — see appointmentLeadMs, which prices a robot's
  // notice at exactly the rate this floor sized the spacing for.
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

  // The findability cap: hold the interval at or under the barrier wait, so the
  // last robot to set off still finds its peers standing there. See
  // Config::max_interval_ms for the inequality this exists to keep, and for the
  // generation-19 change of subject that left the arithmetic untouched.
  //
  // IT CANNOT CUT BELOW THE REACHABILITY FLOOR, and when the two conflict the
  // floor wins. An interval shorter than the drive does not shorten the drive;
  // it just stops describing it, and the robot is late anyway. Either way the
  // result is `interval_ms > max_interval_ms`, which is the broken-inequality
  // shape the node warns on — and the WARN is keyed on that comparison rather
  // than on the flags, because `capped` does NOT have to be true here: it asks
  // whether the cap cut the TOUR term, so a cap that sits above the tours and
  // below the floor overrules nothing and reports nothing. See the derivation
  // below. The clamp is symmetric, like the cap: every robot computes the same
  // floor from the same committed cell and the same shared config, so agreement
  // survives it.
  if (cfg.max_interval_ms > 0 && interval > cfg.max_interval_ms) {
    interval = std::max(cfg.max_interval_ms, floor_ms);
  }

  // THE FLAGS DESCRIBE WHICH BOUND THE ANSWER SITS ON, NOT WHICH BRANCH RAN
  // (2026-09-18). They used to be set inside the two branches above, and that
  // made them exactly inverted in the one configuration where they matter most.
  // When the floor exceeds BOTH the tour interval and the cap — production
  // constants and a 20x20 ROI reach this, e.g. floor 456 s against a 240 s cap
  // — the first branch does not run (the floor only beat the tour interval, so
  // `floored` stayed false only if the tour interval was already above it;
  // where it wasn't, it ran) and the cap branch then assigns
  // max(cap, floor) == floor and stamps `capped`. The solve returns
  // interval_ms == floor_ms while reporting floored=0 capped=1: a tally of "how
  // often did the reachability floor bind" reads ZERO precisely for the cases
  // where it bound, and the cap is credited with a value it was overruled on.
  //
  // DERIVED FROM THE THREE BOUNDS, NOT FROM THE RETURNED VALUE (2026-09-18,
  // second revision). The first revision derived them from `interval` alone and
  // that lost the one configuration both flags exist for. The returned value is
  //
  //     interval = max(floor, min(tour, cap))
  //
  // and asking "which bound is the answer equal to" cannot separate "the cap
  // never had anything to cut" from "the cap cut and the floor overruled it",
  // because the answer sits on the floor either way. In the cap-below-floor
  // configuration — the exact case RendezvousCap.CannotCutBelowTheReachability-
  // Floor covers, cap 5 s against a 20 s floor and an 84.7 s tour term — that
  // derivation returned floored=0 AND capped=0: the findability inequality was
  // broken and the plan said nothing bound at all.
  //
  // Each flag is asked of its own bound instead, against what that bound was
  // competing with:
  //   capped  — the cap CUT the objective's answer. True whenever the cap is
  //             below the tour term, whether or not the floor then overruled
  //             it: the cap did pull the period in from the tours, it just did
  //             not get everything it asked for. This is what
  //             RendezvousPlan::capped has always documented.
  //   floored — the floor RAISED the result above what cap-and-objective
  //             between them asked for. min(tour, cap) is that ask.
  //
  // Both false means nothing bound. BOTH TRUE IS THE BROKEN FINDABILITY
  // INEQUALITY and is not a contradiction: it says the cap cut and the floor
  // refused the cut, which is precisely `interval_ms > max_interval_ms`, the
  // condition the node warns on. The previous comment here claimed the two
  // "cannot both be true"; that claim is what made the case invisible.
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

  // THE ONE UPGRADE. Tested BEFORE the equality below, deliberately: the
  // proposer can re-derive and land on the same three integers with a real
  // choice behind them this time, and that is still an upgrade — what changes
  // is not where the team meets but whether the run may be reported as having
  // exercised the scheduler at all. Taking it clears this robot's flag, which
  // is what makes the transition terminal on both sides.
  if (held_provisional && !peer_provisional) return Adopt::kUpgrade;

  // Already holding it. The overwhelmingly common case: the proposer
  // re-publishes its triple on every heartbeat for the whole run.
  if (peer_equals_held) return Adopt::kIgnore;

  // THE REPLACEMENT THIS ROBOT IS OWED. Tested AFTER the equality above, also
  // deliberately: the proposer republishes on every heartbeat, and while it is
  // still republishing the OLD triple the numbers match and nothing has been
  // answered yet. Only a triple that actually differs can settle the request,
  // so the flag must survive the re-publications and be spent on the change.
  //
  // `!peer_provisional` mirrors the proposer's own guard on the same decision:
  // a replacement that is the centroid placeholder would trade a tour-informed
  // meeting for a position, and the request is better left outstanding than
  // answered with that. A proposer walking backwards to provisional is a fault,
  // and falls through to the refusal below where faults belong.
  if (held_reagree_due && !peer_provisional) return Adopt::kReagree;

  // Neither the first triple, nor the one upgrade, nor a replacement anyone
  // asked for. A restarted proposer, a fleet that disagrees about which robot
  // is robot 0, or two campaigns sharing a bus. There is no correct silent
  // resolution: taking it splits the fleet across two generations, and so does
  // refusing it, so the caller keeps what it has AND says so loudly. Keeping is
  // the lesser evil only because the triple already held is the one its peers
  // have echoed.
  return Adopt::kConflict;
}

}  // namespace explo_planner
