#pragma once
/// @file rendezvous_scheduler.hpp
/// @brief Where and when a separated pair should meet, derived from the tours
///        they were already driving. See docs/mtare_evolution_plan.md §3.5.
///
/// THE IDEA THIS FILE EXISTS TO EXPRESS: a meeting is not a detour to a
/// landmark, it is a CONSTRAINT ON THE TOURS the robots were already going to
/// drive. Two robots sweeping a shared ROI pass near each other repeatedly; the
/// rendezvous should be the cheapest of those space-time near-misses, not a
/// third place both must pay to reach.
///
/// That is a reaction to two measured failures, not a preference:
///
///   * The v2/v3 design optimised a geometric minimax 1-centre over frontier
///     cells — the distance BETWEEN FRONTIERS, a quantity no robot pays. It is
///     blind to where the robots are, where they are going, and what the trip
///     costs. mTARE's own version is worse: it offsets past the robot rows of
///     its distance matrix, so robot positions are excluded from the objective
///     by construction and nothing bounds the meeting point's distance from the
///     team (§3.5.1).
///   * The shipped fallback — the midpoint of the last-contact pose pair — sits
///     BEHIND both robots on ground they have already covered. In campaign mh1
///     the value gate declined 100 of 110 evaluations, every one on cost and
///     none for lack of knowledge, which is precisely the signature of a
///     destination worth nothing: reconnecting there can only cost more than
///     not reconnecting.
///
/// So the objective here is TOUR-RELATIVE. Score a candidate cell by the
/// makespan penalty of forcing every robot's tour through it, and take the
/// argmin. The robot whose tour already contains the cell pays nothing; the
/// other pays one insertion. Both terms are the allocator's own `routeCostMm`,
/// so there is no second cost model to keep consistent with the first.
///
/// WHY THERE IS NO AGREEMENT PROTOCOL IN THIS FILE. global_allocator.hpp
/// already guarantees tours are bit-identical ACROSS PROCESSES — integer-mm
/// quantisation, a total tie-break order, no clock, no random, no
/// address-derived value. A rendezvous derived purely from that output is
/// bit-identical too. There is no proposal, no echo, no lowest-id adoption, no
/// freeze-on-agreed and no convergence bound to prove, because agreement is a
/// CONSEQUENCE of identical arithmetic over a shared world — the same argument
/// the allocator already makes for allocation itself. This file inherits that
/// property and must not spend it:
///
///   * every quantity that reaches a comparison is integral (mm, ms);
///   * every tie breaks on cell id, ascending, which both robots agree on;
///   * the vehicle set is walked in fleet-id order, so a caller's vector order
///     cannot reach the result;
///   * nothing here reads a clock. `mission_elapsed_ms` is a parameter, and it
///     is MISSION-ELAPSED rather than an absolute stamp because field clocks
///     drift hours apart (the 2026-07-06 bunker/curt bags were 4531 s apart
///     while recording simultaneously).
///
/// WHAT THIS DELIBERATELY DOES NOT DO: re-derive the appointment after contact
/// is lost. Once the worlds diverge the tours diverge, so a re-derived
/// rendezvous would differ between robots — which is the one thing the design
/// cannot survive. The caller freezes the plan computed from the last
/// BIDIRECTIONALLY CONFIRMED exchange and drives that; see §3.5 and the node's
/// `reconnect_rec_` for the same discipline applied to the pose pair.

#include <cstdint>
#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"
#include "explo_planner/global_allocator.hpp"

namespace explo_planner {

/// An appointment: a cell to meet in and a mission-elapsed time to be there.
///
/// Everything that produced it is carried alongside, for the same reason the
/// reconnection gate carries its arithmetic: an appointment nobody can
/// re-derive offline is one nobody can audit, and the whole design rests on
/// two robots computing the same numbers from different processes.
struct RendezvousPlan {
  /// The agreed cell, or -1 when none could be derived (see `refused`).
  int cell = -1;

  /// Mission-elapsed milliseconds at which both robots should be at `cell`.
  /// -1 when there is no plan.
  long long t_meet_ms = -1;

  /// Makespan penalty of the winning cell, in the allocator's quantised unit.
  /// Zero is normal and good: it means the winner was already on the critical
  /// robot's tour and the meeting costs the team nothing.
  long long penalty_mm = -1;

  /// The UNCAPPED interval — max over robots of "time to reach `cell` along
  /// its own tour". `t_meet_ms - mission_elapsed_ms` differs from this exactly
  /// when `capped` is true, and the difference is how late the slowest robot
  /// will be.
  long long interval_ms = -1;

  /// True when the divergence cap pulled the meeting earlier than the tours
  /// imply. The far robot is then structurally late by `interval_ms -
  /// (t_meet_ms - mission_elapsed_ms)`; the wait cap absorbs it. Reported
  /// rather than hidden because a run where this is always true is one where
  /// the cap, not the objective, is choosing the meeting time.
  bool capped = false;

  /// The midpoint floor won the argmin — no tour cell was worth its detour, so
  /// the behaviour degrades to exactly what the shipped implementation does.
  bool floor_won = false;

  /// Admissible candidates considered, and the two reasons a candidate was
  /// dropped. A plan with `candidates == 1` is one where only the floor
  /// survived, which is a different situation from one where the floor won on
  /// merit, and the P5 non-vacuity rule needs to tell them apart.
  int candidates = 0;
  int rejected_unreachable = 0;
  int rejected_excluded = 0;

  /// Non-empty when no appointment could be derived at all. The caller must
  /// fall back to its pre-P5 behaviour rather than treating this as "meet at
  /// cell -1"; an empty plan is a NORMAL outcome early in a run, before the
  /// allocator has any tours to speak of.
  std::string refused;

  bool valid() const { return cell >= 0 && refused.empty(); }
};

class RendezvousScheduler {
public:
  struct Config {
    /// Conservative drive speed, in mm/s. Integer so that travel times
    /// quantise exactly and two robots converting the same distance land on
    /// the same millisecond. A float speed would reintroduce, at the last
    /// division, precisely the cross-process disagreement the allocator spent
    /// its whole design removing.
    ///
    /// Non-positive disables the scheduler outright (every solve refuses),
    /// because a zero speed makes every arrival time infinite and there is no
    /// safe value to substitute.
    long long speed_mm_s = 500;

    /// Multiplier on travel time in the departure test, in thousandths. 1200 =
    /// leave when 1.2x the estimated drive remains. It exists because the
    /// estimate is a straight-line-ish graph distance at a nominal speed and
    /// the real drive is neither; being early costs waiting, being late costs
    /// the meeting.
    int depart_safety_milli = 1200;

    /// Additive slack on the departure test, in ms — the fixed overhead of
    /// abandoning what you were doing and getting a goal accepted.
    long long depart_margin_ms = 0;

    /// Upper bound on the interval from now to `t_meet`, in ms. <= 0 means
    /// uncapped. The node feeds this from the map-divergence model it already
    /// runs (`rate_sum`, explo_planner_node.cpp): meet sooner when the two
    /// maps are diverging fast enough that the exchange is worth more than the
    /// tours alone imply. That model is symmetric in the two robots, so both
    /// ends cap identically — which is the only reason a cap is admissible
    /// here at all.
    long long max_interval_ms = 0;

    /// Cells to drop from consideration: the caller's no-show list. This is
    /// the anti-deadlock rule. mTARE waits at its rendezvous forever and has
    /// no handler for a peer that never arrives (§3.5.1, edge 5); here a cell
    /// that has twice failed to produce a meeting stops being proposed and the
    /// argmin moves on.
    ///
    /// It NEVER applies to the floor. The floor is the midpoint of two poses
    /// that were in radio contact, so it is at most half a comms range from
    /// each robot and is the one destination that cannot be argued away — if
    /// exclusion could empty the candidate set, the no-show handler would have
    /// reinvented the deadlock it exists to prevent.
    std::vector<int> exclude;
  };

  /// Derive the appointment.
  ///
  /// `robots` is the vehicle set exactly as it was handed to
  /// GlobalAllocator::solve, and `alloc` is that solve's result — index-aligned
  /// with `robots`, which is what lets this function attribute a tour to a
  /// starting cell without searching by id. Handing in an allocation solved
  /// from a DIFFERENT vehicle set is the one misuse that cannot be detected
  /// here and will silently produce a plan neither robot can keep.
  ///
  /// `floor_cell` is the last-contact midpoint mapped through the grid, or -1
  /// when that point lies outside the ROI (in which case the caller still has
  /// the raw midpoint and should keep using it). It is admitted unconditionally
  /// — see Config::exclude and the reachability note on `solve`'s admissibility
  /// rule.
  ///
  /// `mission_elapsed_ms` is the shared clock. It is the ONLY time input, and
  /// it is a parameter rather than a call to now() so this stays a pure
  /// function of values both robots hold.
  static RendezvousPlan solve(const CellWorld& world,
                              const std::vector<AllocRobot>& robots,
                              const Allocation& alloc,
                              int floor_cell,
                              long long mission_elapsed_ms,
                              const Config& cfg);

  /// Time to cover `mm` at the configured speed, in ms. Integer division:
  /// truncation is deterministic and costs at most a millisecond, where a
  /// double would cost cross-process agreement.
  static long long travelMs(long long mm, long long speed_mm_s);

  /// THE DEPARTURE RULE (§3.5): leave when the remaining countdown no longer
  /// covers the drive.
  ///
  ///     t_meet - now  <=  travel * safety + margin
  ///
  /// PER-ROBOT, which is the whole point: `travel` is MY distance to the cell,
  /// so the robot further away leaves earlier, DEPARTURES STAGGER AND ARRIVALS
  /// COINCIDE. Nothing about the trigger is synchronised and nothing needs to
  /// be — the shared quantity is `t_meet`, not the moment of leaving.
  ///
  /// This is precisely the leave-early test mTARE computes and then throws
  /// away: `rendezvous_manager.cpp:141-142` comments the comparison out and
  /// leaves `time_to_rendezvous` as a dead parameter, so its robots depart AT
  /// zero and each is late by its own travel time — a lateness that is
  /// heterogeneous across the team and therefore cannot be compensated by any
  /// single interval constant (§3.5.1, edge 1).
  ///
  /// Returns true once overdue, so a robot that discovers a passed deadline
  /// leaves immediately rather than waiting for the next one.
  static bool shouldDepart(long long t_meet_ms, long long mission_elapsed_ms,
                           long long travel_ms, const Config& cfg);
};

}  // namespace explo_planner
