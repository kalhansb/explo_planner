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
///   * The then-shipped fallback — the midpoint of the last-contact pose pair —
///     sits BEHIND both robots on ground they have already covered. In campaign
///     mh1 the value gate declined 100 of 110 evaluations, every one on cost and
///     none for lack of knowledge, which is precisely the signature of a
///     destination worth nothing: reconnecting there can only cost more than
///     not reconnecting. (The midpoint was removed outright on 2026-09-16 — see
///     the removal notes in dispatchReconnect. The FLOOR MACHINERY HERE IS NOT
///     GONE WITH IT and is not test-only: since 2026-09-18 the node fills
///     `floor_cell` with the centroid of the team's own vehicle cells on every
///     solve. Read `floor_cell` on `solve` below for why that is a different
///     quantity from the midpoint and what it is there to prevent.)
///
/// So the objective here is TOUR-RELATIVE. Score a candidate cell by the
/// makespan penalty of forcing every robot's tour through it, and take the
/// argmin. The robot whose tour already contains the cell pays nothing; the
/// other pays one insertion. Both terms are the allocator's own `routeCostMm`,
/// so there is no second cost model to keep consistent with the first.
///
/// THIS FILE IS NOT THE AGREEMENT PROTOCOL. It used to be — the claim here was
/// that no protocol was needed at all, because global_allocator.hpp guarantees
/// tours are bit-identical ACROSS PROCESSES (integer-mm quantisation, a total
/// tie-break order, no clock, no random, no address-derived value), so a
/// rendezvous derived purely from that output would be bit-identical too, and
/// agreement would be a CONSEQUENCE of identical arithmetic over a shared
/// world.
///
/// THE ARITHMETIC HELD. THE SHARED WORLD DID NOT. Every determinism property
/// below is still true and still enforced; what was false was the premise that
/// the two robots feed this function the same inputs. They do not: each solves
/// over its own privately merged map, and a merge is a function of what the
/// radio happened to deliver. Measured on the banked ts3 n2+n3 cells, where
/// each robot solved privately at its own arming instant: 21 of 64 separated
/// pairs picked the same cell, and the median t_meet disagreement was 91 s at
/// N=2 and 125 s at N=3. Nine pairs disagreed with IDENTICAL `shared_hash` —
/// the candidate COUNT differed in 5 of 19 — so the hash was never a complete
/// witness of a shared world either.
///
/// So the agreement protocol now lives ONE LEVEL UP, in the node: the robot
/// with the lowest fleet id calls this function, publishes the resulting
/// (cell, interval, t_meet) triple on TeamWorld, and everyone else adopts those
/// three integers verbatim and echoes them back until all N hold the same
/// triple. See TeamWorld.msg's rendezvous block and the node's P5 state block.
/// What that makes THIS file is the proposer's search, not a replicated
/// computation.
///
/// THE INSTANT IS ON THE WIRE, NOT RE-DERIVED (2026-09-17). Until then only
/// (cell, interval) were published and each robot supplied its own origin —
/// the moment IT saw the team stop being mutually whole. At N=2 that is one
/// physical event and the two origins landed 0.84 s apart. At N>=3 it is not
/// one event at all, because the graph comes apart edge by edge and each robot
/// sees a different edge go first: the ts4 N=3 cell put three robots' origins
/// at 13.18 / 34.18 / 33.98 s, so three robots holding a byte-identical
/// interval kept it 21 s apart. Publishing the instant removes the per-robot
/// term outright and leaves only the robots' mission-clock baselines, measured
/// ~1.2 s apart across the same cells.
///
/// The determinism discipline is kept anyway, because it is what makes a
/// proposal auditable offline and because a solve that varied with vector
/// order would be untestable:
///
///   * every quantity that reaches a comparison is integral (mm, ms);
///   * every tie breaks on cell id, ascending;
///   * the vehicle set is walked in fleet-id order, so a caller's vector order
///     cannot reach the result;
///   * nothing here reads a clock. `mission_elapsed_ms` is a parameter, and it
///     is MISSION-ELAPSED rather than an absolute stamp because field clocks
///     drift hours apart (the 2026-07-06 bunker/curt bags were 4531 s apart
///     while recording simultaneously). The node passes its own mission-elapsed
///     ms at proposal time, so `t_meet_ms` comes out in the one frame every
///     robot in the fleet can evaluate without a clock exchange.
///
/// WHAT THIS DELIBERATELY DOES NOT DO: get called after contact is lost. Once
/// the worlds diverge the tours diverge, so a re-derived rendezvous would
/// differ between robots — which is the one thing the design cannot survive.
/// The node solves only while the team reads COMPLETE and freezes the agreed
/// pair the instant it does not; see §3.5 and the node's `reconnect_rec_` for
/// the same discipline applied to the pose pair.
/// Moved comments: doc/rendezvous_scheduler_notes.md

#include <cstdint>
#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"
#include "explo_planner/global_allocator.hpp"

namespace explo_planner {

/// An appointment: a cell and a mission-elapsed time to meet there, plus
/// everything that produced it so it can be re-derived offline.
/// (notes: rdv-plan-auditable)
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

  /// Recurrence period on the wire; must match byte for byte across the fleet.
  /// On a solve() result t_meet_ms = mission_elapsed_ms + interval_ms; not on
  /// the node's appointment_, where arming rewrites t_meet_ms.
  /// (notes: rdv-interval-ms)
  long long interval_ms = -1;

  /// Raw tour term (max over robots of time to reach cell along its own tour)
  /// before the floor or cap. Under a cap the slowest robot is late by
  /// tour_interval_ms - interval_ms. (notes: rdv-tour-interval)
  long long tour_interval_ms = -1;

  /// True when the reachability floor raised the interval above
  /// min(tour_interval_ms, max_interval_ms). Expected true on most rows; not a
  /// defect. (notes: rdv-floored-flag)
  bool floored = false;

  /// True when the cap pulled the period below what the tours imply; the
  /// barrier wait absorbs the lateness. The floor outranks the cap. Not
  /// exclusive with floored: both true means interval_ms > max_interval_ms.
  /// (notes: rdv-capped-flag)
  bool capped = false;

  /// The committed cell is the caller's floor_cell (not the old midpoint). An
  /// identity, not a verdict: the floor is re-added to the candidates
  /// unconditionally, so read it with penalty_mm. (notes: rdv-floor-won)
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
    /// Conservative drive speed, mm/s, an integer so travel times quantise
    /// identically on every robot. Non-positive disables the scheduler: every
    /// solve refuses. (notes: rdv-speed-mm-s)
    long long speed_mm_s = 500;

    /// Travel-time multiplier in thousandths (1200 = 1.2x). Must be the same
    /// constant for the floor in solve() and
    /// ExploPlannerNode::appointmentLeadMs. Name kept for params-row
    /// compatibility. (notes: rdv-depart-safety-milli)
    int depart_safety_milli = 1200;

    /// Lower bound on the interval, ms; <= 0 means no floor. Stops the interval
    /// degenerating (the objective's raw winner can be one grid step). The node
    /// feeds it from rendezvous_interval_sec, which exists only for this.
    /// (notes: rdv-min-interval)
    long long min_interval_ms = 0;

    /// Upper bound on the interval, ms; <= 0 means uncapped. The findability
    /// bound: the node passes rendezvous_appointment_wait_sec (0, uncapped, on
    /// campaign defaults). The floor outranks this cap.
    /// (notes: rdv-max-interval-findability)
    long long max_interval_ms = 0;

    /// Cells to drop from consideration. Retired: the node clears it before
    /// every solve. It never applies to floor_cell, which is re-added so
    /// exclusion cannot empty the candidate set. (notes: rdv-exclude-retired)
    std::vector<int> exclude;
  };

  /// robots and alloc must be the vehicle set and result of one
  /// GlobalAllocator::solve (index-aligned). floor_cell is a guaranteed
  /// candidate, -1 for none (solve may then refuse). mission_elapsed_ms is the
  /// only time input. (notes: rdv-solve-inputs)
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

  // Recurrence and departure are not computed here: see nextAgreedOccurrence
  // (planner_util.hpp) and ExploPlannerNode::appointmentDue.
  // depart_safety_milli also sizes the node's departure lead.
  // (notes: rdv-departure-rule-history)
};

// ---------------------------------------------------------------------------
// THE HANDSHAKE, as opposed to the solve above.
// ---------------------------------------------------------------------------
//
// Only the handshake decisions: no ROS, clock or I/O. The node keeps control
// flow, logging and its own RendezvousProposal. test_rendezvous_scheduler.cpp
// enumerates the transitions. (notes: rdv-handshake-decisions)
struct RendezvousHandshake {
  /// What a non-proposer does with the proposer's triple. Permitted sequence:
  /// empty, placeholder P, R, R2, ...; every step after the first two must be
  /// earned by keeping the meeting (held_reagree_due).
  /// (notes: rdv-adopt-sequence)
  enum class Adopt {
    kIgnore,     ///< Nothing to adopt, or already holding exactly this.
    kTake,       ///< Hold nothing; take the proposer's triple.
    kUpgrade,    ///< Hold the placeholder and the proposer has moved off it.
    kReagree,    ///< Kept the last meeting; this is the replacement it is owed.
    kConflict,   ///< The proposer is publishing a triple that is none of those.
  };

  /// @param peer_valid        the proposer is publishing a usable triple
  /// @param peer_provisional  ...and it is flagged as the centroid placeholder
  /// @param held_valid        this robot already holds a triple
  /// @param held_provisional  ...and what it holds is the placeholder
  /// @param peer_equals_held  the two triples' three integers are identical
  /// @param held_reagree_due  this robot kept the meeting it holds and is
  ///                          waiting to be told the next place and time
  ///
  /// Flags, not triples, are passed. peer_equals_held compares the three
  /// integers only, independent of the provisional flags.
  /// (notes: rdv-adopt-flags)
  ///
  /// `held_reagree_due` is the CALLER'S to clear, and it must clear it on
  /// kReagree. Left set it degrades to "accept any replacement forever", which
  /// is the fleet-split this enum's refusals exist to prevent.
  static Adopt adopt(bool peer_valid, bool peer_provisional,
                     bool held_valid, bool held_provisional,
                     bool peer_equals_held, bool held_reagree_due);

  /// Update what peer id was heard to hold. A message that disagrees with the
  /// record clears it on arrival; a record only counts if it matches what this
  /// robot holds. (notes: rdv-peer-latch)
  template <class Triple>
  static void updatePeerLatch(const Triple& heard, const Triple& held,
                              Triple& latched) {
    // Order matters: clear on contradiction first, then re-record, so a peer
    // switching triples in one message loses the old record and gains the new
    // one, or keeps none if the new one is not ours.
    // (notes: rdv-peer-latch-order)
    if (!(heard == latched)) latched = Triple{};
    if (heard.valid() && heard == held) latched = heard;
  }
};

}  // namespace explo_planner
