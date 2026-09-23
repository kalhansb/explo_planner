#pragma once
/// @file reconnect_gate.hpp
/// @brief The economic reconnection gate (mTARE's "hybrid"): decide whether
///        interrupting exploration to go find a missing peer is worth what it
///        costs. See docs/mtare_evolution_plan.md §3.6.
///
/// Today's mid-run trigger is a SILENCE CLOCK: the team has read incomplete for
/// T seconds, therefore go. That is a proxy for value, and a poor one — it
/// cannot tell a peer who has been quietly re-covering ground I already hold
/// (nothing to gain) from one carrying half the map I have never seen (a lot).
/// This file replaces the "therefore" with two questions:
///
///   1. KNOWLEDGE. Is there anything to say? Some cell of mine that is not
///      UNSEEN must be absent from the missing peer's `known_by` mask. This is
///      a port of mTARE's HasKnowledgeToShare, and it is meaningful ONLY under
///      the reset-on-change semantics of §3.1: `known_by` collapsing to {self}
///      on every committed status change is what makes "the peer is in this
///      mask" mean "the peer knows the CURRENT status", rather than the useless
///      "the peer once heard something about this cell".
///
///   2. VALUE. Is it worth saying? Price the two futures and compare:
///        C_no = makespan of the plan I would run if we never reconnect, with
///               the missing peer masked to cells it already knows;
///        C_re = the cost of my leg out to the peer, plus the makespan of the
///               plan we would run together once reconnected.
///      Dispatch iff C_re < C_no, STRICTLY. Equality is a no: paying a real
///      drive for a provably identical plan is a loss, and a `<=` here would
///      turn every degenerate case (nothing left to explore, both makespans
///      zero) into a dispatch.
///
/// WHY THIS FAILS OPEN. Every path that cannot actually evaluate the
/// arithmetic — an unconfigured world, an allocator refusal, a peer whose
/// position is unknown — returns `dispatch = true` with a reason in `refused`.
/// The alternative is a gate that silently suppresses reconnection whenever the
/// allocator is unhappy, which from the outside is indistinguishable from a
/// gate that is working. A gate can only be trusted if its quiet state is
/// "dispatched anyway, here is why".
///
/// WHAT IT DELIBERATELY MIS-PRICES, stated here because the numbers are logged
/// and someone will read them:
///
///   * The leg is counted ONCE, from my current cell to the peer's last known
///     cell. The post-meeting displacement is not modelled: the assume-comms
///     makespan is solved from where both robots are NOW, not from where they
///     would be after meeting. That UNDER-prices the manoeuvre, so the gate
///     errs towards dispatching.
///   * In `rendezvous` mode the true leg is roughly half this (both robots
///     move), so the gate OVER-prices there and errs towards suppressing.
///   Neither is corrected, because a travel model accurate enough to matter
///   would need the manoeuvre chosen before the decision that chooses it.
///   Both directions are recoverable from the logged `leg_mm`.
/// Moved comments: doc/explo_planner_code_notes.md

#include <cstdint>
#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"
#include "explo_planner/global_allocator.hpp"

namespace explo_planner {

/// A missing peer, as the gate needs to see it.
struct MissingPeer {
  int  id       = -1;     ///< fleet id; the bit index into `known_by`
  int  cell     = -1;     ///< its last known cell, -1 if we never located it
  bool finished = false;  ///< finished peers are ignored entirely (§3.6)
};

/// The verdict, and every number that produced it. All of it is logged: a
/// dispatch decision nobody can re-derive offline is a decision nobody can
/// audit, and this gate's whole risk is being quietly wrong in one direction.
struct GateVerdict {
  /// The answer. True = go.
  bool dispatch = false;

  /// Did the knowledge gate find anything to share? False here with an empty
  /// `refused` is the interesting suppression: we are provably up to date with
  /// every missing peer.
  bool knowledge = false;

  /// Cells that are not UNSEEN locally and are absent from at least one missing
  /// peer's mask. The knowledge gate is `unshared_cells > 0`; the count itself
  /// is the diagnostic that separates "just barely something to say" from "the
  /// peer is a hundred cells behind".
  int unshared_cells = 0;

  /// The priced futures, in the allocator's quantised unit (mm). -1 means the
  /// value gate did not run (knowledge said no, or something was refused).
  long long c_no_mm = -1;
  long long c_re_mm = -1;
  long long leg_mm  = -1;
  /// The missing peer `leg_mm` was priced against: the nearest locatable one.
  /// -1 when no peer was priced. Gen 34 chases this peer (Q64(1)).
  int leg_peer_id = -1;

  /// Cells no vehicle could take under the no-comms mask. Structurally zero
  /// while self, always in_comms and so exempt from the mask, is in the vehicle
  /// set; kept so the column keeps its meaning.
  /// (notes: reconnect-gate-unassigned-dead)
  int unassigned = 0;

  /// Non-empty when the arithmetic could not be evaluated. `dispatch` is then
  /// true (fail open, see the file header) and this string says why.
  std::string refused;
};

/// robots is the full vehicle set, missing peers included; the gate prices
/// fetching only the nearest locatable peer in missing. cfg.comms_mask is
/// ignored: both solves run masked, differing only in that peer.
/// (notes: reconnect-gate-evaluate)
GateVerdict evaluateReconnectGate(const CellWorld& world,
                                  const std::vector<AllocRobot>& robots,
                                  const std::vector<MissingPeer>& missing,
                                  const GlobalAllocator::Config& cfg);

/// Number of non-UNSEEN cells missing from at least one unfinished missing
/// peer's known_by. Exposed as a count so the knowledge gate is testable in
/// both failure directions. (notes: reconnect-gate-unshared-count)
int unsharedCellCount(const CellWorld& world,
                      const std::vector<MissingPeer>& missing);

/// Makespan of an allocation: the largest single-vehicle route cost, which is
/// when the LAST robot finishes and therefore when exploration is done. Not the
/// sum — a plan that gives one robot everything and the other nothing has the
/// same total as a balanced one and is twice as slow.
long long makespanMm(const Allocation& a);

}  // namespace explo_planner
