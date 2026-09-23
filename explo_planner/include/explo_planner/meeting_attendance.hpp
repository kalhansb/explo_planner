#pragma once
/// @file meeting_attendance.hpp
/// @brief Generation 33: a finished robot still comes to the meeting, so its
///        partner waits for it until it says it is leaving. DESIGN_gen33.md
///        §10, item 5 (decided 2026-09-23).
///
/// THE PROTOCOL. Since generation 32 a robot that finishes exploring with an
/// appointment standing drives to the agreed cell (keepAppointmentOnFinish),
/// exchanges maps there, and only then turns for home. Its partner must
/// therefore keep waiting for it, and stop waiting when it says it is leaving
/// — not when it says it has finished.
///
/// WHAT WAS WRONG. `finished` is announced the moment the map saturates, before
/// the drive to the meeting, and the barrier admits a finished peer as
/// accounted for (peerAccounted's third channel, "it will never arrive"). A
/// partner that heard `finished` once and then lost the radio released the
/// barrier and left while the finished robot was still driving in; the finished
/// robot then reached an empty cell and stood out its cap. The mode level had
/// the same fault: MODE_DONE was derived from the same latch as `finished`, so
/// "will it take part?" (mode >= HOMING) answered no for a robot on its way to
/// the meeting.
///
/// THE FIX IS TWO HALVES OF ONE STATEMENT, and both are here so they can be run:
///   * announcedMode — the publisher says DONE only once the finished robot is
///     no longer keeping its appointment. While it keeps it, the level stays
///     below HOMING: it is taking part.
///   * finishedPeerStillComing — the barrier's question: is there a peer that
///     finished, has not said it is leaving, and that this robot cannot hear?
///     The node holds the appointment barrier for such a peer, bounded by the
///     latched-hold cap (the node's holdingForFinishedPeer).
///
/// `finished` itself is unchanged, and so is every consumer of it:
/// peerAccounted, reachablePeerCount, the allocator, the scheduler and the
/// reconnect gate all read it exactly as before. The hold is a veto inside the
/// appointment barrier's release predicate and nowhere else, because
/// peerAccounted feeds ~30 teamComplete call sites (DESIGN §4, the correction
/// under the finished-consumer table).
/// Moved comments: doc/explo_planner_code_notes.md

#include <cstdint>

#include "explo_planner/team_model.hpp"

namespace explo_planner {

/// TeamWorld.msg's mode levels, mirrored so this file and its tests do not
/// depend on the generated message type. The node static_asserts that they
/// match. Ordered, merged by max, extended only at the top.
constexpr uint8_t kModeExploring = 0;
constexpr uint8_t kModeHoming    = 1;
constexpr uint8_t kModeDone      = 2;

/// Level published in TeamWorld/mode. DONE latches in done_announced once
/// finished and not keeping an appointment (max-merged levels never go down); a
/// finished robot keeping its appointment is not DONE.
/// (notes: meeting-announced-mode)
uint8_t announcedMode(bool finished_announced, bool keeping_appointment,
                      bool homing_announced, bool& done_announced);

/// First peer by fleet id that finished, has not said it is leaving (mode below
/// HOMING) and cannot be heard (not direct, one-way or in the comms closure);
/// -1 if none. Heard or unfinished peers are not its case.
/// (notes: meeting-finished-peer-coming)
int finishedPeerStillComing(const TeamModel& team, int self_id);

/// An appointment walker's two moves, from the node's readings of the team.
/// `settled` is teamSettled and `reachable` the release's door (teamComplete
/// over reachablePeerCount); BOTH count a finished peer as present, heard or
/// not. `holding` is holdingForFinishedPeer: such a peer, unheard and not
/// leaving, is not present, and the veto says so. Only the release applied the
/// veto before, so two finished walkers stopped short could neither release
/// nor drive on (DESIGN_gen33.md §10, known issue 2).
///
/// walkerJoinsBarrier: the settle conversion, and the dwell that clocks it —
/// stop on the road and join the barrier. Mesh only, never while the veto
/// holds.
bool walkerJoinsBarrier(bool settled, bool holding);

/// walkerResumesDrive: a walker the conversion stopped short drives on. The
/// negation of the release's "together" half, `(settled || reachable) &&
/// !holding`. Never true on a reading where walkerJoinsBarrier is, so a walker
/// cannot flip between the two each tick.
bool walkerResumesDrive(bool settled, bool reachable, bool holding);

}  // namespace explo_planner
