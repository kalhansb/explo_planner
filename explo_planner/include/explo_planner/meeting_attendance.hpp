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

#include <cstdint>

#include "explo_planner/team_model.hpp"

namespace explo_planner {

/// TeamWorld.msg's mode levels, mirrored so this file and its tests do not
/// depend on the generated message type. The node static_asserts that they
/// match. Ordered, merged by max, extended only at the top.
constexpr uint8_t kModeExploring = 0;
constexpr uint8_t kModeHoming    = 1;
constexpr uint8_t kModeDone      = 2;

/// The level this robot publishes in TeamWorld/mode.
///
///   finished_announced  the publisher's `finished` latch (coverage latched,
///                       or reached State::DONE)
///   keeping_appointment it is in a manoeuvre started to keep an appointment
///                       (appointment_manoeuvre_) — driving to the agreed
///                       cell, or standing at its barrier
///   homing_announced    the publisher's homing latch
///   done_announced      the DONE latch, owned by the caller; set here
///
/// DONE IS LATCHED, like every level on this field: max-merge cannot go back
/// down, so a level the robot later contradicts would be a permanent lie on
/// every peer. Once finished and not keeping an appointment, the robot is DONE
/// for the rest of the run, even if a later tick reads keeping again.
///
/// A FINISHED ROBOT KEEPING ITS APPOINTMENT READS EXPLORING, which on this
/// field means "no evidence it is leaving" (TeamWorld.msg/robot_mode), not
/// "confirmed exploring". Whether its run is over is `finished`'s question,
/// and `finished` still says yes.
uint8_t announcedMode(bool finished_announced, bool keeping_appointment,
                      bool homing_announced, bool& done_announced);

/// The first peer, by fleet id, that has finished exploring, has not said it
/// is leaving (mode below HOMING), and that this robot cannot currently hear —
/// not direct, not heard one way, not in the comms closure. -1 if none.
///
/// A PEER THIS ROBOT CAN HEAR IS NOT THIS FUNCTION'S CASE. Its mode reaches us
/// fresh, it is accounted for by a radio channel, and the barrier's existing
/// terms decide it — including the still-driving veto, which covers a finished
/// robot walking in while in range. What this adds is the peer whose last word
/// was "finished, coming", heard once and then lost to the radio.
///
/// An UNFINISHED absent peer is not this function's case either: the barrier
/// already waits for it (the unbounded appointment vigil).
int finishedPeerStillComing(const TeamModel& team, int self_id);

}  // namespace explo_planner
