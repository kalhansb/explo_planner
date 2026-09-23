#pragma once
/// @file exchange_drain.hpp
/// @brief Generation 33, R3: end the rendezvous map-exchange hold on the
///        exchange, not on the clock. DESIGN_gen33.md Part 2.
///
/// The settle hold is a guess at how long an exchange takes, applied
/// identically to a meeting that finished in four seconds and one that is still
/// delivering at thirty. With Part 1's per-peer fusion counters the exchange
/// itself is observable, so the trigger can be the thing the hold is for: has
/// every peer this robot believes is here delivered what it had.
///
/// One call per RETURN_SYNC tick while the hold is on, from doReturnSync. The
/// node keeps the state (a DrainWindow), the logging and the latch; this file
/// is the predicate and nothing else, so the part of the design that decides
/// when robots leave meetings can be run by a test instead of only scanned.
///
/// WHY IT LIVES HERE. It used to be inline in explo_planner_node.cpp, which is
/// not in explo_planner_lib and defines main(), so nothing could link it and
/// test-plan 7 and 8's behavioural halves could not be written as tests at all
/// (§10). The extraction is verbatim: same order of tests, same arithmetic,
/// same state transitions, and the node's log lines are unchanged.

#include <cstdint>
#include <vector>

#include "explo_planner/team_model.hpp"

namespace explo_planner {

/// A TUMBLING WINDOW, ON THE SETTLE'S CLOCK. `start_sec` is the start of the
/// window currently being measured, as an offset into the settle (one hold,
/// one epoch, nothing extra to keep in step); -1 while no window is open.
/// `base` is the fusion-counter vector as it read when the window opened; the
/// rate is the difference over the window divided by its length, which is why
/// a window is closed and re-opened rather than slid — a ring buffer would buy
/// sub-W granularity on a decision whose whole point is that it is taken at
/// the end of a quiet interval.
///
/// IT IS ALSO THE HARD FLOOR. No window has elapsed before W seconds, so no
/// drain release can happen before then, without a second mechanism saying so.
///
/// Reset to a default-constructed DrainWindow wherever the hold is cleared, so
/// the next visit starts with no window open.
struct DrainWindow {
  double                start_sec = -1.0;
  std::vector<uint64_t> base;
};

enum class DrainStep {
  kHold,        ///< keep standing on the cell; keep heartbeating
  kDrained,     ///< every believed peer spoke, then fell below R over W
  kUnfinished,  ///< the cap arrived first — leave, but do not call it drained
  /// Nobody here (direct or relayed), and every peer has said it is leaving
  /// (mode >= HOMING): there is no exchange left to wait for. Neither drained
  /// nor unfinished — no exchange was expected — and the node logs it as its
  /// own event. Judged at the full-window instants only, like the rest.
  kAllPeersLeaving,
};

/// What one step decided and what it saw. `mute` and `examined` are what the
/// node's UNFINISHED EXCHANGE line reports; `measurable` is false when the
/// counters could not be differenced at all, which the node warns about.
struct DrainReading {
  DrainStep step       = DrainStep::kHold;
  bool      evaluated  = false;  ///< a full window had elapsed and was judged
  bool      measurable = false;  ///< all three counter vectors were fleet-sized
  int       mute       = 0;      ///< examined peers still at the hold-start level
  int       examined   = 0;      ///< peers believed present (direct or relayed)
  int       others     = 0;      ///< fleet peers in the model, self excluded
  int       leaving    = 0;      ///< of those, how many said homing or done
};

/// One tick of the drain release.
///
///   window       the node's DrainWindow; opened on the first call, rolled
///                forward on every evaluation that does not drain
///   settled_sec  seconds since the hold started
///   counters     the current per-peer deltas_received, indexed by fleet id
///   hold_base    the same vector as it read when the hold started
///   team         the belief model — who this robot thinks is here
///   fleet_size   fleet_.size(); every vector must be this long to be read
///   self_id      skipped; a robot does not wait on its own map
///   rate_vox_sec R, per peer. Validated > 0 by the node's configure()
///   window_sec   W. Validated > 0 and < cap_sec
///   cap_sec      rendezvous_latched_hold_sec — past it, not drained means
///                kUnfinished rather than kHold
DrainReading stepDrainRelease(DrainWindow& window, double settled_sec,
                              const std::vector<uint64_t>& counters,
                              const std::vector<uint64_t>& hold_base,
                              const TeamModel& team, int fleet_size, int self_id,
                              double rate_vox_sec, double window_sec,
                              double cap_sec);

}  // namespace explo_planner
