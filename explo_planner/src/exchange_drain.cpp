// Moved comments: doc/explo_planner_code_notes.md
#include "explo_planner/exchange_drain.hpp"

#include "explo_planner/meeting_attendance.hpp"

namespace explo_planner {

DrainReading stepDrainRelease(DrainWindow& window, double settled_sec,
                              const std::vector<uint64_t>& counters,
                              const std::vector<uint64_t>& hold_base,
                              const TeamModel& team, int fleet_size, int self_id,
                              double rate_vox_sec, double window_sec,
                              double cap_sec) {
  DrainReading r;
  if (window.start_sec < 0.0) {
    window.start_sec = settled_sec;
    window.base      = counters;
  }
  const double win = settled_sec - window.start_sec;
  if (win < window_sec) {
    return r;   // no full window yet — this is also the hard floor.
  }
  r.evaluated = true;
  // measurable needs all three vectors fleet-sized; the counters stay empty
  // until dscovox publishes them. Unmeasured reads as not drained: hold rather
  // than release on absent evidence. (notes: drain-fleet-sized-vectors)
  const size_t n = static_cast<size_t>(fleet_size);
  r.measurable = (counters.size() == n && hold_base.size() == n &&
                  window.base.size() == n);
  // THE LOOP RUNS TO THE END EVEN ONCE IT HAS ITS ANSWER, because the count it
  // carries out is what the log reports: a `drained &&` in the condition would
  // stop at the first mute peer and report "1" for a team where three had gone
  // dark.
  bool drained = r.measurable;
  for (int id = 0; r.measurable && id < fleet_size; ++id) {
    if (id == self_id) continue;
    // Presence is what the team model believes, so a detector false positive
    // can make the robot hold for an absent peer; cap_sec bounds that to a
    // wasted wait, not a stall. (notes: drain-believed-presence)
    if (!team.configured() || id >= team.size()) continue;
    // A peer counts if direct or via_relay: a relayed peer still streams map
    // data. The flags are disjoint (team_model.cpp), so this is the whole
    // filter. finished is not presence; a finished peer that left is skipped.
    // (notes: drain-direct-or-relayed)
    const auto& p = team.peer(id);
    if (!p.direct && !p.via_relay) continue;
    ++r.examined;
    const size_t k = static_cast<size_t>(id);
    const uint64_t nowv = counters[k];
    // Clause 1: did the peer speak at all this visit, a level against
    // hold_base, not a rate. Do not drop it: a blacked-out peer and a drained
    // one both show zero rate. (notes: drain-clause-spoke)
    if (nowv <= hold_base[k]) {
      ++r.mute;
      drained = false;
      continue;
    }
    // Clause 2: has the peer stopped, tested as a window rate below
    // rate_vox_sec rather than as zero growth, which over-holds.
    // (notes: drain-clause-rate)
    const uint64_t wb = window.base[k];
    const double rate =
        (nowv > wb) ? static_cast<double>(nowv - wb) / win : 0.0;
    if (rate >= rate_vox_sec) drained = false;
  }
  // Examining no peer is not drained. Exception: every other peer announced
  // homing or done and none is direct or via_relay, giving kAllPeersLeaving.
  // here is counted apart from examined, which is 0 when unmeasurable.
  // (notes: drain-nobody-read-all-leaving)
  int here = 0;
  if (team.configured()) {
    for (int id = 0; id < fleet_size && id < team.size(); ++id) {
      if (id == self_id) continue;
      const auto& p = team.peer(id);
      ++r.others;
      if (p.mode >= kModeHoming) ++r.leaving;
      if (p.direct || p.via_relay) ++here;
    }
  }
  if (here == 0 && r.others > 0 && r.leaving == r.others) {
    r.step = DrainStep::kAllPeersLeaving;
    return r;
  }
  if (r.examined == 0) drained = false;
  if (!drained) {
    // Roll the window forward and keep holding — up to the cap, which is the
    // same knob that bounds a latched robot's hold and is already validated to
    // exceed the settle.
    window.start_sec = settled_sec;
    window.base      = counters;
    r.step = (settled_sec < cap_sec) ? DrainStep::kHold : DrainStep::kUnfinished;
    return r;
  }
  r.step = DrainStep::kDrained;
  return r;
}

}  // namespace explo_planner
