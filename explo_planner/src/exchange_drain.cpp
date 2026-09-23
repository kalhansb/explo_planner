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
  // THREE VECTORS, ALL FLEET-SIZED OR THE READING IS NOT A READING. The
  // counters are empty until dscovox publishes them, and the hold-start
  // baseline is a copy of whatever they were then — so a meeting that began
  // before the counters came up has no interval to difference. That is
  // UNMEASURED, and the safe reading of unmeasured is "not drained": hold to
  // the cap and say the exchange did not finish, rather than release on the
  // absence of evidence.
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
    // BELIEVED PRESENT, which is what the robot actually has. The detector's
    // false-positive rate (2.948% of scored grid points read direct while the
    // oracle says down) is inherited here and not claimed away: a robot can
    // hold for a peer that is not really there. The cap below is what bounds
    // that, and it bounds it to a wasted wait rather than a stall.
    if (!team.configured() || id >= team.size()) continue;
    // DIRECT OR RELAYED. The question at this loop is "is there somebody here
    // to trade maps with", which is a radio statement, and a two-hop peer
    // answers it: the emulator forwards serialized bytes without deserializing,
    // so dscovox credits the robot that SENSED the voxels rather than the one
    // that bridged them (see ScovoxFusionCounters.msg). Gen-32's census
    // measured relayed rows applying a merge 15.5% of the time against 0.9%
    // overall — the most productive channel on the team — so a direct-only
    // filter would let the busiest stream on the meeting keep arriving while
    // this test declared the exchange finished.
    //
    // The two flags are disjoint, so this is the whole filter and not half of
    // one: team_model.cpp's closure loop opens with `if (p.direct) continue;`,
    // which is why via_relay can never be set on a direct peer.
    //
    // FINISHED IS NOT PRESENT (test-plan 7). `finished` is sticky and says
    // nothing about the radio: a peer that finished and drove off is still
    // finished. Counting it would hold a drained exchange open against a robot
    // that is not here to deliver anything. (A finished peer still on its way
    // here does not reach this loop absent: the barrier holds for it before
    // the hold opens — meeting_attendance.hpp.)
    const auto& p = team.peer(id);
    if (!p.direct && !p.via_relay) continue;
    ++r.examined;
    const size_t k = static_cast<size_t>(id);
    const uint64_t nowv = counters[k];
    // CLAUSE 1 — DID IT SPEAK AT ALL THIS VISIT. A level against the
    // hold-start baseline, not a rate. This is the clause that cannot be
    // optimised away: a peer whose bytes are not crossing the radio and a peer
    // that has sent everything it has BOTH present a rate of zero over the
    // window, and dropping this would make the release fire fastest in exactly
    // the blackout the hold exists to sit through.
    if (nowv <= hold_base[k]) {
      ++r.mute;
      drained = false;
      continue;
    }
    // CLAUSE 2 — HAS IT STOPPED. A rate, not a zero test: a third of the long
    // gen-32 N=2 meetings were still gaining when the robot departed, so "the
    // counter has stopped moving" over-holds, while "the counter has fallen
    // below R" does not.
    const uint64_t wb = window.base[k];
    const double rate =
        (nowv > wb) ? static_cast<double>(nowv - wb) / win : 0.0;
    if (rate >= rate_vox_sec) drained = false;
  }
  // NOBODY READ IS NOT EVERYBODY DRAINED. Every `continue` above is a peer
  // this robot could not read — model unconfigured, id past its end, or
  // believed not here — and with all of them taken the loop falls out leaving
  // `drained` at the true it started on, releasing the hold having examined no
  // one. That is the same absence of evidence `measurable` refuses a few lines
  // up, arriving through a different door: the hold opens on
  // max(active, reachablePeerCount), which counts peers this loop is entitled
  // to skip.
  //
  // EXCEPT WHEN EVERY PEER HAS SAID IT IS LEAVING (2026-09-23). That is not an
  // absence of evidence but positive evidence, first-hand or relayed: each one
  // announced homing or done, and there is nobody left to come and trade maps
  // with. Holding to the cap for them is the wait the partner protocol exists
  // to end — a robot stops waiting when its partner says it is leaving. All of
  // them, not any: one peer that has not said so could still be walking in,
  // and that is the case this backstop is for. And NOBODY HERE, counted on its
  // own rather than read off `examined`: the loop above does not run at all
  // when the counters are unmeasurable, so `examined` is 0 there with a peer
  // standing on the cell — and a peer that is here and leaving may still have
  // bytes crossing, which is the counter's question, not this one's.
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
