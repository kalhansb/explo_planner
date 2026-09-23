#include "explo_planner/meeting_attendance.hpp"

namespace explo_planner {

uint8_t announcedMode(bool finished_announced, bool keeping_appointment,
                      bool homing_announced, bool& done_announced) {
  // Finished and not on the way to, or standing at, a meeting: the run is over
  // and nothing is left to take part in. A keeper reaches this on the tick its
  // appointment manoeuvre ends — the barrier released, the cap expired, or the
  // cell proved unreachable — which is the moment it turns for home.
  if (finished_announced && !keeping_appointment) done_announced = true;
  if (done_announced) return kModeDone;
  if (homing_announced) return kModeHoming;
  return kModeExploring;
}

int finishedPeerStillComing(const TeamModel& team, int self_id) {
  if (!team.configured()) return -1;
  for (int id = 0; id < team.size(); ++id) {
    if (id == self_id) continue;
    const TeamModel::Peer& p = team.peer(id);
    if (!p.finished) continue;             // the unbounded vigil's case
    if (p.mode >= kModeHoming) continue;   // it said it is leaving
    // Heard in any way, its mode is current and the barrier's radio terms
    // decide it. The closure covers a peer bridged by a direct contact, whose
    // mode arrives relayed.
    if (p.direct || p.heard_one_way || team.inComms(id)) continue;
    return id;
  }
  return -1;
}

bool walkerJoinsBarrier(bool settled, bool holding) {
  return settled && !holding;
}

bool walkerResumesDrive(bool settled, bool reachable, bool holding) {
  return !((settled || reachable) && !holding);
}

}  // namespace explo_planner
