#include "explo_planner/proximity_guard.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace explo_planner {

ProximityGuard::ProximityGuard(Config cfg, std::string self_id)
    : cfg_(cfg), self_id_(std::move(self_id)) {
  // A NaN threshold fails every comparison (silently disarming the guard)
  // and a negative one can never trigger — either way the config is garbage,
  // so fall back to the compiled default rather than run half-armed.
  const Config dflt;
  auto fix = [](float& v, float d) {
    if (!std::isfinite(v) || v < 0.0f) v = d;
  };
  fix(cfg_.hold_dist_m, dflt.hold_dist_m);
  fix(cfg_.resume_dist_m, dflt.resume_dist_m);
  fix(cfg_.pose_stale_sec, dflt.pose_stale_sec);
  fix(cfg_.peer_static_sec, dflt.peer_static_sec);
  fix(cfg_.parked_keep_dist_m, dflt.parked_keep_dist_m);
  fix(cfg_.peer_static_move_m, dflt.peer_static_move_m);
  fix(cfg_.hold_release_stale_sec, dflt.hold_release_stale_sec);
  // An inverted hysteresis band (resume < hold) would release a hold at a
  // distance where the very next tick re-enters it — permanent chatter.
  if (cfg_.resume_dist_m < cfg_.hold_dist_m)
    cfg_.resume_dist_m = cfg_.hold_dist_m;
}

void ProximityGuard::onPeerPose(const std::string& peer_id,
                                const Eigen::Vector3f& pos,
                                const rclcpp::Time& now_local) {
  if (peer_id.empty() || peer_id == self_id_) return;
  // A non-finite position (diverged localiser) must not poison the track:
  // NaN distances fail every comparison, which reads as "no peer nearby".
  if (!pos.allFinite()) return;

  for (auto& p : peers_) {
    if (p.id != peer_id) continue;
    p.pos = pos;
    p.last_seen = now_local;
    // Motion detection against the anchor, not the previous sample:
    // localisation jitter of a stationary robot wanders a few cm per sample
    // but stays near the anchor, while real driving leaves it within a couple
    // of updates. XY only, like every distance in this class.
    const float dx = pos.x() - p.move_anchor.x();
    const float dy = pos.y() - p.move_anchor.y();
    const float thresh = cfg_.peer_static_move_m;
    if (dx * dx + dy * dy > thresh * thresh) {
      p.move_anchor = pos;
      p.last_moved = now_local;
    }
    return;
  }

  // First sighting: conservatively counts as moving (last_moved = now), so a
  // peer that appears already inside the hold disc is yielded to immediately
  // rather than after it has proven itself mobile.
  PeerTrack t;
  t.id = peer_id;
  t.pos = pos;
  t.last_seen = now_local;
  t.move_anchor = pos;
  t.last_moved = now_local;
  peers_.push_back(std::move(t));
}

ProximityGuard::Decision ProximityGuard::evaluate(
    const Eigen::Vector3f& self_pos, const rclcpp::Time& now,
    bool holding) const {
  Decision d;
  if (!cfg_.enabled) return d;

  const float trigger = holding ? cfg_.resume_dist_m : cfg_.hold_dist_m;
  const float max_age =
      holding ? cfg_.hold_release_stale_sec : cfg_.pose_stale_sec;

  // Two nearest-peer tracks: peers that FORCE a hold, and (for the note when
  // nothing does) the nearest outranking peer at all with why it was let by.
  float best_hold = std::numeric_limits<float>::infinity();
  float best_near = std::numeric_limits<float>::infinity();
  const std::string* near_id = nullptr;
  const char* near_note = nullptr;
  for (const auto& p : peers_) {
    // Yield only to peers that outrank us: lexicographically smaller id.
    // Id-only, so both robots of a pair compute the same winner (see .hpp).
    if (!(p.id < self_id_)) continue;

    const float dx = p.pos.x() - self_pos.x();
    const float dy = p.pos.y() - self_pos.y();
    const float dist = std::sqrt(dx * dx + dy * dy);

    // Freshness. A negative age means the local clock rewound (bag loop /
    // sim reset) — treat as stale; the next observation re-stamps the track.
    const double age = (now - p.last_seen).seconds();
    const bool fresh = age >= 0.0 && age <= max_age;

    // Parked peers are the costmap's job, and holding against one deadlocks
    // — but only beyond parked_keep_dist_m. Inside the floor "it parked" is
    // no licence to drive even closer; the caller's max-hold escape hatch is
    // the deadlock breaker there.
    const bool parked =
        (now - p.last_moved).seconds() > cfg_.peer_static_sec &&
        dist >= cfg_.parked_keep_dist_m;

    if (fresh && !parked && dist < trigger) {
      if (dist < best_hold) {
        best_hold = dist;
        d.hold = true;
        d.peer_id = p.id;
        d.dist_m = dist;
      }
    } else if (dist < best_near) {
      best_near = dist;
      near_id = &p.id;
      near_note = !fresh ? "stale" : (parked ? "parked" : "clear");
    }
  }
  if (!d.hold) {
    if (near_id) {
      d.peer_id = *near_id;
      d.dist_m = best_near;
      d.note = near_note;
    } else {
      d.note = "no-peer";
    }
  }
  return d;
}

}  // namespace explo_planner
