#include "explo_planner/target_queue.hpp"

namespace explo_planner {

namespace {
// 2D (XY) squared distance between two points; vantages and trunk centres are
// compared in the ground plane (the simulation is ground-restricted).
inline float dist2xy(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  const float dx = a.x() - b.x();
  const float dy = a.y() - b.y();
  return dx * dx + dy * dy;
}
}  // namespace

bool TargetQueue::ingest(uint32_t id, const Eigen::Vector3f& center,
                         float radius, float height, float dedup_radius_m) {
  const float dedup2 = dedup_radius_m > 0.0f
                           ? dedup_radius_m * dedup_radius_m
                           : 0.0f;
  for (const auto& t : targets_) {
    if (t.id == id) return false;                       // same id
    if (dedup2 > 0.0f && dist2xy(t.center, center) <= dedup2)
      return false;                                     // re-reported tree
  }
  Target t;
  t.id = id;
  t.center = center;
  t.radius = radius;
  t.height = height;
  t.status = Target::Status::PENDING;
  targets_.push_back(t);
  return true;
}

bool TargetQueue::hasPending() const {
  for (const auto& t : targets_)
    if (t.status != Target::Status::DONE) return true;
  return false;
}

size_t TargetQueue::pendingCount() const {
  size_t n = 0;
  for (const auto& t : targets_)
    if (t.status != Target::Status::DONE) ++n;
  return n;
}

Target* TargetQueue::activate() {
  if (active_idx_ >= 0 &&
      static_cast<size_t>(active_idx_) < targets_.size() &&
      targets_[active_idx_].status == Target::Status::ACTIVE) {
    return &targets_[active_idx_];  // already have an active target
  }
  for (size_t i = 0; i < targets_.size(); ++i) {
    if (targets_[i].status == Target::Status::PENDING) {
      targets_[i].status = Target::Status::ACTIVE;
      active_idx_ = static_cast<int>(i);
      return &targets_[i];
    }
  }
  active_idx_ = -1;
  return nullptr;
}

Target* TargetQueue::active() {
  if (active_idx_ < 0 ||
      static_cast<size_t>(active_idx_) >= targets_.size())
    return nullptr;
  if (targets_[active_idx_].status != Target::Status::ACTIVE) return nullptr;
  return &targets_[active_idx_];
}

const Target* TargetQueue::active() const {
  if (active_idx_ < 0 ||
      static_cast<size_t>(active_idx_) >= targets_.size())
    return nullptr;
  if (targets_[active_idx_].status != Target::Status::ACTIVE) return nullptr;
  return &targets_[active_idx_];
}

void TargetQueue::markActiveDone() {
  if (Target* t = active()) {
    t->status = Target::Status::DONE;
  }
  active_idx_ = -1;
}

void TargetQueue::recordVantageDwell(const Eigen::Vector3f& vantage_xy,
                                     bool los_clear) {
  Target* t = active();
  if (!t) return;
  t->visited_vantages.push_back(vantage_xy);
  if (los_clear) ++t->clear_los_dwells;
}

bool TargetQueue::isVantageVisited(const Eigen::Vector3f& vantage_xy,
                                   float visited_tol_m) const {
  const Target* t = active();
  if (!t) return false;
  const float tol2 = visited_tol_m * visited_tol_m;
  for (const auto& v : t->visited_vantages)
    if (dist2xy(v, vantage_xy) <= tol2) return true;
  return false;
}

}  // namespace explo_planner
