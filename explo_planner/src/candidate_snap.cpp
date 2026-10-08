#include "explo_planner/candidate_snap.hpp"

#include <algorithm>
#include <cmath>

namespace explo_planner {

bool CandidateSnapper::good(const CostGrid& grid, int gx, int gy,
                            bool allow_unknown,
                            bool require_reachable) const {
  if (!grid.inBounds(gx, gy)) return false;
  if (grid.blockedAt(gx, gy)) return false;
  if (grid.inClearanceZoneAt(gx, gy)) return false;
  if (!allow_unknown && grid.unknownAt(gx, gy)) return false;
  if (require_reachable && !std::isfinite(grid.costAt(gx, gy))) return false;
  return true;
}

void CandidateSnapper::buildOffsets(float resolution) {
  offsets_.clear();
  offsets_res_ = resolution;
  if (!(resolution > 0.0f) || !(radius_m_ > 0.0f)) {
    offsets_.emplace_back(0, 0);
    return;
  }
  // Capped so a mis-set radius on a fine grid cannot allocate a huge disc
  // (200 cells -> ~126 k offsets, 1 MB; 10 m even at 0.05 m resolution).
  constexpr int kMaxRadiusCells = 200;
  const int r = std::min(kMaxRadiusCells,
      static_cast<int>(std::floor(radius_m_ / resolution)));
  const long r2 = static_cast<long>(r) * r;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (static_cast<long>(dx) * dx + static_cast<long>(dy) * dy <= r2) {
        offsets_.emplace_back(dx, dy);
      }
    }
  }
  // Nearest first; ties keep a fixed order so the result is deterministic.
  std::stable_sort(offsets_.begin(), offsets_.end(),
      [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return a.first * a.first + a.second * a.second <
               b.first * b.first + b.second * b.second;
      });
}

CandidateSnapper::Outcome CandidateSnapper::snap(
    const CostGrid& grid, const Eigen::Vector3f& pos, bool allow_unknown,
    bool require_reachable, const std::function<bool(float, float)>& accept,
    Eigen::Vector2f& out) {
  int cx = 0;
  int cy = 0;
  const bool in_grid = grid.cellAt(pos, cx, cy);
  if (in_grid && good(grid, cx, cy, allow_unknown, require_reachable)) {
    return Outcome::kKept;
  }
  // Off the grid there is nothing to search from: the planner treats it as
  // blocked, and so does the reject filter downstream.
  if (!in_grid || !(radius_m_ > 0.0f)) return Outcome::kNoSpot;

  if (offsets_res_ != grid.resolution()) buildOffsets(grid.resolution());
  for (const auto& [dx, dy] : offsets_) {
    const int gx = cx + dx;
    const int gy = cy + dy;
    if (!good(grid, gx, gy, allow_unknown, require_reachable)) continue;
    const Eigen::Vector2f c = grid.cellCenter(gx, gy);
    if (accept && !accept(c.x(), c.y())) continue;
    out = c;
    return Outcome::kSnapped;
  }
  return Outcome::kNoSpot;
}

}  // namespace explo_planner
