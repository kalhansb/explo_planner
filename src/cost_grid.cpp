#include "explo_planner/cost_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>
#include <utility>

namespace explo_planner {

namespace {

// 8-connected neighbour offsets and per-step costs (multiplied by resolution
// at runtime). Order is fixed for determinism — the per-cell cost only
// depends on the multiset of arrivals, not the iteration order, but keeping
// it stable makes the unit tests easier to reason about.
constexpr int kDx[8] = { 1, -1,  0,  0,  1,  1, -1, -1};
constexpr int kDy[8] = { 0,  0,  1, -1,  1, -1,  1, -1};
const float kStepMul[8] = {
    1.0f, 1.0f, 1.0f, 1.0f,
    1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f,
};

}  // namespace

void CostGrid::build(const nav_msgs::msg::OccupancyGrid& planning_map,
                     int8_t obstacle_threshold) {
  dims_x_ = static_cast<int>(planning_map.info.width);
  dims_y_ = static_cast<int>(planning_map.info.height);
  resolution_ = planning_map.info.resolution;
  origin_x_ = static_cast<float>(planning_map.info.origin.position.x);
  origin_y_ = static_cast<float>(planning_map.info.origin.position.y);

  if (dims_x_ <= 0 || dims_y_ <= 0 || resolution_ <= 0.0f) {
    cost_.clear();
    blocked_.clear();
    dims_x_ = 0;
    dims_y_ = 0;
    return;
  }

  const size_t n = static_cast<size_t>(dims_x_) * static_cast<size_t>(dims_y_);
  cost_.assign(n, kInfCost);
  blocked_.assign(n, 0);

  // Sample the obstacle layer once. Only *known* occupied/inflated cells
  // (value >= obstacle_threshold) are impassable. Unknown cells (value -1)
  // are treated as traversable so the flood can path through unexplored
  // space — the planner's per-cell isCellFree() filter already rejects
  // candidates that sit on unknown cells, so reachability's job is only
  // to catch free pockets sealed off by *known* obstacles, not to block
  // paths through the exploration frontier.
  const auto& data = planning_map.data;
  if (data.size() != n) {
    // Defensive: malformed message. Mark everything blocked so the flood
    // is empty and the caller falls back to "no candidate reachable".
    std::fill(blocked_.begin(), blocked_.end(), 1);
    return;
  }
  for (size_t i = 0; i < n; ++i) {
    int8_t v = data[i];
    if (v >= 0 && v >= obstacle_threshold) {
      blocked_[i] = 1;
    }
  }
}

void CostGrid::floodFrom(const Eigen::Vector3f& source_xy, float radius_cap_m) {
  if (dims_x_ <= 0 || dims_y_ <= 0) {
    return;
  }

  // Reset the cost layer and parent pointers without touching the obstacle
  // layer (the obstacle layer is owned by build() and is shared across
  // multiple floods if the planner ever wants per-candidate sources from
  // the same map).
  std::fill(cost_.begin(), cost_.end(), kInfCost);
  parent_.assign(cost_.size(), -1);

  int sx = 0;
  int sy = 0;
  if (!worldToGrid(source_xy.x(), source_xy.y(), sx, sy)) {
    // Out-of-bounds source — leave every cell at kInfCost.
    return;
  }
  if (blocked_[idx(sx, sy)]) {
    // Source on top of an inflated cell. The robot is allowed to be there
    // (it actually is — the inflation includes the robot footprint), so
    // start the flood from cost 0 anyway. We just don't relax through any
    // *other* blocked cell.
  }
  cost_[idx(sx, sy)] = 0.0f;

  // Effective radius cap. Negative / zero / NaN means "no bound" — we still
  // clamp to the grid diagonal so the loop terminates trivially.
  const float diag_m =
      std::hypot(static_cast<float>(dims_x_), static_cast<float>(dims_y_)) *
      resolution_;
  float cap = radius_cap_m;
  if (!(cap > 0.0f) || cap > diag_m) {
    cap = diag_m + 1.0f;
  }

  using Node = std::pair<float, int>;  // (cost, flat index)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
  pq.emplace(0.0f, idx(sx, sy));

  while (!pq.empty()) {
    auto [c, i] = pq.top();
    pq.pop();

    // Stale entry from an earlier (longer) relaxation — skip.
    if (c > cost_[i]) continue;
    // Past the bound — every remaining entry is at least this far.
    if (c > cap) break;

    int gx = i % dims_x_;
    int gy = i / dims_x_;

    for (int k = 0; k < 8; ++k) {
      int nx = gx + kDx[k];
      int ny = gy + kDy[k];
      if (!inBounds(nx, ny)) continue;
      int ni = idx(nx, ny);
      if (blocked_[ni]) continue;

      float nc = c + kStepMul[k] * resolution_;
      if (nc > cap) continue;
      if (nc < cost_[ni]) {
        cost_[ni] = nc;
        parent_[ni] = i;
        pq.emplace(nc, ni);
      }
    }
  }
}

float CostGrid::costTo(const Eigen::Vector3f& xy) const {
  int gx = 0;
  int gy = 0;
  if (!worldToGrid(xy.x(), xy.y(), gx, gy)) return kInfCost;
  return cost_[idx(gx, gy)];
}

bool CostGrid::reachable(const Eigen::Vector3f& xy) const {
  return std::isfinite(costTo(xy));
}

std::vector<Eigen::Vector3f> CostGrid::extractPath(
    const Eigen::Vector3f& goal_xy) const {
  std::vector<Eigen::Vector3f> path;
  if (dims_x_ <= 0 || dims_y_ <= 0 || parent_.empty()) return path;

  int gx = 0, gy = 0;
  if (!worldToGrid(goal_xy.x(), goal_xy.y(), gx, gy)) return path;

  int ci = idx(gx, gy);
  if (!std::isfinite(cost_[ci])) return path;  // unreachable

  // Walk parent pointers from goal back to source (parent == -1).
  std::vector<Eigen::Vector3f> reversed;
  while (ci >= 0) {
    int cx = ci % dims_x_;
    int cy = ci / dims_x_;
    float wx = origin_x_ + (static_cast<float>(cx) + 0.5f) * resolution_;
    float wy = origin_y_ + (static_cast<float>(cy) + 0.5f) * resolution_;
    reversed.emplace_back(wx, wy, 0.0f);
    ci = parent_[ci];
  }

  // Reverse to get source → goal order.
  path.assign(reversed.rbegin(), reversed.rend());
  return path;
}

bool CostGrid::blockedAt(int gx, int gy) const {
  if (!inBounds(gx, gy)) return true;
  return blocked_[idx(gx, gy)] != 0;
}

size_t CostGrid::reachedCellCount() const {
  size_t n = 0;
  for (float c : cost_) {
    if (std::isfinite(c)) ++n;
  }
  return n;
}

bool CostGrid::worldToGrid(float x, float y, int& gx, int& gy) const {
  if (resolution_ <= 0.0f || dims_x_ <= 0 || dims_y_ <= 0) return false;
  gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return inBounds(gx, gy);
}

}  // namespace explo_planner
