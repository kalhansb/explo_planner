// Moved comments: doc/explo_planner_code_notes.md
#include "explo_planner/cost_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
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

  // Only known cells with value >= obstacle_threshold are impassable. Unknown
  // (-1) cells stay traversable so the flood can path through unexplored space;
  // isCellFree() rejects candidates on unknown cells.
  // (notes: costgrid-unknown-traversable)
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
  // Seed the source only if it or one of its 8 neighbours is unblocked. A
  // malformed (all-blocked) map then stays all kInfCost, while a robot standing
  // on inflation still floods. (notes: costgrid-conditional-seed)
  bool can_seed = !blocked_[idx(sx, sy)];
  for (int k = 0; !can_seed && k < 8; ++k) {
    const int nx = sx + kDx[k];
    const int ny = sy + kDy[k];
    if (inBounds(nx, ny) && !blocked_[idx(nx, ny)]) can_seed = true;
  }
  if (!can_seed) return;
  cost_[idx(sx, sy)] = 0.0f;

  // A cap <= 0 or NaN means unbounded (+inf). Do not clamp it to the grid
  // diagonal: walked path lengths exceed it, so reachable cells would read
  // kInfCost. Termination does not depend on the cap.
  // (notes: costgrid-radius-cap-unbounded)
  float cap = radius_cap_m;
  if (!(cap > 0.0f)) {
    cap = std::numeric_limits<float>::infinity();
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
