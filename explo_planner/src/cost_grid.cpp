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
                     int8_t obstacle_threshold,
                     const CostModel& model) {
  dims_x_ = static_cast<int>(planning_map.info.width);
  dims_y_ = static_cast<int>(planning_map.info.height);
  resolution_ = planning_map.info.resolution;
  origin_x_ = static_cast<float>(planning_map.info.origin.position.x);
  origin_y_ = static_cast<float>(planning_map.info.origin.position.y);

  step_mul_.clear();
  flood_clear_r2_ = -1;
  if (dims_x_ <= 0 || dims_y_ <= 0 || resolution_ <= 0.0f) {
    cost_.clear();
    flags_.clear();
    dims_x_ = 0;
    dims_y_ = 0;
    return;
  }

  const size_t n = static_cast<size_t>(dims_x_) * static_cast<size_t>(dims_y_);
  cost_.assign(n, kInfCost);
  flags_.assign(n, 0);

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
    std::fill(flags_.begin(), flags_.end(), kFlagBlocked);
    return;
  }
  for (size_t i = 0; i < n; ++i) {
    int8_t v = data[i];
    if (v < 0) {
      flags_[i] = kFlagUnknown;
    } else if (v >= obstacle_threshold) {
      flags_[i] = kFlagBlocked;
    }
  }

  if (model.clearance_m > 0.0f) markClearanceZone(model.clearance_m);

  if (model.shaped()) {
    step_mul_.assign(n, 1.0f);
    const float w = std::max(0.0f, model.trav_weight) / 100.0f;
    const float pen = std::max(1.0f, model.clearance_penalty);
    const float unk = std::clamp(model.unknown_value, 0.0f, 99.0f);
    for (size_t i = 0; i < n; ++i) {
      if (flags_[i] & kFlagBlocked) continue;
      float m = 1.0f;
      if (data[i] > 0) m += w * static_cast<float>(data[i]);
      else if (data[i] < 0) m += w * unk;
      if (flags_[i] & kFlagClearance) m *= pen;
      step_mul_[i] = m;
    }
  }
}

void CostGrid::markClearanceZone(float clearance_m) {
  // Chamfer 3-4 distances in thirds of a cell: orthogonal step 3, diagonal 4.
  // Two raster passes give every cell its distance to the nearest blocked
  // cell in O(cells), independent of the radius — a per-obstacle disc stamp
  // is O(obstacles * r^2) and blows up with a large clearance on a fine grid.
  const size_t n = flags_.size();
  constexpr uint32_t kFar = std::numeric_limits<uint16_t>::max();
  std::vector<uint16_t> d(n, static_cast<uint16_t>(kFar));
  bool any_blocked = false;
  for (size_t i = 0; i < n; ++i) {
    if (flags_[i] & kFlagBlocked) {
      d[i] = 0;
      any_blocked = true;
    }
  }
  if (!any_blocked) return;

  auto relax = [&](size_t i, int gx, int gy, uint32_t step) {
    if (!inBounds(gx, gy)) return;
    const uint32_t cand = static_cast<uint32_t>(d[idx(gx, gy)]) + step;
    if (cand < d[i]) d[i] = static_cast<uint16_t>(std::min(cand, kFar));
  };
  for (int gy = 0; gy < dims_y_; ++gy) {
    for (int gx = 0; gx < dims_x_; ++gx) {
      const size_t i = static_cast<size_t>(idx(gx, gy));
      if (d[i] == 0) continue;
      relax(i, gx - 1, gy, 3);
      relax(i, gx - 1, gy - 1, 4);
      relax(i, gx, gy - 1, 3);
      relax(i, gx + 1, gy - 1, 4);
    }
  }
  for (int gy = dims_y_ - 1; gy >= 0; --gy) {
    for (int gx = dims_x_ - 1; gx >= 0; --gx) {
      const size_t i = static_cast<size_t>(idx(gx, gy));
      if (d[i] == 0) continue;
      relax(i, gx + 1, gy, 3);
      relax(i, gx + 1, gy + 1, 4);
      relax(i, gx, gy + 1, 3);
      relax(i, gx - 1, gy + 1, 4);
    }
  }

  // Cell-centre distance <= clearance_m, in the same thirds-of-a-cell units.
  const float lim = 3.0f * clearance_m / resolution_;
  for (size_t i = 0; i < n; ++i) {
    if (d[i] > 0 && static_cast<float>(d[i]) <= lim) {
      flags_[i] |= kFlagClearance;
    }
  }
}

void CostGrid::floodFrom(const Eigen::Vector3f& source_xy, float radius_cap_m,
                         float source_clear_m) {
  flood_clear_r2_ = -1;
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
  if (flags_[idx(sx, sy)] & kFlagBlocked) {
    // Source on top of an inflated cell. The robot is allowed to be there
    // (it actually is — the inflation includes the robot footprint), so
    // start the flood from cost 0 anyway. We just don't relax through any
    // *other* blocked cell.
  }
  cost_[idx(sx, sy)] = 0.0f;

  // Blocked cells within source_clear_m of the source are entered anyway.
  // Clamped like the snapper's disc: a mis-set radius must not open the map.
  long clear_r2 = 0;
  if (source_clear_m > 0.0f && std::isfinite(source_clear_m)) {
    const long r = std::min(200L,
        static_cast<long>(std::floor(source_clear_m / resolution_)));
    clear_r2 = r * r;
  }
  flood_sx_ = sx;
  flood_sy_ = sy;
  flood_clear_r2_ = clear_r2 > 0 ? clear_r2 : -1;

  // Effective radius cap. Negative / zero / NaN means "no bound" — genuinely
  // unbounded, i.e. +inf. Do NOT clamp to the grid diagonal: a Dijkstra path
  // length is a *walked* distance and routinely exceeds the straight-line
  // diagonal (serpentine corridors, U-shaped rooms), so a diagonal clamp made
  // reachable cells report kInfCost and the exploitation planner rejected
  // genuinely drivable vantages as unreachable. Termination does not depend on
  // the cap — the grid is finite and each cell is relaxed to a strictly
  // decreasing cost.
  float cap = radius_cap_m;
  if (!(cap > 0.0f)) {
    cap = std::numeric_limits<float>::infinity();
  }

  const bool shaped = !step_mul_.empty();

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
      if (flags_[ni] & kFlagBlocked) {
        const long dx = nx - sx;
        const long dy = ny - sy;
        if (dx * dx + dy * dy > clear_r2) continue;
      }

      // Entering cell ni costs its step length, scaled by ni's multiplier
      // under a shaped CostModel (all multipliers >= 1, so Dijkstra holds).
      float nc = c + kStepMul[k] * resolution_ *
                         (shaped ? step_mul_[ni] : 1.0f);
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

float CostGrid::pathLengthTo(const Eigen::Vector3f& xy) const {
  int gx = 0;
  int gy = 0;
  if (!worldToGrid(xy.x(), xy.y(), gx, gy)) return kInfCost;
  int ci = idx(gx, gy);
  if (!std::isfinite(cost_[ci])) return kInfCost;
  if (step_mul_.empty()) return cost_[ci];
  if (parent_.empty()) return kInfCost;
  // Each step is orthogonal (1 cell) or diagonal (sqrt 2); sum in double so a
  // long path does not drift.
  double cells = 0.0;
  for (int pi = parent_[ci]; pi >= 0; ci = pi, pi = parent_[ci]) {
    const bool diag = (ci % dims_x_ != pi % dims_x_) && (ci / dims_x_ != pi / dims_x_);
    cells += diag ? kStepMul[4] : 1.0;
  }
  return static_cast<float>(cells * resolution_);
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
  return (flags_[idx(gx, gy)] & kFlagBlocked) != 0;
}

bool CostGrid::unknownAt(int gx, int gy) const {
  if (!inBounds(gx, gy)) return false;
  return (flags_[idx(gx, gy)] & kFlagUnknown) != 0;
}

bool CostGrid::inClearanceZoneAt(int gx, int gy) const {
  if (!inBounds(gx, gy)) return false;
  return (flags_[idx(gx, gy)] & kFlagClearance) != 0;
}

bool CostGrid::inClearanceZone(const Eigen::Vector3f& xy) const {
  int gx = 0;
  int gy = 0;
  if (!worldToGrid(xy.x(), xy.y(), gx, gy)) return false;
  return inClearanceZoneAt(gx, gy);
}

size_t CostGrid::reachedCellCount() const {
  size_t n = 0;
  for (size_t i = 0; i < cost_.size(); ++i) {
    if (!std::isfinite(cost_[i])) continue;
    if (flood_clear_r2_ > 0) {
      const long dx = static_cast<long>(i % dims_x_) - flood_sx_;
      const long dy = static_cast<long>(i / dims_x_) - flood_sy_;
      if (dx * dx + dy * dy <= flood_clear_r2_) continue;
    }
    ++n;
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
