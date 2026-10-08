#include "explo_planner/traversability_accumulator.hpp"

#include <algorithm>
#include <cmath>

namespace explo_planner {

const char* TraversabilityAccumulator::statusName(Status s) {
  switch (s) {
    case Status::kIntegrated:   return "integrated";
    case Status::kNoOverlap:    return "outside the accumulator box";
    case Status::kNoKnownCells: return "no known cells";
    case Status::kMalformed:    return "malformed";
    case Status::kTooLarge:     return "too large";
    case Status::kRotated:      return "rotated origin (only axis-aligned windows)";
  }
  return "?";
}

TraversabilityAccumulator::TraversabilityAccumulator(
    const AccumulatorConfig& cfg)
    : origin_x_(cfg.min_x),
      origin_y_(cfg.min_y),
      resolution_(cfg.resolution),
      max_window_cells_(cfg.max_window_cells) {
  const double ex = static_cast<double>(cfg.max_x) - cfg.min_x;
  const double ey = static_cast<double>(cfg.max_y) - cfg.min_y;
  if (!(ex > 0.0) || !(ey > 0.0) || !std::isfinite(ex) ||
      !std::isfinite(ey) || !(cfg.resolution > 0.0f) || cfg.max_cells == 0) {
    return;  // empty grid: every window reads kNoOverlap
  }
  double res = cfg.resolution;
  auto cells = [&](double r) { return std::ceil(ex / r) * std::ceil(ey / r); };
  const double cap = static_cast<double>(cfg.max_cells);
  if (cells(res) > cap) {
    // Coarsen to fit rather than allocate whatever the box asks for: an
    // unconfigured (+-1e9) ROI must not turn into a multi-GB grid.
    res = std::max(res, std::sqrt(ex * ey / cap));
    while (cells(res) > cap) res *= 1.01;
    coarsened_ = true;
  }
  resolution_ = static_cast<float>(res);
  width_ = static_cast<int>(std::ceil(ex / res));
  height_ = static_cast<int>(std::ceil(ey / res));
  grid_.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), -1);
}

TraversabilityAccumulator::Status TraversabilityAccumulator::integrate(
    const nav_msgs::msg::OccupancyGrid& window) {
  const auto& info = window.info;
  const double rw = info.resolution;
  if (!(rw > 0.0) || !std::isfinite(rw) || info.width == 0 ||
      info.height == 0) {
    return Status::kMalformed;
  }
  const uint64_t wn = static_cast<uint64_t>(info.width) * info.height;
  if (wn > max_window_cells_) return Status::kTooLarge;
  if (window.data.size() != wn) return Status::kMalformed;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const auto& q = info.origin.orientation;
  if (!std::isfinite(ox) || !std::isfinite(oy) || !std::isfinite(q.x) ||
      !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
    return Status::kMalformed;
  }
  // Yaw of the origin pose; an all-zero quaternion reads as identity, which
  // is what every publisher that leaves it unset means.
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  if (std::abs(q.x) > 1e-3 || std::abs(q.y) > 1e-3 || std::abs(yaw) > 1e-3) {
    return Status::kRotated;
  }
  if (grid_.empty()) return Status::kNoOverlap;

  // Window footprint in accumulator cells, clipped to the grid. Computed in
  // double and range-checked before the int cast, so a far-away window cannot
  // overflow the index.
  const double ra = resolution_;
  const double fx0 = std::floor((ox - origin_x_) / ra);
  const double fx1 = std::floor((ox + info.width * rw - origin_x_) / ra);
  const double fy0 = std::floor((oy - origin_y_) / ra);
  const double fy1 = std::floor((oy + info.height * rw - origin_y_) / ra);
  if (fx1 < 0.0 || fy1 < 0.0 || fx0 > width_ - 1 || fy0 > height_ - 1) {
    return Status::kNoOverlap;
  }
  const int ax0 = static_cast<int>(std::max(0.0, fx0));
  const int ax1 = static_cast<int>(std::min<double>(width_ - 1, fx1));
  const int ay0 = static_cast<int>(std::max(0.0, fy0));
  const int ay1 = static_cast<int>(std::min<double>(height_ - 1, fy1));
  const size_t fw = static_cast<size_t>(ax1 - ax0 + 1);
  touched_.assign(fw * static_cast<size_t>(ay1 - ay0 + 1), 0);

  bool any_known = false;
  bool changed = false;
  // The window's first write to a cell replaces what earlier windows left
  // there (latest observation wins); further writes from the SAME window —
  // several window cells per accumulator cell — keep the max.
  auto write = [&](int ax, int ay, int8_t v) {
    any_known = true;
    int8_t& cell = grid_[static_cast<size_t>(ay) * width_ + ax];
    uint8_t& seen = touched_[static_cast<size_t>(ay - ay0) * fw + (ax - ax0)];
    if (!seen) {
      seen = 1;
      if (cell != v) {
        if (cell < 0) ++known_cells_;
        cell = v;
        changed = true;
      }
    } else if (v > cell) {
      cell = v;
      changed = true;
    }
  };
  auto clean = [](int8_t v) -> int8_t { return v > 100 ? 100 : v; };

  if (rw <= ra * 1.0001) {
    // Window as fine as or finer than the grid: each window cell lands in the
    // accumulator cell holding its centre.
    for (uint32_t j = 0; j < info.height; ++j) {
      const double gy = std::floor((oy + (j + 0.5) * rw - origin_y_) / ra);
      if (gy < ay0 || gy > ay1) continue;
      const int ay = static_cast<int>(gy);
      const int8_t* row = &window.data[static_cast<size_t>(j) * info.width];
      for (uint32_t i = 0; i < info.width; ++i) {
        if (row[i] < 0) continue;
        const double gx = std::floor((ox + (i + 0.5) * rw - origin_x_) / ra);
        if (gx < ax0 || gx > ax1) continue;
        write(static_cast<int>(gx), ay, clean(row[i]));
      }
    }
  } else {
    // Window coarser than the grid: a window cell covers several accumulator
    // cells; write every one whose centre lies inside it. Clamped in double
    // before the int cast (a huge window resolution must not overflow it).
    auto span = [ra](double f0, double f1, double origin, int lo, int hi,
                     int& k0, int& k1) {
      const double d0 =
          std::max<double>(lo, std::ceil((f0 - origin) / ra - 0.5));
      const double d1 =
          std::min<double>(hi, std::ceil((f1 - origin) / ra - 0.5) - 1.0);
      if (!(d0 <= d1)) return false;
      k0 = static_cast<int>(d0);
      k1 = static_cast<int>(d1);
      return true;
    };
    for (uint32_t j = 0; j < info.height; ++j) {
      int ky0 = 0, ky1 = 0;
      if (!span(oy + j * rw, oy + (j + 1) * rw, origin_y_, ay0, ay1,
                ky0, ky1)) {
        continue;
      }
      const int8_t* row = &window.data[static_cast<size_t>(j) * info.width];
      for (uint32_t i = 0; i < info.width; ++i) {
        if (row[i] < 0) continue;
        int kx0 = 0, kx1 = 0;
        if (!span(ox + i * rw, ox + (i + 1) * rw, origin_x_, ax0, ax1,
                  kx0, kx1)) {
          continue;
        }
        for (int ay = ky0; ay <= ky1; ++ay) {
          for (int ax = kx0; ax <= kx1; ++ax) write(ax, ay, clean(row[i]));
        }
      }
    }
  }

  if (!any_known) return Status::kNoKnownCells;
  if (changed) ++version_;
  return Status::kIntegrated;
}

void TraversabilityAccumulator::toMsg(nav_msgs::msg::OccupancyGrid& out) const {
  out.info.resolution = resolution_;
  out.info.width = static_cast<uint32_t>(width_);
  out.info.height = static_cast<uint32_t>(height_);
  out.info.origin.position.x = origin_x_;
  out.info.origin.position.y = origin_y_;
  out.info.origin.position.z = 0.0;
  out.info.origin.orientation.x = 0.0;
  out.info.origin.orientation.y = 0.0;
  out.info.origin.orientation.z = 0.0;
  out.info.origin.orientation.w = 1.0;
  out.data = grid_;
}

}  // namespace explo_planner
