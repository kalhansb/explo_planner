/// @file grid_frontier.cpp
/// @brief Definitions for the 2D planning_map exploration helpers (see header).

#include "explo_planner/grid_frontier.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>

namespace explo_planner {

namespace {

bool gridUsable(const nav_msgs::msg::OccupancyGrid& g) {
  return g.info.resolution > 0.0f && g.info.width > 0 && g.info.height > 0 &&
         g.data.size() == static_cast<size_t>(g.info.width) *
                              static_cast<size_t>(g.info.height);
}

// floor() of a world coordinate to a cell index, clamped to [-1, n] before the
// int cast so an unbounded ROI edge cannot overflow it.
int cellIndex(double world, double origin, double res, int n) {
  const double k = std::floor((world - origin) / res);
  return static_cast<int>(std::clamp(k, -1.0, static_cast<double>(n)));
}

} // namespace

GridBox gridBoxOf(const nav_msgs::msg::OccupancyGrid& g, const Roi2D& roi) {
  GridBox b;
  if (!gridUsable(g)) return b;
  const int w = static_cast<int>(g.info.width);
  const int h = static_cast<int>(g.info.height);
  const double res = g.info.resolution;
  b.x0 = std::max(0, cellIndex(roi.min_x, g.info.origin.position.x, res, w));
  b.x1 = std::min(w - 1, cellIndex(roi.max_x, g.info.origin.position.x, res, w));
  b.y0 = std::max(0, cellIndex(roi.min_y, g.info.origin.position.y, res, h));
  b.y1 = std::min(h - 1, cellIndex(roi.max_y, g.info.origin.position.y, res, h));
  return b;
}

bool isGridFrontierCell(const nav_msgs::msg::OccupancyGrid& g, int gx, int gy,
                        const GridBox& box) {
  // box comes from gridBoxOf, so inside it is inside the grid.
  if (!box.contains(gx, gy)) return false;
  const int w = static_cast<int>(g.info.width);
  const int8_t v = g.data[static_cast<size_t>(gy) * w + gx];
  if (v < 0 || v >= 50) return false;
  static const int kD[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (const auto& d : kD) {
    const int nx = gx + d[0], ny = gy + d[1];
    if (box.contains(nx, ny) && g.data[static_cast<size_t>(ny) * w + nx] < 0)
      return true;
  }
  return false;
}

std::vector<GridFrontier> findGridFrontiers(
    const nav_msgs::msg::OccupancyGrid& g, const Roi2D& roi, float bin_m,
    int min_cells) {
  std::vector<GridFrontier> out;
  const GridBox box = gridBoxOf(g, roi);
  if (box.empty()) return out;

  const int bw = box.x1 - box.x0 + 1;
  const int bh = box.y1 - box.y0 + 1;
  auto local = [&](int gx, int gy) {
    return static_cast<size_t>(gy - box.y0) * bw + (gx - box.x0);
  };
  // 0 = not a frontier, 1 = frontier not yet labelled, 2 = labelled.
  std::vector<uint8_t> mark(static_cast<size_t>(bw) * bh, 0);
  for (int gy = box.y0; gy <= box.y1; ++gy)
    for (int gx = box.x0; gx <= box.x1; ++gx)
      if (isGridFrontierCell(g, gx, gy, box)) mark[local(gx, gy)] = 1;

  const double res = g.info.resolution;
  const double ox = g.info.origin.position.x;
  const double oy = g.info.origin.position.y;
  const bool split = bin_m > 0.0f && std::isfinite(bin_m);

  std::vector<std::pair<int, int>> comp, stack;
  for (int gy = box.y0; gy <= box.y1; ++gy) {
    for (int gx = box.x0; gx <= box.x1; ++gx) {
      if (mark[local(gx, gy)] != 1) continue;
      comp.clear();
      stack.assign(1, {gx, gy});
      mark[local(gx, gy)] = 2;
      while (!stack.empty()) {
        const auto [cx, cy] = stack.back();
        stack.pop_back();
        comp.emplace_back(cx, cy);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = cx + dx, ny = cy + dy;
            if (!box.contains(nx, ny) || mark[local(nx, ny)] != 1) continue;
            mark[local(nx, ny)] = 2;
            stack.emplace_back(nx, ny);
          }
        }
      }
      if (static_cast<int>(comp.size()) < min_cells) continue;

      // Bins in order of first appearance, so the output order is the scan's.
      std::map<std::pair<int64_t, int64_t>, size_t> bin_of;
      std::vector<std::vector<size_t>> groups;
      for (size_t i = 0; i < comp.size(); ++i) {
        const double wx = ox + (comp[i].first + 0.5) * res;
        const double wy = oy + (comp[i].second + 0.5) * res;
        const std::pair<int64_t, int64_t> key =
            split ? std::make_pair(
                        static_cast<int64_t>(std::floor(wx / bin_m)),
                        static_cast<int64_t>(std::floor(wy / bin_m)))
                  : std::make_pair(int64_t{0}, int64_t{0});
        auto it = bin_of.find(key);
        if (it == bin_of.end()) {
          it = bin_of.emplace(key, groups.size()).first;
          groups.emplace_back();
        }
        groups[it->second].push_back(i);
      }
      for (const auto& grp : groups) {
        double mx = 0.0, my = 0.0;
        for (size_t i : grp) {
          mx += ox + (comp[i].first + 0.5) * res;
          my += oy + (comp[i].second + 0.5) * res;
        }
        mx /= grp.size();
        my /= grp.size();
        size_t best = grp.front();
        double best_d2 = std::numeric_limits<double>::infinity();
        for (size_t i : grp) {
          const double ddx = ox + (comp[i].first + 0.5) * res - mx;
          const double ddy = oy + (comp[i].second + 0.5) * res - my;
          const double d2 = ddx * ddx + ddy * ddy;
          if (d2 < best_d2) { best_d2 = d2; best = i; }
        }
        GridFrontier f;
        f.position = Eigen::Vector3f(
            static_cast<float>(ox + (comp[best].first + 0.5) * res),
            static_cast<float>(oy + (comp[best].second + 0.5) * res), 0.0f);
        f.cells = static_cast<int>(grp.size());
        out.push_back(f);
      }
    }
  }
  return out;
}

int gridVisibleUnknown(const nav_msgs::msg::OccupancyGrid& g,
                       const Roi2D& roi, const Eigen::Vector3f& pos, float yaw,
                       float hfov, int h_rays, float min_range,
                       float max_range) {
  const GridBox roi_box = gridBoxOf(g, roi);
  if (roi_box.empty() || !(max_range > 0.0f) || !(hfov > 0.0f)) return 0;
  const int w = static_cast<int>(g.info.width);
  const int h = static_cast<int>(g.info.height);
  const double res = g.info.resolution;
  // Position in cell units; the start cell must be on the grid.
  const double sx = (pos.x() - g.info.origin.position.x) / res;
  const double sy = (pos.y() - g.info.origin.position.y) / res;
  if (!(sx >= 0.0 && sx < w && sy >= 0.0 && sy < h)) return 0;
  const int x0 = static_cast<int>(std::floor(sx));
  const int y0 = static_cast<int>(std::floor(sy));

  const double t_max = max_range / res;
  const double t_min = std::max(0.0f, min_range) / res;
  // Dedup window: a ray never leaves ceil(t_max) + 1 cells of the start.
  const int R = static_cast<int>(std::ceil(t_max)) + 1;
  const int side = 2 * R + 1;
  std::vector<uint8_t> seen(static_cast<size_t>(side) * side, 0);

  const int n = std::max({1, h_rays,
                          static_cast<int>(std::ceil(hfov * t_max))});
  constexpr double kInf = std::numeric_limits<double>::infinity();
  int count = 0;
  for (int i = 0; i < n; ++i) {
    const double a = yaw - 0.5 * hfov + (i + 0.5) * hfov / n;
    const double dx = std::cos(a), dy = std::sin(a);
    // Amanatides-Woo: t is the ray parameter in cells at which the current
    // cell was entered; t_next_* where the ray crosses the next x / y edge.
    const int step_x = dx > 0.0 ? 1 : -1;
    const int step_y = dy > 0.0 ? 1 : -1;
    double t_next_x = dx > 0.0 ? (x0 + 1 - sx) / dx
                    : dx < 0.0 ? (x0 - sx) / dx : kInf;
    double t_next_y = dy > 0.0 ? (y0 + 1 - sy) / dy
                    : dy < 0.0 ? (y0 - sy) / dy : kInf;
    const double dt_x = dx != 0.0 ? 1.0 / std::abs(dx) : kInf;
    const double dt_y = dy != 0.0 ? 1.0 / std::abs(dy) : kInf;
    int cx = x0, cy = y0;
    double t = 0.0;
    while (t <= t_max) {
      if (cx < 0 || cy < 0 || cx >= w || cy >= h) break;
      const int8_t v = g.data[static_cast<size_t>(cy) * w + cx];
      if (v >= 50) break;
      if (v < 0 && t >= t_min && roi_box.contains(cx, cy)) {
        const int lx = cx - x0 + R, ly = cy - y0 + R;
        if (lx >= 0 && ly >= 0 && lx < side && ly < side) {
          uint8_t& s = seen[static_cast<size_t>(ly) * side + lx];
          if (!s) { s = 1; ++count; }
        }
      }
      if (t_next_x < t_next_y) {
        t = t_next_x; t_next_x += dt_x; cx += step_x;
      } else {
        t = t_next_y; t_next_y += dt_y; cy += step_y;
      }
    }
  }
  return count;
}

std::vector<CellWorld::CellObservation> censusFromPlanMap(
    const nav_msgs::msg::OccupancyGrid& pm, const CellGrid& grid) {
  std::vector<CellWorld::CellObservation> out;
  if (grid.size() <= 0) return out;
  out.assign(static_cast<size_t>(grid.size()), CellWorld::CellObservation{});
  const GridBox box =
      gridBoxOf(pm, {grid.min_x, grid.maxX(), grid.min_y, grid.maxY()});
  if (box.empty()) return out;
  const int w = static_cast<int>(pm.info.width);
  const double res = pm.info.resolution;
  for (int gy = box.y0; gy <= box.y1; ++gy) {
    const float y =
        static_cast<float>(pm.info.origin.position.y + (gy + 0.5) * res);
    for (int gx = box.x0; gx <= box.x1; ++gx) {
      const float x =
          static_cast<float>(pm.info.origin.position.x + (gx + 0.5) * res);
      const int id = grid.idAt(x, y);
      if (id < 0) continue;
      auto& o = out[static_cast<size_t>(id)];
      ++o.total_columns;
      if (pm.data[static_cast<size_t>(gy) * w + gx] < 0) continue;
      ++o.observed_columns;
      ++o.observed_voxels;
      if (isGridFrontierCell(pm, gx, gy, box)) ++o.frontier_voxels;
    }
  }
  return out;
}

} // namespace explo_planner
