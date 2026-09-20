/// @file plan_map_query.cpp
/// @brief Definitions for the 2D planning_map cell queries (see header).

#include "explo_planner/plan_map_query.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace explo_planner {

int8_t planMapCellAt(const nav_msgs::msg::OccupancyGrid& m,
                     const Eigen::Vector3f& pos) {
  if (m.info.resolution <= 0.0f || m.info.width == 0 || m.info.height == 0)
    return kCellNoData;
  // info.width/info.height bound the INDEX computed below, but they are only a
  // claim about the buffer -- nothing in nav_msgs makes data.size() agree with
  // them. A publisher that fills in the metadata and then sends a short (or
  // empty) data vector produces a grid that passes every bounds test here and
  // still reads off the end: a 1140x1140 map carrying 1140 bytes segfaulted the
  // planner under AddressSanitizer. Treat the disagreement as "this grid has no
  // data" rather than trusting the header, which is exactly the test
  // CostGrid::build() already applies to the same message (cost_grid.cpp), so
  // the two agree on what a malformed planning_map is. (2026-09-18)
  if (m.data.size() !=
      static_cast<size_t>(m.info.width) * static_cast<size_t>(m.info.height))
    return kCellNoData;
  int gx = static_cast<int>(std::floor(
      (pos.x() - m.info.origin.position.x) / m.info.resolution));
  int gy = static_cast<int>(std::floor(
      (pos.y() - m.info.origin.position.y) / m.info.resolution));
  if (gx < 0 || gy < 0 ||
      gx >= static_cast<int>(m.info.width) ||
      gy >= static_cast<int>(m.info.height))
    return kCellNoData;
  return m.data[gy * m.info.width + gx];
}

bool isCellFree(const nav_msgs::msg::OccupancyGrid& m,
                const Eigen::Vector3f& pos) {
  int8_t v = planMapCellAt(m, pos);  // kCellNoData/unknown both fail v >= 0
  return v >= 0 && v < 50;
}

bool isCellOccupied(const nav_msgs::msg::OccupancyGrid& m,
                    const Eigen::Vector3f& pos) {
  int8_t v = planMapCellAt(m, pos);
  return v == kCellNoData || v >= 50;
}

double unknownFractionInRoi(const nav_msgs::msg::OccupancyGrid& m,
                            const Roi2D& roi) {
  if (m.info.resolution <= 0.0f || m.info.width == 0 || m.info.height == 0)
    return -1.0;
  // Same buffer-vs-metadata mismatch planMapCellAt() screens above, and it
  // matters more here: the loop below indexes every cell of the clipped ROI, so
  // a short data vector walks off the end for the whole rectangle rather than at
  // one cell. -1.0 is already this function's "cannot be measured" answer (the
  // node's coverage-termination check treats it as "no reading this tick"), so
  // a malformed grid degrades to no reading instead of to a crash -- the
  // conservative direction, since a fabricated unknown-fraction would feed the
  // DONE criterion. (2026-09-18)
  if (m.data.size() !=
      static_cast<size_t>(m.info.width) * static_cast<size_t>(m.info.height))
    return -1.0;

  // Convert ROI world bounds to grid indices, clipped to the map.
  auto to_gx = [&](float x) {
    return static_cast<int>(std::floor(
        (x - m.info.origin.position.x) / m.info.resolution));
  };
  auto to_gy = [&](float y) {
    return static_cast<int>(std::floor(
        (y - m.info.origin.position.y) / m.info.resolution));
  };
  int gx0 = std::max(0, to_gx(roi.min_x));
  int gx1 = std::min(static_cast<int>(m.info.width) - 1, to_gx(roi.max_x));
  int gy0 = std::max(0, to_gy(roi.min_y));
  int gy1 = std::min(static_cast<int>(m.info.height) - 1, to_gy(roi.max_y));
  if (gx0 > gx1 || gy0 > gy1) return -1.0;

  int total = 0, unknown = 0;
  for (int gy = gy0; gy <= gy1; ++gy) {
    for (int gx = gx0; gx <= gx1; ++gx) {
      int8_t v = m.data[gy * m.info.width + gx];
      ++total;
      if (v < 0) ++unknown;
    }
  }
  if (total == 0) return -1.0;
  return static_cast<double>(unknown) / static_cast<double>(total);
}

} // namespace explo_planner
