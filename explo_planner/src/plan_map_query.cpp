/// @file plan_map_query.cpp
/// @brief Definitions for the 2D planning_map cell queries (see header).

#include "explo_planner/plan_map_query.hpp"

#include <algorithm>
#include <cmath>

namespace explo_planner {

int8_t planMapCellAt(const nav_msgs::msg::OccupancyGrid& m,
                     const Eigen::Vector3f& pos) {
  if (m.info.resolution <= 0.0f || m.info.width == 0 || m.info.height == 0 ||
      m.data.size() != static_cast<size_t>(m.info.width) * m.info.height)
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
                const Eigen::Vector3f& pos, int8_t threshold) {
  int8_t v = planMapCellAt(m, pos);  // kCellNoData/unknown both fail v >= 0
  return v >= 0 && v < threshold;
}

bool isCellOccupied(const nav_msgs::msg::OccupancyGrid& m,
                    const Eigen::Vector3f& pos, int8_t threshold) {
  int8_t v = planMapCellAt(m, pos);
  return v == kCellNoData || v >= threshold;
}

double unknownFractionInRoi(const nav_msgs::msg::OccupancyGrid& m,
                            const Roi2D& roi) {
  if (m.info.resolution <= 0.0f || m.info.width == 0 || m.info.height == 0 ||
      m.data.size() != static_cast<size_t>(m.info.width) * m.info.height)
    return -1.0;

  // Convert ROI world bounds to grid indices, clipped to the map. A rotated
  // ROI walks its map-frame bounding box and keeps only cells whose centre is
  // inside the ROI.
  const Roi2D box = roi.mapAabb();
  const bool rotated = roi.transformed();
  auto to_gx = [&](float x) {
    return static_cast<int>(std::floor(
        (x - m.info.origin.position.x) / m.info.resolution));
  };
  auto to_gy = [&](float y) {
    return static_cast<int>(std::floor(
        (y - m.info.origin.position.y) / m.info.resolution));
  };
  int gx0 = std::max(0, to_gx(box.min_x));
  int gx1 = std::min(static_cast<int>(m.info.width) - 1, to_gx(box.max_x));
  int gy0 = std::max(0, to_gy(box.min_y));
  int gy1 = std::min(static_cast<int>(m.info.height) - 1, to_gy(box.max_y));
  if (gx0 > gx1 || gy0 > gy1) return -1.0;

  int total = 0, unknown = 0;
  for (int gy = gy0; gy <= gy1; ++gy) {
    for (int gx = gx0; gx <= gx1; ++gx) {
      if (rotated &&
          !roi.contains(
              static_cast<float>(m.info.origin.position.x +
                                 (gx + 0.5) * m.info.resolution),
              static_cast<float>(m.info.origin.position.y +
                                 (gy + 0.5) * m.info.resolution)))
        continue;
      int8_t v = m.data[gy * m.info.width + gx];
      ++total;
      if (v < 0) ++unknown;
    }
  }
  if (total == 0) return -1.0;
  return static_cast<double>(unknown) / static_cast<double>(total);
}

} // namespace explo_planner
