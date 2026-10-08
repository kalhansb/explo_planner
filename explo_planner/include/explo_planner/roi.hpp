#pragma once
/// @file roi.hpp
/// @brief XY region of interest: a rectangle that may be rotated and offset
///        relative to the map frame.
///
/// The bounds are expressed in the ROI's own frame, whose origin sits at map
/// (origin_x, origin_y) and whose x axis points along map heading `yaw`:
///   u =  cos(yaw) (x - origin_x) + sin(yaw) (y - origin_y)
///   v = -sin(yaw) (x - origin_x) + cos(yaw) (y - origin_y)
/// and a map point is inside iff u in [min_x, max_x] and v in [min_y, max_y]
/// (inclusive). With yaw = 0 and origin (0, 0) — the defaults — the ROI frame
/// IS the map frame and every query reduces bit-for-bit to the legacy
/// axis-aligned box test.

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace explo_planner {

struct Roi2D {
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float yaw      = 0.0f;  ///< ROI x-axis heading in the map frame (rad)
  float origin_x = 0.0f;  ///< Map-frame position of the ROI-frame origin (m)
  float origin_y = 0.0f;

  // cos/sin of yaw, computed on first use: contains() runs per voxel in the
  // map clip and per column in the coverage walk. Keyed on yaw so a field
  // edited after construction still gets the right values.
  mutable float trig_yaw_ = NAN, cos_ = 1.0f, sin_ = 0.0f;
  void trig(float& c, float& s) const {
    if (!(trig_yaw_ == yaw)) {
      trig_yaw_ = yaw;
      cos_ = std::cos(yaw);
      sin_ = std::sin(yaw);
    }
    c = cos_;
    s = sin_;
  }

  /// False when the ROI frame is the map frame (the legacy axis-aligned box).
  bool transformed() const {
    return yaw != 0.0f || origin_x != 0.0f || origin_y != 0.0f;
  }

  /// Map-frame point -> ROI frame.
  Eigen::Vector2f toLocal(float x, float y) const {
    if (!transformed()) return {x, y};
    float c, s;
    trig(c, s);
    const float dx = x - origin_x, dy = y - origin_y;
    return {c * dx + s * dy, -s * dx + c * dy};
  }

  /// Map-frame direction -> ROI frame (rotation only, no offset).
  Eigen::Vector2f dirToLocal(float dx, float dy) const {
    if (yaw == 0.0f) return {dx, dy};
    float c, s;
    trig(c, s);
    return {c * dx + s * dy, -s * dx + c * dy};
  }

  /// ROI-frame point -> map frame.
  Eigen::Vector2f toMap(float u, float v) const {
    if (!transformed()) return {u, v};
    float c, s;
    trig(c, s);
    return {origin_x + c * u - s * v, origin_y + s * u + c * v};
  }

  /// Inclusive containment test for a map-frame point.
  bool contains(float x, float y) const {
    const Eigen::Vector2f p = toLocal(x, y);
    return p.x() >= min_x && p.x() <= max_x &&
           p.y() >= min_y && p.y() <= max_y;
  }

  /// Smallest map-axis-aligned box enclosing the ROI (identity frame). For an
  /// untransformed ROI this is the ROI itself.
  Roi2D mapAabb() const {
    if (!transformed()) return {min_x, max_x, min_y, max_y};
    const Eigen::Vector2f c[4] = {toMap(min_x, min_y), toMap(max_x, min_y),
                                  toMap(max_x, max_y), toMap(min_x, max_y)};
    Roi2D out{c[0].x(), c[0].x(), c[0].y(), c[0].y()};
    for (const auto& p : c) {
      out.min_x = std::min(out.min_x, p.x());
      out.max_x = std::max(out.max_x, p.x());
      out.min_y = std::min(out.min_y, p.y());
      out.max_y = std::max(out.max_y, p.y());
    }
    return out;
  }
};

} // namespace explo_planner
