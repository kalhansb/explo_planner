/// @file lookout.hpp
/// @brief Lookout exploit mode: the pure pieces (config parsing, arrival and
///        hold tests), kept out of explo_planner_node.cpp so they can be unit
///        tested.
///
/// The exploit phase has two modes. TREE (the default) is the vantage ring
/// around released tree targets and is untouched by anything here. LOOKOUT
/// sends each robot to one fixed (x, y, yaw) given by configuration -- the
/// robot makes no placement decision -- then holds it and reports that it is
/// in position. Use: a working machine's lidar cannot see the path entries it
/// must watch; the robots stand there as remote sensors.
#pragma once

#include <cmath>
#include <string>

namespace explo_planner {

enum class ExploitMode { TREE, LOOKOUT };

/// When a lookout robot sets off. ON_DONE: after its exploration is DONE (the
/// map-then-support order). IMMEDIATE: as soon as the planner leaves
/// WAIT_FOR_MAP, with no exploration.
enum class LookoutStart { ON_DONE, IMMEDIATE };

inline bool parseExploitMode(const std::string& s, ExploitMode& out) {
  if (s == "tree")    { out = ExploitMode::TREE;    return true; }
  if (s == "lookout") { out = ExploitMode::LOOKOUT; return true; }
  return false;
}

inline bool parseLookoutStart(const std::string& s, LookoutStart& out) {
  if (s == "on_done")   { out = LookoutStart::ON_DONE;   return true; }
  if (s == "immediate") { out = LookoutStart::IMMEDIATE; return true; }
  return false;
}

/// One robot's lookout point in the map frame; yaw in radians.
struct LookoutPoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double lookoutWrapAngle(double a) {
  return std::remainder(a, 2.0 * M_PI);
}

/// Arrival: planar distance and heading error both inside tolerance (the same
/// test NAVIGATE applies to an exploit vantage, where yaw always counts).
inline bool lookoutReached(double x, double y, double yaw,
                           const LookoutPoint& g, double xy_tol, double yaw_tol) {
  const double d = std::hypot(x - g.x, y - g.y);
  return d <= xy_tol && std::abs(lookoutWrapAngle(yaw - g.yaw)) <= yaw_tol;
}

/// Settle test for the hold: the robot counts as held once its pose has stayed
/// within (pos_eps, yaw_eps) of a reference for `window` seconds. The
/// navigator's own arrival tolerance is tighter than the planner's, so a robot
/// is often still creeping when the planner first sees it inside tolerance;
/// "in position" is reported only once it has actually stopped.
class LookoutSettle {
 public:
  LookoutSettle(double pos_eps, double yaw_eps, double window)
      : pos_eps_(pos_eps), yaw_eps_(yaw_eps), window_(window) {}

  void reset() { have_ref_ = false; }

  /// Feed one pose at time t (s). Returns true once settled; the reference
  /// pose is then the settled pose.
  bool update(double t, double x, double y, double yaw) {
    if (!have_ref_ || std::hypot(x - rx_, y - ry_) > pos_eps_ ||
        std::abs(lookoutWrapAngle(yaw - ryaw_)) > yaw_eps_) {
      rx_ = x; ry_ = y; ryaw_ = yaw; t0_ = t; have_ref_ = true;
      return false;
    }
    return (t - t0_) >= window_;
  }

  double refX() const { return rx_; }
  double refY() const { return ry_; }
  double refYaw() const { return ryaw_; }

 private:
  double pos_eps_, yaw_eps_, window_;
  bool have_ref_{false};
  double rx_{0.0}, ry_{0.0}, ryaw_{0.0}, t0_{0.0};
};

// ---------------------------------------------------------------------------
// Messenger (warning delivery). A lookout's alarm is only useful once it
// reaches the machine it protects, over the radio. At its post the lookout is
// usually out of radio range, so on an alarm it carries the warning: first to
// the last point where it had a link on the way out, checking the link all
// the way and sending the moment it comes up; if the link is still down
// there, on towards the machine until it comes up. Then back to its post.

/// Where a delivery trip is heading: the last-link point, then the machine.
enum class DeliverPhase { LAST_LINK, TO_MULCHER };

/// Goal `standoff` m short of the machine at (mx, my) on the straight line
/// from (x, y), facing it; the current position if already that close.
inline LookoutPoint mulcherStandoffGoal(double x, double y, double mx, double my,
                                        double standoff) {
  const double dx = mx - x, dy = my - y, d = std::hypot(dx, dy);
  LookoutPoint g;
  g.yaw = std::atan2(dy, dx);
  if (d <= standoff) {
    g.x = x;
    g.y = y;
    return g;
  }
  g.x = mx - dx / d * standoff;
  g.y = my - dy / d * standoff;
  return g;
}

/// Link state as last reported: up only if a report exists, it said up, and
/// it is no older than `stale_sec` (a silent link monitor means no link).
inline bool lookoutLinkUp(bool have_report, bool report_up, double t_report,
                          double t_now, double stale_sec) {
  return have_report && report_up && (t_now - t_report) <= stale_sec;
}

}  // namespace explo_planner
