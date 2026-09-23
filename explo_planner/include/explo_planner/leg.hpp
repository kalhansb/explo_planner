#pragma once
/// @file leg.hpp
/// @brief Gen 34's one drive for everything that is not an exploration goal:
///        meeting spots, reconnect moves, chase points, the follow point and
///        homing (DESIGN_gen34.md §8.3 Leg). Pure C++, no ROS.
///
/// A leg takes a point and a tolerance. The caller owns the time bound: the
/// leg never gives up on its own. Its watchdog is gen 33's
/// progress_window_sec / progress_min_distance_m pair: when the robot moved
/// less than the minimum over a window, the leg takes one escape move to a
/// free point 1.5-6 m away (picked by the caller, gen 33's escape picker), then
/// resumes. Escapes are unlimited. There is no blacklist.
///
/// The tracker says what to publish; the node publishes it. A new key starts a
/// new leg. The same key with a moved point retargets it without resetting the
/// watchdog, which is how the follow point tracks a moving peer.

#include <cstdint>
#include <functional>

#include "explo_planner/team_core.hpp"

namespace explo_planner {
namespace gen34 {

struct LegConfig {
  double progress_window_sec = 30.0;
  double progress_min_distance_m = 0.5;
  /// An escape move ends on arrival within this, or after escape_max_sec.
  double escape_arrive_m = 1.0;
  double escape_max_sec = 30.0;
  /// After arrival, drift beyond tol + this re-starts the drive.
  double rearm_margin_m = 1.0;
  /// A retarget smaller than this is not re-published.
  double retarget_min_m = 0.5;
};

class LegTracker {
 public:
  enum class Status : uint8_t { kIdle, kDriving, kEscaping, kArrived };

  struct Command {
    bool publish = false;   ///< publish `goal` now
    Vec2 goal;
    bool brake = false;     ///< stop here (arrival): publish a brake goal
  };

  /// Picks an escape point from `from`; false when none is free.
  using EscapePicker = std::function<bool(const Vec2& from, const Vec2& toward, Vec2* out)>;

  explicit LegTracker(const LegConfig& cfg = LegConfig{}) : cfg_(cfg) {}

  /// One tick of a Leg drive. `d.kind` must be kLeg.
  Command update(double now, const Vec2& pose, const Drive& d, const EscapePicker& pick);

  /// Forget the leg (the drive moved to explore or hold).
  void reset() { status_ = Status::kIdle; key_ = 0; }

  Status status() const { return status_; }
  static const char* statusName(Status s);
  /// The goal currently commanded (target or escape point).
  Vec2 currentGoal() const { return status_ == Status::kEscaping ? escape_ : target_; }
  bool active() const { return status_ == Status::kDriving || status_ == Status::kEscaping; }
  long long escapes() const { return escapes_total_; }
  long long legs() const { return legs_total_; }
  /// Escapes taken on the current leg.
  int legEscapes() const { return leg_escapes_; }

 private:
  void startWindow(double now, const Vec2& pose) { win_start_ = now; win_pose_ = pose; }

  LegConfig cfg_;
  Status status_ = Status::kIdle;
  uint64_t key_ = 0;
  Vec2 target_;
  double tol_ = 1.0;
  Vec2 escape_;
  double escape_start_ = 0.0;
  double win_start_ = 0.0;
  Vec2 win_pose_;
  int leg_escapes_ = 0;
  long long escapes_total_ = 0;
  long long legs_total_ = 0;
};

}  // namespace gen34
}  // namespace explo_planner
