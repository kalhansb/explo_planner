#include "explo_planner/leg.hpp"

namespace explo_planner {
namespace gen34 {

const char* LegTracker::statusName(Status s) {
  switch (s) {
    case Status::kIdle:     return "idle";
    case Status::kDriving:  return "driving";
    case Status::kEscaping: return "escaping";
    case Status::kArrived:  return "arrived";
  }
  return "?";
}

LegTracker::Command LegTracker::update(double now, const Vec2& pose, const Drive& d,
                                       const EscapePicker& pick) {
  Command c;
  if (d.key != key_ || status_ == Status::kIdle) {
    key_ = d.key;
    target_ = d.point;
    tol_ = d.tol;
    leg_escapes_ = 0;
    ++starts_total_;
    if (!counted_any_ || d.key != counted_key_) {
      counted_any_ = true;
      counted_key_ = d.key;
      ++legs_total_;
    }
    startWindow(now, pose);
    if (dist(pose, target_) <= tol_) {
      status_ = Status::kArrived;
      c.brake = true;
      return c;
    }
    status_ = Status::kDriving;
    c.publish = true;
    c.goal = target_;
    return c;
  }

  // Same leg: follow a moved target.
  tol_ = d.tol;
  if (dist(d.point, target_) > cfg_.retarget_min_m) {
    target_ = d.point;
    if (status_ == Status::kDriving) {
      c.publish = true;
      c.goal = target_;
    }
  }

  switch (status_) {
    case Status::kArrived:
      if (dist(pose, target_) > tol_ + cfg_.rearm_margin_m) {
        status_ = Status::kDriving;
        startWindow(now, pose);
        c.publish = true;
        c.goal = target_;
      }
      return c;

    case Status::kEscaping: {
      const bool done = dist(pose, escape_) <= cfg_.escape_arrive_m ||
                        now - escape_start_ >= cfg_.escape_max_sec;
      if (done) {
        status_ = Status::kDriving;
        startWindow(now, pose);
        c.publish = true;
        c.goal = target_;
      }
      return c;
    }

    case Status::kDriving: {
      if (dist(pose, target_) <= tol_) {
        status_ = Status::kArrived;
        c.publish = false;
        c.brake = true;
        return c;
      }
      if (now - win_start_ >= cfg_.progress_window_sec) {
        const double moved = dist(pose, win_pose_);
        startWindow(now, pose);
        if (moved < cfg_.progress_min_distance_m) {
          Vec2 e;
          if (pick && pick(pose, target_, &e)) {
            status_ = Status::kEscaping;
            escape_ = e;
            escape_start_ = now;
            ++leg_escapes_;
            ++escapes_total_;
            c.publish = true;
            c.goal = escape_;
            return c;
          }
          // Nowhere to escape to: re-send the target and keep trying.
          c.publish = true;
          c.goal = target_;
        }
      }
      return c;
    }

    case Status::kIdle:
      return c;
  }
  return c;
}

}  // namespace gen34
}  // namespace explo_planner
