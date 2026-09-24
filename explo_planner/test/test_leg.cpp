// LegTracker: the one drive for every non-exploration goal (DESIGN_gen34.md
// §8.3 Leg). Pure; no ROS.

#include <gtest/gtest.h>

#include "explo_planner/leg.hpp"

using explo_planner::gen34::Drive;
using explo_planner::gen34::DriveKind;
using explo_planner::gen34::LegConfig;
using explo_planner::gen34::LegTracker;
using explo_planner::gen34::Vec2;

namespace {

Drive leg(uint64_t key, Vec2 p, double tol = 1.0) {
  Drive d;
  d.kind = DriveKind::kLeg;
  d.point = p;
  d.tol = tol;
  d.key = key;
  return d;
}

LegTracker::EscapePicker picker(bool ok, Vec2 e = {5, 5}) {
  return [ok, e](const Vec2&, const Vec2&, Vec2* out) {
    if (ok) *out = e;
    return ok;
  };
}

}  // namespace

TEST(Leg, NewKeyPublishesTheTarget) {
  LegTracker t;
  const auto c = t.update(0.0, {0, 0}, leg(1, {10, 0}), picker(true));
  EXPECT_TRUE(c.publish);
  EXPECT_FALSE(c.brake);
  EXPECT_DOUBLE_EQ(c.goal.x, 10.0);
  EXPECT_EQ(t.status(), LegTracker::Status::kDriving);
  EXPECT_EQ(t.legs(), 1);
}

TEST(Leg, StartingInsideToleranceBrakesAtOnce) {
  LegTracker t;
  const auto c = t.update(0.0, {9.5, 0}, leg(1, {10, 0}), picker(true));
  EXPECT_FALSE(c.publish);
  EXPECT_TRUE(c.brake);
  EXPECT_EQ(t.status(), LegTracker::Status::kArrived);
}

TEST(Leg, ArrivalBrakesOnceThenIsQuiet) {
  LegTracker t;
  t.update(0.0, {0, 0}, leg(1, {10, 0}), picker(true));
  auto c = t.update(1.0, {5, 0}, leg(1, {10, 0}), picker(true));
  EXPECT_FALSE(c.publish);
  EXPECT_FALSE(c.brake);
  c = t.update(2.0, {9.2, 0}, leg(1, {10, 0}), picker(true));
  EXPECT_TRUE(c.brake);
  EXPECT_EQ(t.status(), LegTracker::Status::kArrived);
  c = t.update(3.0, {9.2, 0}, leg(1, {10, 0}), picker(true));
  EXPECT_FALSE(c.brake);
  EXPECT_FALSE(c.publish);
}

TEST(Leg, DriftBeyondToleranceRearms) {
  LegTracker t;
  t.update(0.0, {9.5, 0}, leg(1, {10, 0}), picker(true));   // arrived
  // Pushed 1.5 m off: within tol + rearm margin (2 m) stays arrived.
  auto c = t.update(1.0, {8.5, 0}, leg(1, {10, 0}), picker(true));
  EXPECT_FALSE(c.publish);
  // 2.5 m off: drive again.
  c = t.update(2.0, {7.5, 0}, leg(1, {10, 0}), picker(true));
  EXPECT_TRUE(c.publish);
  EXPECT_EQ(t.status(), LegTracker::Status::kDriving);
}

TEST(Leg, SameKeyRetargetsWithoutANewLeg) {
  LegTracker t;
  t.update(0.0, {0, 0}, leg(1, {10, 0}), picker(true));
  // A 0.3 m move is below retarget_min_m: not re-published.
  auto c = t.update(1.0, {1, 0}, leg(1, {10.3, 0}), picker(true));
  EXPECT_FALSE(c.publish);
  c = t.update(2.0, {2, 0}, leg(1, {12, 0}), picker(true));
  EXPECT_TRUE(c.publish);
  EXPECT_DOUBLE_EQ(c.goal.x, 12.0);
  EXPECT_EQ(t.legs(), 1);
}

TEST(Leg, NewKeyStartsANewLeg) {
  LegTracker t;
  t.update(0.0, {0, 0}, leg(1, {10, 0}), picker(true));
  auto c = t.update(1.0, {1, 0}, leg(2, {10, 0}), picker(true));
  EXPECT_TRUE(c.publish);
  EXPECT_EQ(t.legs(), 2);
  EXPECT_EQ(t.starts(), 2);
}

TEST(Leg, WatchdogEscapesThenResumes) {
  LegConfig cfg;
  cfg.progress_window_sec = 30.0;
  cfg.progress_min_distance_m = 0.5;
  cfg.escape_max_sec = 30.0;
  LegTracker t(cfg);
  t.update(0.0, {0, 0}, leg(1, {20, 0}), picker(true, {0, 3}));
  // Stuck at the origin for a whole window.
  auto c = t.update(15.0, {0.1, 0}, leg(1, {20, 0}), picker(true, {0, 3}));
  EXPECT_FALSE(c.publish);
  c = t.update(30.0, {0.2, 0}, leg(1, {20, 0}), picker(true, {0, 3}));
  EXPECT_TRUE(c.publish);
  EXPECT_DOUBLE_EQ(c.goal.y, 3.0);
  EXPECT_EQ(t.status(), LegTracker::Status::kEscaping);
  EXPECT_EQ(t.escapes(), 1);
  EXPECT_EQ(t.legEscapes(), 1);
  // Reaching the escape point resumes the target.
  c = t.update(40.0, {0, 2.5}, leg(1, {20, 0}), picker(true, {0, 3}));
  EXPECT_TRUE(c.publish);
  EXPECT_DOUBLE_EQ(c.goal.x, 20.0);
  EXPECT_EQ(t.status(), LegTracker::Status::kDriving);
}

TEST(Leg, EscapeTimesOut) {
  LegTracker t;
  t.update(0.0, {0, 0}, leg(1, {20, 0}), picker(true, {0, 3}));
  t.update(30.0, {0, 0}, leg(1, {20, 0}), picker(true, {0, 3}));
  ASSERT_EQ(t.status(), LegTracker::Status::kEscaping);
  auto c = t.update(59.0, {0, 0}, leg(1, {20, 0}), picker(true, {0, 3}));
  EXPECT_FALSE(c.publish);
  c = t.update(60.0, {0, 0}, leg(1, {20, 0}), picker(true, {0, 3}));
  EXPECT_TRUE(c.publish);
  EXPECT_EQ(t.status(), LegTracker::Status::kDriving);
}

TEST(Leg, EscapesAreUnlimited) {
  LegTracker t;
  t.update(0.0, {0, 0}, leg(1, {20, 0}), picker(true, {0, 3}));
  double now = 0.0;
  for (int i = 0; i < 5; ++i) {
    now += 30.0;
    t.update(now, {0, 0}, leg(1, {20, 0}), picker(true, {0, 3}));   // escape
    now += 30.0;
    t.update(now, {0, 0}, leg(1, {20, 0}), picker(true, {0, 3}));   // timed out
  }
  EXPECT_EQ(t.escapes(), 5);
  EXPECT_EQ(t.legEscapes(), 5);
}

TEST(Leg, NoEscapePointResendsTheTarget) {
  LegTracker t;
  t.update(0.0, {0, 0}, leg(1, {20, 0}), picker(false));
  const auto c = t.update(30.0, {0, 0}, leg(1, {20, 0}), picker(false));
  EXPECT_TRUE(c.publish);
  EXPECT_DOUBLE_EQ(c.goal.x, 20.0);
  EXPECT_EQ(t.status(), LegTracker::Status::kDriving);
  EXPECT_EQ(t.escapes(), 0);
}

TEST(Leg, ProgressResetsTheWatchdog) {
  LegTracker t;
  t.update(0.0, {0, 0}, leg(1, {20, 0}), picker(true));
  const auto c = t.update(30.0, {3, 0}, leg(1, {20, 0}), picker(true));
  EXPECT_FALSE(c.publish);
  EXPECT_EQ(t.status(), LegTracker::Status::kDriving);
}

TEST(Leg, ResetForgetsTheLeg) {
  LegTracker t;
  t.update(0.0, {0, 0}, leg(1, {20, 0}), picker(true));
  t.reset();
  EXPECT_EQ(t.status(), LegTracker::Status::kIdle);
  // The same key after a reset is a new leg: an explore stint in between must
  // not leave the old one half-armed.
  const auto c = t.update(5.0, {0, 0}, leg(1, {20, 0}), picker(true));
  EXPECT_TRUE(c.publish);
  // One drive, two starts (§11.5).
  EXPECT_EQ(t.legs(), 1);
  EXPECT_EQ(t.starts(), 2);
}
