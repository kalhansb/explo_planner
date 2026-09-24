// Lookout exploit mode (exploit_mode=lookout) and the tree-mode guarantee.
//
// GROUP A calls the pure pieces in lookout.hpp: config parsing, the arrival
// test, the settle test behind "in position".
//
// GROUP B scans the node's source, the same technique as test_gen22_start_hold
// and for the same reason (explo_planner_node.cpp is not in explo_planner_lib
// and defines main()). What it pins is the claim "tree mode behaves exactly as
// before": the mode defaults to tree, every way into a lookout state is behind
// `exploit_mode_ == ExploitMode::LOOKOUT`, and the tree arms the hooks sit in
// still do what they did.
//
// GROUP C scans the harness: the mapping and navigation inputs come off the
// 25 m crop relay, so they see what the old 25 m far clip gave them, while the
// raw 100 m cloud stays on /<r>/velodyne_points for detection.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include "explo_planner/lookout.hpp"

using explo_planner::DeliverPhase;
using explo_planner::ExploitMode;
using explo_planner::LookoutPoint;
using explo_planner::LookoutSettle;
using explo_planner::LookoutStart;
using explo_planner::lookoutLinkUp;
using explo_planner::lookoutReached;
using explo_planner::mulcherStandoffGoal;
using explo_planner::lookoutWrapAngle;
using explo_planner::parseExploitMode;
using explo_planner::parseLookoutStart;

namespace {

// Duplicated from test_gen22_start_hold.cpp rather than shared, as that file
// explains: each source-scan binary stays independently re-runnable.
std::string stripComments(const std::string& text) {
  std::string out;
  out.reserve(text.size());
  bool in_str = false, in_chr = false;
  for (size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    if (in_str || in_chr) {
      out.push_back(c);
      if (c == '\\' && i + 1 < text.size()) {
        out.push_back(text[++i]);
        continue;
      }
      if (in_str && c == '"') in_str = false;
      if (in_chr && c == '\'') in_chr = false;
      continue;
    }
    if (c == '"') { in_str = true; out.push_back(c); continue; }
    if (c == '\'') { in_chr = true; out.push_back(c); continue; }
    if (c == '/' && i + 1 < text.size() && text[i + 1] == '/') {
      while (i < text.size() && text[i] != '\n') ++i;
      out.push_back('\n');
      continue;
    }
    if (c == '/' && i + 1 < text.size() && text[i + 1] == '*') {
      i += 2;
      while (i + 1 < text.size() && !(text[i] == '*' && text[i + 1] == '/')) ++i;
      ++i;
      continue;
    }
    out.push_back(c);
  }
  return out;
}

std::string functionBody(const std::string& text, const std::string& signature) {
  const size_t sig = text.find(signature);
  if (sig == std::string::npos) return "";
  const size_t open = text.find('{', sig);
  if (open == std::string::npos) return "";
  int depth = 0;
  for (size_t i = open; i < text.size(); ++i) {
    if (text[i] == '{') ++depth;
    else if (text[i] == '}' && --depth == 0) {
      return text.substr(open, i - open + 1);
    }
  }
  return "";
}

std::string readFile(const char* path) {
  std::ifstream in(path);
  std::stringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

int countOf(const std::string& hay, const std::string& needle) {
  int n = 0;
  for (size_t at = hay.find(needle); at != std::string::npos;
       at = hay.find(needle, at + needle.size())) {
    ++n;
  }
  return n;
}

std::string nodeSource() { return stripComments(readFile(EXPLO_PLANNER_NODE_CPP)); }
std::string harnessSource() { return readFile(EXPLO_PLANNER_SIM_SH); }

std::string caseArm(const std::string& body, const std::string& label) {
  const size_t at = body.find(label);
  if (at == std::string::npos) return "";
  const size_t next = body.find("case State::", at + label.size());
  return body.substr(at, next == std::string::npos ? std::string::npos
                                                   : next - at);
}

constexpr double kDeg = M_PI / 180.0;

}  // namespace

// ============================== GROUP A ==================================

TEST(LookoutConfig, ParsesBothModesAndRejectsOthers) {
  ExploitMode m = ExploitMode::LOOKOUT;
  EXPECT_TRUE(parseExploitMode("tree", m));
  EXPECT_EQ(m, ExploitMode::TREE);
  EXPECT_TRUE(parseExploitMode("lookout", m));
  EXPECT_EQ(m, ExploitMode::LOOKOUT);
  EXPECT_FALSE(parseExploitMode("Lookout", m));
  EXPECT_FALSE(parseExploitMode("", m));

  LookoutStart s = LookoutStart::IMMEDIATE;
  EXPECT_TRUE(parseLookoutStart("on_done", s));
  EXPECT_EQ(s, LookoutStart::ON_DONE);
  EXPECT_TRUE(parseLookoutStart("immediate", s));
  EXPECT_EQ(s, LookoutStart::IMMEDIATE);
  EXPECT_FALSE(parseLookoutStart("now", s));
}

TEST(LookoutArrival, NeedsBothPositionAndHeading) {
  const LookoutPoint g{90.0, -2.0, 90.0 * kDeg};
  EXPECT_TRUE(lookoutReached(90.1, -2.1, 92.0 * kDeg, g, 0.3, 0.2));
  // Close in position, wrong heading: not arrived (a lookout faces its path).
  EXPECT_FALSE(lookoutReached(90.0, -2.0, 120.0 * kDeg, g, 0.3, 0.2));
  // Right heading, 0.5 m off: not arrived.
  EXPECT_FALSE(lookoutReached(90.5, -2.0, 90.0 * kDeg, g, 0.3, 0.2));
}

TEST(LookoutArrival, HeadingWrapsAcrossPlusMinusPi) {
  const LookoutPoint g{0.0, 0.0, 179.0 * kDeg};
  EXPECT_TRUE(lookoutReached(0.0, 0.0, -179.0 * kDeg, g, 0.3, 0.2));
  EXPECT_NEAR(lookoutWrapAngle(2.0 * M_PI + 0.1), 0.1, 1e-12);
}

TEST(LookoutSettleTest, ReportsOnlyAfterTheWindowStill) {
  LookoutSettle st(0.02, 0.5 * kDeg, 2.0);
  EXPECT_FALSE(st.update(10.0, 1.0, 1.0, 0.0));
  EXPECT_FALSE(st.update(11.0, 1.005, 1.0, 0.0));
  EXPECT_FALSE(st.update(11.9, 1.01, 1.0, 0.1 * kDeg));
  EXPECT_TRUE(st.update(12.0, 1.01, 1.0, 0.1 * kDeg));
  EXPECT_DOUBLE_EQ(st.refX(), 1.0);
}

TEST(LookoutSettleTest, CreepRestartsTheWindow) {
  LookoutSettle st(0.02, 0.5 * kDeg, 2.0);
  EXPECT_FALSE(st.update(0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(st.update(1.5, 0.05, 0.0, 0.0));   // moved 5 cm: restart at 1.5
  EXPECT_FALSE(st.update(3.0, 0.05, 0.0, 0.0));
  EXPECT_TRUE(st.update(3.5, 0.05, 0.0, 0.0));
  // A yaw creep restarts it too.
  EXPECT_FALSE(st.update(4.0, 0.05, 0.0, 1.0 * kDeg));
  EXPECT_FALSE(st.update(5.9, 0.05, 0.0, 1.0 * kDeg));
  EXPECT_TRUE(st.update(6.0, 0.05, 0.0, 1.0 * kDeg));
  st.reset();
  EXPECT_FALSE(st.update(6.1, 0.05, 0.0, 1.0 * kDeg));
}

TEST(LookoutMessenger, StandoffGoalIsShortOfTheMachineFacingIt) {
  const LookoutPoint g = mulcherStandoffGoal(90.0, 0.0, 0.0, 0.0, 5.0);
  EXPECT_NEAR(g.x, 5.0, 1e-12);
  EXPECT_NEAR(g.y, 0.0, 1e-12);
  EXPECT_NEAR(std::abs(g.yaw), M_PI, 1e-12);
  const LookoutPoint d = mulcherStandoffGoal(3.0, 4.0, 0.0, 0.0, 10.0);
  EXPECT_DOUBLE_EQ(d.x, 3.0);                 // already inside: stay put
  EXPECT_DOUBLE_EQ(d.y, 4.0);
  const LookoutPoint e = mulcherStandoffGoal(10.0, 10.0, 10.0, 30.0, 5.0);
  EXPECT_NEAR(e.x, 10.0, 1e-12);
  EXPECT_NEAR(e.y, 25.0, 1e-12);
  EXPECT_NEAR(e.yaw, M_PI / 2, 1e-12);
}

TEST(LookoutMessenger, LinkIsUpOnlyOnAFreshUpReport) {
  EXPECT_FALSE(lookoutLinkUp(false, true, 10.0, 10.0, 2.0));   // no report yet
  EXPECT_TRUE(lookoutLinkUp(true, true, 10.0, 11.5, 2.0));
  EXPECT_FALSE(lookoutLinkUp(true, true, 10.0, 12.5, 2.0));    // stale
  EXPECT_FALSE(lookoutLinkUp(true, false, 10.0, 10.1, 2.0));   // reported down
  EXPECT_NE(DeliverPhase::LAST_LINK, DeliverPhase::TO_MULCHER);
}

// ============================== GROUP B ==================================

TEST(LookoutNodeScan, ModeDefaultsToTreeAndStartToOnDone) {
  const std::string src = nodeSource();
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("dp(\"exploit_mode\", std::string(\"tree\"))"),
            std::string::npos);
  EXPECT_NE(src.find("dp(\"lookout_start\", std::string(\"on_done\"))"),
            std::string::npos);
  EXPECT_NE(src.find("ExploitMode  exploit_mode_  = ExploitMode::TREE;"),
            std::string::npos);
}

TEST(LookoutNodeScan, LookoutModeRefusesAMissingPoint) {
  const std::string src = nodeSource();
  EXPECT_NE(src.find("exploit_mode=lookout needs finite lookout_x"),
            std::string::npos);
}

// Every transition into a lookout state: LOOKOUT_NAV only through
// enterLookoutNav, LOOKOUT_HOLD only from doLookoutNav.
TEST(LookoutNodeScan, LookoutStatesAreEnteredOnlyThroughTheGatedHooks) {
  const std::string src = nodeSource();
  EXPECT_EQ(countOf(src, "transitionTo(State::LOOKOUT_NAV"), 1);
  EXPECT_EQ(countOf(src, "transitionTo(State::LOOKOUT_HOLD"), 1);
  const std::string nav = functionBody(src, "void ExploPlannerNode::doLookoutNav()");
  ASSERT_FALSE(nav.empty());
  EXPECT_NE(nav.find("transitionTo(State::LOOKOUT_HOLD"), std::string::npos);
  const std::string enter =
      functionBody(src, "void ExploPlannerNode::enterLookoutNav(");
  ASSERT_FALSE(enter.empty());
  EXPECT_NE(enter.find("transitionTo(State::LOOKOUT_NAV"), std::string::npos);

  // enterLookoutNav is called from exactly four places: the PLAN hook, the
  // DONE hook, doLookoutHold's drive-back and doLookoutDeliver's return
  // (both already inside lookout mode).
  EXPECT_EQ(countOf(src, "enterLookoutNav(\""), 4);
  const std::string tick = functionBody(src, "void ExploPlannerNode::tick()");
  ASSERT_FALSE(tick.empty());
  EXPECT_EQ(countOf(tick, "enterLookoutNav(\""), 2);
  const std::string hold = functionBody(src, "void ExploPlannerNode::doLookoutHold()");
  ASSERT_FALSE(hold.empty());
  EXPECT_EQ(countOf(hold, "enterLookoutNav(\""), 1);
  const std::string deliver =
      functionBody(src, "void ExploPlannerNode::doLookoutDeliver()");
  ASSERT_FALSE(deliver.empty());
  EXPECT_EQ(countOf(deliver, "enterLookoutNav(\""), 1);
}

TEST(LookoutNodeScan, PlanHookIsGatedAndPlanStillPlans) {
  const std::string tick = functionBody(nodeSource(), "void ExploPlannerNode::tick()");
  const std::string arm = caseArm(tick, "case State::PLAN:");
  ASSERT_FALSE(arm.empty());
  const size_t gate = arm.find("exploit_mode_ == ExploitMode::LOOKOUT &&");
  const size_t imm  = arm.find("lookout_start_ == LookoutStart::IMMEDIATE");
  const size_t hook = arm.find("enterLookoutNav(");
  const size_t plan = arm.find("doPlan();");
  ASSERT_NE(gate, std::string::npos);
  ASSERT_NE(imm, std::string::npos);
  ASSERT_NE(hook, std::string::npos);
  ASSERT_NE(plan, std::string::npos);
  EXPECT_LT(gate, hook);
  EXPECT_LT(imm, hook);
  EXPECT_LT(hook, plan);
  EXPECT_EQ(countOf(arm, "doPlan();"), 1);
}

TEST(LookoutNodeScan, DoneHookIsGatedAndDoneStillDoesWhatItDid) {
  const std::string tick = functionBody(nodeSource(), "void ExploPlannerNode::tick()");
  const std::string arm = caseArm(tick, "case State::DONE:");
  ASSERT_FALSE(arm.empty());
  const size_t gate = arm.find("exploit_mode_ == ExploitMode::LOOKOUT &&");
  const size_t ond  = arm.find("lookout_start_ == LookoutStart::ON_DONE");
  const size_t hook = arm.find("enterLookoutNav(");
  const size_t idle = arm.find("if (done_action_ == \"idle\")");
  ASSERT_NE(gate, std::string::npos);
  ASSERT_NE(ond, std::string::npos);
  ASSERT_NE(hook, std::string::npos);
  ASSERT_NE(idle, std::string::npos);
  EXPECT_LT(gate, hook);
  EXPECT_LT(hook, idle);
  // Tree mode's DONE: target pull-in, latch idling, shutdown, all still here.
  EXPECT_NE(arm.find("transitionTo(State::EXPLOIT_PLAN, \"target-arrived-done-idle\")"),
            std::string::npos);
  EXPECT_NE(arm.find("rclcpp::shutdown();"), std::string::npos);
}

TEST(LookoutNodeScan, LookoutPublishersExistOnlyInLookoutMode) {
  const std::string src = nodeSource();
  const size_t gate = src.find(
      "if (exploit_mode_ == ExploitMode::LOOKOUT) {\n"
      "    lookout_in_position_pub_ = create_publisher");
  EXPECT_NE(gate, std::string::npos);
  EXPECT_EQ(countOf(src, "lookout_in_position_pub_ = create_publisher"), 1);
  EXPECT_EQ(countOf(src, "lookout_pose_pub_ = create_publisher"), 1);
}

TEST(LookoutNodeScan, HoldReportsInPositionOnlyOnceSettled) {
  const std::string hold =
      functionBody(nodeSource(), "void ExploPlannerNode::doLookoutHold()");
  ASSERT_FALSE(hold.empty());
  const size_t settle = hold.find("lookout_settle_.update(");
  const size_t pub    = hold.find("b.data = true;");
  ASSERT_NE(settle, std::string::npos);
  ASSERT_NE(pub, std::string::npos);
  EXPECT_LT(settle, pub);
}

TEST(LookoutNodeScan, StateNamesAreStable) {
  const std::string src = nodeSource();
  EXPECT_NE(src.find("case State::LOOKOUT_NAV:    return \"LOOKOUT_NAV\";"),
            std::string::npos);
  EXPECT_NE(src.find("case State::LOOKOUT_HOLD:   return \"LOOKOUT_HOLD\";"),
            std::string::npos);
}

// LOOKOUT_DELIVER is entered only from an alarm, only by a robot that is in
// position at its post; everything else drops the alarm first.
TEST(LookoutNodeScan, DeliveryStartsOnlyFromAnAlarmAtThePost) {
  const std::string src = nodeSource();
  EXPECT_EQ(countOf(src, "transitionTo(State::LOOKOUT_DELIVER"), 1);
  const std::string on = functionBody(src, "void ExploPlannerNode::onLookoutAlarm(");
  ASSERT_FALSE(on.empty());
  const size_t guard = on.find("if (state_ != State::LOOKOUT_HOLD || !lookout_in_position_) {");
  const size_t linked = on.find("if (lookoutLinkNow()) {");
  const size_t off = on.find("lookout_in_position_ = false;");
  const size_t go = on.find("transitionTo(State::LOOKOUT_DELIVER");
  ASSERT_NE(guard, std::string::npos);
  ASSERT_NE(linked, std::string::npos);
  ASSERT_NE(off, std::string::npos);
  ASSERT_NE(go, std::string::npos);
  EXPECT_LT(guard, linked);     // not watching: dropped before anything else
  EXPECT_LT(linked, off);       // linked: sent at once, the robot stays put
  EXPECT_LT(off, go);           // leaving the post withdraws in_position first
}

TEST(LookoutNodeScan, DeliveryChecksTheLinkBeforeDriving) {
  const std::string deliver =
      functionBody(nodeSource(), "void ExploPlannerNode::doLookoutDeliver()");
  ASSERT_FALSE(deliver.empty());
  const size_t link = deliver.find("if (lookoutLinkNow()) {");
  const size_t send = deliver.find("lookoutSendWarning(");
  const size_t back = deliver.find("enterLookoutNav(\"lookout-return\")");
  const size_t drive = deliver.find("publishGoal(vp);");
  ASSERT_NE(link, std::string::npos);
  ASSERT_NE(send, std::string::npos);
  ASSERT_NE(back, std::string::npos);
  ASSERT_NE(drive, std::string::npos);
  EXPECT_LT(link, send);
  EXPECT_LT(send, back);
  EXPECT_LT(back, drive);
  // last-link point first, then on towards the machine
  EXPECT_NE(deliver.find("lookout_deliver_phase_ = DeliverPhase::TO_MULCHER;"),
            std::string::npos);
}

TEST(LookoutNodeScan, MessengerIoExistsOnlyInLookoutMode) {
  const std::string src = nodeSource();
  const size_t gate = src.find(
      "if (exploit_mode_ == ExploitMode::LOOKOUT) {\n"
      "    lookout_in_position_pub_ = create_publisher");
  ASSERT_NE(gate, std::string::npos);
  const size_t end = src.find("\n  }\n", gate);
  for (const char* what : {"lookout_warning_pub_ = create_publisher",
                           "lookout_link_sub_ = create_subscription",
                           "lookout_alarm_sub_ = create_subscription"}) {
    const size_t at = src.find(what);
    EXPECT_EQ(countOf(src, what), 1) << what;
    EXPECT_GT(at, gate) << what;
    EXPECT_LT(at, end) << what;
  }
  EXPECT_NE(src.find("case State::LOOKOUT_DELIVER: return \"LOOKOUT_DELIVER\";"),
            std::string::npos);
}

// ============================== GROUP C ==================================

TEST(LookoutHarnessScan, MappingAndNavReadTheCroppedCloud) {
  const std::string sh = harnessSource();
  ASSERT_FALSE(sh.empty());
  EXPECT_NE(sh.find("lidar_crop.py\" $r --max-range 25.0"), std::string::npos);
  EXPECT_NE(sh.find("lidar_points_topic:=/$r/velodyne_points_25m"),
            std::string::npos);
}
