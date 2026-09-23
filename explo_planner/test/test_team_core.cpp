// TeamCore component tests (DESIGN_gen34.md §8.3-8.4): presence, exchange,
// team_finished, the plan, bookings, chases, the priority list and homing.
// Each test drives a few cores by hand: the test sets what each robot's
// dscovox and pose say, and the rig passes beacons over the links it is told
// are up. A tick runs every core, then delivers the beacons, then advances the
// clock one second, so robot i sees robot j's beacon from the previous tick.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "explo_planner/team_core.hpp"

using namespace explo_planner::gen34;
using explo_planner::findField;
using explo_planner::LogField;

namespace {

struct FakeOracle : TeamOracle {
  PlanSolve solve;          // what solvePlan answers
  int solves = 0;
  ChaseGateVerdict gate;
  int gate_calls = 0;
  bool has_intercept = false;
  Vec2 intercept_at;
  std::vector<Vec2> blocked;   // not standable within 0.5 m of these

  FakeOracle() {
    solve.ok = true;
    solve.cell = 3;
    solve.center = {10, 10};
    solve.provisional = false;
  }
  PlanSolve solvePlan(const std::vector<TeamRobotView>&) override {
    ++solves;
    return solve;
  }
  double pathDistance(const Vec2& from, int, const Vec2& center) override {
    return dist(from, center);
  }
  bool standable(const Vec2& p) override {
    for (const Vec2& b : blocked)
      if (dist(p, b) < 0.5) return false;
    return true;
  }
  bool lineOfSight(const Vec2&, const Vec2&) override { return true; }
  ChaseGateVerdict chaseGate(const std::vector<ChaseGateView>&) override {
    ++gate_calls;
    return gate;
  }
  bool intercept(const ChaseGateView&, Vec2* out) override {
    if (has_intercept) *out = intercept_at;
    return has_intercept;
  }
};

struct Bot {
  FakeOracle oracle;
  std::unique_ptr<TeamCore> core;
  TickInputs in;
  TickOutputs out;
  std::vector<TeamEvent> events;
};

struct Rig {
  int n;
  std::vector<Bot> b;
  std::vector<std::vector<bool>> up;
  double now = 0.0;

  Rig(int n_, Arm arm, void (*tweak)(TeamConfig&) = nullptr) : n(n_), b(n_) {
    up.assign(n, std::vector<bool>(n, true));
    for (int i = 0; i < n; ++i) {
      TeamConfig c;
      c.arm = arm;
      c.self_id = i;
      c.n = n;
      c.team_hash = 7;
      c.grid_hash = 9;
      // Booking tests cut links and expect the booking on the next tick; the
      // separation hysteresis has its own test.
      c.book_apart_sec = 0.0;
      if (tweak) tweak(c);
      b[i].core = std::make_unique<TeamCore>(c, &b[i].oracle);
      b[i].in.have_pose = true;
      b[i].in.pose = {20.0 * i, 0.0};
      b[i].in.home = {20.0 * i, -5.0};
      b[i].in.have_home = true;
      b[i].in.rcvd_seq.assign(n, 0);
      b[i].in.seq_gaps.assign(n, 0);
    }
  }
  void link(int i, int j, bool on) { up[i][j] = up[j][i] = on; }
  void isolate(int i) {
    for (int j = 0; j < n; ++j)
      if (j != i) link(i, j, false);
  }
  void allLinks(bool on) {
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j)
        if (i != j) up[i][j] = on;
  }
  void step() {
    for (int i = 0; i < n; ++i) {
      b[i].in.now = now;
      b[i].out = b[i].core->tick(b[i].in);
      for (auto& e : b[i].core->drainEvents()) b[i].events.push_back(std::move(e));
    }
    for (int i = 0; i < n; ++i) {
      const Beacon bc = b[i].core->beacon();
      for (int j = 0; j < n; ++j)
        if (j != i && up[i][j]) b[j].core->onBeacon(bc, now);
    }
    now += 1.0;
  }
  void run(double secs) {
    for (double t = 0; t < secs; t += 1.0) step();
  }
  // Steps until pred holds or the budget runs out; true when it held.
  template <class P>
  bool until(P pred, double budget) {
    for (double t = 0; t < budget; t += 1.0) {
      step();
      if (pred()) return true;
    }
    return false;
  }
  TeamCore& core(int i) { return *b[i].core; }
};

// Events of a kind with a given action (or any action when empty).
int countEvents(const Bot& bot, const std::string& kind, const std::string& action = "") {
  int k = 0;
  for (const TeamEvent& e : bot.events) {
    if (e.kind != kind) continue;
    if (action.empty()) { ++k; continue; }
    const LogField* f = findField(e.fields, "action");
    if (f && f->s == action) ++k;
  }
  return k;
}

const TeamEvent* lastEvent(const Bot& bot, const std::string& kind, const std::string& action) {
  for (auto it = bot.events.rbegin(); it != bot.events.rend(); ++it) {
    if (it->kind != kind) continue;
    const LogField* f = findField(it->fields, "action");
    if (f && f->s == action) return &*it;
  }
  return nullptr;
}

long long countOf(const TeamCore& c, const std::string& k) {
  auto it = c.counts().find(k);
  return it == c.counts().end() ? 0 : it->second;
}

// A two-robot rendezvous rig with plan v1 (cell 3 at (10,10), t0 = 302) agreed
// at t=2, then split up at t=3.
Rig splitPair() {
  Rig r(2, Arm::kRendezvous);
  r.b[0].in.pose = {0, 0};
  r.b[1].in.pose = {20, 0};
  r.run(4);
  r.allLinks(false);
  return r;
}

}  // namespace

// ── Beacon admission ────────────────────────────────────────────────────────

TEST(TeamCoreBeacon, RejectsForeignBadAndStale) {
  FakeOracle o;
  TeamConfig c;
  c.n = 3;
  c.self_id = 1;
  c.team_hash = 7;
  c.grid_hash = 9;
  TeamCore core(c, &o);
  Beacon b;
  b.team_hash = 8;
  b.robot_id = 0;
  EXPECT_FALSE(core.onBeacon(b, 0.0));
  b.team_hash = 7;
  b.robot_id = 1;   // self
  EXPECT_FALSE(core.onBeacon(b, 0.0));
  b.robot_id = 3;   // out of range for n=3
  EXPECT_FALSE(core.onBeacon(b, 0.0));
  b.robot_id = 0;
  b.stamp = 10.0;
  EXPECT_TRUE(core.onBeacon(b, 10.0));
  b.stamp = 9.0;    // older than the last one
  EXPECT_FALSE(core.onBeacon(b, 11.0));
  EXPECT_EQ(countOf(core, "beacon_foreign_team"), 1);
  EXPECT_EQ(countOf(core, "beacon_bad_id"), 2);
  EXPECT_EQ(countOf(core, "beacon_stale"), 1);
}

TEST(TeamCoreBeacon, GridMismatchIsHeardButNotAPlanSource) {
  FakeOracle o;
  TeamConfig c;
  c.arm = Arm::kRendezvous;
  c.n = 2;
  c.self_id = 1;
  c.team_hash = 7;
  c.grid_hash = 9;
  TeamCore core(c, &o);
  Beacon b;
  b.team_hash = 7;
  b.grid_hash = 10;
  b.robot_id = 0;
  b.plan.version = 3;
  b.plan.interval = 300;
  b.plan.cell = 4;
  ASSERT_TRUE(core.onBeacon(b, 0.0));
  TickInputs in;
  in.now = 1.0;
  core.tick(in);
  EXPECT_TRUE(core.present(0));
  EXPECT_EQ(core.plan().version, 0u);
  EXPECT_EQ(countOf(core, "beacon_grid_mismatch"), 1);
}

// ── Presence ────────────────────────────────────────────────────────────────

TEST(TeamCorePresence, ContactNeedsTheOtherSideToHearMe) {
  Rig r(2, Arm::kOff);
  r.step();   // t=0: nobody heard yet
  EXPECT_FALSE(r.core(0).present(1));
  r.step();   // t=1: heard, but the t=0 beacon had an empty hears mask
  EXPECT_TRUE(r.core(0).present(1));
  EXPECT_FALSE(r.core(0).inContact(1));
  r.step();   // t=2: both ways
  EXPECT_TRUE(r.core(0).inContact(1));
  EXPECT_TRUE(r.core(0).allConnected());
}

TEST(TeamCorePresence, OneWayLinkIsPresenceWithoutContact) {
  Rig r(2, Arm::kOff);
  r.up[1][0] = true;
  r.up[0][1] = false;   // 0 hears 1; 1 never hears 0
  r.run(5);
  EXPECT_TRUE(r.core(0).present(1));
  EXPECT_FALSE(r.core(0).inContact(1));
  EXPECT_FALSE(r.core(1).present(0));
}

TEST(TeamCorePresence, ExpiresAfterTheWindow) {
  Rig r(2, Arm::kOff);
  r.run(3);
  ASSERT_TRUE(r.core(0).inContact(1));
  r.allLinks(false);
  r.run(10);   // last beacon at t=2; at t=12 the age is 10, still in
  EXPECT_TRUE(r.core(0).present(1));
  r.step();    // t=13: age 11
  EXPECT_FALSE(r.core(0).present(1));
  EXPECT_FALSE(r.core(0).inContact(1));
}

TEST(TeamCorePresence, ThirdPartyPairFromHearsMasks) {
  Rig r(3, Arm::kOff);
  r.link(1, 2, false);
  r.run(4);
  EXPECT_TRUE(r.core(0).pairInContact(0, 1));
  EXPECT_TRUE(r.core(0).pairInContact(0, 2));
  EXPECT_FALSE(r.core(0).pairInContact(1, 2));
  EXPECT_FALSE(r.core(0).allConnected());
  r.link(1, 2, true);
  r.run(3);
  EXPECT_TRUE(r.core(0).allConnected());
}

// ── Exchange ────────────────────────────────────────────────────────────────

TEST(TeamCoreExchange, CompletesWhenBothSidesHaveTheTargets) {
  Rig r(2, Arm::kOff);
  r.b[0].in.sent_seq = 5;
  r.b[1].in.sent_seq = 8;
  r.run(3);   // contact at t=2
  ASSERT_TRUE(r.core(0).inContact(1));
  EXPECT_EQ(r.core(0).exchange(1).state, ExchangeRecord::State::kRunning);
  EXPECT_EQ(r.core(0).exchange(1).target_rx, 8u);
  EXPECT_EQ(r.core(0).exchange(1).target_tx, 5u);
  // Their sends keep growing; the targets do not move.
  r.b[0].in.sent_seq = 50;
  r.b[1].in.sent_seq = 80;
  r.b[0].in.rcvd_seq[1] = 8;
  r.b[1].in.rcvd_seq[0] = 5;
  r.run(2);
  EXPECT_TRUE(r.core(0).exchanged(1));
  EXPECT_TRUE(r.core(1).exchanged(0));
  EXPECT_EQ(countEvents(r.b[0], "exchange", "done"), 1);
  EXPECT_EQ(countOf(r.core(0), "exchange_done_trivial"), 0);
  // The beacon says so, for third parties.
  EXPECT_TRUE(r.core(0).beacon().exchanged_mask & 0x2u);
}

TEST(TeamCoreExchange, NothingToSendIsATrivialDone) {
  Rig r(2, Arm::kOff);
  r.run(3);
  EXPECT_TRUE(r.core(0).exchanged(1));
  EXPECT_EQ(countOf(r.core(0), "exchange_done_trivial"), 1);
}

TEST(TeamCoreExchange, UnmeasuredSeqStartsNoExchange) {
  // No fusion counters yet: every seq reads 0, and 0 >= 0 is not a swap.
  Rig r(2, Arm::kOff);
  r.b[0].in.seq_valid = false;
  r.run(3);
  ASSERT_TRUE(r.core(0).inContact(1));
  EXPECT_FALSE(r.core(0).exchange(1).active);
  EXPECT_FALSE(r.core(0).exchanged(1));
  EXPECT_EQ(countOf(r.core(0), "exchange_start"), 0);
  // The first measured tick starts it.
  r.b[0].in.seq_valid = true;
  r.run(1);
  EXPECT_TRUE(r.core(0).exchanged(1));
  EXPECT_EQ(countOf(r.core(0), "exchange_done_trivial"), 1);
}

TEST(TeamCoreExchange, AlreadyCurrentAtContactIsTrivial) {
  // The maps went through at an earlier contact: nothing moves at this one.
  Rig r(2, Arm::kOff);
  r.b[0].in.sent_seq = 5;
  r.b[1].in.sent_seq = 8;
  r.b[0].in.rcvd_seq[1] = 8;
  r.b[1].in.rcvd_seq[0] = 5;
  r.run(3);
  EXPECT_TRUE(r.core(0).exchanged(1));
  EXPECT_EQ(countOf(r.core(0), "exchange_done_trivial"), 1);
  // Apart, robot 1 maps more; the next contact has something to send.
  r.allLinks(false);
  r.run(15);
  r.b[1].in.sent_seq = 9;
  r.allLinks(true);
  r.run(3);
  EXPECT_EQ(r.core(0).exchange(1).state, ExchangeRecord::State::kRunning);
  r.b[0].in.rcvd_seq[1] = 9;
  r.run(2);
  EXPECT_TRUE(r.core(0).exchanged(1));
  EXPECT_EQ(countOf(r.core(0), "exchange_done"), 2);
  EXPECT_EQ(countOf(r.core(0), "exchange_done_trivial"), 1);
}

TEST(TeamCoreExchange, StallGivesUpAndContactLossEndsTheRecord) {
  Rig r(2, Arm::kOff);
  r.b[0].in.sent_seq = 5;
  r.b[1].in.sent_seq = 8;
  r.run(3);
  // One frame of progress, then nothing.
  r.b[0].in.rcvd_seq[1] = 3;
  r.run(1);
  const double progress_at = r.core(0).exchange(1).last_progress;
  ASSERT_TRUE(r.until([&] { return r.core(0).exchangeGaveUp(1); }, 200));
  EXPECT_NEAR(r.now - 1.0 - progress_at, 120.0, 1.0);
  const TeamEvent* e = lastEvent(r.b[0], "exchange", "gave_up_stall");
  ASSERT_NE(e, nullptr);
  // A continuing contact does not restart a finished record.
  r.run(30);
  EXPECT_EQ(countEvents(r.b[0], "exchange", "start"), 1);
  // Lose contact and regain it: a new record with new targets.
  r.allLinks(false);
  r.run(15);
  EXPECT_FALSE(r.core(0).exchangeGaveUp(1));
  r.allLinks(true);
  r.run(3);
  EXPECT_EQ(countEvents(r.b[0], "exchange", "start"), 2);
  EXPECT_EQ(r.core(0).exchange(1).state, ExchangeRecord::State::kRunning);
}

TEST(TeamCoreExchange, TotalBoundGivesUpDespiteProgress) {
  Rig r(2, Arm::kOff);
  r.b[1].in.sent_seq = 100000;
  r.run(3);
  ASSERT_EQ(r.core(0).exchange(1).state, ExchangeRecord::State::kRunning);
  for (int t = 0; t < 700 && !r.core(0).exchangeGaveUp(1); ++t) {
    r.b[0].in.rcvd_seq[1] += 1;
    r.step();
  }
  EXPECT_TRUE(r.core(0).exchangeGaveUp(1));
  EXPECT_NE(lastEvent(r.b[0], "exchange", "gave_up_total"), nullptr);
}

TEST(TeamCoreExchange, ContactLostMidwayIsLogged) {
  Rig r(2, Arm::kOff);
  r.b[1].in.sent_seq = 10;
  r.run(3);
  r.allLinks(false);
  r.run(15);
  EXPECT_EQ(countEvents(r.b[0], "exchange", "contact_lost"), 1);
  EXPECT_FALSE(r.core(0).exchange(1).active);
}

// ── team_finished ───────────────────────────────────────────────────────────

TEST(TeamCoreFinished, AllHeardFinishedLatchesIt) {
  Rig r(3, Arm::kOff);
  for (auto& b : r.b) b.in.finished = true;
  r.b[2].in.finished = false;
  r.run(5);
  EXPECT_FALSE(r.core(0).teamFinished());
  r.b[2].in.finished = true;
  r.run(2);
  EXPECT_TRUE(r.core(0).teamFinished());
  const TeamEvent* e = nullptr;
  for (const auto& ev : r.b[0].events)
    if (ev.kind == "team_finished") e = &ev;
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(findField(e->fields, "reason")->s, "all-heard");
}

TEST(TeamCoreFinished, SpreadsByBeaconAndStaysLatched) {
  Rig r(3, Arm::kOff);
  for (auto& b : r.b) b.in.finished = true;
  r.isolate(2);   // nobody hears 2, so nobody has all-heard
  r.run(4);
  EXPECT_FALSE(r.core(0).teamFinished());
  EXPECT_FALSE(r.core(1).teamFinished());
  // 1 learns it from 2 (one beacon); 0, who never hears 2, from 1's beacon.
  Beacon b;
  b.team_hash = 7;
  b.grid_hash = 9;
  b.robot_id = 2;
  b.stamp = r.now;
  b.finished = true;
  b.team_finished = true;
  ASSERT_TRUE(r.core(1).onBeacon(b, r.now));
  r.run(3);
  EXPECT_TRUE(r.core(0).teamFinished());
  EXPECT_FALSE(r.core(0).present(2));
  r.allLinks(false);
  r.run(20);
  EXPECT_TRUE(r.core(0).teamFinished());
}

// ── Plan ────────────────────────────────────────────────────────────────────

TEST(TeamCorePlan, RobotZeroProposesWhenTheTeamIsTogetherAndOthersAdopt) {
  Rig r(2, Arm::kRendezvous);
  r.b[0].oracle.solve.provisional = true;
  r.run(3);   // all connected at t=2
  ASSERT_EQ(r.core(0).plan().version, 1u);
  EXPECT_TRUE(r.core(0).plan().provisional);
  EXPECT_DOUBLE_EQ(r.core(0).plan().t0, 2.0 + 300.0);
  EXPECT_DOUBLE_EQ(r.core(0).plan().interval, 300.0);
  r.run(1);
  EXPECT_EQ(r.core(1).plan().version, 1u);
  EXPECT_EQ(countEvents(r.b[1], "plan", "adopted"), 1);
  // Provisional: re-solved every 5 s; still provisional means no new version.
  r.run(12);
  EXPECT_EQ(r.core(0).plan().version, 1u);
  EXPECT_GE(r.b[0].oracle.solves, 3);
  // A real solve firms it, keeping t0 and the interval.
  r.b[0].oracle.solve.provisional = false;
  r.b[0].oracle.solve.cell = 5;
  r.b[0].oracle.solve.center = {30, 30};
  r.run(6);
  EXPECT_EQ(r.core(0).plan().version, 2u);
  EXPECT_FALSE(r.core(0).plan().provisional);
  EXPECT_EQ(r.core(0).plan().cell, 5);
  EXPECT_DOUBLE_EQ(r.core(0).plan().t0, 302.0);
  EXPECT_EQ(r.core(0).previousPlan().version, 1u);
  EXPECT_EQ(countEvents(r.b[0], "plan", "firmed"), 1);
  // Firm: no more solving.
  const int solves = r.b[0].oracle.solves;
  r.run(20);
  EXPECT_EQ(r.b[0].oracle.solves, solves);
}

TEST(TeamCorePlan, FirmingKeepsTheRenewedSlot) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed &&
                                   r.core(1).booking().departed; }, 400));
  // The renewal at the meeting comes back provisional.
  r.b[0].oracle.solve.provisional = true;
  r.b[0].in.pose = r.core(0).booking().spot;
  r.b[1].in.pose = r.core(1).booking().spot;
  r.allLinks(true);
  ASSERT_TRUE(r.until([&] { return countEvents(r.b[0], "plan", "renewed") == 1; }, 30));
  ASSERT_TRUE(r.core(0).plan().provisional);
  ASSERT_EQ(r.core(0).plan().renewed_at_slot, 0);
  // Still together, a real solve firms it; slot 0 stays proved met.
  r.b[0].oracle.solve.provisional = false;
  ASSERT_TRUE(r.until([&] { return countEvents(r.b[0], "plan", "firmed") == 1; }, 10));
  EXPECT_FALSE(r.core(0).plan().provisional);
  EXPECT_EQ(r.core(0).plan().renewed_at_slot, 0);
  r.run(2);
  EXPECT_EQ(r.core(1).plan().version, r.core(0).plan().version);
  EXPECT_EQ(r.core(1).plan().renewed_at_slot, 0);
}

TEST(TeamCorePlan, NoPlanWithoutTheWholeTeam) {
  Rig r(3, Arm::kRendezvous);
  r.link(1, 2, false);
  r.run(20);
  EXPECT_EQ(r.core(0).plan().version, 0u);
  EXPECT_EQ(r.b[0].oracle.solves, 0);
}

TEST(TeamCorePlan, RefusalsAreThrottled) {
  Rig r(2, Arm::kRendezvous);
  r.b[0].oracle.solve.ok = false;
  r.b[0].oracle.solve.refused = "no-cells";
  r.run(3 + 5 * 13);
  EXPECT_EQ(r.core(0).plan().version, 0u);
  EXPECT_GE(countOf(r.core(0), "plan_solve_refused"), 13);
  EXPECT_EQ(countEvents(r.b[0], "plan", "refused"), 2);   // 1st and 13th
}

TEST(TeamCorePlan, ArmsWithoutBookingsNeverPlan) {
  for (Arm a : {Arm::kOff, Arm::kPursuit}) {
    Rig r(2, a);
    r.run(20);
    EXPECT_EQ(r.core(0).plan().version, 0u) << armName(a);
  }
}

TEST(TeamCorePlan, SplitRecoveryAlternatesOddSlotsToThePreviousCell) {
  FakeOracle o;
  TeamConfig c;
  c.arm = Arm::kRendezvous;
  c.n = 2;
  c.self_id = 1;
  c.team_hash = 7;
  c.grid_hash = 9;
  TeamCore core(c, &o);
  auto plan = [](uint32_t v, int cell, double cx) {
    Plan p;
    p.version = v;
    p.cell = cell;
    p.center = {cx, 0};
    p.t0 = 100;
    p.interval = 300;
    return p;
  };
  Beacon b;
  b.team_hash = 7;
  b.grid_hash = 9;
  b.robot_id = 0;
  b.stamp = 1;
  b.plan = plan(1, 3, 30);
  core.onBeacon(b, 1);
  TickInputs in;
  in.now = 1;
  core.tick(in);
  ASSERT_EQ(core.plan().version, 1u);
  b.stamp = 2;
  b.plan = plan(2, 8, 80);
  core.onBeacon(b, 2);
  in.now = 2;
  core.tick(in);
  ASSERT_EQ(core.plan().version, 2u);
  int cell;
  Vec2 ctr;
  // The only peer shows v2: no split.
  core.cellForSlot(3, &cell, &ctr);
  EXPECT_EQ(cell, 8);
  // Robot 0 shows v1 again (a stale peer view): odd slots use v1's cell.
  b.stamp = 3;
  b.plan = plan(1, 3, 30);
  core.onBeacon(b, 3);
  core.cellForSlot(3, &cell, &ctr);
  EXPECT_EQ(cell, 3);
  EXPECT_DOUBLE_EQ(ctr.x, 30.0);
  core.cellForSlot(4, &cell, &ctr);
  EXPECT_EQ(cell, 8);
}

// ── Booking ─────────────────────────────────────────────────────────────────

TEST(TeamCoreBooking, BooksTheFirstReachableSlotAndDepartsByLead) {
  Rig r = splitPair();
  r.run(15);   // presence expires, not all connected: book
  const Booking& B = r.core(0).booking();
  ASSERT_TRUE(B.held);
  EXPECT_EQ(B.slot, 0);
  EXPECT_DOUBLE_EQ(B.slot_time, 302.0);
  EXPECT_FALSE(B.departed);
  EXPECT_EQ(r.b[0].out.activity, Activity::kExplore);
  // lead = 14.14 m / 0.4 m/s * 1.2 = 42.4 s: departs at t >= 259.6.
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed; }, 400));
  EXPECT_NEAR(r.now - 1.0, 260.0, 1.0);
  EXPECT_EQ(r.b[0].out.activity, Activity::kMeet);
  EXPECT_EQ(r.b[0].out.drive.kind, DriveKind::kLeg);
  // Ring spot: robot 0 at angle 0, 3 m out.
  EXPECT_NEAR(r.b[0].out.drive.point.x, 13.0, 1e-9);
  EXPECT_NEAR(r.b[0].out.drive.point.y, 10.0, 1e-9);
  EXPECT_EQ(r.b[0].out.drive.purpose, "meet");
  // The beacon now names the meeting.
  EXPECT_EQ(r.core(0).beacon().meeting_slot, 0);
}

TEST(TeamCoreBooking, SkipsASlotThePeerCannotReach) {
  Rig r(2, Arm::kRendezvous);
  r.b[0].in.pose = {0, 0};
  r.b[1].in.pose = {10, 200};   // 190 m from the cell: lead 570 s
  r.run(4);
  r.allLinks(false);
  r.run(15);
  const Booking& B = r.core(0).booking();
  ASSERT_TRUE(B.held);
  EXPECT_EQ(B.slot, 1);   // 302 is too soon for robot 1; 602 fits
}

TEST(TeamCoreBooking, BlockedRingSpotStepsOutward) {
  Rig r = splitPair();
  r.b[0].oracle.blocked = {{13, 10}, {14, 10}};
  r.run(15);
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed; }, 400));
  EXPECT_NEAR(r.core(0).booking().spot.x, 15.0, 1e-9);
  EXPECT_NEAR(r.core(0).booking().spot_radius, 5.0, 1e-9);
}

TEST(TeamCoreBooking, FullMeetingRenewsThePlanAndReleasesEveryone) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed &&
                                   r.core(1).booking().departed; }, 400));
  // Both drive to their spots (the test moves them) and the link returns.
  r.b[0].in.pose = r.core(0).booking().spot;
  r.b[1].in.pose = r.core(1).booking().spot;
  r.b[0].in.sent_seq = 40;
  r.b[1].in.sent_seq = 60;
  r.allLinks(true);
  r.run(4);
  EXPECT_TRUE(r.core(0).booking().arrived);
  EXPECT_EQ(r.b[0].out.activity, Activity::kMeet);
  EXPECT_TRUE(r.core(0).proximityExempt(1));
  // Not met until the maps are through.
  EXPECT_EQ(r.core(0).booking().met_mask, 0u);
  r.b[0].in.rcvd_seq[1] = 60;
  r.b[1].in.rcvd_seq[0] = 40;
  ASSERT_TRUE(r.until([&] { return !r.core(0).booking().held &&
                                   !r.core(1).booking().held; }, 30));
  EXPECT_EQ(countEvents(r.b[0], "booking", "full_met"), 1);
  EXPECT_EQ(countEvents(r.b[0], "booking", "met"), 1);
  EXPECT_EQ(countEvents(r.b[1], "booking", "met"), 1);
  EXPECT_EQ(countEvents(r.b[0], "plan", "renewed"), 1);
  EXPECT_EQ(r.core(0).plan().version, 2u);
  EXPECT_EQ(r.core(0).plan().renewed_at_slot, 0);
  EXPECT_EQ(r.core(1).plan().version, 2u);
  EXPECT_EQ(r.core(0).missedStreak(), 0);
  // Released while still together: explore, no new booking.
  r.run(2);
  EXPECT_EQ(r.b[0].out.activity, Activity::kExplore);
  EXPECT_FALSE(r.core(0).booking().held);
  EXPECT_FALSE(r.core(0).proximityExempt(1));
}

TEST(TeamCoreBooking, BookingWaitsForASustainedSeparation) {
  Rig r(2, Arm::kRendezvous, [](TeamConfig& c) { c.book_apart_sec = 15.0; });
  r.b[0].in.pose = {0, 0};
  r.b[1].in.pose = {20, 0};
  r.run(4);
  ASSERT_TRUE(r.core(0).plan().valid());
  // Apart for 10 s, then back in contact: no booking.
  r.allLinks(false);
  ASSERT_TRUE(r.until([&] { return !r.core(0).allConnected(); }, 30));
  r.run(10);
  EXPECT_FALSE(r.core(0).booking().held);
  r.allLinks(true);
  ASSERT_TRUE(r.until([&] { return r.core(0).allConnected(); }, 10));
  // Apart again: the clock restarts and the booking comes 15 s later.
  r.allLinks(false);
  ASSERT_TRUE(r.until([&] { return !r.core(0).allConnected(); }, 30));
  const double apart = r.now - 1.0;
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().held; }, 60));
  EXPECT_NEAR(r.now - 1.0 - apart, 15.0, 1.0);
  EXPECT_EQ(countEvents(r.b[0], "booking", "booked"), 1);
}

TEST(TeamCoreBooking, DeferredDepartureOutlastsAShortLapse) {
  Rig r(3, Arm::kRendezvous, [](TeamConfig& c) { c.book_apart_sec = 15.0; });
  r.b[0].in.pose = {0, 0};
  r.run(4);
  ASSERT_TRUE(r.core(0).plan().valid());
  r.allLinks(false);
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().held; }, 60));
  // Together again, with robot 1's maps still coming in when robot 0's
  // departure falls due (302 - 42.4 s): the encounter is the meeting.
  r.b[1].in.sent_seq = 100000;
  r.allLinks(true);
  auto step = [&] {
    r.b[0].in.rcvd_seq[1] += 1;
    r.step();
  };
  while (r.now < 262) step();
  ASSERT_TRUE(r.core(0).booking().depart_deferred);
  ASSERT_FALSE(r.core(0).booking().departed);
  // Robots 1 and 2 lose each other for a while: the team is not all connected
  // for under book_apart_sec, and the 0-1 exchange runs on. Still deferred.
  r.link(1, 2, false);
  int lapse = 0;
  for (int t = 0; t < 20; ++t) {
    step();
    if (!r.core(0).allConnected()) ++lapse;
  }
  r.link(1, 2, true);
  for (int t = 0; t < 5; ++t) step();
  ASSERT_GE(lapse, 5);
  ASSERT_LT(lapse, 15);
  ASSERT_TRUE(r.core(0).allConnected());
  EXPECT_FALSE(r.core(0).booking().departed);
  EXPECT_EQ(countOf(r.core(0), "booking_depart_deferred"), 1);
  // A separation that lasts book_apart_sec ends the deferral.
  r.link(1, 2, false);
  ASSERT_TRUE(r.until([&] { return !r.core(0).allConnected(); }, 20));
  const double apart = r.now - 1.0;
  for (int t = 0; t < 30 && !r.core(0).booking().departed; ++t) step();
  EXPECT_TRUE(r.core(0).booking().departed);
  EXPECT_NEAR(r.now - 1.0 - apart, 15.0, 1.0);
}

TEST(TeamCoreBooking, AMetSlotIsNotBookedAgainWhileItIsStillMakeable) {
  Rig r = splitPair();
  r.b[0].in.finished = true;
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed; }, 60));
  // An early full meeting at the cell, some 280 s before slot 0.
  r.b[0].in.pose = r.core(0).booking().spot;
  r.b[1].in.pose = {7, 10};   // 3 m from the centre, like a ring spot
  r.allLinks(true);
  ASSERT_TRUE(r.until([&] { return countEvents(r.b[0], "booking", "met") == 1 &&
                                   !r.core(1).booking().held &&
                                   r.core(1).plan().renewed_at_slot == 0; }, 30));
  EXPECT_EQ(r.b[0].out.activity, Activity::kWait);
  // Apart again. Slot 0 was met and every robot can still make it, so nothing
  // is booked: robot 0 waits and robot 1 explores.
  r.allLinks(false);
  r.run(200);
  EXPECT_FALSE(r.core(0).booking().held);
  EXPECT_FALSE(r.core(1).booking().held);
  EXPECT_EQ(r.b[0].out.activity, Activity::kWait);
  EXPECT_EQ(r.b[1].out.activity, Activity::kExplore);
  // Once slot 0 is too close for a robot's lead (3 m: 9 s), slot 1 is booked.
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().held; }, 200));
  EXPECT_NEAR(r.now - 1.0, 294.0, 1.0);
  EXPECT_EQ(r.core(0).booking().slot, 1);
  EXPECT_TRUE(r.core(0).booking().departed);
  ASSERT_TRUE(r.until([&] { return r.core(1).booking().held; }, 5));
  EXPECT_EQ(r.core(1).booking().slot, 1);
  EXPECT_EQ(countEvents(r.b[0], "booking", "met"), 1);
}

TEST(TeamCoreBooking, ASlotIsMetOnlyWithinItsOwnInterval) {
  Rig r(2, Arm::kRendezvous);
  r.b[0].in.pose = {0, 0};
  r.b[1].in.pose = {10, 200};   // lead 570 s: slot 0 is out of reach
  r.run(4);
  r.b[0].in.finished = true;
  r.allLinks(false);
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed; }, 30));
  EXPECT_EQ(r.core(0).booking().slot, 1);   // 602; its interval opens at 302
  // Robot 1 turns up at the cell long before that.
  r.b[0].in.pose = r.core(0).booking().spot;
  r.b[1].in.pose = {7, 10};
  r.allLinks(true);
  r.run(60);
  EXPECT_TRUE(r.core(0).allConnected());
  EXPECT_TRUE(r.core(0).exchanged(1));
  EXPECT_EQ(r.core(0).booking().met_mask, 0u);   // maps swapped, slot 1 not spent
  EXPECT_EQ(r.b[0].out.activity, Activity::kMeet);
  ASSERT_TRUE(r.until([&] { return countEvents(r.b[0], "booking", "met") == 1; }, 400));
  EXPECT_NEAR(r.now - 1.0, 302.0, 3.0);
  EXPECT_EQ(r.core(0).plan().renewed_at_slot, 1);
}

TEST(TeamCoreBooking, AbsentPeerIsMissedAtTheWindow) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed; }, 400));
  r.b[0].in.pose = r.core(0).booking().spot;
  ASSERT_TRUE(r.until([&] { return countEvents(r.b[0], "booking", "missed") == 1; }, 400));
  // Window: slot 302 + 120.
  EXPECT_NEAR(r.now - 1.0, 422.0, 1.0);
  EXPECT_EQ(r.core(0).missedStreak(), 1);
  // Books the next slot in the same tick.
  EXPECT_TRUE(r.core(0).booking().held);
  EXPECT_EQ(r.core(0).booking().slot, 1);
  EXPECT_FALSE(r.core(0).booking().departed);
  EXPECT_EQ(r.b[0].out.activity, Activity::kExplore);
}

TEST(TeamCoreBooking, FinishedRobotGoesSlotToSlotWithoutABrake) {
  Rig r = splitPair();
  r.b[0].in.finished = true;
  r.run(15);
  ASSERT_TRUE(r.core(0).booking().departed);
  r.b[0].in.pose = r.core(0).booking().spot;
  ASSERT_TRUE(r.until([&] { return countEvents(r.b[0], "booking", "missed") == 1; }, 800));
  // Missed slot 0, booked and departed for slot 1 in the same tick: still Meet.
  EXPECT_EQ(r.b[0].out.activity, Activity::kMeet);
  EXPECT_EQ(r.core(0).booking().slot, 1);
  EXPECT_TRUE(r.core(0).booking().departed);
  EXPECT_EQ(countEvents(r.b[0], "team_activity"), 3);   // explore, wait, meet
}

TEST(TeamCoreBooking, FinishedRobotGoesHomeAfterTwoMissedSlots) {
  Rig r = splitPair();
  r.b[0].in.finished = true;
  // Finished: departs as soon as it books, and waits at the cell.
  r.run(15);
  ASSERT_TRUE(r.core(0).booking().departed);
  r.b[0].in.pose = r.core(0).booking().spot;
  ASSERT_TRUE(r.until([&] { return r.core(0).missedStreak() == 2; }, 800));
  EXPECT_NEAR(r.now - 1.0, 722.0, 1.0);
  r.run(1);
  EXPECT_FALSE(r.core(0).booking().held);
  EXPECT_EQ(r.b[0].out.activity, Activity::kHome);
  const TeamEvent* h = lastEvent(r.b[0], "homing", "start");
  ASSERT_NE(h, nullptr);
  EXPECT_EQ(findField(h->fields, "reason")->s, "missed-streak");
  r.b[0].in.pose = r.b[0].in.home;
  r.run(2);
  EXPECT_EQ(r.b[0].out.activity, Activity::kDone);
  EXPECT_EQ(r.b[0].out.drive.kind, DriveKind::kHold);
  EXPECT_TRUE(r.core(0).beacon().home);
  // Stays done: no more bookings.
  r.run(700);
  EXPECT_EQ(r.b[0].out.activity, Activity::kDone);
  EXPECT_EQ(countEvents(r.b[0], "booking", "booked"), 2);
}

TEST(TeamCoreBooking, EncounterBeforeDepartureCancelsTheBooking) {
  Rig r = splitPair();
  r.run(15);
  ASSERT_TRUE(r.core(0).booking().held);
  r.allLinks(true);
  r.run(4);
  EXPECT_FALSE(r.core(0).booking().held);
  EXPECT_EQ(countEvents(r.b[0], "booking", "encounter"), 1);
  // Together: no new booking.
  r.run(10);
  EXPECT_FALSE(r.core(0).booking().held);
  // Apart again: books.
  r.allLinks(false);
  r.run(15);
  EXPECT_TRUE(r.core(0).booking().held);
}

TEST(TeamCoreBooking, EncounterWaitsForEveryPairsExchange) {
  Rig r(3, Arm::kRendezvous);
  r.run(4);
  r.allLinks(false);
  r.run(15);
  ASSERT_TRUE(r.core(0).booking().held);
  r.b[1].in.sent_seq = 10;   // 2 must receive 1's maps
  r.allLinks(true);
  r.run(4);
  EXPECT_TRUE(r.core(0).allConnected());
  EXPECT_TRUE(r.core(0).booking().held);   // pair (1,2) not exchanged yet
  r.b[2].in.rcvd_seq[1] = 10;
  r.b[0].in.rcvd_seq[1] = 10;
  r.run(4);
  EXPECT_FALSE(r.core(0).booking().held);
}

TEST(TeamCoreBooking, StalledExchangeClosesAtTheWindowWithoutMet) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed &&
                                   r.core(1).booking().departed; }, 400));
  r.b[0].in.pose = r.core(0).booking().spot;
  // Robot 1 is at the cell too but its maps never arrive.
  r.b[1].in.pose = r.core(1).booking().spot;
  r.b[1].in.sent_seq = 1000;
  r.allLinks(true);
  ASSERT_TRUE(r.until([&] { return !r.core(0).booking().held; }, 1000));
  EXPECT_NEAR(r.now - 1.0, 422.0, 1.0);
  EXPECT_EQ(countEvents(r.b[0], "exchange", "gave_up_stall"), 1);
  EXPECT_EQ(countEvents(r.b[0], "booking", "met"), 0);
  EXPECT_EQ(countEvents(r.b[0], "booking", "missed"), 1);
  EXPECT_EQ(r.core(0).missedStreak(), 0);   // robot 1 was heard
}

TEST(TeamCoreBooking, ProgressingExchangeHoldsTheWindowOpen) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed &&
                                   r.core(1).booking().departed; }, 400));
  r.b[0].in.pose = r.core(0).booking().spot;
  r.b[1].in.pose = r.core(1).booking().spot;
  r.b[1].in.sent_seq = 100000;
  r.allLinks(true);
  // Maps keep arriving: the window stays open past 422 until the exchange's
  // own total bound (contact at 263 + 600) ends it.
  for (int t = 0; t < 1000 && r.core(0).booking().held; ++t) {
    r.b[0].in.rcvd_seq[1] += 1;
    r.step();
  }
  EXPECT_NEAR(r.now - 1.0, 863.0, 2.0);
  EXPECT_EQ(countEvents(r.b[0], "exchange", "gave_up_total"), 1);
}

TEST(TeamCoreBooking, NeverArrivingIsMissedAtTheWindow) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed; }, 400));
  ASSERT_TRUE(r.until([&] { return countEvents(r.b[0], "booking", "missed") == 1; }, 1000));
  EXPECT_NEAR(r.now - 1.0, 422.0, 1.0);
}

TEST(TeamCoreBooking, BackstopEndsALateExchangeThatHoldsTheWindow) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed; }, 400));
  r.b[0].in.pose = r.core(0).booking().spot;
  // Contact at t~362, 60 s past the slot, with maps flowing: the running
  // exchange holds the window (422) open, and its own bound (362 + 600) is past
  // the backstop (302 + 600).
  r.b[1].in.sent_seq = 100000;
  while (r.now < 360) r.step();
  r.allLinks(true);
  for (int t = 0; t < 1000 && r.core(0).booking().held; ++t) {
    r.b[0].in.rcvd_seq[1] += 1;
    r.step();
  }
  EXPECT_NEAR(r.now - 1.0, 902.0, 1.0);
  EXPECT_EQ(countEvents(r.b[0], "booking", "backstop"), 1);
}

TEST(TeamCoreBooking, BackstopCapsTheWaitForAnUnseenRenewal) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed; }, 400));
  r.b[0].in.pose = r.core(0).booking().spot;
  // As above, but the rest of the maps arrive at t=870, 32 s before the
  // backstop (302 + 600), and robot 1 stops hearing robot 0 in that tick: it
  // never shows the renewal, and the patience (870 + 120) outlasts the backstop.
  r.b[1].in.sent_seq = 100000;
  while (r.now < 360) r.step();
  r.allLinks(true);
  while (r.now < 870) {
    r.b[0].in.rcvd_seq[1] += 1;
    r.step();
  }
  r.b[0].in.rcvd_seq[1] = 100000;
  r.up[0][1] = false;
  r.step();
  ASSERT_TRUE(r.core(0).booking().full_met);
  ASSERT_EQ(r.core(0).plan().version, 2u);
  ASSERT_TRUE(r.until([&] { return countEvents(r.b[0], "booking", "met") == 1; }, 200));
  EXPECT_NEAR(r.now - 1.0, 902.0, 1.0);
  EXPECT_EQ(countOf(r.core(0), "booking_renewal_unseen"), 1);
  EXPECT_EQ(countEvents(r.b[0], "booking", "backstop"), 0);
}

TEST(TeamCoreBooking, LostPairAtTheCellHigherIdMovesLowerHolds) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed &&
                                   r.core(1).booking().departed; }, 400));
  r.b[0].in.pose = r.core(0).booking().spot;
  r.b[1].in.pose = r.core(1).booking().spot;
  r.b[1].in.sent_seq = 1000;   // keeps the exchange running
  r.allLinks(true);
  r.run(5);
  ASSERT_TRUE(r.core(1).inContact(0));
  // Robot 0 drifts to (5,5), then they lose each other (a wall between).
  r.b[0].in.pose = {5, 5};
  r.run(2);
  r.allLinks(false);
  r.run(12);
  const Booking& B1 = r.core(1).booking();
  EXPECT_TRUE(B1.rc_active);
  EXPECT_EQ(B1.rc_peer, 0);
  EXPECT_EQ(r.b[1].out.drive.purpose, "reconnect");
  EXPECT_NEAR(r.b[1].out.drive.point.x, 5.0, 1e-9);
  EXPECT_FALSE(r.core(0).booking().rc_active);   // the lower id holds
  EXPECT_EQ(r.b[0].out.drive.purpose, "meet");
  // Robot 1 regains contact on the way: holds there.
  r.allLinks(true);
  r.run(4);
  EXPECT_TRUE(r.core(1).booking().rc_holding);
  EXPECT_EQ(r.b[1].out.drive.kind, DriveKind::kHold);
  EXPECT_EQ(countOf(r.core(1), "reconnect_moves"), 1);
}

TEST(TeamCoreBooking, ReconnectMoveIsOncePerLostContact) {
  Rig r = splitPair();
  ASSERT_TRUE(r.until([&] { return r.core(1).booking().departed; }, 400));
  r.b[0].in.pose = r.core(0).booking().spot;
  r.b[1].in.pose = r.core(1).booking().spot;
  r.b[1].in.sent_seq = 1000;
  r.allLinks(true);
  r.run(5);
  r.allLinks(false);
  r.run(12);
  ASSERT_TRUE(r.core(1).booking().rc_active);
  // Reaches the point without contact: back to the spot, no second move.
  r.b[1].in.pose = r.core(1).booking().rc_point;
  r.run(2);
  EXPECT_FALSE(r.core(1).booking().rc_active);
  EXPECT_EQ(r.b[1].out.drive.purpose, "meet");
  r.run(20);
  EXPECT_EQ(countOf(r.core(1), "reconnect_moves"), 1);
  EXPECT_EQ(countOf(r.core(1), "reconnect_no_contact"), 1);
}

TEST(TeamCoreBooking, TeamFinishedCancelsAPendingBooking) {
  Rig r = splitPair();
  r.run(15);
  ASSERT_TRUE(r.core(0).booking().held);
  ASSERT_FALSE(r.core(0).booking().departed);
  Beacon b = r.core(1).beacon();
  b.stamp = r.now;
  b.team_finished = true;
  r.core(0).onBeacon(b, r.now);
  r.run(1);
  EXPECT_FALSE(r.core(0).booking().held);
  EXPECT_EQ(countEvents(r.b[0], "booking", "team-finished"), 1);
}

TEST(TeamCoreBooking, TeamFinishedEndsADepartedBookingToo) {
  Rig r = splitPair();
  r.b[0].in.finished = true;
  r.run(15);
  ASSERT_TRUE(r.core(0).booking().departed);
  ASSERT_EQ(r.b[0].out.activity, Activity::kMeet);
  Beacon b = r.core(1).beacon();
  b.stamp = r.now;
  b.team_finished = true;
  r.core(0).onBeacon(b, r.now);
  r.run(1);
  EXPECT_FALSE(r.core(0).booking().held);
  EXPECT_EQ(countEvents(r.b[0], "booking", "team-finished"), 1);
  EXPECT_EQ(r.b[0].out.activity, Activity::kHome);
  EXPECT_EQ(findField(lastEvent(r.b[0], "homing", "start")->fields, "reason")->s,
            "team-finished");
  r.run(30);
  EXPECT_FALSE(r.core(0).booking().held);
  EXPECT_EQ(countEvents(r.b[0], "booking", "booked"), 1);
}

// ── Chase ───────────────────────────────────────────────────────────────────

namespace {
Rig chasePair(Arm arm = Arm::kPursuit) {
  Rig r(2, arm);
  r.b[0].in.pose = {0, 0};
  r.b[1].in.pose = {20, 0};
  r.run(4);
  r.allLinks(false);
  return r;
}
}  // namespace

TEST(TeamCoreChase, SilenceTriggersTheGateAndIntercept) {
  Rig r = chasePair();
  r.b[0].oracle.gate.dispatch = true;
  r.b[0].oracle.gate.peer = 1;
  r.b[0].oracle.has_intercept = true;
  r.b[0].oracle.intercept_at = {40, 40};
  r.run(85);
  EXPECT_FALSE(r.core(0).chase().held);   // silence 90 s not reached
  ASSERT_TRUE(r.until([&] { return r.core(0).chase().held; }, 20));
  const Chase& C = r.core(0).chase();
  EXPECT_EQ(C.peer, 1);
  ASSERT_EQ(C.points.size(), 2u);   // intercept, last-position (no goal)
  EXPECT_EQ(C.points[0].kind, "intercept");
  EXPECT_EQ(C.points[1].kind, "last-position");
  EXPECT_EQ(r.b[0].out.activity, Activity::kChase);
  EXPECT_EQ(r.b[0].out.drive.purpose, "chase");
  EXPECT_NEAR(r.b[0].out.drive.point.x, 40.0, 1e-9);
  // Point budget: 56.6 m / 0.4 * 2 + 30.
  EXPECT_NEAR(C.point_deadline - C.start, 56.5685 / 0.4 * 2.0 + 30.0, 0.01);
}

TEST(TeamCoreChase, DeclinedGateRetriesOnItsPeriod) {
  Rig r = chasePair();
  r.b[0].oracle.gate.dispatch = false;
  r.run(120);
  EXPECT_FALSE(r.core(0).chase().held);
  EXPECT_GE(r.b[0].oracle.gate_calls, 2);
  EXPECT_LE(r.b[0].oracle.gate_calls, 4);
  EXPECT_GE(countEvents(r.b[0], "chase", "declined"), 2);
}

TEST(TeamCoreChase, RefusedGateFailsOpen) {
  Rig r = chasePair();
  r.b[0].oracle.gate.refused = "no-cells";
  ASSERT_TRUE(r.until([&] { return r.core(0).chase().held; }, 120));
  EXPECT_EQ(countOf(r.core(0), "chase_gate_failopen"), 1);
  EXPECT_EQ(r.core(0).chase().points[0].kind, "last-position");
}

TEST(TeamCoreChase, NeverBeforeTheTeamWasTogether) {
  Rig r(2, Arm::kPursuit);
  r.allLinks(false);
  r.up[1][0] = true;   // 0 hears 1 for a while, never in contact
  r.b[0].oracle.gate.dispatch = true;
  r.run(4);
  r.allLinks(false);
  r.run(300);
  EXPECT_TRUE(r.core(0).peer(1).ever);
  EXPECT_EQ(r.b[0].oracle.gate_calls, 0);
}

TEST(TeamCoreChase, FinishedPeerAndOldContactAreNotChased) {
  Rig r = chasePair();
  r.b[0].oracle.gate.dispatch = true;
  Beacon b = r.core(1).beacon();
  b.stamp = r.now;
  b.finished = true;
  r.core(0).onBeacon(b, r.now);
  r.run(200);
  EXPECT_EQ(r.b[0].oracle.gate_calls, 0);
  // An unfinished peer silent past chase_max_contact_age_sec is not chased.
  Rig r2 = chasePair();
  r2.b[0].oracle.gate.dispatch = false;
  r2.run(1000);
  const int calls = r2.b[0].oracle.gate_calls;
  r2.b[0].oracle.gate.dispatch = true;
  r2.run(100);
  EXPECT_EQ(r2.b[0].oracle.gate_calls, calls);
}

TEST(TeamCoreChase, OneWayHearingDoesNotRenewTheContactAge) {
  // Robot 0 goes on hearing robot 1 long after their last contact (t~14), and
  // loses it at t=880. At silence 90 s the contact is over 900 s old.
  Rig r = chasePair();
  r.b[0].oracle.gate.dispatch = true;
  r.up[1][0] = true;
  while (r.now < 880) r.step();
  ASSERT_TRUE(r.core(0).present(1));
  ASSERT_FALSE(r.core(0).inContact(1));
  r.allLinks(false);
  r.run(250);
  EXPECT_EQ(r.b[0].oracle.gate_calls, 0);
  EXPECT_FALSE(r.core(0).chase().held);
}

TEST(TeamCoreChase, PointsAdvanceOnReachAndOnExpiry) {
  Rig r = chasePair();
  r.b[0].oracle.gate.dispatch = true;
  r.b[0].oracle.has_intercept = true;
  r.b[0].oracle.intercept_at = {40, 40};
  ASSERT_TRUE(r.until([&] { return r.core(0).chase().held; }, 120));
  r.b[0].in.pose = {40, 40};
  r.run(2);
  EXPECT_EQ(r.core(0).chase().idx, 1u);
  EXPECT_EQ(countEvents(r.b[0], "chase", "point_reached"), 1);
  // Never reaches the last position: its deadline expires, the chase fails.
  ASSERT_TRUE(r.until([&] { return !r.core(0).chase().held; }, 400));
  EXPECT_EQ(countEvents(r.b[0], "chase", "point_expired"), 1);
  EXPECT_EQ(countEvents(r.b[0], "chase", "failed"), 1);
  // Cooldown before the next.
  const long long starts = countOf(r.core(0), "chase_start");
  r.run(85);
  EXPECT_EQ(countOf(r.core(0), "chase_start"), starts);
}

TEST(TeamCoreChase, ContactFollowsUntilExchangedThenDone) {
  Rig r = chasePair();
  r.b[0].oracle.gate.dispatch = true;
  ASSERT_TRUE(r.until([&] { return r.core(0).chase().held; }, 120));
  r.b[1].in.sent_seq = 30;
  r.b[1].in.pose = {30, 0};
  r.allLinks(true);
  r.run(3);
  EXPECT_TRUE(r.core(0).chase().following);
  EXPECT_EQ(r.b[0].out.activity, Activity::kFollow);
  EXPECT_EQ(r.b[0].out.drive.purpose, "follow");
  // Follow point: 10 m short of the peer on my side.
  EXPECT_NEAR(dist(r.b[0].out.drive.point, r.b[1].in.pose), 10.0, 1e-6);
  const uint64_t key = r.b[0].out.drive.key;
  r.b[1].in.pose = {35, 0};
  r.run(2);
  EXPECT_EQ(r.b[0].out.drive.key, key);   // same leg, moved point
  r.b[0].in.rcvd_seq[1] = 30;
  r.run(2);
  EXPECT_FALSE(r.core(0).chase().held);
  EXPECT_EQ(countEvents(r.b[0], "chase", "done"), 1);
  EXPECT_EQ(r.b[0].out.activity, Activity::kExplore);
}

TEST(TeamCoreChase, LimitEndsIt) {
  Rig r = chasePair();
  r.b[0].oracle.gate.dispatch = true;
  ASSERT_TRUE(r.until([&] { return r.core(0).chase().held; }, 120));
  r.b[1].in.sent_seq = 1000000;
  r.allLinks(true);
  // Contact with steady progress so the exchange neither ends nor stalls.
  for (int t = 0; t < 700 && r.core(0).chase().held; ++t) {
    r.b[0].in.rcvd_seq[1] += 1;
    r.step();
  }
  EXPECT_EQ(countEvents(r.b[0], "chase", "limit") + countEvents(r.b[0], "chase", "failed"), 1);
}

TEST(TeamCoreChase, DepartedBookingPreemptsItInHybrid) {
  Rig r(2, Arm::kHybrid);
  r.b[0].in.pose = {0, 0};
  r.b[1].in.pose = {20, 0};
  r.run(4);
  r.allLinks(false);
  r.b[0].oracle.gate.dispatch = true;
  // A far intercept point: its time bound outlasts the wait for departure.
  r.b[0].oracle.has_intercept = true;
  r.b[0].oracle.intercept_at = {200, 200};
  // Booked slot 0 (302). The chase starts at silence 90 s (t~93), before the
  // departure margin (302 - 42 - 120 = t 140) closes the trigger, and runs
  // until the departure at t~260 pre-empts it.
  ASSERT_TRUE(r.until([&] { return r.core(0).chase().held; }, 200));
  ASSERT_TRUE(r.until([&] { return r.core(0).booking().departed; }, 400));
  EXPECT_FALSE(r.core(0).chase().held);
  EXPECT_EQ(r.b[0].out.activity, Activity::kMeet);
  EXPECT_EQ(countEvents(r.b[0], "chase", "pre-empted"), 1);
  // No chase starts while the departed booking holds.
  const long long starts = countOf(r.core(0), "chase_start");
  r.run(100);
  EXPECT_EQ(countOf(r.core(0), "chase_start"), starts);
}

TEST(TeamCoreChase, HybridHoldsOffInsideTheDepartureMargin) {
  Rig r(2, Arm::kHybrid);
  r.b[0].in.pose = {0, 0};
  r.b[1].in.pose = {20, 0};
  r.run(4);
  r.allLinks(false);
  r.b[0].oracle.gate.dispatch = false;
  r.run(200);   // to t~204: the margin window starts at 302 - 42 - 120 = 140
  const int calls = r.b[0].oracle.gate_calls;
  r.run(50);
  EXPECT_EQ(r.b[0].oracle.gate_calls, calls);
}

// ── Priority list, arms, homing ─────────────────────────────────────────────

TEST(TeamCoreMission, ArmOffExploresThenGoesHomeThenDone) {
  Rig r(2, Arm::kOff);
  r.run(20);
  EXPECT_EQ(r.b[0].out.activity, Activity::kExplore);
  EXPECT_FALSE(r.core(0).booking().held);
  EXPECT_FALSE(r.core(0).chase().held);
  r.b[0].in.finished = true;
  r.run(1);
  EXPECT_EQ(r.b[0].out.activity, Activity::kHome);
  EXPECT_EQ(r.b[0].out.drive.purpose, "home");
  EXPECT_EQ(findField(lastEvent(r.b[0], "homing", "start")->fields, "reason")->s, "arm");
  r.b[0].in.pose = r.b[0].in.home;
  r.run(2);
  EXPECT_EQ(r.b[0].out.activity, Activity::kDone);
}

TEST(TeamCoreMission, HomingGivesUpOnItsBound) {
  Rig r(1, Arm::kOff);
  r.b[0].in.finished = true;
  r.run(601);   // t=600 gives up; the activity shows it a tick later
  EXPECT_EQ(r.b[0].out.activity, Activity::kHome);
  r.run(1);
  EXPECT_EQ(r.b[0].out.activity, Activity::kDone);
  EXPECT_EQ(countEvents(r.b[0], "homing", "gave_up"), 1);
}

TEST(TeamCoreMission, FinishedWithAPlanWaitsWhileTheTeamIsTogether) {
  Rig r(2, Arm::kRendezvous);
  r.run(4);
  r.b[0].in.finished = true;
  r.run(5);
  EXPECT_EQ(r.b[0].out.activity, Activity::kWait);
  EXPECT_EQ(r.b[0].out.drive.kind, DriveKind::kHold);
  EXPECT_GE(r.b[0].out.wait_start, 0.0);
  // The peer leaves: book, depart at once (finished), meet.
  r.allLinks(false);
  r.run(15);
  EXPECT_EQ(r.b[0].out.activity, Activity::kMeet);
}

TEST(TeamCoreMission, NoPlanFinishedGoesHomeAndAPlanInterruptsIt) {
  Rig r(2, Arm::kRendezvous);
  r.allLinks(false);
  r.b[0].in.finished = true;
  r.run(3);
  EXPECT_EQ(r.b[0].out.activity, Activity::kHome);
  EXPECT_EQ(findField(lastEvent(r.b[0], "homing", "start")->fields, "reason")->s, "no-plan");
  // A plan arrives by beacon: book, depart (finished), Meet interrupts homing.
  Beacon b;
  b.team_hash = 7;
  b.grid_hash = 9;
  b.robot_id = 1;
  b.stamp = r.now;
  b.position = {20, 0};
  b.plan.version = 1;
  b.plan.cell = 3;
  b.plan.center = {10, 10};
  b.plan.t0 = r.now + 300;
  b.plan.interval = 300;
  r.core(0).onBeacon(b, r.now);
  r.run(2);
  EXPECT_EQ(r.b[0].out.activity, Activity::kMeet);
  EXPECT_EQ(countEvents(r.b[0], "homing", "interrupted"), 1);
  EXPECT_FALSE(r.core(0).homeDone());
}

TEST(TeamCoreMission, DoneIsTerminalWhenAPlanArrivesAfter) {
  Rig r(2, Arm::kRendezvous);
  r.allLinks(false);
  r.b[0].in.finished = true;
  r.run(3);
  ASSERT_EQ(r.b[0].out.activity, Activity::kHome);
  r.b[0].in.pose = r.b[0].in.home;
  r.run(2);
  ASSERT_EQ(r.b[0].out.activity, Activity::kDone);
  // A plan arrives once the robot is home: adopted and carried on, not booked.
  Beacon b;
  b.team_hash = 7;
  b.grid_hash = 9;
  b.robot_id = 1;
  b.stamp = r.now;
  b.position = {20, 0};
  b.plan.version = 1;
  b.plan.cell = 3;
  b.plan.center = {10, 10};
  b.plan.t0 = r.now + 300;
  b.plan.interval = 300;
  r.core(0).onBeacon(b, r.now);
  r.run(700);
  EXPECT_EQ(r.core(0).plan().version, 1u);
  EXPECT_EQ(r.b[0].out.activity, Activity::kDone);
  EXPECT_TRUE(r.core(0).homeDone());
  EXPECT_EQ(countEvents(r.b[0], "booking", "booked"), 0);
  EXPECT_EQ(countEvents(r.b[0], "homing", "interrupted"), 0);
  EXPECT_EQ(countEvents(r.b[0], "homing", "start"), 1);
}

TEST(TeamCoreMission, TeamFinishedSendsEveryoneHome) {
  Rig r(2, Arm::kRendezvous);
  r.run(4);
  for (auto& b : r.b) b.in.finished = true;
  r.run(2);
  EXPECT_TRUE(r.core(0).teamFinished());
  EXPECT_EQ(r.b[0].out.activity, Activity::kHome);
  EXPECT_EQ(findField(lastEvent(r.b[0], "homing", "start")->fields, "reason")->s,
            "team-finished");
}

TEST(TeamCoreMission, ActivityEventsCarryDwell) {
  Rig r(2, Arm::kOff);
  r.run(10);
  r.b[0].in.finished = true;
  r.run(1);
  auto it = std::find_if(r.b[0].events.rbegin(), r.b[0].events.rend(),
                         [](const TeamEvent& x) { return x.kind == "team_activity"; });
  ASSERT_NE(it, r.b[0].events.rend());
  const TeamEvent& e = *it;
  EXPECT_EQ(findField(e.fields, "from")->s, "explore");
  EXPECT_EQ(findField(e.fields, "to")->s, "home");
  EXPECT_NEAR(findField(e.fields, "dwell_sec")->d, 10.0, 1e-9);
}

TEST(TeamCoreMission, BeaconMirrorsTheState) {
  Rig r(2, Arm::kRendezvous);
  r.b[0].in.sent_seq = 12;
  r.b[0].in.rcvd_seq[1] = 4;
  r.run(4);
  const Beacon b = r.core(0).beacon();
  EXPECT_EQ(b.robot_id, 0);
  EXPECT_EQ(b.map_sent_seq, 12u);
  ASSERT_EQ(b.map_rcvd_seq.size(), 2u);
  EXPECT_EQ(b.map_rcvd_seq[0], 12u);   // own slot carries the sent seq
  EXPECT_EQ(b.map_rcvd_seq[1], 4u);
  EXPECT_EQ(b.hears_mask, 0x2u);
  EXPECT_EQ(b.plan.version, 1u);
  EXPECT_EQ(b.meeting_slot, -1);
  EXPECT_FALSE(b.has_goal);
}
