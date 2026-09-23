/// @file test_meeting_attendance.cpp
/// @brief Generation 33, §10 item 5: a finished robot still comes to the
///        meeting, exchanges maps, and only then says it is leaving; its
///        partner waits for it until it does. DESIGN_gen33.md §10.
///
/// Two halves of one statement, both run here:
///   * announcedMode — what a robot says on TeamWorld/mode. A finished robot
///     keeping its appointment stays below HOMING; DONE comes when it stops
///     keeping it, and is latched.
///   * finishedPeerStillComing — what the partner's barrier asks. A peer that
///     finished, has not said it is leaving, and cannot be heard is still on
///     its way.
///
/// finishedPeerStillComing is driven through a real TeamModel for the reason
/// test_exchange_drain.cpp gives: which peers count as heard is a TeamModel
/// question, and a fake model would answer it however the test wanted.
///
/// The node's wiring — the publisher's call, the veto in
/// manoeuvreReleaseEligible, the cap, the stamp and its clears — is not
/// reachable from here and is pinned by the scan in
/// test_gen21_latched_hold.cpp (Gen33MeetingAttendance).
///
/// MUTATION-VERIFIED 2026-09-23 against meeting_attendance.cpp, each failing
/// the test named for it, source restored byte-exact afterwards (sha256
/// re-checked). One sequence with test_gen21_latched_hold.cpp (M44-M53) and
/// test_exchange_drain.cpp (M54-M58):
///
///   M59  DONE latched on `finished` alone, keeping ignored
///        -> AnnouncedMode.AKeeperSaysDoneOnlyOnceItStopsKeepingTheAppointment
///   M60  DONE recomputed each tick instead of latched
///        -> AnnouncedMode.DoneStaysDoneEvenIfKeepingReadsTrueAgain
///           and ...TheLevelIsMonotoneUnderLatchedInputs
///   M61  the "it said it is leaving" skip deleted
///        -> FinishedPeerStillComing.APeerThatSaidDoneOrHomingIsNotComing
///           and ...ARelayedLevelIsBelieved
///   M62  `heard_one_way` dropped from the heard test
///        -> FinishedPeerStillComing.APeerThatCanBeHeardIsLeftToTheBarrier
///   M63  the closure dropped from the heard test
///        -> FinishedPeerStillComing.APeerInTheClosureIsNotSilent
///   M64  the `finished` filter deleted
///        -> FinishedPeerStillComing.AnUnfinishedPeerIsNotThisCase
///
/// The self skip is not in the ledger: TeamModel never marks its own entry
/// finished (the gossip loop skips self), so deleting it is equivalent.
/// Moved comments: doc/explo_planner_code_notes.md

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "explo_planner/meeting_attendance.hpp"
#include "explo_planner/team_model.hpp"

using explo_planner::announcedMode;
using explo_planner::finishedPeerStillComing;
using explo_planner::FleetIdentity;
using explo_planner::kModeDone;
using explo_planner::kModeExploring;
using explo_planner::kModeHoming;
using explo_planner::makeFleetIdentity;
using explo_planner::robotBit;
using explo_planner::TeamModel;

namespace {

constexpr int kAtlas  = 0;   // this robot, in every test
constexpr int kBestla = 1;
constexpr int kCerd   = 2;

/// Late enough that a peer heard only at t = 0 is past the 5 s TTL.
constexpr double kNowSec = 10.0;

TeamModel::Config cfg() {
  TeamModel::Config c;
  c.direct_ttl_sec     = 5.0;
  c.closure_enabled    = true;
  c.gossip_max_age_sec = 120.0;
  return c;
}

TeamModel model(const FleetIdentity& fleet) {
  TeamModel m;
  EXPECT_EQ(m.configure(fleet, cfg()), "") << "fixture failed to configure";
  return m;
}

FleetIdentity fleet2() { return makeFleetIdentity({"atlas", "bestla"}, "atlas"); }
FleetIdentity fleet3() {
  return makeFleetIdentity({"atlas", "bestla", "cerd"}, "atlas");
}

/// A message from `sender` naming `mask` as its direct contacts, plus itself.
TeamModel::Observation msg(int sender, uint32_t mask, bool finished,
                           uint8_t mode) {
  TeamModel::Observation o;
  o.sender_id     = sender;
  o.in_range_mask = mask | robotBit(sender);
  o.finished      = finished;
  o.mode          = mode;
  return o;
}

void hear(TeamModel& m, const TeamModel::Observation& o, double t) {
  ASSERT_EQ(m.observe(o, t), "") << "fixture message refused";
}

/// One publisher tick, with the caller's DONE latch carried across calls the
/// way the node's done_announced_ member carries it.
struct Publisher {
  bool done = false;
  uint8_t tick(bool finished, bool keeping, bool homing) {
    return announcedMode(finished, keeping, homing, done);
  }
};

}  // namespace

// ===========================================================================
// announcedMode — WHAT A ROBOT SAYS.
// ===========================================================================

/// The keeper's whole run, tick by tick. A robot that saturates with an
/// appointment standing must not say DONE before the drive to the meeting.
/// (notes: attendance-keeper-whole-run)
TEST(AnnouncedMode, AKeeperSaysDoneOnlyOnceItStopsKeepingTheAppointment) {
  Publisher pub;
  // Exploring.
  EXPECT_EQ(pub.tick(false, false, false), kModeExploring);
  // Map saturated; keepAppointmentOnFinish has started the drive.
  EXPECT_EQ(pub.tick(true, true, false), kModeExploring)
      << "a finished robot driving to its meeting said it was leaving";
  // Standing at the barrier, exchanging.
  EXPECT_EQ(pub.tick(true, true, false), kModeExploring);
  EXPECT_FALSE(pub.done) << "the latch closed while the robot was keeping it";
  // The barrier released; the manoeuvre ended into RETURN_HOME.
  EXPECT_EQ(pub.tick(true, false, true), kModeDone);
  EXPECT_TRUE(pub.done);
  // Home.
  EXPECT_EQ(pub.tick(true, false, true), kModeDone);
}

/// DONE IS LATCHED. The field is max-merged on every peer and never comes
/// back down, so a robot that said DONE and later read "keeping" again (a
/// re-armed leg on some path) must go on saying DONE: dropping to EXPLORING
/// would contradict a level the team already holds.
TEST(AnnouncedMode, DoneStaysDoneEvenIfKeepingReadsTrueAgain) {
  Publisher pub;
  EXPECT_EQ(pub.tick(true, false, false), kModeDone);
  EXPECT_EQ(pub.tick(true, true, false), kModeDone)
      << "the DONE latch reopened";
  EXPECT_EQ(pub.tick(true, true, true), kModeDone);
}

/// A ROBOT THAT FINISHES WITH NOTHING TO KEEP IS DONE AT ONCE — the old
/// behaviour, unchanged for every robot that is not a keeper.
TEST(AnnouncedMode, AFinishWithNoAppointmentIsDoneOnTheFirstTick) {
  Publisher pub;
  EXPECT_EQ(pub.tick(true, false, false), kModeDone);
}

/// HOMING WITHOUT FINISHING (budget return, mission return before
/// saturation) is HOMING, and becomes DONE only if the run is later finished.
TEST(AnnouncedMode, UnfinishedHomingIsHoming) {
  Publisher pub;
  EXPECT_EQ(pub.tick(false, false, true), kModeHoming);
  EXPECT_FALSE(pub.done);
  EXPECT_EQ(pub.tick(true, false, true), kModeDone);
}

/// THE LEVEL NEVER GOES DOWN, over every sequence of three ticks whose inputs
/// behave as the node's do: `finished` and `homing` are latches (never true
/// then false), `keeping` is free. Exhaustive rather than sampled, because
/// the case that breaks max-merge is one specific ordering.
TEST(AnnouncedMode, TheLevelIsMonotoneUnderLatchedInputs) {
  int sequences = 0;
  for (int a = 0; a < 8; ++a)
    for (int b = 0; b < 8; ++b)
      for (int c = 0; c < 8; ++c) {
        const int seq[3] = {a, b, c};
        bool ok = true;
        for (int i = 1; i < 3 && ok; ++i) {
          // bit 0 finished, bit 1 homing: a latch may only gain bits.
          if ((seq[i - 1] & 0b011) & ~(seq[i] & 0b011)) ok = false;
        }
        if (!ok) continue;
        ++sequences;
        Publisher pub;
        uint8_t prev = kModeExploring;
        for (int i = 0; i < 3; ++i) {
          const bool fin  = seq[i] & 0b001;
          const bool hom  = seq[i] & 0b010;
          const bool keep = seq[i] & 0b100;
          const uint8_t now = pub.tick(fin, keep, hom);
          EXPECT_GE(now, prev) << "sequence " << a << "," << b << "," << c
                               << " lowered the level at tick " << i;
          prev = now;
        }
      }
  EXPECT_GT(sequences, 100) << "the enumeration lost its cases";
}

// ===========================================================================
// finishedPeerStillComing — WHAT THE PARTNER ASKS.
// ===========================================================================

/// THE CASE, N = 2. bestla said "finished" once, at t = 0, still below HOMING
/// — it is driving to the meeting — and then the radio lost it. It is still
/// coming, and the partner must say so.
TEST(FinishedPeerStillComing, AFinishedPeerThatHasNotSaidItIsLeavingIsComing) {
  TeamModel m = model(fleet2());
  hear(m, msg(kBestla, robotBit(kAtlas), true, kModeExploring), 0.0);
  m.tick(kNowSec);
  ASSERT_TRUE(m.peer(kBestla).finished);
  ASSERT_FALSE(m.peer(kBestla).direct || m.peer(kBestla).heard_one_way ||
               m.inComms(kBestla));
  EXPECT_EQ(finishedPeerStillComing(m, kAtlas), kBestla);
}

/// IT SAID IT IS LEAVING: DONE, or HOMING. Either one ends the wait.
TEST(FinishedPeerStillComing, APeerThatSaidDoneOrHomingIsNotComing) {
  for (const uint8_t mode : {kModeHoming, kModeDone}) {
    TeamModel m = model(fleet2());
    hear(m, msg(kBestla, robotBit(kAtlas), true, mode), 0.0);
    m.tick(kNowSec);
    ASSERT_TRUE(m.peer(kBestla).finished);
    EXPECT_EQ(finishedPeerStillComing(m, kAtlas), -1)
        << "mode " << int(mode) << " was read as still coming";
  }
}

/// HEARD IS NOT THIS FUNCTION'S CASE — direct, or one way. Its mode is
/// current, and the barrier's existing radio terms (and the inbound veto)
/// decide it.
TEST(FinishedPeerStillComing, APeerThatCanBeHeardIsLeftToTheBarrier) {
  {
    TeamModel m = model(fleet2());
    for (double t = 0.0; t <= kNowSec; t += 1.0)
      hear(m, msg(kBestla, robotBit(kAtlas), true, kModeExploring), t);
    m.tick(kNowSec);
    ASSERT_TRUE(m.peer(kBestla).direct);
    EXPECT_EQ(finishedPeerStillComing(m, kAtlas), -1) << "direct";
  }
  {
    TeamModel m = model(fleet2());
    for (double t = 0.0; t <= kNowSec; t += 1.0)
      hear(m, msg(kBestla, 0u, true, kModeExploring), t);   // does not name us
    m.tick(kNowSec);
    ASSERT_FALSE(m.peer(kBestla).direct);
    ASSERT_TRUE(m.peer(kBestla).heard_one_way);
    EXPECT_EQ(finishedPeerStillComing(m, kAtlas), -1) << "one way";
  }
}

/// AN UNFINISHED ABSENT PEER IS NOT THIS FUNCTION'S CASE: the barrier already
/// waits for it, unbounded, because peerAccounted does not count it.
TEST(FinishedPeerStillComing, AnUnfinishedPeerIsNotThisCase) {
  TeamModel m = model(fleet2());
  hear(m, msg(kBestla, robotBit(kAtlas), false, kModeExploring), 0.0);
  m.tick(kNowSec);
  EXPECT_EQ(finishedPeerStillComing(m, kAtlas), -1);
}

/// N = 3, BRIDGED. cerd is finished and cannot hear atlas, but bestla hears
/// both and says so: cerd is in the comms closure, its mode arrives relayed,
/// and it is not the silent case.
TEST(FinishedPeerStillComing, APeerInTheClosureIsNotSilent) {
  TeamModel m = model(fleet3());
  for (double t = 0.0; t <= kNowSec; t += 1.0)
    hear(m, msg(kBestla, robotBit(kAtlas) | robotBit(kCerd), false,
                kModeExploring), t);
  hear(m, msg(kCerd, robotBit(kBestla), true, kModeExploring), 0.0);
  m.tick(kNowSec);
  ASSERT_TRUE(m.peer(kCerd).finished);
  ASSERT_TRUE(m.peer(kCerd).via_relay);
  EXPECT_EQ(finishedPeerStillComing(m, kAtlas), -1);
}

/// N = 3, RELAYED WORDS. cerd is silent to atlas and outside the closure. Its
/// "finished" and its level reach atlas only through bestla's gossip. A
/// relayed DONE ends the wait exactly as a first-hand one does; a relayed
/// "finished" with no relayed level is still coming.
TEST(FinishedPeerStillComing, ARelayedLevelIsBelieved) {
  const auto gossip = [](bool with_done) {
    TeamModel::Observation o =
        msg(kBestla, robotBit(kAtlas), false, kModeExploring);   // not cerd
    o.finished_gossip = {0, 0, 1};
    o.mode_gossip     = {0, 0, with_done ? kModeDone : kModeExploring};
    return o;
  };
  {
    TeamModel m = model(fleet3());
    for (double t = 0.0; t <= kNowSec; t += 1.0) hear(m, gossip(true), t);
    m.tick(kNowSec);
    ASSERT_TRUE(m.peer(kCerd).finished);
    ASSERT_FALSE(m.inComms(kCerd));
    EXPECT_EQ(finishedPeerStillComing(m, kAtlas), -1) << "relayed DONE";
  }
  {
    TeamModel m = model(fleet3());
    for (double t = 0.0; t <= kNowSec; t += 1.0) hear(m, gossip(false), t);
    m.tick(kNowSec);
    ASSERT_TRUE(m.peer(kCerd).finished);
    ASSERT_FALSE(m.inComms(kCerd));
    EXPECT_EQ(finishedPeerStillComing(m, kAtlas), kCerd)
        << "relayed finished, no level";
  }
}

/// A ROBOT DOES NOT WAIT FOR ITSELF, and an unconfigured model has nobody.
TEST(FinishedPeerStillComing, SelfAndUnconfiguredAreNeverComing) {
  TeamModel unconfigured;
  EXPECT_EQ(finishedPeerStillComing(unconfigured, kAtlas), -1);

  // From bestla's side: atlas is the silent finished peer, bestla is self.
  TeamModel m;
  ASSERT_EQ(m.configure(makeFleetIdentity({"atlas", "bestla"}, "bestla"), cfg()),
            "");
  hear(m, msg(kAtlas, robotBit(kBestla), true, kModeExploring), 0.0);
  m.tick(kNowSec);
  EXPECT_EQ(finishedPeerStillComing(m, kBestla), kAtlas);
}
