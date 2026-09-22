/// @file test_exchange_drain.cpp
/// @brief Generation 33's drain release, run rather than scanned: test-plan 7
///        (exchange presence) and the behavioural half of test-plan 8 (a
///        blackout is not a drain). DESIGN_gen33.md §7.
///
/// Every test here is one robot's side of one meeting, stepped once a second
/// the way doReturnSync steps it: the peers' messages land on a real TeamModel,
/// the model ticks, and stepDrainRelease is handed the counter vector for that
/// second. Nothing is stubbed between the belief model and the predicate,
/// because the defects this file is for live exactly at that seam — which
/// peers the loop decides to read is a TeamModel question, and a fake model
/// would answer it however the test wanted.
///
/// The node's wiring of the result — which step returns, which step logs
/// UNFINISHED EXCHANGE and which logs the release — is not reachable from
/// here, and is pinned by the scan in test_gen21_latched_hold.cpp
/// (Gen33DrainRelease, M38-M40).
///
/// ONE CLOCK SHAPE FOR THE WHOLE FILE, chosen so the boundaries are exact
/// rather than approximately right: R = 1 vox/s, W = 10 s, cap = 60 s, steps
/// of 1 s. Full windows are judged at 10, 20, ..., 60, so the cap falls on an
/// evaluation instant and an off-by-one in the cap comparison moves the
/// outcome by a whole window instead of by nothing.
///
/// MUTATION-VERIFIED 2026-09-22 against exchange_drain.cpp, each failing the
/// test named for it, source restored byte-exact afterwards (sha256
/// re-checked). One sequence with test_gen21_latched_hold.cpp; M27-M31 were
/// first run there against the node's inline copy and are re-run here against
/// the extraction:
///
///   M27  presence filter narrowed back to `!p.direct` alone
///        -> ExchangePresence.ARelayedPeerIsWaitedFor
///   M28  `if (r.examined == 0) drained = false;` deleted
///        -> ExchangePresence.NobodyReadIsNotEverybodyDrained
///   M29  clause 1 deleted, leaving the rate test alone
///        -> BlackoutIsNotADrain.ACounterThatNeverMovedRunsToTheCapUnfinished
///   M30  clause 2 deleted, leaving the level test alone
///        -> BlackoutIsNotADrain.ACounterThatRoseThenWentFlatReleasesDrained
///   M31  `bool drained = r.measurable;` seeded with bare `true`
///        EQUIVALENT while M28's backstop stands: with nothing measurable the
///        loop does not run, nothing is examined, and the backstop forces the
///        same `false`. DrainUnmeasured is the test that fails the moment both
///        are gone (M31+M28 applied together, shown to fail it).
///   M35  finished counted as present — `&& !p.finished` added to the filter
///        -> ExchangePresence.AFinishedPeerThatIsNotHereIsNotWaitedFor
///   M36  the window not rolled forward on an evaluation that does not drain
///        -> BlackoutIsNotADrain.ACounterThatRoseThenWentFlatReleasesDrained
///   M37  the cap comparison `<` relaxed to `<=`
///        -> BlackoutIsNotADrain.ACounterThatNeverMovedRunsToTheCapUnfinished
///   M43  the over-correction of M35: finished peers skipped outright
///        -> ExchangePresence.AFinishedPeerThatIsHereIsWaitedFor
///
/// M43 is out of order because M41-M42 went to scovox's counter-liveness test
/// (test-plan 9) before this control was added.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <vector>

#include "explo_planner/exchange_drain.hpp"
#include "explo_planner/team_model.hpp"

using explo_planner::DrainReading;
using explo_planner::DrainStep;
using explo_planner::DrainWindow;
using explo_planner::FleetIdentity;
using explo_planner::makeFleetIdentity;
using explo_planner::robotBit;
using explo_planner::stepDrainRelease;
using explo_planner::TeamModel;

namespace {

constexpr int    kAtlas  = 0;   // this robot, in every test
constexpr int    kBestla = 1;
constexpr int    kCerd   = 2;

constexpr double kRateVoxSec = 1.0;    // R
constexpr double kWindowSec  = 10.0;   // W
constexpr double kCapSec     = 60.0;   // rendezvous_latched_hold_sec
/// Mission time the hold starts at. Late enough that a peer heard only at
/// t = 0 is past the 5 s TTL by then.
constexpr int    kHoldStartSec = 10;
/// Where a run stops if nothing has ended the hold: a whole extra window past
/// the cap, so a hold that overruns it is a failed assertion, not a hang.
constexpr int    kLastStepSec = 80;

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
TeamModel::Observation msg(int sender, uint32_t mask, bool finished = false) {
  TeamModel::Observation o;
  o.sender_id     = sender;
  o.in_range_mask = mask | robotBit(sender);
  o.finished      = finished;
  return o;
}

void hear(TeamModel& m, const TeamModel::Observation& o, double t) {
  ASSERT_EQ(m.observe(o, t), "") << "fixture message refused";
}

/// Whatever arrives at mission second `t`.
using Deliver = std::function<void(TeamModel&, double)>;
/// The fleet-sized deltas_received vector at settle second `s`. Entry 0 is
/// this robot's own and is never read.
using Counters = std::function<std::vector<uint64_t>(int)>;

/// The mission up to the moment the hold starts, so the model's belief at
/// s = 0 is whatever the traffic before it made it.
void prerollToHoldStart(TeamModel& m, const Deliver& deliver) {
  for (int t = 0; t < kHoldStartSec; ++t) {
    deliver(m, t);
    m.tick(t);
  }
}

/// How a hold ended: the first step that was not kHold, the settle second it
/// came at, what that step saw, and every second a full window was judged.
struct Hold {
  DrainStep           step   = DrainStep::kHold;
  int                 at_sec = -1;
  DrainReading        reading;
  std::vector<int>    evaluated_at;
};

Hold runHold(TeamModel& m, const Deliver& deliver, const Counters& counters) {
  // The node stamps the baseline when the hold starts; so does this.
  const std::vector<uint64_t> hold_base = counters(0);
  DrainWindow window;
  Hold h;
  for (int s = 0; s <= kLastStepSec; ++s) {
    const double t = kHoldStartSec + s;
    deliver(m, t);
    m.tick(t);
    const DrainReading r = stepDrainRelease(
        window, s, counters(s), hold_base, m, m.size(), kAtlas, kRateVoxSec,
        kWindowSec, kCapSec);
    if (r.evaluated) h.evaluated_at.push_back(s);
    if (r.step != DrainStep::kHold) {
      h.step    = r.step;
      h.at_sec  = s;
      h.reading = r;
      return h;
    }
  }
  return h;
}

const char* name(DrainStep s) {
  switch (s) {
    case DrainStep::kHold:       return "kHold (never ended)";
    case DrainStep::kDrained:    return "kDrained";
    case DrainStep::kUnfinished: return "kUnfinished";
  }
  return "?";
}

/// The rate the predicate saw at `at`, per the test's own schedule: the
/// difference over the window that closed there. Used to assert the
/// precondition test-plan 8 states for both of its cases — a rate of zero at
/// the moment of decision — so a schedule edit cannot quietly turn 8(a) into
/// a test of something the rate alone already decides.
double rateAtDecision(const Counters& counters, int peer, int at) {
  const int from = at - static_cast<int>(kWindowSec);
  const auto k = static_cast<size_t>(peer);
  return static_cast<double>(counters(at)[k] - counters(from)[k]) / kWindowSec;
}

}  // namespace

// ===========================================================================
// TEST-PLAN 7 — EXCHANGE PRESENCE.
// ===========================================================================

/// A FINISHED PEER WITH NO FRESH CONTACT IS NOT WAITED FOR (the A3 regression).
///
/// `finished` is sticky and relayed — it says the peer's run is over, not that
/// the peer is here. §4.1 of the design splits the two questions A3 conflated:
/// the barrier asks "should I stop waiting?" and is right to count a finished
/// peer; the exchange asks "is someone here to trade maps with?", which is a
/// radio statement, and a robot that finished and drove off is not an answer
/// to it.
///
/// cerd announces it is finished at t = 0 and is never heard again; bestla is
/// here, talks for five seconds, and goes flat. The exchange that is actually
/// happening is over by the second window. If cerd is counted, it is mute
/// against its own baseline for the whole hold, and the robot stands on the
/// cell to the cap waiting for a map that is not coming.
TEST(ExchangePresence, AFinishedPeerThatIsNotHereIsNotWaitedFor) {
  TeamModel m = model(fleet3());
  const Deliver deliver = [](TeamModel& tm, double t) {
    if (t == 0.0) hear(tm, msg(kCerd, robotBit(kAtlas), /*finished=*/true), t);
    // bestla names atlas and not cerd, so nothing bridges cerd either.
    hear(tm, msg(kBestla, robotBit(kAtlas)), t);
  };
  const Counters counters = [](int s) {
    return std::vector<uint64_t>{0, 1000u + 100u * static_cast<uint64_t>(std::min(s, 5)),
                                 500};
  };

  prerollToHoldStart(m, deliver);
  m.tick(kHoldStartSec);
  // The case, not a nearby one: cerd is finished and neither direct nor
  // relayed; bestla is a direct contact.
  ASSERT_TRUE(m.peer(kCerd).finished);
  ASSERT_FALSE(m.peer(kCerd).direct);
  ASSERT_FALSE(m.peer(kCerd).via_relay);
  ASSERT_TRUE(m.peer(kBestla).direct);

  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kDrained)
      << "got " << name(h.step) << " at s=" << h.at_sec << " (mute "
      << h.reading.mute << " of " << h.reading.examined << "): a peer that "
      << "finished and is not here was waited for as if it were";
  EXPECT_EQ(h.at_sec, 20) << "bestla went flat at s=5, so the first window "
                             "(0-10) still carries its burst and the second "
                             "(10-20) is the first quiet one";
  EXPECT_EQ(h.reading.examined, 1) << "only bestla is here";
  EXPECT_EQ(h.reading.mute, 0);
  EXPECT_FALSE(m.peer(kCerd).direct || m.peer(kCerd).via_relay)
      << "cerd became reachable during the hold, so this was not the case";
}

/// THE PAIR TO THE TEST ABOVE: A FINISHED PEER THAT IS HERE IS WAITED FOR.
///
/// The filter is on presence, and presence only. The over-correction of A3 —
/// skip finished peers — is wrong in the case generation 32 made the common
/// one: keepAppointmentOnFinish sends a robot that has finished exploring to
/// its standing appointment, so the partner on the cell is often finished and
/// has its whole map still to hand over. Here cerd is finished, direct, and
/// has delivered nothing since the hold began: the robot must hold for it to
/// the cap and leave on the unfinished outcome, not release.
TEST(ExchangePresence, AFinishedPeerThatIsHereIsWaitedFor) {
  TeamModel m = model(fleet3());
  const Deliver deliver = [](TeamModel& tm, double t) {
    hear(tm, msg(kCerd, robotBit(kAtlas), /*finished=*/true), t);
    hear(tm, msg(kBestla, robotBit(kAtlas)), t);
  };
  const Counters counters = [](int s) {
    return std::vector<uint64_t>{0, 1000u + 100u * static_cast<uint64_t>(std::min(s, 5)),
                                 500};
  };

  prerollToHoldStart(m, deliver);
  m.tick(kHoldStartSec);
  ASSERT_TRUE(m.peer(kCerd).finished);
  ASSERT_TRUE(m.peer(kCerd).direct);

  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kUnfinished)
      << "got " << name(h.step) << " at s=" << h.at_sec << ": a finished peer "
         "standing on the cell was skipped, so its map was abandoned";
  EXPECT_EQ(h.at_sec, 60);
  EXPECT_EQ(h.reading.examined, 2);
  EXPECT_EQ(h.reading.mute, 1) << "cerd, which never left its baseline";
}

/// A RELAYED PEER IS WAITED FOR.
///
/// cerd cannot hear atlas at all, but bestla can hear both and says so, so the
/// model carries cerd as in comms by relay. The emulator forwards serialized
/// bytes without deserializing them and dscovox credits the robot that SENSED
/// the voxels, so cerd's counter moves on atlas's side even though nothing
/// from cerd ever arrives first-hand — gen 32 measured relayed rows as the
/// most productive exchange channel per row (§2.8). A direct-only filter lets
/// that stream keep arriving while it calls the exchange finished.
///
/// bestla is quiet after its first two seconds; cerd keeps delivering at
/// 2 vox/s — twice R — until s = 40. The release belongs to cerd going flat.
TEST(ExchangePresence, ARelayedPeerIsWaitedFor) {
  TeamModel m = model(fleet3());
  const Deliver deliver = [](TeamModel& tm, double t) {
    hear(tm, msg(kBestla, robotBit(kAtlas) | robotBit(kCerd)), t);
  };
  const Counters counters = [](int s) {
    return std::vector<uint64_t>{
        0, 1000u + 100u * static_cast<uint64_t>(std::min(s, 2)),
        500u + 20u * static_cast<uint64_t>(std::min(s, 40))};
  };

  prerollToHoldStart(m, deliver);
  m.tick(kHoldStartSec);
  ASSERT_TRUE(m.peer(kBestla).direct);
  ASSERT_FALSE(m.peer(kCerd).direct);
  ASSERT_TRUE(m.peer(kCerd).via_relay);

  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kDrained) << "got " << name(h.step);
  EXPECT_EQ(h.at_sec, 50)
      << "released at s=" << h.at_sec << " while the relayed peer was still "
         "delivering at twice R (it goes flat at s=40, so 40-50 is its first "
         "quiet window)";
  EXPECT_EQ(h.reading.examined, 2) << "bestla direct and cerd by relay";
}

/// NOBODY READ IS NOT EVERYBODY DRAINED.
///
/// Every `continue` in the loop is a peer the robot could not read, and with
/// all of them taken a loop seeded `true` would release having examined no
/// one — at the first full window, on nothing. Here the only peer finished
/// at t = 0 and has not been heard since: the robot must not call that a
/// drained exchange. It holds to the cap and leaves UNFINISHED.
///
/// THIS IS ALSO TEST-PLAN 7'S N = 2 CASE, AND IT DOES NOT MATCH ITS WORDING.
/// The plan says a finished peer with no fresh contact does "not open a hold".
/// At N = 2 the hold does open: the barrier counts the finished partner as
/// accounted for (the barrier's question, where counting it is right), and
/// the hold opens on that count. The drain then examines nobody, and this
/// test is what happens next — a hold to the cap, logged as an unfinished
/// exchange. That is the safe reading of the evidence the predicate has, and
/// it is pinned here as the shipped behaviour. Whether the hold should open
/// at all is a design decision recorded as open in DESIGN_gen33.md §10, not
/// something this test settles.
TEST(ExchangePresence, NobodyReadIsNotEverybodyDrained) {
  TeamModel m = model(fleet2());
  const Deliver deliver = [](TeamModel& tm, double t) {
    if (t == 0.0) hear(tm, msg(kBestla, robotBit(kAtlas), /*finished=*/true), t);
  };
  const Counters counters = [](int) { return std::vector<uint64_t>{0, 1000}; };

  prerollToHoldStart(m, deliver);
  m.tick(kHoldStartSec);
  ASSERT_TRUE(m.peer(kBestla).finished);
  ASSERT_FALSE(m.peer(kBestla).direct || m.peer(kBestla).via_relay);

  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kUnfinished)
      << "got " << name(h.step) << " at s=" << h.at_sec
      << ": the hold released having examined " << h.reading.examined
      << " peer(s)";
  EXPECT_EQ(h.at_sec, 60);
  EXPECT_EQ(h.reading.examined, 0);
  EXPECT_TRUE(h.reading.measurable)
      << "the counters were fleet-sized; this must fail on presence, not on "
         "an unreadable vector, or it is DrainUnmeasured again";
}

// ===========================================================================
// TEST-PLAN 8 — BLACKOUT IS NOT A DRAIN. The behavioural half; the scan half
// is Gen33DrainRelease in test_gen21_latched_hold.cpp.
// ===========================================================================
//
// Both cases hold the peer BELIEVED PRESENT and both present a rate of 0 at
// the moment of decision. They differ only in whether the counter ever left
// its hold-start baseline. A predicate reduced to a rate test cannot tell them
// apart, and releases fastest in the one the hold exists to sit through.

/// (a) THE COUNTER NEVER MOVES. bestla is a direct contact every second and
/// not one voxel of its map arrives. The predicate must not release, must run
/// to the cap, and must end on kUnfinished — the event the node logs as
/// UNFINISHED EXCHANGE — never on kDrained.
TEST(BlackoutIsNotADrain, ACounterThatNeverMovedRunsToTheCapUnfinished) {
  TeamModel m = model(fleet2());
  const Deliver deliver = [](TeamModel& tm, double t) {
    hear(tm, msg(kBestla, robotBit(kAtlas)), t);
  };
  const Counters counters = [](int) { return std::vector<uint64_t>{0, 1000}; };

  prerollToHoldStart(m, deliver);
  m.tick(kHoldStartSec);
  ASSERT_TRUE(m.peer(kBestla).direct);

  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kUnfinished)
      << "got " << name(h.step) << " at s=" << h.at_sec
      << ": a peer that delivered nothing was read as drained";
  EXPECT_EQ(h.at_sec, 60) << "the cap is 60 s and a window closes exactly on "
                             "it; ending anywhere else moved the cap";
  EXPECT_EQ(h.reading.examined, 1);
  EXPECT_EQ(h.reading.mute, 1);
  EXPECT_EQ(h.evaluated_at, (std::vector<int>{10, 20, 30, 40, 50, 60}))
      << "every full window was judged, and every one held";
  // The precondition, stated so that it is checked rather than assumed.
  EXPECT_EQ(rateAtDecision(counters, kBestla, h.at_sec), 0.0);
}

/// (b) THE COUNTER ROSE, THEN WENT FLAT FOR W. bestla delivers 40 vox/s until
/// s = 15 and nothing after. The window 0-10 is busy, 10-20 still carries the
/// tail of it, and 20-30 is the first quiet one: the predicate must release
/// there, and as kDrained.
///
/// Also the rolling window. Each evaluation that does not drain must re-open
/// the window where it closed, so the next rate is over the next W seconds
/// and nothing older. A window left open from s = 0 dilutes the burst into an
/// ever-longer denominator and never gets below R before the cap.
TEST(BlackoutIsNotADrain, ACounterThatRoseThenWentFlatReleasesDrained) {
  TeamModel m = model(fleet2());
  const Deliver deliver = [](TeamModel& tm, double t) {
    hear(tm, msg(kBestla, robotBit(kAtlas)), t);
  };
  const Counters counters = [](int s) {
    return std::vector<uint64_t>{0, 1000u + 40u * static_cast<uint64_t>(std::min(s, 15))};
  };

  prerollToHoldStart(m, deliver);
  m.tick(kHoldStartSec);
  ASSERT_TRUE(m.peer(kBestla).direct);

  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kDrained)
      << "got " << name(h.step) << " at s=" << h.at_sec
      << ": an exchange that finished was not released";
  EXPECT_EQ(h.at_sec, 30)
      << "released at s=" << h.at_sec << "; the first window with a rate "
         "below R is 20-30";
  EXPECT_EQ(h.evaluated_at, (std::vector<int>{10, 20, 30}))
      << "windows are W apart; judging every second means the window did not "
         "roll";
  EXPECT_EQ(h.reading.mute, 0);
  EXPECT_EQ(h.reading.examined, 1);
  EXPECT_EQ(rateAtDecision(counters, kBestla, h.at_sec), 0.0);
}

// ===========================================================================
// UNMEASURED
// ===========================================================================

/// NO COUNTERS IS NEVER DRAINED.
///
/// dscovox publishes the counters only once it is up, and the node's vectors
/// are empty until then, so a meeting that began earlier has nothing to
/// difference. That reading is unmeasured, and it must hold to the cap and
/// leave unfinished rather than release on the absence of evidence. The node
/// warns on `evaluated && !measurable`, so the reading has to say so too.
TEST(DrainUnmeasured, NoCountersIsNeverDrained) {
  TeamModel m = model(fleet2());
  const Deliver deliver = [](TeamModel& tm, double t) {
    hear(tm, msg(kBestla, robotBit(kAtlas)), t);
  };
  const Counters counters = [](int) { return std::vector<uint64_t>{}; };

  prerollToHoldStart(m, deliver);
  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kUnfinished) << "got " << name(h.step);
  EXPECT_EQ(h.at_sec, 60);
  EXPECT_TRUE(h.reading.evaluated);
  EXPECT_FALSE(h.reading.measurable);
}
