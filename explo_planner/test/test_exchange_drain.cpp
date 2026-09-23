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
///
/// MUTATION-VERIFIED 2026-09-23, the all-peers-leaving outcome (§10 item 5;
/// M44-M53 are the node scans in test_gen21_latched_hold.cpp):
///
///   M54  `r.leaving == r.others` relaxed to `r.leaving > 0`
///        -> AllPeersLeaving.OnePeerStillComingKeepsTheHold
///   M55  the `here == 0` term deleted
///        -> AllPeersLeaving.ALeavingPeerThatIsStillHereIsReadOnItsCounter
///           and ...IsNotSkippedWhenUnmeasured
///   M56  `here == 0` read off `r.examined == 0` instead
///        -> AllPeersLeaving.ALeavingPeerThatIsHereIsNotSkippedWhenUnmeasured
///   M57  the `r.others > 0` term deleted
///        -> AllPeersLeaving.NoPeersIsNotEveryPeerLeaving
///   M58  leaving counted at DONE only, not HOMING
///        -> AllPeersLeaving.EveryPeerHomingOrDoneEndsTheHoldAtN3
/// Moved comments: doc/explo_planner_code_notes.md

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <vector>

#include "explo_planner/exchange_drain.hpp"
#include "explo_planner/meeting_attendance.hpp"
#include "explo_planner/team_model.hpp"

using explo_planner::DrainReading;
using explo_planner::DrainStep;
using explo_planner::DrainWindow;
using explo_planner::FleetIdentity;
using explo_planner::kModeDone;
using explo_planner::kModeExploring;
using explo_planner::kModeHoming;
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
/// `mode` is the sender's TeamWorld level (0 exploring, 1 homing, 2 done).
TeamModel::Observation msg(int sender, uint32_t mask, bool finished = false,
                           uint8_t mode = 0) {
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
    case DrainStep::kAllPeersLeaving: return "kAllPeersLeaving";
  }
  return "?";
}

/// The per-peer rate over the kWindowSec window closing at the given second,
/// from the test's own schedule. Used to assert both blackout cases see a zero
/// rate at the decision. (notes: drain-rate-at-decision)
double rateAtDecision(const Counters& counters, int peer, int at) {
  const int from = at - static_cast<int>(kWindowSec);
  const auto k = static_cast<size_t>(peer);
  return static_cast<double>(counters(at)[k] - counters(from)[k]) / kWindowSec;
}

}  // namespace

// ===========================================================================
// TEST-PLAN 7 — EXCHANGE PRESENCE.
// ===========================================================================

/// finished is sticky and relayed: it says the peer's run is over, not that it
/// is here. The exchange waits only for peers present by radio, so a finished
/// peer that left is not waited for. (notes: drain-finished-absent-peer)
TEST(ExchangePresence, AFinishedPeerThatIsNotHereIsNotWaitedFor) {
  TeamModel m = model(fleet3());
  const Deliver deliver = [](TeamModel& tm, double t) {
    if (t == 0.0)
      hear(tm, msg(kCerd, robotBit(kAtlas), /*finished=*/true, kModeDone), t);
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

/// The filter is on presence only: a finished peer that is direct is still
/// waited for, as it may have its whole map to hand over. Here it delivers
/// nothing, so the hold runs to the cap, unfinished.
/// (notes: drain-finished-present-peer)
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

/// A peer in comms only by relay is waited for: relayed data is credited to the
/// robot that sensed it, so its counter moves here. cerd delivers until s = 40,
/// so the release belongs to it going flat. (notes: drain-relayed-peer)
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

/// A hold that examined no one must not release. The only peer finished, has
/// not said it is leaving, and is not heard: the hold runs to the cap and ends
/// UNFINISHED, the honest outcome for a no-show. (notes: drain-nobody-read)
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
// EVERY PEER HAS SAID IT IS LEAVING — the partner protocol's other half
// (§10 item 5). A robot stops waiting when its partner says homing or done.
// ===========================================================================

/// N = 2: THE PARTNER CAME, EXCHANGED, AND LEFT — or never meant to come. It
/// said DONE and is not here. Nothing is left to trade, so the hold ends at
/// the first full window, and as its own outcome: not drained (nobody spoke)
/// and not unfinished (nothing was owed).
TEST(AllPeersLeaving, APartnerThatSaidDoneAndIsGoneEndsTheHold) {
  TeamModel m = model(fleet2());
  const Deliver deliver = [](TeamModel& tm, double t) {
    if (t == 0.0)
      hear(tm, msg(kBestla, robotBit(kAtlas), /*finished=*/true, kModeDone), t);
  };
  const Counters counters = [](int) { return std::vector<uint64_t>{0, 1000}; };

  prerollToHoldStart(m, deliver);
  m.tick(kHoldStartSec);
  ASSERT_FALSE(m.peer(kBestla).direct || m.peer(kBestla).via_relay);

  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kAllPeersLeaving) << "got " << name(h.step);
  EXPECT_EQ(h.at_sec, 10) << "the first full window, and not before it";
  EXPECT_EQ(h.reading.examined, 0);
  EXPECT_EQ(h.reading.others, 1);
  EXPECT_EQ(h.reading.leaving, 1);
}

/// N = 3: ALL OF THEM, BY EITHER LEVEL. bestla is done; cerd is homing
/// without having finished (a budget return). Both are gone, both said so.
TEST(AllPeersLeaving, EveryPeerHomingOrDoneEndsTheHoldAtN3) {
  TeamModel m = model(fleet3());
  const Deliver deliver = [](TeamModel& tm, double t) {
    if (t != 0.0) return;
    hear(tm, msg(kBestla, robotBit(kAtlas), /*finished=*/true, kModeDone), t);
    hear(tm, msg(kCerd, robotBit(kAtlas), /*finished=*/false, kModeHoming), t);
  };
  const Counters counters = [](int) {
    return std::vector<uint64_t>{0, 1000, 500};
  };

  prerollToHoldStart(m, deliver);
  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kAllPeersLeaving) << "got " << name(h.step);
  EXPECT_EQ(h.at_sec, 10);
  EXPECT_EQ(h.reading.others, 2);
  EXPECT_EQ(h.reading.leaving, 2);
}

/// ALL, NOT ANY. bestla is done and gone; cerd finished but has not said it is
/// leaving — it may still be walking in. One leaving peer does not release a
/// hold another may still arrive at: this is the no-show case, to the cap.
TEST(AllPeersLeaving, OnePeerStillComingKeepsTheHold) {
  TeamModel m = model(fleet3());
  const Deliver deliver = [](TeamModel& tm, double t) {
    if (t != 0.0) return;
    hear(tm, msg(kBestla, robotBit(kAtlas), /*finished=*/true, kModeDone), t);
    hear(tm, msg(kCerd, robotBit(kAtlas), /*finished=*/true, kModeExploring), t);
  };
  const Counters counters = [](int) {
    return std::vector<uint64_t>{0, 1000, 500};
  };

  prerollToHoldStart(m, deliver);
  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kUnfinished)
      << "got " << name(h.step) << " at s=" << h.at_sec
      << ": one peer leaving released the hold another may still arrive at";
  EXPECT_EQ(h.at_sec, 60);
  EXPECT_EQ(h.reading.leaving, 1);
  EXPECT_EQ(h.reading.others, 2);
}

/// LEAVING, BUT HERE: THE COUNTER STILL DECIDES. bestla says DONE — its
/// manoeuvre ended, it is turning for home — but it is still in range and its
/// map is still crossing at 2 vox/s until s = 30. What it said is not the
/// exchange; the bytes are. The hold releases on the counter going flat.
TEST(AllPeersLeaving, ALeavingPeerThatIsStillHereIsReadOnItsCounter) {
  TeamModel m = model(fleet2());
  const Deliver deliver = [](TeamModel& tm, double t) {
    hear(tm, msg(kBestla, robotBit(kAtlas), /*finished=*/true, kModeDone), t);
  };
  const Counters counters = [](int s) {
    return std::vector<uint64_t>{0, 1000u + 20u * static_cast<uint64_t>(std::min(s, 30))};
  };

  prerollToHoldStart(m, deliver);
  m.tick(kHoldStartSec);
  ASSERT_TRUE(m.peer(kBestla).direct);

  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kDrained)
      << "got " << name(h.step) << " at s=" << h.at_sec
      << ": a leaving peer still delivering was cut off by what it said";
  EXPECT_EQ(h.at_sec, 40) << "30-40 is its first quiet window";
  EXPECT_EQ(h.reading.examined, 1);
}

/// With unmeasurable counters examined is 0 even with the peer on the cell, so
/// presence must come from the model, not that zero. A leaving peer that is
/// here does not end an unmeasured hold before the cap.
/// (notes: drain-leaving-peer-unmeasured)
TEST(AllPeersLeaving, ALeavingPeerThatIsHereIsNotSkippedWhenUnmeasured) {
  TeamModel m = model(fleet2());
  const Deliver deliver = [](TeamModel& tm, double t) {
    hear(tm, msg(kBestla, robotBit(kAtlas), /*finished=*/true, kModeDone), t);
  };
  const Counters counters = [](int) { return std::vector<uint64_t>{}; };

  prerollToHoldStart(m, deliver);
  const Hold h = runHold(m, deliver, counters);
  EXPECT_EQ(h.step, DrainStep::kUnfinished)
      << "got " << name(h.step) << " at s=" << h.at_sec;
  EXPECT_EQ(h.at_sec, 60);
  EXPECT_FALSE(h.reading.measurable);
}

/// AN UNCONFIGURED MODEL KNOWS NO PEERS, so "every one of them is leaving" is
/// vacuous, not true. It holds, as it did before this outcome existed.
TEST(AllPeersLeaving, NoPeersIsNotEveryPeerLeaving) {
  TeamModel m;   // never configured
  const std::vector<uint64_t> c{0, 1000};
  DrainWindow window;
  DrainReading r;
  for (int s = 0; s <= 60; ++s) {
    r = stepDrainRelease(window, s, c, c, m, 2, kAtlas, kRateVoxSec,
                         kWindowSec, kCapSec);
    if (r.step != DrainStep::kHold) break;
  }
  EXPECT_EQ(r.step, DrainStep::kUnfinished) << "got " << name(r.step);
  EXPECT_EQ(r.others, 0);
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

/// bestla delivers 40 vox/s until s = 15; 20-30 is the first quiet window, so
/// the release is there, as kDrained. Each non-draining evaluation re-opens the
/// window where it closed, or the burst never drops below R.
/// (notes: drain-rose-then-flat)
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

/// Empty counter vectors (dscovox not yet up) are unmeasured: the hold runs to
/// the cap and ends unfinished, never drained. The reading must say so, since
/// the node warns on an evaluated, unmeasurable reading.
/// (notes: drain-unmeasured-never-drained)
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
