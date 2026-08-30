/// @file test_team_model.cpp
/// @brief The comms model: handshake, TTL, two-hop closure, gossip freshness.
///
/// The three properties worth stating up front, because most of what follows
/// is a way of pinning one of them down:
///
///   - a link is MUTUAL or it is not a link (doc/limitations.md §10);
///   - closure is a statement about COMMS and never about DATA, so a peer can
///     be IN_COMMS while everything we know about it is minutes old;
///   - a peer's clock is never read as a time, only ever differenced into an
///     age on the peer's own clock and re-based onto ours.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "explo_planner/team_model.hpp"

using explo_planner::CommsStatus;
using explo_planner::FleetIdentity;
using explo_planner::kMaxTeamSize;
using explo_planner::makeFleetIdentity;
using explo_planner::maskHas;
using explo_planner::robotBit;
using explo_planner::TeamModel;

namespace {

TeamModel::Config goodCfg() {
  TeamModel::Config c;
  c.direct_ttl_sec     = 5.0;
  c.closure_enabled    = true;
  c.gossip_max_age_sec = 120.0;
  return c;
}

/// Three robots, this one is id 0. Three is the smallest fleet in which
/// closure, gossip and the N>=3 half of the handshake are all reachable at
/// all — at N=2 every one of them degenerates.
FleetIdentity fleet3() {
  return makeFleetIdentity({"atlas", "bestla", "cerd"}, "atlas");
}

TeamModel makeModel(const TeamModel::Config& cfg = goodCfg()) {
  TeamModel m;
  EXPECT_EQ(m.configure(fleet3(), cfg), "") << "fixture failed to configure";
  return m;
}

/// A message from `sender` whose direct-contact mask is `mask`. The sender
/// always names itself, as a real one does.
TeamModel::Observation msg(int sender, uint32_t mask) {
  TeamModel::Observation o;
  o.sender_id     = sender;
  o.in_range_mask = mask | robotBit(sender);
  return o;
}

}  // namespace

// ===========================================================================
// Configuration
// ===========================================================================

TEST(TeamModelConfig, RefusesAnIdentityItCannotIndex) {
  TeamModel m;
  // The legacy default: team_robot_names unset. Legal for the node, useless
  // here, and accepted-as-empty it would answer LOST_COMMS about everybody
  // with no way to tell that from a dead radio.
  EXPECT_NE(m.configure(FleetIdentity{}, goodCfg()), "");
  EXPECT_FALSE(m.configured());

  FleetIdentity nameless;
  nameless.configured = true;
  nameless.names      = {"atlas", "bestla"};
  nameless.self_id    = -1;
  EXPECT_NE(m.configure(nameless, goodCfg()), "");

  FleetIdentity oversize;
  oversize.configured = true;
  oversize.self_id    = 0;
  oversize.names.assign(static_cast<size_t>(kMaxTeamSize) + 1, "r");
  EXPECT_NE(m.configure(oversize, goodCfg()), "");

  TeamModel::Config bad = goodCfg();
  bad.direct_ttl_sec = 0.0;
  EXPECT_NE(m.configure(fleet3(), bad), "") << "a zero TTL makes every peer "
                                               "lost on the tick it arrived";
  bad = goodCfg();
  bad.gossip_max_age_sec = -1.0;
  EXPECT_NE(m.configure(fleet3(), bad), "");

  // Calibration: the refusals above must not be passing because configure()
  // refuses everything.
  EXPECT_EQ(m.configure(fleet3(), goodCfg()), "");
  EXPECT_TRUE(m.configured());
  EXPECT_EQ(m.size(), 3);
  EXPECT_EQ(m.selfId(), 0);
}

TEST(TeamModelConfig, SelfNeedsNoSpecialCase) {
  TeamModel m = makeModel();
  EXPECT_TRUE(m.inComms(0));
  EXPECT_EQ(m.lastKnownAgeSec(0, 900.0), 0.0);
  EXPECT_EQ(m.lastDirectAgeSec(0, 900.0), 0.0);
  EXPECT_TRUE(maskHas(m.directMask(), 0));
  EXPECT_TRUE(maskHas(m.inCommsMask(), 0));
  EXPECT_EQ(m.lostCount(), 2) << "and the two peers are lost until heard from";
}

TEST(TeamModelConfig, ReconfiguringClears) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 10.0), "");
  m.tick(10.0);
  ASSERT_TRUE(m.inComms(1));
  ASSERT_EQ(m.configure(fleet3(), goodCfg()), "");
  EXPECT_FALSE(m.inComms(1)) << "a contact from the previous fleet definition "
                                "must not survive into the new one";
  EXPECT_LT(m.lastKnownAgeSec(1, 10.0), 0.0);
}

// ===========================================================================
// The handshake
// ===========================================================================

TEST(TeamModelHandshake, OneWayContactIsNotInComms) {
  // The §10 failure, exactly: we receive from bestla, bestla cannot hear us.
  // Its mask names itself and cerd, but not us.
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(2)), 100.0), "");
  m.tick(100.0);

  EXPECT_FALSE(m.inComms(1)) << "a receipt is half a link; reconnecting over "
                                "it would share nothing";
  EXPECT_FALSE(m.peer(1).direct);
  EXPECT_TRUE(m.peer(1).heard_one_way) << "and it is reported as its own "
                                          "condition, not as an absence";
  EXPECT_LT(m.lastDirectAgeSec(1, 100.0), 0.0) << "never direct";
  // But we DID receive data, and that is a separate fact from the link state.
  EXPECT_EQ(m.lastKnownAgeSec(1, 100.0), 0.0);

  // Paired control on the one bit that differs: without it this test would
  // pass against a model that never reports IN_COMMS at all.
  TeamModel ok = makeModel();
  ASSERT_EQ(ok.observe(msg(1, robotBit(0) | robotBit(2)), 100.0), "");
  ok.tick(100.0);
  EXPECT_TRUE(ok.inComms(1));
  EXPECT_TRUE(ok.peer(1).direct);
  EXPECT_FALSE(ok.peer(1).heard_one_way);
  EXPECT_EQ(ok.lastDirectAgeSec(1, 100.0), 0.0);
}

TEST(TeamModelHandshake, PublishedMaskIsReceiptNotHandshake) {
  // If we published the mutual result, neither robot could ever name the
  // other: each would be waiting to be named first. So the published mask is
  // the half we can observe alone, and it names a one-way peer.
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(2)), 100.0), "");
  m.tick(100.0);
  EXPECT_TRUE(maskHas(m.directMask(), 1)) << "we receive from it, and saying "
                                             "so is what breaks the deadlock";
  EXPECT_FALSE(maskHas(m.inCommsMask(), 1)) << "the conclusion is ours alone";
}

TEST(TeamModelHandshake, ContactExpiresWithTheTtl) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 10.0), "");

  m.tick(12.0);
  EXPECT_TRUE(m.inComms(1));
  m.tick(15.0);
  EXPECT_TRUE(m.inComms(1)) << "the TTL boundary itself is still contact";
  m.tick(15.1);
  EXPECT_FALSE(m.inComms(1));
  EXPECT_FALSE(m.peer(1).heard_one_way) << "expired is not one-way: nothing "
                                           "is arriving at all";

  // The link is gone; the knowledge of when it was last up is not, and the
  // knowledge gate keys on precisely that.
  EXPECT_NEAR(m.lastDirectAgeSec(1, 15.1), 5.1, 1e-9);
  EXPECT_NEAR(m.lastKnownAgeSec(1, 15.1), 5.1, 1e-9);

  // And it heals on the next message rather than needing a reset.
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 20.0), "");
  m.tick(20.0);
  EXPECT_TRUE(m.inComms(1));
  EXPECT_EQ(m.lastDirectAgeSec(1, 20.0), 0.0);
}

TEST(TeamModelHandshake, ReportsWhatThePeerClaimsAboutItself) {
  TeamModel m = makeModel();
  TeamModel::Observation o = msg(1, robotBit(0));
  o.finished      = true;
  o.have_position = true;
  o.x = 3.0; o.y = -4.0; o.z = 0.5;
  ASSERT_EQ(m.observe(o, 100.0), "");
  m.tick(100.0);
  EXPECT_TRUE(m.peer(1).finished);
  EXPECT_TRUE(m.peer(1).have_position);
  EXPECT_TRUE(m.peer(1).position_first_hand);
  EXPECT_EQ(m.peer(1).position_x, 3.0);
  EXPECT_EQ(m.peer(1).in_range_mask, robotBit(0) | robotBit(1));
}

// ===========================================================================
// Closure
// ===========================================================================

TEST(TeamModelClosure, ReachesThroughABridgeAndSaysItIsARelay) {
  // A—B—C. We are A; B hears both of us; C we cannot hear at all.
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(0) | robotBit(2)), 100.0), "");
  m.tick(100.0);

  EXPECT_TRUE(m.inComms(2)) << "a reconnection dispatch to cerd would be a "
                               "long drive to deliver what bestla is already "
                               "relaying";
  EXPECT_TRUE(m.peer(2).via_relay);
  EXPECT_FALSE(m.peer(2).direct);
  EXPECT_EQ(m.lostCount(), 0);
  EXPECT_TRUE(maskHas(m.inCommsMask(), 2));
  EXPECT_FALSE(maskHas(m.directMask(), 2))
      << "the published mask is direct contacts; putting the closure result "
         "there would make the handshake circular";
}

TEST(TeamModelClosure, IsAboutCommsAndNeverAboutData) {
  // The trap. Same fixture as above: cerd is IN_COMMS, and we have never held
  // one byte about cerd. A consumer that keyed on the status where it meant
  // freshness would chase a position that does not exist.
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(0) | robotBit(2)), 100.0), "");
  m.tick(100.0);
  ASSERT_TRUE(m.inComms(2));
  EXPECT_LT(m.lastKnownAgeSec(2, 100.0), 0.0) << "never heard anything";
  EXPECT_LT(m.lastDirectAgeSec(2, 100.0), 0.0);
  EXPECT_FALSE(m.peer(2).have_position);
}

TEST(TeamModelClosure, WillNotExpandThroughABridgeThatCannotHearUs) {
  // bestla relays cerd, but bestla cannot hear US — so nothing we know is
  // reaching cerd through it, and the closure would be a fiction.
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(2)), 100.0), "");
  m.tick(100.0);
  EXPECT_FALSE(m.inComms(1));
  EXPECT_FALSE(m.inComms(2));
}

TEST(TeamModelClosure, ExpiresWithTheBridge) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(0) | robotBit(2)), 100.0), "");
  m.tick(101.0);
  ASSERT_TRUE(m.inComms(2));
  m.tick(106.0);
  EXPECT_FALSE(m.inComms(1)) << "the bridge went stale";
  EXPECT_FALSE(m.inComms(2)) << "and it must take the closure with it, not "
                                "leave a peer permanently reachable through a "
                                "robot we stopped hearing";
  EXPECT_FALSE(m.peer(2).via_relay);
}

TEST(TeamModelClosure, IsTwoHopsAndTheThirdIsHonestlyMissing) {
  // A—B—C—D with four robots. B's row reaches us on B's own message, so B—C
  // is visible; C's row does not, so C—D is not. This is a limitation of what
  // TeamWorld carries, and the model reports D lost rather than guessing.
  TeamModel m;
  ASSERT_EQ(m.configure(makeFleetIdentity({"a", "b", "c", "d"}, "a"),
                        goodCfg()),
            "");
  ASSERT_EQ(m.observe(msg(1, robotBit(0) | robotBit(2)), 100.0), "");
  m.tick(100.0);
  EXPECT_TRUE(m.inComms(2));
  EXPECT_FALSE(m.inComms(3));
  EXPECT_EQ(m.lostCount(), 1);
}

TEST(TeamModelClosure, CanBeTurnedOffToRecoverThePriorBehaviour) {
  TeamModel::Config cfg = goodCfg();
  cfg.closure_enabled = false;
  TeamModel m = makeModel(cfg);
  ASSERT_EQ(m.observe(msg(1, robotBit(0) | robotBit(2)), 100.0), "");
  m.tick(100.0);
  EXPECT_TRUE(m.inComms(1)) << "direct contact is not part of the closure";
  EXPECT_FALSE(m.inComms(2));
  EXPECT_FALSE(m.peer(2).via_relay);
}

// ===========================================================================
// Gossip
// ===========================================================================

namespace {

/// A message from `sender` carrying gossip. `heard` is indexed by fleet id and
/// holds times ON THE SENDER'S CLOCK; the sender's own entry is its publish
/// time, which is the reference every other entry is differenced against.
TeamModel::Observation gossipMsg(int sender, uint32_t mask,
                                 const std::vector<double>& heard) {
  TeamModel::Observation o = msg(sender, mask);
  o.last_heard_sec = heard;
  return o;
}

}  // namespace

TEST(TeamModelGossip, TransfersAsAnIntervalNotATimestamp) {
  // The two clocks are deliberately an order of magnitude apart: field robots'
  // clocks have been observed hours out (see the bunker-curt bags), so
  // anything that read the sender's numbers as local times would land in the
  // far past or the future here, not merely be a little off.
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(gossipMsg(1, robotBit(0), {-1.0, 100.0, 70.0}), 1000.0),
            "");
  m.tick(1000.0);

  EXPECT_TRUE(m.peer(2).known);
  EXPECT_FALSE(m.inComms(2)) << "gossip is knowledge about a peer, not a link "
                                "to it: bestla's mask does not name cerd";
  EXPECT_NEAR(m.lastKnownAgeSec(2, 1000.0), 30.0, 1e-9)
      << "100 - 70 on the sender's clock is a 30 s age wherever it lands";
  EXPECT_LT(m.lastDirectAgeSec(2, 1000.0), 0.0);

  // And it ages on our clock from there, with no further messages.
  EXPECT_NEAR(m.lastKnownAgeSec(2, 1060.0), 90.0, 1e-9);
}

TEST(TeamModelGossip, CarriesARelayedPositionAndLabelsItAsRelayed) {
  TeamModel m = makeModel();
  TeamModel::Observation o =
      gossipMsg(1, robotBit(0), {-1.0, 100.0, 90.0});
  o.gx = {0.0, 0.0, 12.0};
  o.gy = {0.0, 0.0, -3.0};
  o.gz = {0.0, 0.0, 0.0};
  o.have_gossip_pos = {0, 0, 1};
  ASSERT_EQ(m.observe(o, 500.0), "");

  EXPECT_TRUE(m.peer(2).have_position);
  EXPECT_FALSE(m.peer(2).position_first_hand)
      << "an operator reading a position must be able to see it came second-"
         "hand, because its age and its accuracy are different questions";
  EXPECT_EQ(m.peer(2).position_x, 12.0);
  EXPECT_EQ(m.peer(2).position_y, -3.0);
}

TEST(TeamModelGossip, StopsCountingPastTheMaxAge) {
  TeamModel m = makeModel();
  // 200 s old, against a 120 s cutoff. Treated as never heard, NOT as heard
  // 200 s ago: a very stale interval added to a message is a number that would
  // still compare against thresholds while meaning nothing.
  ASSERT_EQ(m.observe(gossipMsg(1, robotBit(0), {-1.0, 300.0, 100.0}), 1000.0),
            "");
  EXPECT_FALSE(m.peer(2).known);
  EXPECT_LT(m.lastKnownAgeSec(2, 1000.0), 0.0);

  // Calibration on the same message shape, inside the cutoff.
  TeamModel ok = makeModel();
  ASSERT_EQ(ok.observe(gossipMsg(1, robotBit(0), {-1.0, 300.0, 200.0}), 1000.0),
            "");
  EXPECT_TRUE(ok.peer(2).known);
  EXPECT_NEAR(ok.lastKnownAgeSec(2, 1000.0), 100.0, 1e-9);
}

TEST(TeamModelGossip, IsUninterpretableWithoutTheSendersOwnEntry) {
  // No reference point, so no age can be computed for anything in the array.
  // Guessing one — say, taking the sender's latest entry as its publish time —
  // would systematically under-age every relay and make stale positions look
  // current, so the whole array is dropped instead.
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(gossipMsg(1, robotBit(0), {-1.0, -1.0, 70.0}), 1000.0),
            "");
  EXPECT_FALSE(m.peer(2).known);

  TeamModel shortm = makeModel();
  ASSERT_EQ(shortm.observe(gossipMsg(1, robotBit(0), {-1.0}), 1000.0), "");
  EXPECT_FALSE(shortm.peer(2).known) << "an array too short to hold the "
                                        "sender's own entry is the same case";
}

TEST(TeamModelGossip, OnlyEverMovesFreshnessForward) {
  TeamModel m = makeModel();
  // First-hand contact with cerd at t=1000.
  ASSERT_EQ(m.observe(msg(2, robotBit(0)), 1000.0), "");
  m.tick(1000.0);
  ASSERT_EQ(m.lastKnownAgeSec(2, 1000.0), 0.0);

  // A relay arriving later, carrying an OLDER reading of the same robot. It
  // must not overwrite first-hand knowledge with its stale copy.
  ASSERT_EQ(m.observe(gossipMsg(1, robotBit(0), {-1.0, 200.0, 150.0}), 1001.0),
            "");
  EXPECT_NEAR(m.lastKnownAgeSec(2, 1001.0), 1.0, 1e-9)
      << "a 50 s-old relay must not age our 1 s-old first-hand contact";

  // And a relay that keeps re-sending the same reading must not keep
  // refreshing it. Note what "the same reading re-sent" looks like on the
  // wire: the entry for cerd is unchanged at 190, but the SENDER'S OWN entry
  // advances every publish, because it is the sender's publish time. So the
  // age grows message by message and `at` stays put — which is the point.
  // (Constructing this with a frozen sender entry instead would be a
  // duplicated message, not a re-send; see the note in observe().)
  TeamModel r = makeModel();
  ASSERT_EQ(r.observe(gossipMsg(1, robotBit(0), {-1.0, 200.0, 190.0}), 500.0),
            "");
  ASSERT_NEAR(r.lastKnownAgeSec(2, 500.0), 10.0, 1e-9);
  ASSERT_EQ(r.observe(gossipMsg(1, robotBit(0), {-1.0, 260.0, 190.0}), 560.0),
            "");
  EXPECT_NEAR(r.lastKnownAgeSec(2, 560.0), 70.0, 1e-9)
      << "the same reading, 60 s later, is 60 s older — not brand new";
}

TEST(TeamModelGossip, IgnoresASenderClockThatRanBackwards) {
  TeamModel m = makeModel();
  // The sender claims it heard cerd in its own future. Negative ages would
  // date the reading after our receipt time and make it permanently the
  // freshest thing we hold.
  ASSERT_EQ(m.observe(gossipMsg(1, robotBit(0), {-1.0, 100.0, 150.0}), 1000.0),
            "");
  EXPECT_FALSE(m.peer(2).known);
}

// ===========================================================================
// Message-level refusals
// ===========================================================================

TEST(TeamModelObserve, RefusesRatherThanMisapplying) {
  TeamModel unconf;
  EXPECT_NE(unconf.observe(msg(1, robotBit(0)), 10.0), "");

  TeamModel m = makeModel();
  EXPECT_NE(m.observe(msg(-1, 0), 10.0), "");
  EXPECT_NE(m.observe(msg(3, 0), 10.0), "") << "outside this fleet";
  EXPECT_NE(m.observe(msg(0, 0), 10.0), "") << "our own message, looped back";
  EXPECT_NE(m.observe(msg(1, robotBit(0)), -1.0), "");
  EXPECT_NE(m.observe(msg(1, robotBit(0)),
                      std::numeric_limits<double>::quiet_NaN()),
            "");

  // A gossip array longer than our fleet means the sender is running a
  // different team_robot_names, so its ids address different robots than ours.
  // Truncating would apply the sender's robot 0 to our robot 0.
  EXPECT_NE(m.observe(gossipMsg(1, robotBit(0), {0.0, 1.0, 2.0, 3.0}), 10.0),
            "");
  EXPECT_FALSE(m.peer(1).known) << "and the refusal must be total: no part of "
                                   "a message from a different fleet applies";

  // Calibration: a well-formed message on the same path is accepted.
  EXPECT_EQ(m.observe(msg(1, robotBit(0)), 10.0), "");
  EXPECT_TRUE(m.peer(1).known);
}

TEST(TeamModelObserve, TickIsIdempotentAndSafeBeforeAnyMessage) {
  TeamModel m = makeModel();
  m.tick(0.0);
  m.tick(0.0);
  EXPECT_EQ(m.lostCount(), 2);
  EXPECT_FALSE(m.peer(1).known);

  TeamModel unconf;
  unconf.tick(0.0);  // must not touch empty vectors
  EXPECT_FALSE(unconf.configured());
  EXPECT_EQ(unconf.lostCount(), 0);
  EXPECT_FALSE(unconf.inComms(0));
}
