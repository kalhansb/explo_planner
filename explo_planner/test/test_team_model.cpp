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

// ===========================================================================
// Position freshness
//
// Rule 3 of the file header, made testable. lastKnownAgeSec() answers "when
// did we last learn ANYTHING about this robot"; positionAgeSec() answers "how
// old is the pose we hold for it". Both refresh paths in observe() advance
// last_known_sec BEFORE testing whether the message carried a position at all,
// so the two genuinely come apart, and the consumer that steers on a peer pose
// — a chase, the separation term — is the one that would be wrong about it.
// ===========================================================================

TEST(TeamModelPositionAge, NegativeUntilAPositionArrives) {
  TeamModel m = makeModel();
  EXPECT_LT(m.positionAgeSec(1, 0.0), 0.0) << "nothing heard yet";

  // Heard, but the message carried no position.
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 10.0), "");
  EXPECT_GE(m.lastKnownAgeSec(1, 10.0), 0.0) << "we did hear from it";
  EXPECT_LT(m.positionAgeSec(1, 10.0), 0.0)
      << "but a status-only message is not a position";
}

TEST(TeamModelPositionAge, FirstHandPositionAgesOnTheLocalClock) {
  TeamModel m = makeModel();
  TeamModel::Observation o = msg(1, robotBit(0));
  o.have_position = true;
  o.x = 5.0; o.y = -2.0;
  ASSERT_EQ(m.observe(o, 100.0), "");

  EXPECT_NEAR(m.positionAgeSec(1, 100.0), 0.0, 1e-9);
  EXPECT_NEAR(m.positionAgeSec(1, 137.5), 37.5, 1e-9);
}

TEST(TeamModelPositionAge, AStatusOnlyUpdateDoesNotRefreshThePose) {
  // THE BUG THIS ACCESSOR EXISTS FOR. A peer that keeps talking while its pose
  // stops updating reads fresh on lastKnownAgeSec forever. A separation term
  // or a pursuit keyed on that would steer on a pose of any age and never see
  // a reason to doubt it.
  TeamModel m = makeModel();
  TeamModel::Observation with_pos = msg(1, robotBit(0));
  with_pos.have_position = true;
  with_pos.x = 5.0; with_pos.y = -2.0;
  ASSERT_EQ(m.observe(with_pos, 100.0), "");

  // 60 s later, same peer, still talking, but nothing about where it is.
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 160.0), "");

  EXPECT_NEAR(m.lastKnownAgeSec(1, 160.0), 0.0, 1e-9)
      << "we heard from it a moment ago";
  EXPECT_NEAR(m.positionAgeSec(1, 160.0), 60.0, 1e-9)
      << "but the pose we hold is a minute old";
  EXPECT_EQ(m.peer(1).position_x, 5.0) << "and it is still the old pose";
}

TEST(TeamModelPositionAge, RelayedPositionIsDatedWhenItWasHeardNotWhenItArrived) {
  // A gossiped pose is as old as the SENDER's last contact with the subject,
  // converted to our clock. Stamping it with our receipt time instead would
  // make a two-minute-old relayed pose look one tick old — which is exactly
  // the freshness bound a separation term is trusting.
  TeamModel m = makeModel();
  TeamModel::Observation o = gossipMsg(1, robotBit(0), {-1.0, 100.0, 40.0});
  o.gx = {0.0, 0.0, 12.0};
  o.gy = {0.0, 0.0, -3.0};
  o.gz = {0.0, 0.0, 0.0};
  o.have_gossip_pos = {0, 0, 1};
  ASSERT_EQ(m.observe(o, 500.0), "");

  ASSERT_TRUE(m.peer(2).have_position);
  // 100 - 40 = 60 s old on the sender's clock when it left; the same 60 s here.
  EXPECT_NEAR(m.positionAgeSec(2, 500.0), 60.0, 1e-9);
  EXPECT_NEAR(m.positionAgeSec(2, 530.0), 90.0, 1e-9);
}

TEST(TeamModelPositionAge, NeverNegativeWhenTheClockRunsBackwards) {
  TeamModel m = makeModel();
  TeamModel::Observation o = msg(1, robotBit(0));
  o.have_position = true;
  ASSERT_EQ(m.observe(o, 100.0), "");
  // A caller asking about an earlier instant must not get a negative age that
  // would pass a "fresher than max_age" test by being absurd.
  EXPECT_GE(m.positionAgeSec(1, 90.0), 0.0);
}

TEST(TeamModelPositionAge, OutOfRangeIdsAndSelfReadAsNoPosition) {
  TeamModel m = makeModel();
  EXPECT_LT(m.positionAgeSec(-1, 0.0), 0.0);
  EXPECT_LT(m.positionAgeSec(99, 0.0), 0.0);
  // Self is not special-cased to 0: the model never stores a position for
  // self, so "0 seconds old" would be a confident age for a pose that is not
  // there, and a consumer would happily use the (0, 0) it came with.
  EXPECT_LT(m.positionAgeSec(m.selfId(), 0.0), 0.0);
}

TEST(TeamModelPositionAge, ReconfiguringClearsIt) {
  TeamModel m = makeModel();
  TeamModel::Observation o = msg(1, robotBit(0));
  o.have_position = true;
  ASSERT_EQ(m.observe(o, 100.0), "");
  ASSERT_GE(m.positionAgeSec(1, 100.0), 0.0);

  ASSERT_EQ(m.configure(fleet3(), goodCfg()), "");
  EXPECT_LT(m.positionAgeSec(1, 100.0), 0.0);
}

// ===========================================================================
// mode — the three-level "what is this robot doing with the rest of its run"
// (generation 33, R4). Same two channels as `finished`, one more level, and
// MAX in place of OR.
// ===========================================================================

/// First-hand is authoritative and is therefore the one channel that can
/// LOWER the level. The case is a node that restarts mid-run: it genuinely
/// un-homes, and the robot itself is the only witness worth believing.
TEST(TeamModelMode, FirstHandIsAuthoritativeAndCanLower) {
  TeamModel m = makeModel();
  EXPECT_EQ(m.peer(1).mode, 0u) << "no evidence must read as EXPLORING";

  TeamModel::Observation o = msg(1, robotBit(0));
  o.mode = 1;                                   // HOMING
  ASSERT_EQ(m.observe(o, 10.0), "");
  EXPECT_EQ(m.peer(1).mode, 1u);

  o.mode = 0;                                   // it restarted
  ASSERT_EQ(m.observe(o, 20.0), "");
  EXPECT_EQ(m.peer(1).mode, 0u)
      << "a first-hand EXPLORING is the sender's own statement about itself "
         "and must overwrite, exactly as `finished` does";
}

/// Relay can only ever raise. A relayed EXPLORING is "the sender has no
/// evidence", never "that robot is still exploring", so it carries nothing
/// and must not overwrite a level already held.
TEST(TeamModelMode, RelayOnlyEverRaisesTheLevel) {
  TeamModel m = makeModel();

  TeamModel::Observation up = msg(1, robotBit(0));
  up.mode_gossip = {0u, 0u, 2u};                // robot 2 is DONE
  ASSERT_EQ(m.observe(up, 10.0), "");
  EXPECT_EQ(m.peer(2).mode, 2u);
  EXPECT_TRUE(m.peer(2).known)
      << "a relayed level is also evidence the robot exists";

  TeamModel::Observation down = msg(1, robotBit(0));
  down.mode_gossip = {0u, 0u, 1u};              // a staler second-hand read
  ASSERT_EQ(m.observe(down, 20.0), "");
  EXPECT_EQ(m.peer(2).mode, 2u) << "max-merge, so a lower relay is a no-op";

  down.mode_gossip = {0u, 0u, 0u};
  ASSERT_EQ(m.observe(down, 30.0), "");
  EXPECT_EQ(m.peer(2).mode, 2u) << "and a relayed zero carries no information";
}

/// NOT age-gated, for finished_gossip's reason exactly: a robot that is homing
/// or done stops moving and goes quiet, so its last-heard entry ages out of
/// gossip_max_age_sec precisely when its level starts to matter. Gating a
/// monotonic fact on freshness switches the relay off at that moment.
TEST(TeamModelMode, RelayIsNotAgeGated) {
  TeamModel m = makeModel();
  // Sender's own entry is its publish time; robot 2's is 600 s older, which is
  // five times gossip_max_age_sec and would drop a position outright.
  TeamModel::Observation o = gossipMsg(1, robotBit(0), {-1.0, 1000.0, 400.0});
  o.mode_gossip = {0u, 0u, 1u};
  ASSERT_EQ(m.observe(o, 50.0), "");
  EXPECT_EQ(m.peer(2).mode, 1u);
  EXPECT_FALSE(m.peer(2).have_position)
      << "fixture: the position half of the same message IS age-gated, which "
         "is what makes this a contrast rather than a coincidence";
}

/// The length guard refuses the whole message rather than truncating: a
/// longer array means the sender is running a different team definition, so
/// its ids address different robots than ours do.
TEST(TeamModelMode, OversizedModeGossipIsRefusedWholesale) {
  TeamModel m = makeModel();
  TeamModel::Observation o = msg(1, robotBit(0));
  o.mode = 2;
  o.mode_gossip = {0u, 0u, 1u, 1u};             // four entries, three robots
  EXPECT_NE(m.observe(o, 10.0), "");
  EXPECT_EQ(m.peer(1).mode, 0u)
      << "refused means nothing from the message is believed, including the "
         "sender's own first-hand level";
}

// ---------------------------------------------------------------------------
// R1 instrumentation (generation 33) — acquire_sec / held_sec
//
// Timing the detector, never changing it: every assertion below is about WHEN
// a transition was reported, and none of them is about whether `direct` was
// right. The one design property they exist to pin is that both readings are
// differences of PACKET stamps rather than tick stamps, so the tests drive the
// tick at times deliberately offset from the arrivals. A measurement that
// picked up the tick clock would read those offsets and fail.
// ---------------------------------------------------------------------------

TEST(TeamModelR1, AcquisitionIsMeasuredFromTheFirstOneWayPacket) {
  TeamModel m = makeModel();

  // t=100: heard, but the peer's mask does not name us. One-way — the period
  // this measurement is defined over opens here.
  ASSERT_EQ(m.observe(msg(1, robotBit(2)), 100.0), "");
  m.tick(100.4);
  ASSERT_TRUE(m.peer(1).heard_one_way);
  EXPECT_LT(m.peer(1).acquire_sec, 0.0) << "nothing has been acquired yet";

  // t=102: still one-way. The anchor must NOT move to this packet.
  ASSERT_EQ(m.observe(msg(1, robotBit(2)), 102.0), "");
  m.tick(102.4);
  EXPECT_LT(m.peer(1).acquire_sec, 0.0);

  // t=103: the mask names us. Handshake complete.
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 103.0), "");
  m.tick(103.9);
  ASSERT_TRUE(m.peer(1).direct);
  EXPECT_DOUBLE_EQ(m.peer(1).acquire_sec, 3.0)
      << "measured 103 - 100 from the packets; 103.9 - 100.4 would be the "
         "tick clock and 103 - 102 would be the wrong anchor";

  // ONE-SHOT. The link is still up and nothing has transitioned, so the next
  // tick must report no event -- otherwise a reader counting non-negative
  // readings counts one acquisition per tick for as long as the link holds.
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 104.0), "");
  m.tick(104.5);
  ASSERT_TRUE(m.peer(1).direct);
  EXPECT_LT(m.peer(1).acquire_sec, 0.0) << "acquire_sec latched past its tick";
}

TEST(TeamModelR1, AnInstantHandshakeAcquiresInZero) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 100.0), "");
  m.tick(100.4);
  ASSERT_TRUE(m.peer(1).direct);
  EXPECT_DOUBLE_EQ(m.peer(1).acquire_sec, 0.0)
      << "the first packet of the run completed the handshake, so the one-way "
         "period was empty -- 0, not -1 and not the tick offset";
}

TEST(TeamModelR1, HoldIsReportedOnlyWhenThePeerIsStillAudible) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 100.0), "");
  m.tick(100.1);
  ASSERT_TRUE(m.peer(1).direct);

  // The peer keeps talking and stops naming us: the §10 failure on a link
  // that was up. That is the break this measurement is defined over.
  ASSERT_EQ(m.observe(msg(1, robotBit(2)), 107.5), "");
  m.tick(107.9);
  ASSERT_FALSE(m.peer(1).direct);
  ASSERT_TRUE(m.peer(1).heard_one_way);
  EXPECT_DOUBLE_EQ(m.peer(1).held_sec, 7.5);
  EXPECT_LT(m.peer(1).acquire_sec, 0.0) << "a break is not an acquisition";
}

TEST(TeamModelR1, AHandshakeLostToSilenceReportsNoHoldAtAll) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 100.0), "");
  m.tick(100.1);
  ASSERT_TRUE(m.peer(1).direct);

  // Nothing arrives. The TTL expires and the link drops -- but this is a
  // DELIVERY failure, and the whole §2.7 finding is that it has a different
  // cause from a handshake that broke. Averaging it into the handshake
  // statistic would hide exactly the distinction Part 0 exists to draw.
  m.tick(106.0);
  ASSERT_FALSE(m.peer(1).direct);
  ASSERT_FALSE(m.peer(1).heard_one_way) << "fixture: this must be silence";
  EXPECT_LT(m.peer(1).held_sec, 0.0)
      << "a link that ended because the packets stopped has no closing packet "
         "to stamp and must not be reported as a hold";
}

TEST(TeamModelR1, ReacquisitionAfterABreakDoesNotChargeThePrecedingHold) {
  TeamModel m = makeModel();

  // Up at t=100, broken while audible at t=110: a 10 s hold.
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 100.0), "");
  m.tick(100.1);
  ASSERT_EQ(m.observe(msg(1, robotBit(2)), 110.0), "");
  m.tick(110.1);
  ASSERT_DOUBLE_EQ(m.peer(1).held_sec, 10.0) << "fixture";

  // Re-acquired at t=112, WITHOUT the packets ever stopping, so the receiving
  // run is unbroken and its first packet is still the one at t=100.
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 112.0), "");
  m.tick(112.1);
  ASSERT_TRUE(m.peer(1).direct);
  EXPECT_DOUBLE_EQ(m.peer(1).acquire_sec, 2.0)
      << "measured from the break at 110, not from the run's first packet at "
         "100 -- charging the hold to the next acquisition would report a "
         "12 s handshake for a link that took 2";
}

TEST(TeamModelR1, ASilenceResetsTheAnchorSoTheNextRunMeasuresItself) {
  TeamModel m = makeModel();

  // One-way from t=100, then total silence past the TTL.
  ASSERT_EQ(m.observe(msg(1, robotBit(2)), 100.0), "");
  m.tick(100.1);
  m.tick(200.0);
  ASSERT_FALSE(m.peer(1).heard_one_way) << "fixture: TTL must have expired";

  // A new run, 100 s later, acquiring in 1 s.
  ASSERT_EQ(m.observe(msg(1, robotBit(2)), 300.0), "");
  m.tick(300.1);
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 301.0), "");
  m.tick(301.1);
  ASSERT_TRUE(m.peer(1).direct);
  EXPECT_DOUBLE_EQ(m.peer(1).acquire_sec, 1.0)
      << "the dead 200 s between runs is not acquisition latency";
}
