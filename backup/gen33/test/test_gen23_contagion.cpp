/// @file test_gen23_contagion.cpp
/// @brief Generation 23's contagious arming: one robot's broken view of the
///        team arms the whole team, in one hop, without deadlocking.
///
/// THE PROBLEM THIS GENERATION CLOSES. Take an A—B—C bridge: A and B cannot
/// hear each other, C hears both. Through generation 22 the appointment
/// predicate was each robot's own `teamComplete(<its own peer count>,
/// rendezvous_expected_peers_)`, so A armed, B armed, and C — whose own count
/// is full — did not. A and B drove to the meeting point and waited for a robot
/// that was still off exploring. The directive is literal: if any robot is
/// disconnected, ALL robots go.
///
/// WHY A LOCAL FIX COULD NOT WORK, and this is the load-bearing fact. Every
/// peer count this node can compute — `livePeerCount`, the raw claim table, and
/// `accountedPeerCount`, which widens it to TeamWorld's `direct` handshake and
/// to peers that announced `finished` (NOT to two-hop-closed peers: it reads
/// `Peer::direct`, never `inComms()`, so the closure never enters the count) —
/// describes what THIS robot can reach. None of them describes what a
/// PEER can reach. Coordination fills its claim table from `onIntent()` alone;
/// the comms emulator forwards nothing; RobotIntent carries no peer list, no
/// mask, no hop count. So C cannot compute A's view of the team from anything
/// C already has, at any width. The bit has to be on the wire.
///
/// AND A CLOSURE-BASED FIX WOULD HAVE BEEN WRONG IN THE OTHER DIRECTION.
/// TeamModel's two-hop closure says A—B—C is connected, which is a statement
/// about COMMS and tempts the reading "so the maps are flowing, nobody needs to
/// meet". They are not. There is NO MAP RELAY: dscovox_node subscribes to its
/// peers' raw `scovox_node/scovox_bin` and publishes only fused products for
/// LOCAL consumers, so nothing forwards A's voxels to B through C. In the
/// bridge, A and B are genuinely partitioned on the thing the rendezvous
/// exists to exchange. A predicate keyed on closure would SUPPRESS a meeting
/// that is needed. (The node comment that used to assert "the map still flows
/// A->C->B" was checked and is FALSE. It has been removed. Do not restore it.)
///
/// ONE HOP IS ENOUGH, AT EVERY N AND EVERY TOPOLOGY, and this is why there is
/// no relay of the bit and no second round. Partition the UNFINISHED robots by
/// each robot's OWN read:
///
///   * a robot whose own read is BROKEN arms from its own read, no message
///     needed;
///   * a robot whose own read is WHOLE has heard every unfinished peer
///     first-hand inside one TTL — that is what the count's two reachability
///     channels mean — so it receives their announced bits itself.
///
/// Every unfinished robot therefore arms iff SOME unfinished robot's own
/// break-bit is true — unanimous, in one hop, with no forwarding.
///
/// THE QUANTIFIER IS OVER UNFINISHED ROBOTS AND THAT IS NOT A HEDGE. A finished
/// peer counts toward the read through the third channel with no contact at all
/// — `finished` relays and never clears — so WHOLE does not imply "receiving
/// from all N-1 directly". It costs nothing: a finished robot's bit is thrown
/// away at both ends on purpose (publishTeamWorld forces team_incomplete=false
/// once state_ == DONE, and peerReportsTeamBreak skips any peer with `finished`
/// set), so the robot whose bit could go unseen is precisely the robot whose
/// bit nobody would read. TeamWorld.msg states the one residual window, where a
/// peer's intents arrive inside coord_claim_ttl_sec but its TeamWorld does not
/// arrive inside direct_ttl_sec. Contrast `in_range_mask`, whose
/// closure is exactly two hops (team_model.cpp: "A—B—C is covered…, A—B—C—D is
/// not"): that bound does not apply here because the bit never needs to travel
/// further than from a broken robot to a whole one, and a whole robot is
/// adjacent to everybody.
///
/// THE DEADLOCK THE DESIGN AVOIDS, and the single thing most likely to be
/// "simplified" back in. The announced bit is the producer's OWN FIRST-HAND
/// `teamComplete` READ. It is NOT the producer's derived armed state. If it
/// were the armed state: C arms because A announced armed, C announces armed,
/// A sees C armed, and now neither can ever clear — a permanent appointment in
/// a fully connected team. The clear is asymmetric ON PURPOSE: A clears on its
/// own read recovering, C clears when no peer is announcing a break. GROUP B
/// pins the producer side of that; GROUP A pins the consumer side (TeamModel
/// stores the bit against the SENDER and the gossip path never touches it, so
/// a claim cannot re-enter the fleet wearing a different robot's name).
///
/// THE FIVE-SITE RULE IS THE REST OF THE FILE. Generations 18 and 19 are both
/// the same defect with opposite polarity, and both came from arming on one
/// predicate and releasing on another:
///
///   gen 18 — armed weak, cleared strict: the latch never cleared at N>=3 and
///            the arm sat inert for 70% of a 604 s cell.
///   gen 19 — armed strict, released weak: the arm NEVER STOPPED, ~25 spurious
///            regroups per 3000 s cell. N=2 was unaffected, so it would have
///            reached analysis disguised as a team-size effect.
///
/// The rule is: ARM ON !P, END ON P, FOR ONE P. Generation 23 changes what P
/// is — from teamComplete to teamSettled — which means it changes it at all
/// five sites or it is gen 18 again:
///
///   1. the arm                          doPlan
///   2. the supersede                    doPlan
///   3. the appointment classifier       transitionTo
///      (+ the reconnect_end classifier beside it)
///   4. the barrier release              doReturnSync / doReturnNav
///   5. the rendezvous_spent_ release    heartbeatTick
///
/// GROUP D asserts all five together, which is the assertion this file exists
/// for. GROUP E asserts the equally important negative: pursuit, the presence
/// clock, the event field and the CSV column DELIBERATELY stay on teamComplete.
/// Pursuit is per-robot by definition — a robot chases the peer IT cannot hear
/// — and the metric columns describe this robot's own connectivity, which is
/// what every banked analysis has read them as. Widening those would silently
/// redefine columns across generations.
///
/// WHAT THIS CHANGE RISKS, stated so the smoke knows what to look for. The
/// appointment barrier's wait is UNBOUNDED (rendezvous_appointment_wait_sec
/// defaults to 0 = no cap) and generation 23 makes its release STRICTER. A
/// robot that arrives, sees everyone, but keeps hearing a peer announce a break
/// holds. Two things bound it: at the meeting point everyone is co-located, so
/// the announcing peer's own read heals; and peerReportsTeamBreak() exempts
/// FINISHED peers. GROUP C pins the exemption; the smoke's job is a waited_sec
/// distribution that ENDS.
///
/// THE EXEMPTION ON ITS OWN IS NOT ENOUGH, AND AN EARLIER VERSION OF THIS
/// COMMENT CLAIMED IT WAS. It said the exemption "removes the one case that
/// could hold indefinitely". It does not, because the exemption is applied BY
/// THE LISTENER, to a bit the listener must have heard FIRST-HAND. Presence is
/// counted on three channels (the claim table, TeamWorld's `direct` handshake,
/// and `finished` — see peerAccounted; NOT the two-hop closure, which
/// accountedPeerCount never consults) and the contagion is read on one, so a
/// robot that never hears the finisher keeps announcing a break that nobody can
/// exempt:
///
///   N=3. A latches done while out of contact with B. They were adjacent at
///   spawn, so B holds `finished=false` for A and nothing ages it out — the
///   field has no TTL by design. B reads 1 of 2 peers forever and announces
///   team_incomplete forever. C hears both, exempts A correctly, and is held
///   open by B's complaint. Both drive to the agreed cell and wait at a barrier
///   with no cap that neither can release. The cell runs to the duration cap.
///
/// GROUP G closes it by RELAYING `finished`, which is sound for this one field
/// and for no other on the wire: it is MONOTONIC and it is a statement a robot
/// makes ABOUT ITSELF, so a relayed copy can never contradict a first-hand one
/// and can never feed back. `team_incomplete` is neither, which is why
/// GossipNeverCarriesTheBit is still true and must stay true.
///
/// TWO KINDS OF TEST IN ONE FILE, on purpose.
///
///   GROUPS A: REAL CALLS. team_model.cpp IS in explo_planner_lib's source
///   list, so the wire field's storage semantics — first-hand only, no TTL of
///   its own, cleared by the sender and by nobody else — are executed, not
///   scanned. This is the half of the design where a mistake is silent,
///   because a relayed claim still produces a plausible-looking armed team.
///
///   GROUPS B-F: SOURCE SCANS, same technique and same stated limit as
///   test_gen20_rendezvous.cpp, test_gen21_latched_hold.cpp and
///   test_gen22_start_hold.cpp. Every node invariant lives in
///   explo_planner_node.cpp, which is NOT in that source list — an explicit
///   list of files in CMakeLists.txt, not a glob — so nothing links it and no
///   test can call it. A scan shows a predicate is PRESENT and ORDERED; it
///   cannot show it is REACHED.
///
///   AND DO NOT READ "not in the list" AS "one line away from testable". The
///   list edit is one line; it does not make the node callable. This TU also
///   defines main(), so the first test that references a node symbol drags
///   that object out of the archive and collides with gtest_main's main().
///   Reaching these invariants behaviourally means splitting main() and the
///   node class apart first — a real change, not a build-file tweak.
///
/// stripComments() IS NOT OPTIONAL, and this file is a sharper case for it than
/// any before. The node's own comments at the sites under test spell out the
/// contrast longhand — doPursue's block literally reads "teamComplete AND NOT
/// teamSettled, DELIBERATELY", and GROUP E asserts doPursue contains ZERO
/// occurrences of `teamSettled(`. Over raw text that assertion fails against
/// correct code; over stripped text it is a statement about the code. The
/// inverse failure is the one that actually ships: a cleanup edit that deletes
/// the calls and leaves the comments would pass a raw-text scan on a node with
/// the whole generation removed.
/// Moved comments: doc/test_gen23_contagion_notes.md

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "explo_planner/team_model.hpp"

using explo_planner::FleetIdentity;
using explo_planner::makeFleetIdentity;
using explo_planner::robotBit;
using explo_planner::TeamModel;

// ===========================================================================
// Fixtures for the executable half.
// ===========================================================================

namespace {

TeamModel::Config goodCfg() {
  TeamModel::Config c;
  c.direct_ttl_sec     = 5.0;
  c.closure_enabled    = true;
  c.gossip_max_age_sec = 120.0;
  return c;
}

/// Three robots, this one is id 0. Three is the smallest fleet in which the
/// bridge this generation exists for is expressible at all.
FleetIdentity fleet3() {
  return makeFleetIdentity({"atlas", "bestla", "cerd"}, "atlas");
}

/// Four robots, this one is id 0 — the hub of GROUP A's star.
FleetIdentity fleet4() {
  return makeFleetIdentity({"atlas", "bestla", "cerd", "dvalin"}, "atlas");
}

TeamModel makeModel(const FleetIdentity& fleet = fleet3(),
                    const TeamModel::Config& cfg = goodCfg()) {
  TeamModel m;
  EXPECT_EQ(m.configure(fleet, cfg), "") << "fixture failed to configure";
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

/// The same, announcing the sender's own first-hand "the team is not whole".
TeamModel::Observation brokenMsg(int sender, uint32_t mask) {
  TeamModel::Observation o = msg(sender, mask);
  o.team_incomplete = true;
  return o;
}

/// A message carrying third-party gossip. heard is indexed by fleet id, in
/// times on the sender's clock; the sender's own entry is its publish time.
/// Duplicated from test_team_model.cpp on purpose, not shared.
/// (notes: fixture-gossip-msg)
TeamModel::Observation gossipMsg(int sender, uint32_t mask,
                                 const std::vector<double>& heard) {
  TeamModel::Observation o = msg(sender, mask);
  o.last_heard_sec = heard;
  return o;
}

}  // namespace

// ===========================================================================
// GROUP A. THE WIRE FIELD, EXECUTED. What TeamModel does with the bit.
// ===========================================================================

/// Pins that team_incomplete is stored against the sender that announced it and
/// against no other peer, so an arm can be attributed.
/// (notes: wire-bit-stored-against-sender)
TEST(Gen23ContagionWire, TheBitIsStoredAgainstTheSenderThatMadeIt) {
  TeamModel m = makeModel();
  // cerd (2) can hear us and says its own view of the team is broken. bestla
  // (1) can hear us and says its view is fine.
  ASSERT_EQ(m.observe(brokenMsg(2, robotBit(0)), 100.0), "");
  ASSERT_EQ(m.observe(msg(1, robotBit(0)), 100.0), "");
  m.tick(100.0);

  EXPECT_TRUE(m.peer(2).team_incomplete);
  EXPECT_FALSE(m.peer(1).team_incomplete)
      << "one peer's announcement has landed on another peer's record; the "
         "node would then arm citing a robot that never said anything";
  EXPECT_TRUE(m.peer(2).direct);
  EXPECT_TRUE(m.peer(1).direct);
}

/// observe() stores team_incomplete first-hand only; the gossip block must not
/// touch it, or a relayed break returns to its sender and the appointment never
/// clears. Pins both directions: no smear, no clear.
/// (notes: wire-gossip-never-carries-bit)
TEST(Gen23ContagionWire, GossipNeverCarriesTheBit) {
  {
    TeamModel m = makeModel();
    // bestla (1) hears us, says ITS OWN view is broken, and relays what it
    // knows about cerd (2). Nothing here is a statement by cerd.
    TeamModel::Observation relay = gossipMsg(1, robotBit(0), {-1.0, 100.0, 90.0});
    relay.team_incomplete = true;
    ASSERT_EQ(m.observe(relay, 1000.0), "");
    m.tick(1000.0);

    EXPECT_TRUE(m.peer(1).team_incomplete) << "the relay's own bit is first-hand";
    EXPECT_TRUE(m.peer(2).known) << "the gossip did land as knowledge";
    EXPECT_FALSE(m.peer(2).team_incomplete)
        << "the relay's claim about the team has been re-attributed to the "
           "robot it was gossiping about. That is the deadlock TeamWorld.msg "
           "warns about, reached by relaying rather than by announcing the "
           "armed state.";
  }
  {
    TeamModel m = makeModel();
    // cerd (2) says so itself, first-hand, then goes quiet. bestla (1) then
    // gossips about cerd with a healthy view of its own.
    ASSERT_EQ(m.observe(brokenMsg(2, robotBit(0)), 100.0), "");
    m.tick(100.0);
    ASSERT_TRUE(m.peer(2).team_incomplete);

    ASSERT_EQ(m.observe(gossipMsg(1, robotBit(0), {-1.0, 200.0, 199.0}), 200.0),
              "");
    m.tick(200.0);
    EXPECT_TRUE(m.peer(2).team_incomplete)
        << "a third party's gossip has cleared a claim only cerd can make or "
           "withdraw. Liveness is what expires this bit, not hearsay — see "
           "TheBitHasNoTtlOfItsOwn.";
  }
}

/// The stored team_incomplete survives TTL expiry on purpose; direct and
/// heard_one_way both go false, and peerReportsTeamBreak() skips a peer with
/// neither. Do not clear the bit on expiry. (notes: wire-bit-no-ttl-liveness)
TEST(Gen23ContagionWire, TheBitHasNoTtlOfItsOwnSoLivenessIsTheConsumersJob) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(brokenMsg(2, robotBit(0)), 100.0), "");
  m.tick(100.0);
  ASSERT_TRUE(m.peer(2).team_incomplete);
  ASSERT_TRUE(m.peer(2).direct);

  // Past direct_ttl_sec with nothing further from cerd.
  m.tick(110.0);
  EXPECT_FALSE(m.peer(2).direct);
  EXPECT_FALSE(m.peer(2).heard_one_way)
      << "if either liveness flag survived the TTL the node's gate would keep "
         "believing a peer that has been silent for a minute";
  EXPECT_TRUE(m.peer(2).team_incomplete)
      << "the stored claim is expected to survive; it is the liveness flags "
         "above, not this field, that stop the node from acting on it";
}

/// A peer we hear but that cannot hear us still delivers its bit: it is the
/// robot most in need of the meeting, so the consumer must not gate on direct
/// alone. (notes: wire-one-way-carries-bit)
TEST(Gen23ContagionWire, OneWayContactStillCarriesTheBit) {
  TeamModel m = makeModel();
  // cerd's mask does not name us: we hear it, it does not hear us.
  ASSERT_EQ(m.observe(brokenMsg(2, 0u), 100.0), "");
  m.tick(100.0);

  EXPECT_FALSE(m.peer(2).direct) << "a link is mutual or it is not a link";
  EXPECT_TRUE(m.peer(2).heard_one_way);
  EXPECT_TRUE(m.peer(2).team_incomplete)
      << "the §10 case must remain readable; this is the robot most in need of "
         "the rendezvous and the least able to ask for one";
}

/// The sender's false clears its earlier true (assignment, not a sticky OR).
/// The asymmetric clear depends on it; a latch would leave a permanent
/// appointment. (notes: wire-sender-can-withdraw)
TEST(Gen23ContagionWire, TheSenderCanWithdrawItsOwnClaim) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(brokenMsg(2, robotBit(0)), 100.0), "");
  m.tick(100.0);
  ASSERT_TRUE(m.peer(2).team_incomplete);

  ASSERT_EQ(m.observe(msg(2, robotBit(0)), 101.0), "");
  m.tick(101.0);
  EXPECT_FALSE(m.peer(2).team_incomplete)
      << "the bit latched on. A peer that recovers cannot then release the "
         "team, and the appointment stands for the rest of the run.";
}

/// N=4 star: the hub's own read is complete, each leaf announces a break, and
/// the hub receives all three first-hand. One hop suffices at any N: no relay,
/// no second round. (notes: wire-hub-one-hop-n4)
TEST(Gen23ContagionWire, TheHubHearsEveryLeafsBreakFirstHandAtNFour) {
  TeamModel m = makeModel(fleet4());
  ASSERT_EQ(m.observe(brokenMsg(1, robotBit(0)), 100.0), "");
  ASSERT_EQ(m.observe(brokenMsg(2, robotBit(0)), 100.0), "");
  ASSERT_EQ(m.observe(brokenMsg(3, robotBit(0)), 100.0), "");
  m.tick(100.0);

  for (int id = 1; id <= 3; ++id) {
    EXPECT_TRUE(m.peer(id).direct) << "leaf " << id << " is not a direct peer";
    EXPECT_TRUE(m.peer(id).team_incomplete)
        << "leaf " << id << "'s announcement did not arrive first-hand; if the "
           "hub cannot read it the fleet needs a relay and one hop stops being "
           "enough";
  }
}

/// AND IT IS CLEARED BY A RECONFIGURE, LIKE EVERY OTHER PEER FIELD.
///
/// A stale `true` surviving a fleet change would arm a team that has never
/// exchanged a message.
TEST(Gen23ContagionWire, ReconfiguringClearsIt) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(brokenMsg(2, robotBit(0)), 100.0), "");
  m.tick(100.0);
  ASSERT_TRUE(m.peer(2).team_incomplete);

  ASSERT_EQ(m.configure(fleet3(), goodCfg()), "");
  EXPECT_FALSE(m.peer(2).team_incomplete);
  EXPECT_FALSE(m.peer(2).known);
}

// ===========================================================================
// Source-scan helpers for GROUPS B-F.
// ===========================================================================

namespace {

/// Returns the source with // and /* */ comments removed, string literals
/// preserved. Duplicated in each scan test on purpose: a shared header would
/// let one node mutation break all of them.
/// (notes: scan-strip-comments-duplicated)
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

/// The body of a function definition, by brace matching from its signature.
/// Returns "" when the signature is not found, which every caller ASSERTs on.
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

/// Runs of spaces and tabs collapsed to one space. For assertions where the
/// declaration's column alignment would otherwise decide whether the scan
/// matches anything — a source scan that quietly matches nothing is a check
/// that has stopped checking, which is worse than not having written it.
std::string squeezeSpaces(const std::string& text) {
  std::string out;
  out.reserve(text.size());
  bool prev_space = false;
  for (const char c : text) {
    const bool sp = (c == ' ' || c == '\t');
    if (sp) {
      if (!prev_space) out.push_back(' ');
    } else {
      out.push_back(c);
    }
    prev_space = sp;
  }
  return out;
}

}  // namespace

// ===========================================================================
// GROUP B. THE PRODUCER ANNOUNCES ITS OWN READ, NOT ITS ARMED STATE.
// ===========================================================================

/// publishTeamWorld announces this robot's own read over accountedPeerCount.
/// Never appointment_armed_ or peerReportsTeamBreak(), which close a deadlock
/// cycle; appointment_manoeuvre_ is banned from this field only.
/// (notes: producer-own-read-not-armed)
TEST(Gen23ContagionProducer, AnnouncesItsOwnReadAndNeverItsArmedState) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string pub =
      functionBody(text, "void ExploPlannerNode::publishTeamWorld()");
  ASSERT_FALSE(pub.empty())
      << "no publishTeamWorld() definition — it has been renamed or reshaped "
         "and this test has stopped testing anything";

  EXPECT_NE(pub.find("m.team_incomplete ="), std::string::npos)
      << "publishTeamWorld no longer sets team_incomplete. The wire field is "
         "the only channel there is: the peer count is computed from links "
         "this robot holds and nothing relays a peer list, so a robot that "
         "does not announce its own break cannot be heard by the robot that "
         "is still connected to everyone.";
  EXPECT_NE(pub.find("!teamComplete("), std::string::npos)
      << "the announced bit is no longer the negation of this robot's own "
         "teamComplete read";
  EXPECT_NE(pub.find("accountedPeerCount("), std::string::npos)
      << "the announced bit is not being computed from the accounted peer "
         "count. On bare livePeerCount a finished peer that parked out of "
         "range is missing forever, so this robot announces a break it can "
         "never withdraw and the contagion holds the whole team at a barrier "
         "with no cap.";

  EXPECT_EQ(countOf(pub, "appointment_armed_"), 0)
      << "publishTeamWorld reads the appointment state. Announcing the derived "
         "armed state instead of the robot's own read deadlocks: C arms from "
         "A, C announces armed, A holds because C is armed, and neither can "
         "ever clear. See TeamWorld.msg.";
  const size_t ti = pub.find("m.team_incomplete =");
  const size_t ti_end = pub.find(';', ti);
  ASSERT_NE(ti_end, std::string::npos) << "unterminated team_incomplete assignment";
  const std::string ti_stmt = pub.substr(ti, ti_end - ti);
  EXPECT_EQ(countOf(ti_stmt, "appointment_manoeuvre_"), 0)
      << "the announced read is now a function of the appointment state — the "
         "same cycle as appointment_armed_, one state further along. The bit "
         "other robots arm on has to be this robot's own view of the team and "
         "nothing else.";
  EXPECT_EQ(countOf(pub, "peerReportsTeamBreak("), 0)
      << "the announced bit has been made a function of what PEERS said, which "
         "is the relay this design exists to avoid — the claim would re-enter "
         "the fleet wearing this robot's name";
}

/// With zero expected peers teamComplete is false, so the
/// rendezvous_expected_peers_ guard must lead and short-circuit it: an inert or
/// single-robot configuration announces no break.
/// (notes: producer-inert-expectation)
TEST(Gen23ContagionProducer, AnnouncesNothingWithoutAConfiguredExpectation) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string pub =
      functionBody(text, "void ExploPlannerNode::publishTeamWorld()");
  ASSERT_FALSE(pub.empty()) << "no publishTeamWorld() definition";

  const size_t at = pub.find("m.team_incomplete =");
  ASSERT_NE(at, std::string::npos);
  const size_t guard = pub.find("rendezvous_expected_peers_ > 0", at);
  const size_t negate = pub.find("!teamComplete(", at);
  ASSERT_NE(guard, std::string::npos)
      << "the inert-configuration guard is gone. With expected_peers = 0, "
         "teamComplete() returns false, so this robot would announce a "
         "permanent break it can never withdraw.";
  ASSERT_NE(negate, std::string::npos);
  EXPECT_LT(guard, negate)
      << "the guard no longer leads the expression. It has to short-circuit "
         "the teamComplete term, not sit downstream of it.";
}

/// THE CONSUMER COPIES IT VERBATIM INTO THE OBSERVATION.
///
/// The one line that connects the wire to GROUP A. Without it every assertion
/// above is about a field nothing ever writes.
TEST(Gen23ContagionProducer, TheConsumerCopiesTheBitIntoTheObservation) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string drain =
      functionBody(text, "void ExploPlannerNode::drainTeamWorld()");
  ASSERT_FALSE(drain.empty()) << "no drainTeamWorld() definition";

  EXPECT_NE(drain.find("o.team_incomplete = msg.team_incomplete;"),
            std::string::npos)
      << "drainTeamWorld no longer copies the bit through. TeamModel would "
         "then hold false for every peer forever and the whole generation is "
         "inert — which looks exactly like generation 22 and passes every "
         "smoke gate.";
}

// ===========================================================================
// GROUP C. peerReportsTeamBreak(): liveness, the finished exemption, and
//          nothing else.
// ===========================================================================

/// peerReportsTeamBreak() believes only a peer it receives from now, direct or
/// heard_one_way. The stored bit outlives the TTL, so this gate prevents a
/// permanent appointment; direct alone drops the one-way case.
/// (notes: predicate-liveness-gate)
TEST(Gen23ContagionPredicate, BelievesOnlyAPeerItIsCurrentlyReceivingFrom) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn =
      functionBody(text, "bool ExploPlannerNode::peerReportsTeamBreak() const");
  ASSERT_FALSE(fn.empty())
      << "no peerReportsTeamBreak() definition — the contagion predicate has "
         "been renamed or removed";

  EXPECT_NE(fn.find("if (!p.direct && !p.heard_one_way) continue;"),
            std::string::npos)
      << "the liveness gate is gone or has changed shape. Without it a robot "
         "that announced a break and then dropped off the air holds the team "
         "armed forever off a message nobody can refresh; with `direct` alone "
         "the one-way peer this exists to catch is silently skipped.";
  EXPECT_NE(fn.find("if (p.team_incomplete) return true;"), std::string::npos)
      << "the predicate no longer reads the peer's announced bit";
  EXPECT_NE(fn.find("if (id == team_model_.selfId()) continue;"),
            std::string::npos)
      << "self is no longer skipped. Reading our own slot makes this robot its "
         "own contagion source, which is at best a redundant restatement of "
         "teamComplete and at worst a self-latch.";
  EXPECT_NE(fn.find("if (!team_model_.configured()) return false;"),
            std::string::npos)
      << "the unconfigured guard is gone; peer() would be indexed into an "
         "empty vector";

  EXPECT_EQ(countOf(fn, "inComms("), 0)
      << "the predicate has been keyed on transitive closure. Closure is a "
         "statement about COMMS, and there is no map relay — in an A-B-C "
         "bridge A and B are genuinely partitioned on the thing the meeting "
         "exists to exchange, so a closure-keyed predicate SUPPRESSES a "
         "needed rendezvous.";
  EXPECT_EQ(countOf(fn, "last_known_sec"), 0)
      << "the predicate is reading gossip freshness. Gossip never carries this "
         "bit (see GossipNeverCarriesTheBit); reading a gossip field here "
         "means it is being reconstructed from a relay.";
}

/// peerReportsTeamBreak() skips a finished peer, or one at MODE_HOMING or
/// above, before reading its bit: it is not coming to the meeting and the
/// barrier wait is unbounded. Arm and release share this function.
/// (notes: predicate-finished-homing-exempt)
TEST(Gen23ContagionPredicate, ExemptsAFinishedPeer) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn =
      functionBody(text, "bool ExploPlannerNode::peerReportsTeamBreak() const");
  ASSERT_FALSE(fn.empty()) << "no peerReportsTeamBreak() definition";

  const size_t exempt = fn.find("if (p.finished ||");
  const size_t homing =
      fn.find("p.mode >= explo_planner_msgs::msg::TeamWorld::MODE_HOMING");
  const size_t read   = fn.find("if (p.team_incomplete) return true;");
  ASSERT_NE(exempt, std::string::npos)
      << "the finished-peer exemption is gone. The appointment barrier has no "
         "cap (rendezvous_appointment_wait_sec defaults to 0), so a parked "
         "robot that can never close one distant pair now holds the whole team "
         "to the duration cap and censors the cell.";
  ASSERT_NE(homing, std::string::npos)
      << "the HOMING half of the exemption is gone. A robot driving home is "
         "not coming to the meeting either, and it reads finished=false for "
         "the whole drive — so the barrier holds on it for exactly as long as "
         "the return leg takes.";
  ASSERT_NE(read, std::string::npos);
  EXPECT_LT(exempt, read)
      << "the exemption no longer precedes the read, so a finished peer's bit "
         "is believed before it is skipped";
  EXPECT_LT(homing, read)
      << "the homing half no longer precedes the read";
}

// ===========================================================================
// GROUP D. THE FIVE SITES MOVE AS ONE. This is what the file is for.
// ===========================================================================

/// teamSettled is teamComplete and no peer reporting a break, defined once so
/// the five call sites cannot drift apart. (notes: five-sites-team-settled-def)
TEST(Gen23ContagionFiveSites, TeamSettledIsTeamCompletePlusTheContagion) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn =
      functionBody(text, "bool ExploPlannerNode::teamSettled(int live_peers) const");
  ASSERT_FALSE(fn.empty()) << "no teamSettled() definition";

  EXPECT_NE(fn.find("teamComplete(live_peers, rendezvous_expected_peers_)"),
            std::string::npos)
      << "teamSettled no longer contains this robot's own read. Contagion is "
         "an ADDITION to the generation-22 predicate, not a replacement: a "
         "robot that cannot hear its peers must still arm on its own account.";
  EXPECT_NE(fn.find("!peerReportsTeamBreak()"), std::string::npos)
      << "teamSettled no longer consults its peers, which is generation 22 "
         "with extra steps — the C of the A-B-C bridge stops arming";
}

/// The arm fires on the first sample of !teamSettled; the supersede needs
/// teamSettled held for reconnect_release_confirm_sec via dwellHeld. Never
/// dwellConfirmed here: heartbeatTick is the window's only writer.
/// (notes: five-sites-arm-supersede)
TEST(Gen23ContagionFiveSites, TheArmAndTheSupersedeReadOnePredicate) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string plan = functionBody(text, "void ExploPlannerNode::doPlan()");
  ASSERT_FALSE(plan.empty()) << "no doPlan() definition";

  EXPECT_NE(plan.find("if (!teamSettled(live_for_arm)) {"), std::string::npos)
      << "the arm is no longer on !teamSettled. On bare teamComplete this is "
         "generation 22: in an A-B-C bridge the bridging robot never arms, A "
         "and B drive to a meeting point, and nobody else comes.";
  EXPECT_NE(plan.find("dwellHeld(teamSettled(live_for_supersede),"),
            std::string::npos)
      << "the supersede is no longer a dwelt read of teamSettled. Arming on one "
         "predicate and cancelling on another IS the generation-18/19 defect; "
         "whichever of the two is weaker decides whether the appointment never "
         "fires or never stops. Cancelling on an UNDWELT read is the same "
         "defect by the other axis: same predicate, one sample against six "
         "seconds, so a flicker cancels a meeting the peer is driving to.";
  // The arm must NOT acquire one. It is the asymmetry that makes the pair
  // correct, and "make them symmetric" is the obvious wrong repair for anyone
  // reading only the line above.
  EXPECT_EQ(plan.find("dwellHeld(!teamSettled(live_for_arm)"),
            std::string::npos)
      << "the ARM has grown a dwell. It must fire on the first sample: a "
         "confirm window here is a departure delay, on the one side of the "
         "pair where late is the harmful direction.";
  // And it must not be the mutating twin. dwellConfirmed here would give this
  // site its own frozen window and put the two answers back out of step.
  EXPECT_EQ(plan.find("dwellConfirmed("), std::string::npos)
      << "doPlan writes a dwell window. The team-back window has exactly one "
         "writer, heartbeatTick, because a window ticked from a branch this "
         "rarely entered freezes rather than decays and then fires on one "
         "reading.";
}

/// The arm's reason is separation when this robot's own read fails,
/// peer-separation when it hears everyone but a peer announced a break. The
/// selector must be teamComplete: teamSettled is false in both.
/// (notes: five-sites-arm-reason-string)
TEST(Gen23ContagionFiveSites, TheArmRecordsWhichHalfOfThePredicateFired) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string plan = functionBody(text, "void ExploPlannerNode::doPlan()");
  ASSERT_FALSE(plan.empty()) << "no doPlan() definition";

  const size_t arm = plan.find("if (!teamSettled(live_for_arm)) {");
  ASSERT_NE(arm, std::string::npos);
  const size_t call = plan.find("armAppointment(", arm);
  ASSERT_NE(call, std::string::npos) << "the arm site no longer arms";

  EXPECT_NE(plan.find("teamComplete(live_for_arm, rendezvous_expected_peers_)",
                      arm),
            std::string::npos)
      << "the reason selector is no longer this robot's own read. Under "
         "!teamSettled it is teamComplete that separates 'I cannot hear them' "
         "from 'I can hear everyone and a peer cannot', and teamSettled is "
         "false in both — the string would stop distinguishing anything.";
  EXPECT_NE(plan.find("\"peer-separation\"", arm), std::string::npos)
      << "the contagion arming reason is gone. It is the only signal that "
         "separates a contagion arm from an ordinary one anywhere in the "
         "cell's output; without it the generation is unmeasurable.";
  EXPECT_NE(plan.find("\"separation\"", arm), std::string::npos)
      << "the original arming reason is gone; banked parsers key on it";
}

/// reconnect_end labels the manoeuvre (manoeuvreReleaseEligible); the
/// appointment classifier uses teamSettled or appointment_barrier_released.
/// Capture that before clearing appointment_manoeuvre_; clear before
/// classifying. (notes: five-sites-transition-classifiers)
TEST(Gen23ContagionFiveSites, TheClassifiersReadTheRightHelperForWhatTheyLabel) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string tr =
      functionBody(text, "void ExploPlannerNode::transitionTo(State s, const char* reason)");
  ASSERT_FALSE(tr.empty()) << "no transitionTo() definition";

  EXPECT_NE(tr.find("releaseHeld(manoeuvreReleaseEligible(live))"),
            std::string::npos)
      << "the reconnect_end classifier no longer judges the manoeuvre by the "
         "same predicate that will release it. The log line and the event both "
         "carry this one answer; computing it differently from the barrier is "
         "how a log and the thing it describes drift apart.";
  EXPECT_NE(tr.find("const bool team_back = releaseHeld(teamSettled(live_now)) ||"),
            std::string::npos)
      << "the appointment classifier's mesh half is no longer teamSettled, or "
         "the generation-27 OR with the captured barrier verdict is gone. It "
         "labels an APPOINTMENT, which arms on !teamSettled and supersedes on "
         "teamSettled; the OR's other operand is appointment_barrier_released, "
         "pinned with its ordering below. Anything else here is the five-site "
         "rule broken at the site that writes the outcome into the event "
         "stream.";

  const size_t capture =
      tr.find("appointment_barrier_released = appointment_manoeuvre_ &&");
  const size_t clear = tr.find("appointment_manoeuvre_ = false;");
  const size_t classify = tr.find("const bool team_back =");
  ASSERT_NE(capture, std::string::npos)
      << "the generation-27 capture is gone from transitionTo. The classifier "
         "cannot re-ask manoeuvreReleaseEligible after the flag clear, so "
         "without the capture every reachable-door release banks as a no-show "
         "with arrived=true.";
  ASSERT_NE(clear, std::string::npos)
      << "the manoeuvre flag is no longer cleared in transitionTo";
  ASSERT_NE(classify, std::string::npos)
      << "the appointment classifier's `const bool team_back =` is gone from "
         "transitionTo, so there is no classifier left to order the clear "
         "against. Whatever replaced it needs this test rewritten, not deleted.";
  EXPECT_LT(capture, clear)
      << "the barrier verdict is captured AFTER appointment_manoeuvre_ is "
         "cleared. The capture reads the cleared flag, is constant false, and "
         "every generation-27 door release silently banks as a no-show — the "
         "defect the capture exists to prevent, reintroduced by ordering.";
  EXPECT_LT(clear, classify)
      << "appointment_manoeuvre_ is no longer cleared BEFORE the appointment "
         "classifier.\n"
         "\n"
         "THIS ASSERTS A PROXY, AND IT DOES DEMAND IT. The message here read "
         "'exists to record the ordering, not to demand it' until 2026-09-18, "
         "which is not what an EXPECT does: this fails the suite and blocks a "
         "build, exactly as intended.\n"
         "\n"
         "What is actually required is the RULE — the appointment classifier "
         "must not read manoeuvreReleaseEligible() while the flag it depends "
         "on has already been cleared, because there it degrades silently to "
         "bare teamComplete, i.e. the generation-22 predicate restored at one "
         "of the five sites with nothing visible in review. The ordering is "
         "the cheap, source-scannable stand-in for that rule.\n"
         "\n"
         "So if you are moving the clear BELOW the classifier on purpose, this "
         "failure is not a veto: change the ordering and this assertion "
         "together, in one commit, and state in it which helper the classifier "
         "now reads. If you did not mean to move it, you have just introduced "
         "the degradation above.";
}

/// doReturnSync is the barrier. In doReturnNav the helper is deliberately
/// redundant, since the leading !appointment_manoeuvre_ guard short-circuits
/// it, but it keeps the five sites on one predicate.
/// (notes: five-sites-barrier-one-helper)
TEST(Gen23ContagionFiveSites, BothReturnPathsReleaseOnOneHelper) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  const std::string sync =
      functionBody(text, "void ExploPlannerNode::doReturnSync()");
  ASSERT_FALSE(sync.empty()) << "no doReturnSync() definition";
  EXPECT_NE(sync.find("if (releaseConfirmed(manoeuvreReleaseEligible(active))) {"),
            std::string::npos)
      << "the appointment barrier no longer releases on "
         "manoeuvreReleaseEligible. On bare teamComplete the team leaves the "
         "meeting point while a peer it can hear is still announcing a break — "
         "generation 19's polarity, released weaker than it armed.";

  const std::string nav =
      functionBody(text, "void ExploPlannerNode::doReturnNav()");
  ASSERT_FALSE(nav.empty()) << "no doReturnNav() definition";
  EXPECT_NE(nav.find("if (!appointment_manoeuvre_ && "
                     "releaseConfirmed(manoeuvreReleaseEligible(active))) {"),
            std::string::npos)
      << "the en-route release has changed shape. The guard must lead so that "
         "&& short-circuits an appointment out of this path entirely, and the "
         "helper must stay so the five sites read as one predicate.";
}

/// doReturnSync logs a throttled line when a peer's announced break is what
/// holds the barrier, so a contagion hold can be told apart from a broken
/// release. (notes: five-sites-barrier-contagion-log)
TEST(Gen23ContagionFiveSites, TheBarrierNamesAContagionHold) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string sync =
      functionBody(text, "void ExploPlannerNode::doReturnSync()");
  ASSERT_FALSE(sync.empty()) << "no doReturnSync() definition";

  EXPECT_NE(sync.find("peerReportsTeamBreak()"), std::string::npos)
      << "the contagion-hold diagnostic is gone. Without it a barrier held by "
         "a peer's announcement looks exactly like a barrier that has stopped "
         "releasing, and the smoke cannot tell them apart.";
  EXPECT_NE(sync.find("still reports the team "), std::string::npos)
      << "the diagnostic's text no longer says what is holding the barrier";
}

/// rendezvous_spent_ clears only in heartbeatTick, on teamSettled dwelt by
/// dwellConfirmed, called as a standalone statement outside the
/// !appointment_armed_ test: an unticked window freezes, then fires on one
/// sample. (notes: five-sites-latch-release-dwell)
TEST(Gen23ContagionFiveSites, TheLatchReleaseIsOnTheSamePredicate) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string hb =
      functionBody(text, "void ExploPlannerNode::heartbeatTick()");
  ASSERT_FALSE(hb.empty()) << "no heartbeatTick() definition";

  const size_t release = hb.find("rendezvous_spent_ = false;");
  ASSERT_NE(release, std::string::npos)
      << "the latch release is gone from heartbeatTick — nothing can re-arm "
         "after the first outage";
  EXPECT_EQ(countOf(hb, "rendezvous_spent_ = false;"), 1)
      << "a second release path has appeared; the latch is only a latch if "
         "there is one way out of it";

  // The window is written here and nowhere else in the file: one writer, so
  // that asking the question cannot advance or disarm it. doPlan's supersede
  // reads the same pair through dwellHeld.
  const size_t write = hb.find("const bool team_back_dwelt =");
  // Generation 33 known issue 2: the predicate is teamSettled with the
  // finished-peer veto (walkerJoinsBarrier), since the settle conversion reads
  // this window too. The veto is false outside an appointment manoeuvre.
  const size_t call = hb.find("dwellConfirmed(walkerJoinsBarrier(");
  EXPECT_NE(write, std::string::npos)
      << "the team-back dwell is no longer bound to a named standalone "
         "statement in heartbeatTick. If it was inlined into the "
         "`!appointment_armed_` test it FROZE: an un-ticked window keeps `armed` "
         "true with a stale `since_sec`, and the first sample after a standing "
         "appointment closes fires immediately. That is the undwelt behaviour "
         "back again, wearing the guard's name.";
  EXPECT_NE(call, std::string::npos)
      << "heartbeatTick no longer writes a dwell window over teamSettled";
  EXPECT_EQ(countOf(text, "dwellConfirmed(walkerJoinsBarrier("), 1)
      << "a second site writes the team-back window. Two writers over one pair "
         "means asking the question changes its answer — the supersede site "
         "must use dwellHeld.";
  if (write != std::string::npos && call != std::string::npos) {
    // The named statement IS the dwellConfirmed call, not a separate thing that
    // happens to sit near one: nothing but whitespace between them.
    EXPECT_LT(write, call);
    EXPECT_LT(call - write, size_t{40})
        << "`team_back_dwelt` is no longer initialised directly from "
           "dwellConfirmed(walkerJoinsBarrier(teamSettled(...), ...)) — "
           "something has been interposed";
    EXPECT_LT(call, release)
        << "the window is stepped after the latch is cleared, so the release "
           "acts on the previous heartbeat's answer";
  }
  EXPECT_EQ(hb.find("appointment_armed_ && dwellConfirmed"), std::string::npos)
      << "the writer has been short-circuited behind the `!appointment_armed_` "
         "test. That is the frozen-window failure exactly.";

  // The guard is the two hundred characters immediately above the write.
  const size_t from = release > 240 ? release - 240 : 0;
  const std::string guard = hb.substr(from, release - from);
  EXPECT_NE(guard.find("team_back_dwelt"), std::string::npos)
      << "the latch release is no longer on the dwelt teamSettled read. On the "
         "raw predicate this is generation 19: one claim inside the 5 s TTL "
         "clears the latch for a tick, the arming site re-fires inside the same "
         "outage, and one-appointment-per-outage becomes one per tick.";
  EXPECT_NE(guard.find("!appointment_armed_"), std::string::npos)
      << "the release no longer requires the arm to be down, so a reunion "
         "observed mid-appointment would hand out a second arming while the "
         "first is still live";
}

// ===========================================================================
// GROUP E. WHAT DELIBERATELY DID NOT MOVE.
// ===========================================================================

/// doPursue must not call teamSettled: pursuit chases the peer this robot
/// cannot hear. Counted over stripped text, because doPursue's own comments
/// mention teamSettled. (notes: unchanged-pursuit-own-read)
TEST(Gen23ContagionUnchanged, PursuitStaysOnTheRobotsOwnRead) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string pursue =
      functionBody(text, "void ExploPlannerNode::doPursue()");
  ASSERT_FALSE(pursue.empty()) << "no doPursue() definition";

  EXPECT_EQ(countOf(pursue, "teamSettled("), 0)
      << "pursuit has been widened to the team predicate. A robot that can "
         "hear every peer would then chase on a third party's behalf, which "
         "collapses the distinction between the pursuit and rendezvous arms.";
  EXPECT_NE(pursue.find("teamComplete("), std::string::npos)
      << "pursuit's release no longer reads this robot's own count at all";
}

/// team_last_complete_time_ is this robot's own connectivity history and stays
/// on teamComplete; heartbeatTick holds exactly one teamSettled call, the latch
/// release. (notes: unchanged-presence-clock)
TEST(Gen23ContagionUnchanged, ThePresenceClockStaysOnTheRobotsOwnRead) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string hb =
      functionBody(text, "void ExploPlannerNode::heartbeatTick()");
  ASSERT_FALSE(hb.empty()) << "no heartbeatTick() definition";

  const size_t stamp = hb.find("team_last_complete_time_ = pres_now;");
  ASSERT_NE(stamp, std::string::npos)
      << "the presence clock no longer stamps in heartbeatTick";
  const size_t from = stamp > 200 ? stamp - 200 : 0;
  const std::string guard = hb.substr(from, stamp - from);
  EXPECT_NE(guard.find("teamComplete("), std::string::npos)
      << "the presence clock's guard is no longer this robot's own read";
  EXPECT_EQ(guard.find("teamSettled("), std::string::npos)
      << "the presence clock has been widened to include peers' claims. It is "
         "this robot's connectivity history and banked analyses read it as "
         "such; redefining it changes what old columns mean.";

  EXPECT_EQ(countOf(hb, "teamSettled("), 1)
      << "heartbeatTick holds a number of teamSettled calls other than the one "
         "latch release. Either the release has been duplicated or another "
         "quantity in the heartbeat has been quietly widened.";
}

/// The team_complete event field and metrics columns stay on teamComplete;
/// changing the quantity needs a new column name.
/// (notes: unchanged-team-complete-column)
TEST(Gen23ContagionUnchanged, TheReportedTeamCompleteColumnIsStillTeamComplete) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  const size_t ev = text.find("e.team_complete     = teamComplete(");
  EXPECT_NE(ev, std::string::npos)
      << "the event's team_complete field is no longer computed from "
         "teamComplete. The column would keep its name and change its meaning, "
         "which is the worst available outcome for a banked series.";

  const std::string metrics =
      functionBody(text, "void ExploPlannerNode::fillCommonMetrics(StepMetrics& m)");
  ASSERT_FALSE(metrics.empty()) << "no fillCommonMetrics() definition";
  EXPECT_EQ(countOf(metrics, "teamSettled("), 0)
      << "a metrics column has been switched to the team predicate. Same "
         "problem as the event field: the CSV heading does not change, so "
         "nothing downstream can tell.";
}

// ===========================================================================
// GROUP F. manoeuvreReleaseEligible's asymmetry.
// ===========================================================================

/// manoeuvreReleaseEligible is a ternary on appointment_manoeuvre_: an
/// appointment waits for teamSettled or the closure door; any other manoeuvre
/// releases on teamComplete, never held by a third robot's break.
/// (notes: eligibility-appointment-only)
TEST(Gen23ContagionEligibility, OnlyAnAppointmentWaitsForTheWholeTeam) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn = functionBody(
      text, "bool ExploPlannerNode::manoeuvreReleaseEligible(int live_peers) const");
  ASSERT_FALSE(fn.empty()) << "no manoeuvreReleaseEligible() definition";

  EXPECT_NE(fn.find("appointment_manoeuvre_"), std::string::npos)
      << "the helper no longer distinguishes an appointment from an ordinary "
         "manoeuvre";
  EXPECT_NE(fn.find("teamSettled(live_peers)"), std::string::npos)
      << "the appointment branch no longer waits for the team to settle, so "
         "the barrier releases on the generation-22 predicate";
  EXPECT_NE(fn.find("teamComplete(live_peers, rendezvous_expected_peers_)"),
            std::string::npos)
      << "the non-appointment branch has been widened to teamSettled. Every "
         "pursuit and every mid-run reconnect would then be held open by a "
         "third robot's announcement, which is a team meeting by another name "
         "in the arm that is defined by not having one.";
}

/// The appointment branch of manoeuvreReleaseEligible also releases when every
/// expected peer is reachable (reachablePeerCount). The arm, supersede, latch
/// clear and producer must not read it. (notes: eligibility-reachable-door)
TEST(Gen23ContagionEligibility, TheReachableDoorOpensOnlyAtTheBarrier) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn = functionBody(
      text, "bool ExploPlannerNode::manoeuvreReleaseEligible(int live_peers) const");
  ASSERT_FALSE(fn.empty()) << "no manoeuvreReleaseEligible() definition";

  EXPECT_NE(
      fn.find("teamComplete(reachablePeerCount(), rendezvous_expected_peers_)"),
      std::string::npos)
      << "the reachable door is gone from the appointment branch. The gen-26 "
         "N=3 defect returns: a bridged gather (closure whole, mesh never "
         "complete) holds the unbounded barrier to the duration cap and the "
         "cell banks CLEAN with zero rendezvous_outcome rows.";

  const std::string plan = functionBody(text, "void ExploPlannerNode::doPlan()");
  ASSERT_FALSE(plan.empty()) << "no doPlan() definition";
  EXPECT_EQ(countOf(plan, "reachablePeerCount("), 0)
      << "the arm or the supersede has acquired the closure door. In an A-B-C "
         "bridge the ends are genuinely partitioned on the thing the meeting "
         "exists to exchange, so a closure-keyed arm suppresses a needed "
         "rendezvous.";

  const std::string hb =
      functionBody(text, "void ExploPlannerNode::heartbeatTick()");
  ASSERT_FALSE(hb.empty()) << "no heartbeatTick() definition";
  EXPECT_EQ(countOf(hb, "reachablePeerCount("), 0)
      << "the spent-latch clear has been widened to the closure. The strict "
         "clear is the loop-breaker that makes the weak release safe: weaken "
         "both and a door release can re-arm on the same break, which is the "
         "generation-19 churn by a new route.";

  const std::string producer =
      functionBody(text, "void ExploPlannerNode::publishTeamWorld()");
  ASSERT_FALSE(producer.empty()) << "no publishTeamWorld() definition";
  EXPECT_EQ(countOf(producer, "reachablePeerCount("), 0)
      << "the producer has been keyed to the closure. team_incomplete must "
         "stay a first-hand claim (the one-hop invariant); a closure-keyed "
         "claim about the team travels round the cycle "
         "GossipNeverCarriesTheBit forbids.";
}

// ===========================================================================
// GROUP G. THE RELAYED `finished` BIT, which is what keeps the unbounded
//          barrier bounded at N>=3.
//
// The contagion is read first-hand, so a robot that never hears a finisher
// cannot exempt it. finished is relayed because it is latched (the merge is a
// pure OR) and is about its subject, not the team, so it cannot cycle.
// (notes: relay-finished-soundness)
// ===========================================================================

namespace {

/// A message from `sender` that relays third-party `finished` bits, indexed by
/// fleet id. Deliberately separate from gossipMsg(): the relay is merged by a
/// different loop, under none of the pairing or freshness rules that apply to
/// relayed positions, and a fixture that bundled them would hide that.
TeamModel::Observation finishedMsg(int sender, uint32_t mask,
                                   const std::vector<uint8_t>& finished) {
  TeamModel::Observation o = msg(sender, mask);
  o.finished_gossip = finished;
  return o;
}

}  // namespace

/// A relayed finished bit is stored against the robot it is about, not the
/// relay, the opposite of team_incomplete; marking the relay would drop it from
/// allocation. (notes: relay-lands-on-subject)
TEST(Gen23FinishedRelay, LandsOnTheSubjectAndNotOnTheRelay) {
  TeamModel m = makeModel();  // atlas(0) is us
  // bestla(1) hears us and reports that cerd(2) is finished. bestla says
  // nothing about itself beyond being in contact.
  ASSERT_EQ(m.observe(finishedMsg(1, robotBit(0), {0u, 0u, 1u}), 100.0), "");
  m.tick(100.0);

  EXPECT_TRUE(m.peer(2).finished)
      << "the relayed bit did not land. This is the N>=3 barrier hang: the "
         "robot that can hear the finisher cannot tell the robot that cannot.";
  EXPECT_TRUE(m.peer(2).known) << "a relayed bit is knowledge about that robot";
  EXPECT_FALSE(m.peer(1).finished)
      << "the relay has been marked finished by carrying somebody else's bit. "
         "bestla is mid-run and would drop out of allocation and out of every "
         "reconnect decision the rest of the fleet makes.";
}

/// The relay merge skips false entries rather than assigning them, so a relay
/// that stops mentioning a finished robot never clears it.
/// (notes: relay-pure-or)
TEST(Gen23FinishedRelay, NeverClearsOnAQuietRelay) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(finishedMsg(1, robotBit(0), {0u, 0u, 1u}), 100.0), "");
  m.tick(100.0);
  ASSERT_TRUE(m.peer(2).finished);

  // bestla loses contact with cerd and now relays all-false.
  ASSERT_EQ(m.observe(finishedMsg(1, robotBit(0), {0u, 0u, 0u}), 101.0), "");
  m.tick(101.0);
  EXPECT_TRUE(m.peer(2).finished)
      << "a relay's silence cleared the bit. `finished` is monotonic — a robot "
         "that has stopped exploring does not resume — so a false here means "
         "'no evidence', never 'still going'.";

  // And an empty array — a producer that sends nothing at all — is the same
  // non-statement.
  ASSERT_EQ(m.observe(finishedMsg(1, robotBit(0), {}), 102.0), "");
  m.tick(102.0);
  EXPECT_TRUE(m.peer(2).finished) << "an empty relay array cleared the bit";
}

/// Relayed finished ignores gossip_max_age_sec, unlike relayed positions: a
/// stale finished is still true, and a finished robot goes quiet. It also
/// survives our direct TTL. (notes: relay-not-age-gated)
TEST(Gen23FinishedRelay, SurvivesBothTheGossipAgeAndTheDirectTtl) {
  TeamModel m = makeModel();
  // bestla publishes at t=1000 on its clock and last heard cerd at t=1.0 —
  // 999 s, far past gossip_max_age_sec (120 s). The position half of that
  // message would be discarded as stale.
  TeamModel::Observation o = finishedMsg(1, robotBit(0), {0u, 0u, 1u});
  o.last_heard_sec = {-1.0, 1000.0, 1.0};
  ASSERT_EQ(m.observe(o, 100.0), "");
  m.tick(100.0);
  EXPECT_TRUE(m.peer(2).finished)
      << "the relay was age-gated. A monotonic fact has no freshness to check, "
         "and this gate would fire on every finished robot, because going "
         "quiet is what finishing looks like on the wire.";

  // Nothing further from anybody, well past direct_ttl_sec.
  m.tick(200.0);
  EXPECT_FALSE(m.peer(1).direct) << "fixture: the relay itself should have aged out";
  EXPECT_TRUE(m.peer(2).finished)
      << "the bit expired with the link that carried it. peerAccounted counts "
         "a finished peer precisely because it is no longer reachable.";
}

/// The relay loop skips the sender, so the subject's own first-hand finished is
/// an assignment and can clear the bit. Peers still relaying the old value
/// re-assert it; only a node restart hits this.
/// (notes: relay-first-hand-outranks)
TEST(Gen23FinishedRelay, TheSubjectsOwnWordOutranksARelayedCopyOfIt) {
  TeamModel m = makeModel();
  ASSERT_EQ(m.observe(finishedMsg(1, robotBit(0), {0u, 0u, 1u}), 100.0), "");
  m.tick(100.0);
  ASSERT_TRUE(m.peer(2).finished);

  // cerd itself, first-hand, saying it is not finished.
  TeamModel::Observation own = msg(2, robotBit(0));
  own.finished = false;
  ASSERT_EQ(m.observe(own, 101.0), "");
  m.tick(101.0);
  EXPECT_FALSE(m.peer(2).finished)
      << "a relayed copy has overridden the subject's own statement about "
         "itself. The relay loop must skip the sender's own entry.";

  // And a relay carrying the same message in the same tick does not undo it,
  // because the relay is merged only for robots other than the sender.
  ASSERT_EQ(m.observe(finishedMsg(2, robotBit(0), {0u, 0u, 1u}), 102.0), "");
  m.tick(102.0);
  EXPECT_FALSE(m.peer(2).finished)
      << "cerd's own message relayed a bit about cerd back onto cerd";
}

/// A finished relay array longer than the fleet refuses the whole message
/// rather than truncating: the sender's ids differ, and a never-clearing bit on
/// the wrong robot is permanent. (notes: relay-fleet-length-backstop)
TEST(Gen23FinishedRelay, RefusesAnArrayLongerThanTheFleet) {
  TeamModel m = makeModel();  // three robots
  const std::string why =
      m.observe(finishedMsg(1, robotBit(0), {0u, 0u, 1u, 1u}), 100.0);
  EXPECT_NE(why, "")
      << "a four-entry relay was accepted by a three-robot fleet. The whole "
         "message is refused rather than truncated, because truncating applies "
         "the sender's robot 0 to our robot 0.";
  EXPECT_FALSE(m.peer(2).finished) << "the refusal still merged something";
}

/// End to end from B's seat: A finishes out of contact with B, whose
/// stale-false entry for A nothing ages out; only C's one-hop relay can correct
/// it. (notes: relay-n3-hang-end-to-end)
TEST(Gen23FinishedRelay, ClosesTheNThreeBarrierHangFromTheSeatThatHangs) {
  // We are bestla (1). atlas is 0, cerd is 2.
  TeamModel m = makeModel(makeFleetIdentity({"atlas", "bestla", "cerd"}, "bestla"));

  // Spawn: atlas and bestla are in contact and atlas is mid-run.
  ASSERT_EQ(m.observe(msg(0, robotBit(1)), 100.0), "");
  m.tick(100.0);
  ASSERT_TRUE(m.peer(0).direct);
  ASSERT_FALSE(m.peer(0).finished);

  // They drift apart. atlas is now unreachable from bestla for the rest of the
  // run, and the stale entry is what generation 23 reads.
  m.tick(200.0);
  ASSERT_FALSE(m.peer(0).direct);
  ASSERT_FALSE(m.peer(0).finished)
      << "fixture: this is the stale-false that causes the hang, and it must "
         "still be false here or the test is not reproducing the defect";

  // atlas latches done at t=250, heard only by cerd. cerd relays it at t=300.
  ASSERT_EQ(m.observe(finishedMsg(2, robotBit(1), {1u, 0u, 0u}), 300.0), "");
  m.tick(300.0);

  EXPECT_TRUE(m.peer(0).finished)
      << "bestla still believes atlas is exploring. It will announce "
         "team_incomplete for the rest of the run, cerd will be held open by "
         "that announcement, and both will wait at an uncapped barrier until "
         "the duration cap censors the cell.";
}

// ---------------------------------------------------------------------------
// The node half of GROUP G: source scans, same limits as GROUPS B-F.
// ---------------------------------------------------------------------------

/// publishTeamWorld must publish the finished_announced_ latch, not recompute
/// it: State::DONE can be left, and only a monotonic bit may be relayed by a
/// pure OR. (notes: relay-published-bit-latched)
TEST(Gen23FinishedRelay, ThePublishedBitIsLatched) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string pub =
      functionBody(text, "void ExploPlannerNode::publishTeamWorld()");
  ASSERT_FALSE(pub.empty()) << "no publishTeamWorld() definition";

  EXPECT_NE(pub.find("m.finished = finished_announced_;"), std::string::npos)
      << "the published bit is being recomputed rather than read from the "
         "latch. A bit that can go back to false must not be relayed, because "
         "the relay is a pure OR with nothing to clear it.";
  EXPECT_EQ(countOf(pub, "m.finished = coverage_latched_"), 0)
      << "the pre-latch expression is back";

  // The only assignment of `false` anywhere is the member initialiser. Counted
  // over a space-collapsed copy so that the declaration's column alignment is
  // not what decides whether this check runs — a scan that silently matches
  // nothing is the failure mode this whole file is written against.
  const std::string flat = squeezeSpaces(text);
  EXPECT_EQ(countOf(flat, "finished_announced_ = false"), 1)
      << "something clears the latch. Monotonicity is not a property of this "
         "variable's name; it is the single-assignment discipline, and it is "
         "what GROUP G's pure-OR merge rests on.";
  EXPECT_GT(countOf(flat, "finished_announced_ = true"), 0)
      << "nothing ever sets the latch, so the fleet never learns that anybody "
         "finished and the barrier hang is back";
}

/// publishTeamWorld sizes robot_finished and sets it for each peer believed
/// finished, first-hand or relayed, without the position gossip's pairing rule:
/// the bit is idempotent and only shrinks the wait set.
/// (notes: relay-producer-fills-array)
TEST(Gen23FinishedRelay, TheProducerFillsTheArray) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string pub =
      functionBody(text, "void ExploPlannerNode::publishTeamWorld()");
  ASSERT_FALSE(pub.empty()) << "no publishTeamWorld() definition";

  EXPECT_NE(pub.find("m.robot_finished.assign("), std::string::npos)
      << "the array is never sized, so it goes out empty and every receiver "
         "reads it as 'no evidence' — the relay is inert and looks fine";
  EXPECT_NE(pub.find("if (p.finished) m.robot_finished[i] = true;"),
            std::string::npos)
      << "the producer no longer forwards what it knows about its peers, so "
         "only a robot's own bit ever reaches the wire and the one hop the "
         "design needs does not exist";
}

/// drainTeamWorld must copy robot_finished into finished_gossip, or the relay
/// is dead on the receive side. (notes: relay-consumer-copies-array)
TEST(Gen23FinishedRelay, TheConsumerCopiesTheArrayIntoTheObservation) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string drain =
      functionBody(text, "void ExploPlannerNode::drainTeamWorld()");
  ASSERT_FALSE(drain.empty()) << "no drainTeamWorld() definition";

  EXPECT_NE(drain.find("o.finished_gossip"), std::string::npos)
      << "drainTeamWorld no longer populates the relay array. TeamModel then "
         "merges an empty vector for every message, the relay is dead on the "
         "receive side, and the N>=3 barrier hang is back with every producer "
         "-side test still passing.";
  EXPECT_NE(drain.find("msg.robot_finished"), std::string::npos)
      << "the observation is being filled from something other than the wire "
         "field";
}

// ===========================================================================
// GROUP H. `appointment_inbound`: the appointment barrier's second term.
//
// The barrier's comms test can release before a partner reaches the cell.
// appointment_inbound is keyed on the RETURN_NAV state, never distance, so it
// clears at every drive exit; the wait is unbounded.
// (notes: inbound-barrier-second-term)
// ===========================================================================

/// appointment_inbound is first-hand only and never relayed: it is
/// non-monotonic, unlike the latched finished. Pins both directions: no smear
/// onto the gossip subject, no clear by a third party.
/// (notes: inbound-first-hand-only)
TEST(Gen23InboundWire, TheBitIsFirstHandAndNeverRelayed) {
  {
    TeamModel m = makeModel();
    // bestla (1) hears us, says IT is still driving to the agreed cell, and
    // relays what it knows about cerd (2). Nothing here is a claim by cerd.
    TeamModel::Observation relay = gossipMsg(1, robotBit(0), {-1.0, 100.0, 90.0});
    relay.appointment_inbound = true;
    ASSERT_EQ(m.observe(relay, 1000.0), "");
    m.tick(1000.0);

    EXPECT_TRUE(m.peer(1).appointment_inbound)
        << "the sender's own bit is first-hand and must land";
    EXPECT_TRUE(m.peer(2).known) << "the gossip did land as knowledge";
    EXPECT_FALSE(m.peer(2).appointment_inbound)
        << "one robot's 'I am still driving' has been re-attributed to the "
           "robot it was gossiping about. The barrier would then be held by a "
           "peer that has already arrived, or by one that never left.";
  }
  {
    TeamModel m = makeModel();
    // cerd (2) says so itself, first-hand, then goes quiet. bestla (1) gossips
    // about cerd with no claim of its own.
    TeamModel::Observation o = msg(2, robotBit(0));
    o.appointment_inbound = true;
    ASSERT_EQ(m.observe(o, 100.0), "");
    m.tick(100.0);
    ASSERT_TRUE(m.peer(2).appointment_inbound);

    ASSERT_EQ(m.observe(gossipMsg(1, robotBit(0), {-1.0, 200.0, 199.0}), 200.0),
              "");
    m.tick(200.0);
    EXPECT_TRUE(m.peer(2).appointment_inbound)
        << "a third party's gossip has withdrawn a claim only cerd can make or "
           "withdraw. Like team_incomplete, this bit has no TTL of its own and "
           "it is liveness that stops the consumer believing it.";
  }
}

/// appointment_inbound_seen, the bridge's report, lands only under its sender
/// and clears by the sender's next message; this must hold through every gossip
/// merge loop in observe(). (notes: inbound-seen-sender-only)
TEST(Gen23InboundWire, TheSeenReportLandsUnderItsSenderOnlyAndClears) {
  TeamModel m = makeModel();
  // bestla (1) reports that somebody IT hears is still driving, and gossips
  // about cerd (2). Nothing here is a claim by cerd.
  TeamModel::Observation report = gossipMsg(1, robotBit(0), {-1.0, 100.0, 90.0});
  report.appointment_inbound_seen = true;
  // Also relays cerd's finished bit on purpose, so one message drives the
  // finished_gossip merge as well as the position loop over cerd's row.
  // (notes: inbound-seen-both-loops)
  report.finished_gossip = {0, 0, 1};
  ASSERT_EQ(m.observe(report, 1000.0), "");
  m.tick(1000.0);
  EXPECT_TRUE(m.peer(1).appointment_inbound_seen)
      << "the sender's own report is first-hand and must land";
  EXPECT_FALSE(m.peer(2).appointment_inbound_seen)
      << "one robot's 'I hear someone still driving' has been re-attributed "
         "to the robot it was gossiping about — the echo the raw-only "
         "derivation rule exists to prevent, built into the store instead";

  // The drive it reported ends; bestla's next message says so.
  ASSERT_EQ(m.observe(gossipMsg(1, robotBit(0), {-1.0, 110.0, 90.0}), 1010.0),
            "");
  m.tick(1010.0);
  EXPECT_FALSE(m.peer(1).appointment_inbound_seen)
      << "the report latched: the sender has withdrawn it and the model is "
         "still holding the barrier's veto on";
}

/// appointment_inbound must be keyed on appointment_manoeuvre_ and the
/// RETURN_NAV state, which clears at every drive exit, never on arrival or
/// distance to the cell: the wait it feeds is unbounded.
/// (notes: inbound-keyed-on-driving)
TEST(Gen23InboundProducer, TheBitIsKeyedOnDrivingNotOnArrival) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string pub =
      functionBody(text, "void ExploPlannerNode::publishTeamWorld()");
  ASSERT_FALSE(pub.empty()) << "no publishTeamWorld() definition";

  const size_t at = pub.find("m.appointment_inbound");
  ASSERT_NE(at, std::string::npos)
      << "publishTeamWorld no longer writes the field. It is then false on every "
         "message, the barrier is back to its comms-only predicate, and the "
         "smoke's 272 s of standing alone returns with the receive side still "
         "tested.";
  const size_t end = pub.find(';', at);
  ASSERT_NE(end, std::string::npos) << "unterminated assignment";
  const std::string stmt = squeezeSpaces(pub.substr(at, end - at));

  EXPECT_NE(stmt.find("appointment_manoeuvre_"), std::string::npos)
      << "the bit is no longer confined to an appointment, so an ordinary "
         "pursuit would hold somebody else's barrier";
  EXPECT_NE(stmt.find("State::RETURN_NAV"), std::string::npos)
      << "the bit is no longer keyed on the driving state. Whatever replaced it "
         "must clear at the nav budget and the no-progress watchdog as well as "
         "at arrival, or an unreachable meeting point holds the whole team at a "
         "barrier with no cap.";
  EXPECT_EQ(stmt.find("arrive"), std::string::npos)
      << "an arrival test has been substituted for the state test — see above "
         "for why that hangs rather than releases";
  EXPECT_EQ(stmt.find("appointmentPoint"), std::string::npos)
      << "the bit is being derived from distance to the agreed cell, which is "
         "exactly the rewrite this test exists to stop";
}

/// appointment_inbound_seen is published from peerInboundToAppointment(), never
/// peerReportsInboundToAppointment(), which reads the published field and would
/// start an echo nothing can clear. (notes: inbound-seen-raw-bits-only)
TEST(Gen23InboundProducer, TheSeenReportIsDerivedFromRawBitsOnly) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string pub =
      functionBody(text, "void ExploPlannerNode::publishTeamWorld()");
  ASSERT_FALSE(pub.empty()) << "no publishTeamWorld() definition";

  const std::string sq = squeezeSpaces(pub);
  EXPECT_NE(
      sq.find("m.appointment_inbound_seen = peerInboundToAppointment();"),
      std::string::npos)
      << "the report is no longer the raw predicate's verdict, verbatim. "
         "Whatever replaced it must not read peers' copies of the seen field, "
         "or the echo above is live.";
  EXPECT_EQ(pub.find("peerReportsInboundToAppointment"), std::string::npos)
      << "the producer is publishing the peer-report predicate, which reads "
         "the very field being published: A says seen because B says seen, "
         "and the barrier is held by an echo nothing can clear.";
}

/// AND THE CONSUMER COPIES BOTH INTO THE OBSERVATION. Exact statements, not
/// substrings: the raw bit's name is a prefix of its sibling's, so a bare
/// find("msg.appointment_inbound") would pass on the sibling's copy alone.
TEST(Gen23InboundWire, TheConsumerCopiesTheBitIntoTheObservation) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string drain =
      functionBody(text, "void ExploPlannerNode::drainTeamWorld()");
  ASSERT_FALSE(drain.empty()) << "no drainTeamWorld() definition";

  const std::string sq = squeezeSpaces(drain);
  EXPECT_NE(sq.find("o.appointment_inbound = msg.appointment_inbound;"),
            std::string::npos)
      << "the wire field is written and never read, which looks exactly like "
         "the generation before it";
  EXPECT_NE(
      sq.find("o.appointment_inbound_seen = msg.appointment_inbound_seen;"),
      std::string::npos)
      << "the one-hop report is written and never read; the closure door's "
         "veto is then blind to every peer the door admits through a bridge";
}

/// The still-driving term belongs in manoeuvreReleaseEligible's appointment
/// branch, never in teamSettled, whose other consumers ask whether the team is
/// together. (notes: inbound-term-not-in-settled)
TEST(Gen23InboundEligibility, TheAppointmentBranchWaitsForAPeerStillDriving) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn = functionBody(
      text, "bool ExploPlannerNode::manoeuvreReleaseEligible(int live_peers) const");
  ASSERT_FALSE(fn.empty()) << "no manoeuvreReleaseEligible() definition";

  EXPECT_NE(fn.find("!peerInboundToAppointment()"), std::string::npos)
      << "the barrier releases on connectivity alone again. The first robot to "
         "come into radio range of the meeting point ends the meeting, and its "
         "partner finishes the walk to an empty cell — the generation-23 smoke "
         "measured 36 s of settle against 272 s of standing.";

  const std::string settled =
      functionBody(text, "bool ExploPlannerNode::teamSettled(int live_peers) const");
  ASSERT_FALSE(settled.empty()) << "no teamSettled() definition";
  EXPECT_EQ(settled.find("peerInboundToAppointment"), std::string::npos)
      << "the term has been hoisted into teamSettled, where the arm, the "
         "supersede and the presence dwell all read it. Those are asking "
         "whether the team is together, not whether anyone is still on the way.";
}

/// peerInboundToAppointment() has no finished exemption on purpose: a robot
/// that latched coverage may still be driving to the meeting. It keeps the
/// liveness gate, since the bit has no TTL.
/// (notes: inbound-no-finished-exemption)
TEST(Gen23InboundEligibility, ThePredicateHasNoFinishedExemption) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn = functionBody(
      text, "bool ExploPlannerNode::peerInboundToAppointment() const");
  ASSERT_FALSE(fn.empty()) << "no peerInboundToAppointment() definition";

  const std::string sq = squeezeSpaces(fn);
  EXPECT_NE(sq.find("if (!p.direct && !p.heard_one_way) continue;"),
            std::string::npos)
      << "the liveness gate is gone. A peer that dropped off the air mid-drive "
         "leaves a true in the table that nothing ages out, and the barrier is "
         "held by a robot nobody can hear.";
  EXPECT_EQ(fn.find("finished"), std::string::npos)
      << "a finished-peer exemption has been added, copied from "
         "peerReportsTeamBreak where it is load-bearing. Here it drops a robot "
         "that latched coverage on its way to the meeting and is still driving "
         "to it — which is this generation's defect, reintroduced for the one "
         "robot most likely to meet it.";
  EXPECT_NE(fn.find("p.appointment_inbound"), std::string::npos)
      << "the predicate is reading something other than the peer's own bit";
}

/// manoeuvreReleaseEligible also vetoes on peerReportsInboundToAppointment(): a
/// peer admitted through the closure door never sends its own bit here, so only
/// the bridge's report shows it driving. Not in teamSettled.
/// (notes: inbound-door-veto-bridge)
TEST(Gen23InboundEligibility, TheDoorVetoCoversTheBridgeHop) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn = functionBody(
      text, "bool ExploPlannerNode::manoeuvreReleaseEligible(int live_peers) const");
  ASSERT_FALSE(fn.empty()) << "no manoeuvreReleaseEligible() definition";

  EXPECT_NE(fn.find("!peerReportsInboundToAppointment()"), std::string::npos)
      << "the release predicate has lost the one-hop veto. A robot whose "
         "reachable count fills through a bridge leaves while the peer behind "
         "the bridge is still driving in — the gen-27 pre-build review's "
         "blocking finding, reintroduced.";

  const std::string settled =
      functionBody(text, "bool ExploPlannerNode::teamSettled(int live_peers) const");
  ASSERT_FALSE(settled.empty()) << "no teamSettled() definition";
  EXPECT_EQ(settled.find("peerReportsInboundToAppointment"), std::string::npos)
      << "the report term has been hoisted into teamSettled, whose other "
         "consumers — the arm, the supersede, the presence dwell — ask "
         "whether the team is together, not whether anyone is still driving.";
}

/// AND THE REPORT PREDICATE IS PAIRED WITH LIVENESS, LIKE ITS SIBLING.
TEST(Gen23InboundEligibility, TheReportPredicateReadsTheSeenBitFirstHand) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn = functionBody(
      text, "bool ExploPlannerNode::peerReportsInboundToAppointment() const");
  ASSERT_FALSE(fn.empty()) << "no peerReportsInboundToAppointment() definition";

  const std::string sq = squeezeSpaces(fn);
  EXPECT_NE(sq.find("if (!p.direct && !p.heard_one_way) continue;"),
            std::string::npos)
      << "the liveness gate is gone. The report has no TTL of its own, so a "
         "reporter that went silent mid-settle leaves a true in the table "
         "that nothing ages out.";
  EXPECT_NE(fn.find("p.appointment_inbound_seen"), std::string::npos)
      << "the predicate is reading something other than the peer's report";
  EXPECT_EQ(fn.find("finished"), std::string::npos)
      << "a finished-peer exemption has been added. A reporter's own run "
         "ending does not invalidate what it currently receives; dropping its "
         "report re-blinds the veto for exactly the bridge case it exists for.";
}

// ===========================================================================
// GENERATION 28 — the settle conversion is reversible.
//
// The settle conversion lets an appointment walker join the barrier from the
// road once the team settles; these tests pin the resume when that settle
// lapses. (notes: settle-lapse-return-path)
// ===========================================================================

/// THE CONVERSION IS MARKED, SO THE BARRIER CAN TELL IT FROM THE OTHER EXITS.
TEST(Gen28SettleLapse, OnlyTheConversionSetsTheResumeFlag) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string nav = functionBody(text, "void ExploPlannerNode::doReturnNav()");
  ASSERT_FALSE(nav.empty()) << "no doReturnNav() definition";

  EXPECT_EQ(countOf(nav, "appointment_settle_converted_ = true"), 1)
      << "the conversion no longer marks itself, so doReturnSync cannot tell "
         "a walker stopped by a settle from one stopped by an unreachable "
         "cell — and the resume either never fires or fires on the exits "
         "whose own evidence says the drive has already failed.";

  // The budget and no-progress exits must stay unmarked: their evidence is
  // that the cell could NOT be reached, so resuming them loops a failed drive.
  const std::size_t budget = nav.find("return-budget");
  const std::size_t noprog = nav.find("return-no-progress");
  ASSERT_NE(budget, std::string::npos) << "no nav-budget exit";
  ASSERT_NE(noprog, std::string::npos) << "no no-progress exit";
  const std::size_t marked = nav.find("appointment_settle_converted_ = true");
  ASSERT_NE(marked, std::string::npos);
  EXPECT_LT(marked, budget)
      << "the resume flag is set at or after the nav-budget exit; a drive that "
         "ran out of budget would be sent back down the same road.";
  EXPECT_LT(marked, noprog)
      << "the resume flag is set at or after the no-progress exit; a drive "
         "that stopped making progress would be re-issued unchanged.";
}

/// AND EVERY NEW LEG CLEARS IT, SO ONE LAPSE COSTS AT MOST ONE RESUME.
TEST(Gen28SettleLapse, StartReturnToClearsTheResumeFlag) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn = functionBody(
      text,
      "void ExploPlannerNode::startReturnTo(const Eigen::Vector3f& dest,");
  ASSERT_FALSE(fn.empty()) << "no startReturnTo() definition";

  EXPECT_NE(fn.find("appointment_settle_converted_ = false"), std::string::npos)
      << "a resumed leg inherits the flag that resumed it. Its own nav-budget "
         "exit would then read as a settle conversion and resume again, which "
         "is an unbounded loop on a cell the navigator cannot reach.";
}

/// THE RESUME ASKS THE RELEASE'S OWN "TEAM IS TOGETHER" QUESTION, NEGATED.
TEST(Gen28SettleLapse, TheResumeFiresOnlyWhenTheTeamIsActuallyApart) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn = functionBody(text, "void ExploPlannerNode::doReturnSync()");
  ASSERT_FALSE(fn.empty()) << "no doReturnSync() definition";

  const std::string sq = squeezeSpaces(fn);
  EXPECT_NE(sq.find("startReturnTo(resume_target, \"appointment\", "
                    "\"return-settle-lapsed\")"),
            std::string::npos)
      << "the barrier has lost the resume. A walker whose settle lapses stands "
         "where it stopped until the duration cap — the gen-27 N=2 defect.";

  // doReturnSync must read appointment_settle_converted_: without it the resume
  // also fires after the return-budget and return-no-progress exits, and a
  // budget exit resumes into another, unbounded.
  // (notes: settle-lapse-flag-read)
  EXPECT_NE(sq.find("appointment_settle_converted_"), std::string::npos)
      << "doReturnSync no longer reads appointment_settle_converted_, so the "
         "resume is not restricted to the settle conversion that stopped the "
         "walk — a budget or no-progress park would resume and re-park.";

  // Both halves of manoeuvreReleaseEligible's "together" disjunction have to
  // be negated here. Dropping either one resumes a robot whose team is fine.
  // Since generation 33 (known issue 2) the negation is walkerResumesDrive,
  // which also takes the finished-peer veto; its logic runs in
  // test_meeting_attendance.cpp (WalkerMoves).
  const std::string flat = [&] {
    std::string f;
    for (const char c : fn)
      if (c != ' ' && c != '\n' && c != '\t' && c != '\r') f.push_back(c);
    return f;
  }();
  const size_t at_resume = flat.find("walkerResumesDrive(");
  ASSERT_NE(at_resume, std::string::npos)
      << "the resume test no longer asks walkerResumesDrive";
  const std::string args = flat.substr(at_resume, 160);
  EXPECT_NE(args.find("walkerResumesDrive(teamSettled(active),"),
            std::string::npos)
      << "the mesh half is gone from the resume test: a robot whose team is "
         "settled would be sent back onto the road.\n" << args;
  EXPECT_NE(args.find(",teamComplete(reachablePeerCount(),"
                      "rendezvous_expected_peers_),"),
            std::string::npos)
      << "the closure half is gone from the resume test: a bridged team — "
         "exactly what generation 27 releases on — would be torn up and sent "
         "driving again.\n" << args;
  EXPECT_NE(sq.find("!appointment_arrived_"), std::string::npos)
      << "a robot that reached the agreed cell can be resumed. There is "
         "nowhere for it to go and the vigil at the cell is the design.";

  // The aliasing trap: startReturnTo blanks current_goal_ before reading its
  // own argument, so the destination must be copied out first.
  EXPECT_EQ(sq.find("startReturnTo(current_goal_.position"), std::string::npos)
      << "current_goal_.position is passed straight into startReturnTo, which "
         "assigns current_goal_ = CandidateViewpoint{} before it reads dest — "
         "the robot would be dispatched to the world origin.";
}
