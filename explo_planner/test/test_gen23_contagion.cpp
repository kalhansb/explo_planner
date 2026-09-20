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

/// A message carrying third-party gossip. `heard` is indexed by fleet id and
/// holds times ON THE SENDER'S CLOCK; the sender's own entry is its publish
/// time. Duplicated from test_team_model.cpp rather than shared — see the
/// stripComments note in that file for why these scan binaries stay
/// independently re-runnable.
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

/// THE BIT ATTACHES TO THE SENDER AND ONLY TO THE SENDER.
///
/// The trivial-looking assertion is the one that keeps the design honest: a
/// claim about the team, stored against the robot that made it, can be
/// attributed. A claim stored anywhere else is a rumour.
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

/// GOSSIP NEVER CARRIES THE BIT. THIS IS THE DEADLOCK GUARD.
///
/// observe()'s first-hand block copies `team_incomplete` beside `finished`; the
/// third-party gossip block below it relays positions and last-heard times and
/// deliberately touches neither. If it relayed this one, C would re-announce
/// A's break as though it were C's own read, A would see its own bit come
/// back, and the appointment could never clear — the same deadlock as
/// announcing the armed state, arriving by a different route.
///
/// Both directions are asserted, because they fail differently. A relay whose
/// OWN view is broken must not smear that onto the robot it is gossiping
/// about; and a relay whose own view is fine must not clear a bit it was never
/// told anything about.
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

/// THE BIT HAS NO TTL OF ITS OWN, WHICH IS WHY THE CONSUMER MUST GATE ON
/// LIVENESS.
///
/// This is a property of TeamModel and a requirement on the node in one test.
/// The stored bit is whatever the peer last said and nothing ages it out — so
/// a robot that announced a break and then dropped off the air entirely leaves
/// a `true` sitting in the table forever. Read alone, that is a latch: the team
/// would stay armed off a message nobody can refresh. What makes it safe is
/// that `direct` and `heard_one_way` BOTH go false on TTL expiry, and
/// peerReportsTeamBreak() skips a peer that has neither (GROUP C pins that
/// side).
///
/// Asserting the bit SURVIVES is deliberate, not an accident of the
/// implementation being tested as-is. Clearing it on expiry would look tidier
/// and would be wrong: the peer's last known state is still the best available
/// answer if it comes back inside a moment, and it is the node's liveness gate,
/// not the storage, that decides whether to believe it.
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

/// ONE-WAY CONTACT STILL CARRIES THE BIT, AND THAT IS THE POINT.
///
/// doc/limitations.md §10's failure mode: we receive from the peer, the peer
/// cannot hear us. Its own read of the team is therefore broken — it is
/// exactly the robot that needs the meeting — and the announcement is the only
/// way it has of telling us, because by construction it cannot complete a
/// handshake. Gating the node's consumer on `direct` ALONE would discard
/// precisely the case the field exists for.
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

/// THE SENDER CAN WITHDRAW IT. ASSIGNMENT, NOT A STICKY OR.
///
/// The asymmetric clear only works if the producer's `false` is as
/// authoritative as its `true`. A consumer that latched on first `true` would
/// give the fleet a permanent appointment after the first transient dropout
/// anywhere in it.
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

/// THE HUB CASE AT N=4, WHICH IS WHERE ONE-HOP SUFFICIENCY EARNS ITS KEEP.
///
/// A star: atlas hears bestla, cerd and dvalin and all three name it back, so
/// atlas's OWN read is complete and generation 22 would have refused to arm.
/// The three leaves hear only atlas — each sees one peer where it expects
/// three — so each announces a break, and atlas receives all three FIRST-HAND
/// because a robot with a complete read is adjacent to everybody. No relay, no
/// second round, whatever N is.
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

/// The source with `//` and `/* */` comments removed, string literals
/// preserved. Duplicated from test_gen20/21/22 rather than shared: these are
/// standalone source-scan binaries, and a shared header between test
/// executables whose whole job is to be independently re-runnable against a
/// mutated node would make one mutation break all of them.
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

/// THE ANNOUNCED BIT IS `!teamComplete(accountedPeerCount, expected)`,
/// FIRST-HAND.
///
/// `accountedPeerCount`, not `livePeerCount`, and the difference is the whole
/// reason the announcement terminates. The live count is direct links only, so
/// a robot that has finished and parked out of radio range would be missing
/// from it forever and this robot would announce a break for the rest of the
/// run — with the contagion in place, that is the whole team held at an
/// uncapped barrier. `accountedPeerCount` counts a peer that is direct, or
/// finished, or reachable through the two-hop closure, which is the same union
/// peerAccounted applies everywhere else.
///
/// This is the anti-deadlock invariant at its source. Publishing the derived
/// armed state instead — `appointment_armed_`, or anything downstream of
/// peerReportsTeamBreak() — closes a cycle: C arms because A announced, C
/// announces, A holds because C announced. Nothing in a fully reconnected team
/// could then clear it.
///
/// The assertion is therefore as much about what is ABSENT as what is present.
/// `appointment_armed_` is banned from the function outright: it is derived
/// from what peers announced, so publishing it on ANY field closes the cycle.
/// `appointment_manoeuvre_` is banned from THIS field's right-hand side rather
/// than from the function, because GROUP H's `appointment_inbound` is keyed on
/// it deliberately and does not close a cycle — a robot can only be held at the
/// barrier while it is in RETURN_SYNC, where its own inbound bit is false, so
/// the mutual hold is unreachable and every exit from the drive clears the bit
/// within nav_budget_sec without waiting on anyone.
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

/// AN UNCONFIGURED EXPECTATION ANNOUNCES NOTHING.
///
/// `teamComplete(x, 0)` is FALSE, so an unguarded `!teamComplete(...)` would
/// make every single-robot and every rendezvous-inert configuration broadcast
/// a permanent, unclearable "the team is broken" to a fleet of nobody — and
/// any listener that did exist would arm on it and never release.
/// shared_params.yaml documents `rendezvous_expected_peers: 0` as the inert
/// setting; no expectation means the question has no answer, and the honest
/// wire value for that is "I am not reporting a break".
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

/// IT BELIEVES ONLY A PEER IT IS RECEIVING FROM RIGHT NOW — AND BOTH LIVENESS
/// FLAGS COUNT.
///
/// GROUP A's TheBitHasNoTtlOfItsOwn is the other half of this: the stored bit
/// survives the TTL on purpose, so this gate is the only thing standing
/// between a silent robot's last message and a permanent appointment.
///
/// `heard_one_way` must be accepted alongside `direct`. One-way contact is the
/// §10 mode, and a robot in it has a genuinely broken read, is announcing so,
/// and cannot complete a handshake by construction. Requiring `direct` would
/// discard the case with the strongest claim to a meeting.
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

/// A FINISHED PEER DOES NOT ARM THE TEAM.
///
/// `finished` already means "no longer someone worth reconnecting to"
/// (TeamWorld.msg), and the reason bites harder here than anywhere else: the
/// appointment barrier's wait is UNBOUNDED, so a parked robot that can never
/// hear one distant peer would hold the whole team at the meeting point until
/// the run's duration cap and censor the cell. It also buys nothing — map
/// still flows OUT of a finished robot over any link carrying its scovox_bin,
/// and it has stopped needing map to flow in.
///
/// This is NOT the generation-18/19 split reappearing. That defect was arming
/// and releasing on DIFFERENT predicates; here the arm and the release both
/// call this one function, so the exemption weakens both sides identically.
/// What it does leave — deliberately — is that a finished peer still counts
/// toward teamComplete, so if it goes SILENT the team arms on the ordinary
/// rule.
TEST(Gen23ContagionPredicate, ExemptsAFinishedPeer) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string fn =
      functionBody(text, "bool ExploPlannerNode::peerReportsTeamBreak() const");
  ASSERT_FALSE(fn.empty()) << "no peerReportsTeamBreak() definition";

  const size_t exempt = fn.find("if (p.finished) continue;");
  const size_t read   = fn.find("if (p.team_incomplete) return true;");
  ASSERT_NE(exempt, std::string::npos)
      << "the finished-peer exemption is gone. The appointment barrier has no "
         "cap (rendezvous_appointment_wait_sec defaults to 0), so a parked "
         "robot that can never close one distant pair now holds the whole team "
         "to the duration cap and censors the cell.";
  ASSERT_NE(read, std::string::npos);
  EXPECT_LT(exempt, read)
      << "the exemption no longer precedes the read, so a finished peer's bit "
         "is believed before it is skipped";
}

// ===========================================================================
// GROUP D. THE FIVE SITES MOVE AS ONE. This is what the file is for.
// ===========================================================================

/// teamSettled IS teamComplete PLUS THE CONTAGION, AND NOTHING ELSE.
///
/// One definition, so that "change P" is a one-line change and the five call
/// sites cannot drift apart. The conjunction is the literal reading of the
/// directive: the team is settled when this robot can hear everyone AND nobody
/// it can hear says otherwise.
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

/// 1 AND 2. THE ARM AND THE SUPERSEDE, WHICH LIVE IN THE SAME FUNCTION.
///
/// `arm on !P, end on P, for one P`. doPlan holds both halves within a few
/// hundred lines of each other, which is exactly why they are the easiest pair
/// to break in opposite directions: generation 18 and generation 19 are this
/// pair disagreeing, each time in a way N=2 could not see.
///
/// ONE PREDICATE, TWO EVIDENCE THRESHOLDS (generation 23), and the second half
/// of that is new. Matching the predicate was only ever half the rule. The arm
/// fires on the first sample of `!P` — coming apart is shown by a missing
/// beacon, and a robot that waits for confirmation departs late for a meeting it
/// has already been told about. The supersede requires `P` to have HELD for
/// reconnect_release_confirm_sec, because coming back is shown by a single
/// arriving claim inside a 5 s TTL, which a range-edge flicker produces exactly
/// as readily as a reunion — and cancelling on it strands a peer already driving
/// to the cell. Until 2026-09-18 this site had no dwell while the barrier
/// guarding the same reunion refused the same evidence for 6 s.
///
/// dwellHeld, NOT dwellConfirmed, and the test pins that too. The window is
/// written once per heartbeat; this block sits inside `appointment_armed_ &&
/// !appointment_departed_`, and doPlan is entered only in State::PLAN — roughly
/// once per step, not at 10 Hz. A continuity window ticked from here would
/// measure "two consecutive plan entries agreed", which two samples thirty
/// seconds apart satisfy trivially, and between appointments it would not decay
/// at all. See FlickerDwell.AnUnTickedWindowFreezesRatherThanDecaying.
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

/// AND THE ARM'S SECOND REASON STRING, WHICH IS HOW THE CONTAGION IS COUNTED.
///
/// RendezvousAgreedEvent has no `reason` field and adding one would bump a
/// schema several banked parsers pin, so the measurement rides the plaintext
/// "Rendezvous schedule [%s]" line instead. "separation" is this robot's own
/// read failing — the generation-22 string, unchanged, so banked parsers still
/// find it. "peer-separation" means this robot can hear everyone and armed
/// SOLELY because a peer announced a break: it is the C of the bridge, and
/// counting those is how much the contagion actually did.
///
/// The selector must be teamComplete, not teamSettled: under !teamSettled,
/// teamComplete distinguishes the two causes exactly, whereas teamSettled is
/// false in both and the string would be constant.
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

/// 3. THE TWO CLASSIFIERS IN transitionTo, AND WHY THEY READ DIFFERENT
///    HELPERS.
///
/// They label DIFFERENT OBJECTS and may legitimately disagree — the
/// reconnect_end classifier labels the MANOEUVRE that ended, the appointment
/// classifier labels the APPOINTMENT. The manoeuvre is judged by
/// manoeuvreReleaseEligible, whose answer depends on whether the manoeuvre was
/// an appointment; the appointment is judged by teamSettled — its arm and
/// supersede predicate — OR, since generation 27, by the barrier's own
/// verdict, captured into `appointment_barrier_released` while
/// `appointment_manoeuvre_` was still true. Without that second half a
/// reachable-door release (closure whole, mesh not) would bank as a no-show
/// with arrived=true, which is the smoke20 sign reversal restored at the site
/// that writes the event stream.
///
/// THERE IS ALSO A HARD REASON, and it is the one that changed this design
/// mid-implementation: `appointment_manoeuvre_` HAS ALREADY BEEN CLEARED by
/// the time control reaches the appointment classifier. Calling
/// manoeuvreReleaseEligible() there would read a `false` flag and silently
/// degrade to bare teamComplete — the generation-22 predicate, restored at one
/// of the five sites, invisible in review. The capture is the sanctioned
/// carrier across that clear, so it has an ordering of its own to pin: the
/// assertions below stop either from being re-"simplified" back.
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

/// 4. THE BARRIER. BOTH RETURN PATHS RELEASE ON THE SAME HELPER.
///
/// doReturnSync is the barrier proper. doReturnNav's use is DELIBERATELY
/// REDUNDANT — it is guarded by `!appointment_manoeuvre_`, so `&&`
/// short-circuits and on that path the helper can only ever evaluate its
/// teamComplete branch. It is written as the shared helper anyway so the five
/// sites read as one predicate and a later change to the guard cannot leave an
/// appointment releasing there on the wrong half of it.
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

/// AND THE BARRIER SAYS SO WHEN THE CONTAGION IS WHAT IS HOLDING IT.
///
/// "Everyone is here and we are still waiting" is indistinguishable from a
/// broken release in a log. This is the change's main risk made visible: an
/// unbounded wait under a stricter predicate. The throttled line names the
/// cause so a smoke reader can tell a contagion hold from a defect.
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

/// 5. THE LATCH RELEASE. THE SITE GENERATION 18 GOT WRONG.
///
/// `rendezvous_spent_` is what stops one outage arming twice, and the ONLY
/// path back from spent is a reunion observed on the heartbeat. Gen 18 armed
/// on a weaker predicate than it cleared on; a weaker predicate HERE is that
/// same defect in this generation's terms — the latch would release while a
/// peer is still announcing a break, and the next arm would fire against a
/// team that never regrouped.
///
/// AND THIS IS THE SOLE WRITER OF THE TEAM-BACK DWELL WINDOW (generation 23).
/// Three things have to hold together and each fails silently on its own:
///
///   * the predicate is teamSettled (the five-site rule, above);
///   * it is DWELT, so a flicker cannot clear the latch and hand a second
///     arming to the same outage — the generation-19 ratchet;
///   * and dwellConfirmed is called OUTSIDE the `!appointment_armed_` test, as
///     a standalone statement. This is the load-bearing detail and the one with
///     no signature to protect it. A dwell window that is not ticked does not
///     decay, it FREEZES: written as `!appointment_armed_ && dwellConfirmed(…)`
///     the clock would stop for the whole life of a standing appointment, and
///     the first heartbeat after closeAppointment would find `now - since` far
///     past the confirm time and release on ONE sample — a guard that reads as
///     a guard and filters nothing. See
///     FlickerDwell.AnUnTickedWindowFreezesRatherThanDecaying for the same
///     failure at the level of the function itself.
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
  const size_t call = hb.find("dwellConfirmed(teamSettled(");
  EXPECT_NE(write, std::string::npos)
      << "the team-back dwell is no longer bound to a named standalone "
         "statement in heartbeatTick. If it was inlined into the "
         "`!appointment_armed_` test it FROZE: an un-ticked window keeps `armed` "
         "true with a stale `since_sec`, and the first sample after a standing "
         "appointment closes fires immediately. That is the undwelt behaviour "
         "back again, wearing the guard's name.";
  EXPECT_NE(call, std::string::npos)
      << "heartbeatTick no longer writes a dwell window over teamSettled";
  EXPECT_EQ(countOf(text, "dwellConfirmed(teamSettled("), 1)
      << "a second site writes the team-back window. Two writers over one pair "
         "means asking the question changes its answer — the supersede site "
         "must use dwellHeld.";
  if (write != std::string::npos && call != std::string::npos) {
    // The named statement IS the dwellConfirmed call, not a separate thing that
    // happens to sit near one: nothing but whitespace between them.
    EXPECT_LT(write, call);
    EXPECT_LT(call - write, size_t{40})
        << "`team_back_dwelt` is no longer initialised directly from "
           "dwellConfirmed(teamSettled(...)) — something has been interposed";
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

/// PURSUIT STAYS PER-ROBOT. teamComplete AND NOT teamSettled, ON PURPOSE.
///
/// Pursuit is a robot chasing the peer IT cannot hear. Widening its trigger to
/// teamSettled would send a robot that can hear everybody off chasing on
/// somebody else's behalf, which is not pursuit — it is a rendezvous without a
/// meeting point. The arms are defined by what they do differently and this is
/// the line between two of them.
///
/// The assertion is a ZERO COUNT over STRIPPED text, and doPursue's own comment
/// block spells out "teamComplete AND NOT teamSettled, DELIBERATELY" in prose.
/// Over raw text this test fails against correct code.
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

/// THE PRESENCE CLOCK STAYS ON teamComplete, AND heartbeatTick HOLDS EXACTLY
/// ONE teamSettled.
///
/// `team_last_complete_time_` is this robot's connectivity history and several
/// banked analyses read the columns downstream of it. Redefining it to include
/// peers' claims would change what every one of those numbers means, silently,
/// across a generation boundary. The count pins the other direction too: the
/// only teamSettled in this function is the latch release GROUP D asserts.
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

/// THE EVENT FIELD AND THE CSV COLUMN STAY ON teamComplete.
///
/// `team_complete` is a named column in banked output. Whatever it is computed
/// from must not change between generations without the column changing name,
/// or every cross-generation read of it is comparing two different quantities
/// that happen to share a heading.
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

/// A NON-APPOINTMENT MANOEUVRE MUST NOT BE HELD BY SOMEBODY ELSE'S BREAK.
///
/// The helper is a ternary on `appointment_manoeuvre_`, and the asymmetry is
/// the design, not a shortcut. An APPOINTMENT is a team event: everyone was
/// summoned, so it ends when the team is settled — or, since generation 27,
/// when every expected peer is reachable in the comms closure; the next test
/// pins where that door may and may not appear. A pursuit or a mid-run
/// reconnect is one robot's business with one peer; holding it open because a
/// third robot announced a break would convert every pursuit into an
/// unscheduled team meeting — and the pursuit arm is defined by not having
/// one.
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

/// THE REACHABLE DOOR OPENS AT THE BARRIER, AND ONLY AT THE BARRIER.
///
/// Generation 27. The gen-26 N=3 smoke parked all three robots at the agreed
/// cell to the duration cap — pairs 0-1 and 1-2 up 100%, pair 0-2 up 1.3%
/// through one trunk at 8 m — because the release read the same first-hand
/// mesh the arm reads, and a gathered A-B-C bridge satisfies the closure
/// while never satisfying the mesh. The fix widens the RELEASE alone: the
/// appointment branch of manoeuvreReleaseEligible also opens when every
/// expected peer is reachable in the comms closure (reachablePeerCount).
///
/// The confinement is the test. The arm and the supersede must NOT acquire
/// the door — in an A-B-C bridge the ends are genuinely partitioned on the
/// maps the meeting exists to exchange (there is no map relay), so a
/// closure-keyed arm SUPPRESSES a needed rendezvous, the doctrine at the top
/// of this file. The latch clear in heartbeatTick must stay on the strict
/// mesh predicate, because that strictness is what makes releasing weaker
/// than arming loop-safe. And the producer must stay first-hand, or the
/// claim travels the cycle GossipNeverCarriesTheBit forbids.
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
// THE DEFECT THIS GROUP EXISTS FOR, because a test that only pins the fix is
// half a test. Presence is counted on THREE channels — peerAccounted takes
// `direct`, `finished` or a claim-table entry — and the contagion is read on
// ONE: peerReportsTeamBreak skips a peer that is neither `direct` nor
// `heard_one_way`, and exempts `finished` only for a peer it can already hear.
// So the exemption is applied by the LISTENER, to first-hand knowledge, and a
// robot that never hears the finisher has no way to apply it:
//
//   N=3. A latches done out of contact with B. They were adjacent at spawn, so
//   B holds `finished=false` for A; nothing ages that out, because the field
//   has no TTL by design (TheBitHasNoTtlOfItsOwn). B reads 1 of 2 forever and
//   announces team_incomplete forever. C hears both, exempts A correctly, and
//   is held open by B. Both drive to the agreed cell and wait at a barrier with
//   no cap (rendezvous_appointment_wait_sec = 0) that neither can release, to
//   the duration cap, in every cell where an ordinary thing happens. It is also
//   a REGRESSION: before generation 23 the barrier released on bare
//   teamComplete, which A's stale-but-present entry satisfied.
//
// WHY RELAYING THIS ONE FIELD IS SOUND WHEN RELAYING team_incomplete IS NOT.
// Two independent properties, and the bit needs both:
//
//   MONOTONIC — the publisher latches it (finished_announced_), so a relayed
//   copy can never contradict a first-hand one and the merge is a pure OR that
//   needs no arbitration, no freshness and no tie-break.
//
//   ABOUT ITSELF — it is not a claim about the team, so there is no cycle for
//   it to travel round. C relaying "A is finished" says nothing about C, so A
//   cannot receive its own claim back wearing C's name. That is precisely the
//   deadlock GossipNeverCarriesTheBit forbids for team_incomplete, and the
//   reason that test stays true while these ones exist.
//
// The bit only ever REMOVES a robot from the set the team waits for, and it is
// idempotent, so the worst a spurious relay can do is release a barrier early
// — never hold one open, which is the failure with no bound.
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

/// A RELAYED BIT IS ATTRIBUTED TO THE ROBOT IT IS ABOUT, NOT TO THE RELAY.
///
/// The mirror of TheBitIsStoredAgainstTheSenderThatMadeIt, and it has to come
/// out the OPPOSITE way: team_incomplete is stored against the sender because
/// it is a claim the sender is making, `finished` is stored against the subject
/// because the sender is only carrying it. Getting this backwards would mark
/// every relay finished and empty the team out of the allocator.
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

/// PURE OR. A RELAY THAT STOPS MENTIONING IT DOES NOT CLEAR IT.
///
/// The merge skips false entries entirely rather than assigning them, and that
/// is the whole reason the relay needs no arbitration: two peers relaying
/// different vintages of the same monotonic fact cannot disagree in a way that
/// matters. An assignment here would let the LAST message win, so a peer that
/// had not yet heard the finisher would keep un-finishing it and the team would
/// oscillate between waiting and not waiting.
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

/// NOT AGE-GATED, UNLIKE EVERY OTHER RELAYED FIELD — AND THAT IS THE POINT.
///
/// The position gossip beside it is dropped past gossip_max_age_sec, correctly:
/// a stale position is a wrong answer. A stale `finished` is still the right
/// answer, and the age gate would suppress it exactly when it is needed —
/// a finished robot stops moving, stops planning and goes quiet, so its
/// last-heard entry ages out at the moment the bit starts to matter.
///
/// Both clocks are asserted: the relay's own view of the subject can be
/// arbitrarily old, and the bit survives our own direct TTL afterwards.
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

/// FIRST-HAND STILL WINS FOR THE SENDER'S OWN ENTRY, INCLUDING A FALSE.
///
/// The relay loop skips the sender, so a message from cerd about cerd is an
/// assignment and can clear. That is deliberate, and its limit is stated here
/// rather than papered over: the clear is not DURABLE against peers who still
/// believe the old value, because the next relay re-asserts it. It cannot arise
/// in a campaign run — the publisher latches, so no process ever publishes
/// false after true — and the case it is kept for is a node restart, where the
/// robot itself is the only witness worth believing.
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

/// THE FLEET-LENGTH BACKSTOP COVERS THE NEW ARRAY TOO.
///
/// An over-long array means the sender indexes robots differently than we do,
/// so entry k is not about the robot we think. For a bit that never clears,
/// applying one to the wrong robot is permanent: it would park a live robot out
/// of the allocator for the rest of the run with no way back.
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

/// THE REGRESSION, END TO END, FROM B'S SEAT. This is the test that would have
/// failed before the relay existed.
///
/// A and B are adjacent at spawn, drift apart, and A latches done while out of
/// contact. B's entry for A is stale-false and nothing can age it out. C is the
/// only robot that can carry the news, and one hop is all it has.
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

/// THE PUBLISHED BIT IS A LATCH, NOT A RECOMPUTATION.
///
/// This is the precondition for relaying it at all. The pre-generation-23
/// expression was `coverage_latched_ || state_ == State::DONE`, and only the
/// first of those is monotonic: State::DONE reverts to EXPLORE when the exploit
/// sub-loop runs. That path is unreachable for this campaign
/// (exploitation_enabled: false), and resting a wire invariant on a config
/// value is how an invariant stops being true without anything looking wrong.
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

/// THE PRODUCER FILLS THE RELAY ARRAY, AND FILLS IT WITHOUT THE PAIRING RULE.
///
/// The position gossip beside it is published only for peers whose freshness
/// pairs up; `finished` is published for any peer believed finished, first-hand
/// or relayed, because the bit only ever removes a robot from the set the team
/// waits for and is idempotent in doing so.
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

/// AND THE CONSUMER COPIES IT INTO THE OBSERVATION.
///
/// The counterpart to TheConsumerCopiesTheBitIntoTheObservation. Without this
/// line the wire field is written and never read, which is the failure mode
/// that looks exactly like the generation before it.
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
// THE DEFECT THIS GROUP EXISTS FOR, measured in the generation-23 smoke rather
// than argued from the source. The barrier's release predicate is a COMMS test
// and the drive to the agreed cell only ends on ARRIVAL, and the two halves do
// not compose. In ts4smoke23_n2_mtare_rendezvous_r20_ttl0_seed1 both robots
// committed the same cell at the same t_meet_sec — the timing contract was
// exact — and then:
//
//   atlas arrives at t_rel 349.5, settles its 36 s, releases at 385.58.
//   bestla, dispatched to the same cell at 249.58, arrives at 412.0 — 26 s
//   after atlas left — and stands there alone with waited_sec=271.88.
//
// Neither half misbehaved. atlas released because the radio reaches tens of
// metres in the 2026-09 regime, so `teamSettled` went true while most of
// bestla's walk was still ahead of it; bestla did not release en route because
// doReturnNav's en-route release is gated `!appointment_manoeuvre_` on purpose,
// the CELL being the agreed thing. 40% of a robot's run, in the two treatment
// arms, on the primary endpoint.
//
// WHY THE BIT SAYS "STILL DRIVING" AND NOT "NOT YET ARRIVED", which is the
// whole safety argument and the thing these tests are really guarding. The
// appointment wait cap is unbounded (rendezvous_appointment_wait_sec = 0), so
// any term added here that can stay true forever hangs the team forever. Every
// exit from the drive — arrival, the nav budget, the no-progress watchdog —
// leaves RETURN_NAV, so a bit keyed on the STATE always clears within
// nav_budget_sec whether or not the cell turned out reachable. A bit keyed on
// distance-to-cell would not, and that is the tidier-looking rewrite a later
// reader is most likely to reach for.
// ===========================================================================

/// THE BIT IS FIRST-HAND ONLY, LIKE team_incomplete AND UNLIKE finished.
///
/// It is non-monotonic — it goes true when the drive starts and false when the
/// drive ends — so a relayed copy could hold the barrier off an echo of a claim
/// its own subject has already withdrawn. The relay that GROUP G adds for
/// `finished` is sound precisely because that bit is latched; this one is not,
/// and the two must not be tidied into one mechanism.
///
/// Both directions, because they fail differently: a relay must not smear its
/// own bit onto the robot it is gossiping about, and must not clear a bit it
/// was never told anything about.
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

/// THE ONE-HOP REPORT LANDS UNDER ITS SENDER ONLY, AND CLEARS BY ASSIGNMENT.
///
/// Generation 27's closure door admits peers heard only through a bridge, so
/// the bridge's report (appointment_inbound_seen) is the still-driving veto's
/// only witness for them — but the report is itself non-monotonic twice over:
/// the drive it reports ends, and the reporter's own horizon changes. It must
/// therefore land only under its sender, and the sender's next message must
/// withdraw it by plain assignment. Two merge loops in observe() touch a
/// gossiped robot's row — relayed positions and finished_gossip — and the
/// sender-only claim must hold through BOTH, so this message exercises both.
TEST(Gen23InboundWire, TheSeenReportLandsUnderItsSenderOnlyAndClears) {
  TeamModel m = makeModel();
  // bestla (1) reports that somebody IT hears is still driving, and gossips
  // about cerd (2). Nothing here is a claim by cerd.
  TeamModel::Observation report = gossipMsg(1, robotBit(0), {-1.0, 100.0, 90.0});
  report.appointment_inbound_seen = true;
  // The same message ALSO relays cerd's finished bit. finishedMsg() keeps the
  // two gossip channels apart on purpose; bundling them here is the point —
  // the finished_gossip merge visits cerd's row too, and a careless merge
  // there could smear the seen-report onto the gossip subject just as the
  // position loop could. One message, both loops, one sender-only assertion.
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

/// THE PRODUCER KEYS THE BIT ON THE DRIVING STATE, NOT ON AN ARRIVAL TEST.
///
/// This is the safety argument, pinned. `appointment_manoeuvre_ && state_ ==
/// State::RETURN_NAV` clears at every exit from the drive; a distance test
/// against the agreed cell would stay true forever on a robot whose meeting
/// point turned out unreachable, and the wait cap it feeds is unbounded. The
/// nav budget and the no-progress watchdog exist because that case is routine.
///
/// The assertion is scoped to the assignment statement rather than to the whole
/// function so that it says what it means: the publisher may talk about arrival
/// for other reasons, this field may not.
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

/// THE PRODUCER DERIVES THE ONE-HOP REPORT FROM RAW FIRST-HAND BITS ONLY.
///
/// The one statement in generation 27 where the tidy-looking rewrite is the
/// deadlock: publishing peerReportsInboundToAppointment() here — "surely the
/// wider predicate is the more complete report" — lets A say seen because B
/// says seen. Both then hold their barriers off an echo nothing can clear,
/// which is the armed-state relay trap arriving one field later.
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

/// THE APPOINTMENT BRANCH CARRIES THE SECOND TERM, AND ONLY THE APPOINTMENT
/// BRANCH.
///
/// GROUP F pins the ternary's shape; this pins what generation 23's smoke added
/// to it. The term belongs in manoeuvreReleaseEligible and not in teamSettled:
/// that helper has other consumers — the arm, the supersede, the presence dwell
/// — asking a different question, and none of them should acquire a term about
/// who is still driving.
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

/// THE PREDICATE HAS NO FINISHED EXEMPTION, AND THAT IS NOT AN OVERSIGHT.
///
/// peerReportsTeamBreak() must skip a finished peer because its bit never
/// clears and would hang an unbounded barrier. Copying that skip here for
/// symmetry — the obvious tidy-up, and the reason this test is written as a
/// negative — would reproduce the very defect the field was added for, for
/// exactly the robot most likely to hit it: one that latched coverage on its
/// way to the meeting and is still driving there.
///
/// The liveness gate IS shared, and for the same reason in both: the bit has no
/// TTL of its own, so a peer that goes silent mid-drive must stop holding the
/// barrier. Its silence then holds the release through teamComplete instead,
/// which is the stronger test and the one that can clear.
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

/// THE DOOR'S VETO SEES THE BRIDGE HOP, AND ONLY THE RELEASE PREDICATE
/// CARRIES IT.
///
/// The closure door admits peers this robot cannot hear; their own
/// appointment_inbound never arrives (never relayed), so without the bridge's
/// report the door releases while a third robot is still walking in behind
/// the bridge — the gen-23 lopsided release, reinstated through the door for
/// exactly the peers the door newly admits.
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
// The gen-25 conversion lets an appointment walker join the barrier from the
// road once the team has settled, which is right while the team IS settled.
// Through generation 27 it was one-way, and the ts4 gen-27 N=2 rendezvous
// smoke cell measured the cost: both robots converted mid-drive, the link
// died thirty seconds later and never came back, and both stood 13.0 m and
// 7.2 m short of the agreed cell for the remaining 330 s with no outcome row
// and frozen coverage. These tests pin the return path.
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

  // The flag must be READ here, not merely written at the conversion. Without
  // this conjunct the resume also arms on return-budget and return-no-progress,
  // which give up on the road on purpose: budget-exit resumes into another
  // budget-exit, unbounded. A mutant deleting exactly this term survived the
  // whole suite until the gen-28 review ran it by hand.
  EXPECT_NE(sq.find("appointment_settle_converted_"), std::string::npos)
      << "doReturnSync no longer reads appointment_settle_converted_, so the "
         "resume is not restricted to the settle conversion that stopped the "
         "walk — a budget or no-progress park would resume and re-park.";

  // Both halves of manoeuvreReleaseEligible's "together" disjunction have to
  // be negated here. Dropping either one resumes a robot whose team is fine.
  EXPECT_NE(sq.find("!teamSettled(active)"), std::string::npos)
      << "the mesh half is gone from the resume test: a robot whose team is "
         "settled would be sent back onto the road.";
  EXPECT_NE(sq.find("!teamComplete(reachablePeerCount(), "
                    "rendezvous_expected_peers_)"),
            std::string::npos)
      << "the closure half is gone from the resume test: a bridged team — "
         "exactly what generation 27 releases on — would be torn up and sent "
         "driving again.";
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
