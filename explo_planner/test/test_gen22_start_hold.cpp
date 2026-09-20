// T11. Contract tests for GENERATION 22's pre-mission hold.
//
// WHAT GENERATION 22 CHANGED, in one sentence: no robot plans or navigates for
// the first `mission_start_hold_sec` of the mission, in EVERY arm, so that the
// rendezvous handshake completes while the team is still standing on top of
// itself instead of racing the fleet's own dispersal.
//
// THE DEFECT IT CLOSES was measured, not hypothesised. In ts4 smoke20's N=3
// hybrid cell, bestla's last peer went silent at t+65.1 s and atlas authored the
// upgraded (final, tour-informed) rendezvous triple at t+65.7 s — 0.6 s later.
// bestla never received it, stayed armed on the provisional centroid placeholder
// (cell 44), and atlas and husky armed cell 47. All three departed on
// `appointment-due`, all three ARRIVED, and the run then sat in RETURN_SYNC from
// t~258 s to teardown at t~654 s with a third of the map unknown. Two robots
// waited at one place, one waited at another, and every robot's log said it kept
// the agreement. Realized rate in the banked smokes: 1 cell in 13, N=3 only.
//
// WHY A HOLD AND NOT A PROTOCOL RULE. Committing the placeholder->final upgrade
// atomically across a fleet that can partition mid-round is the Two Generals
// problem, and it has no solution: proposer-commits-on-echo,
// follower-commits-on-receipt, and a third announce round each only move WHICH
// robot is left behind. The two exact fixes are to remove the second generation
// or to remove the window in which the fleet can disperse while the upgrade is
// in flight.
//
// GENERATION 22 ATTEMPTED THE SECOND AND ACHIEVES NEITHER, which this file has
// to say plainly because an earlier revision of this banner claimed otherwise.
// The hold makes the INITIAL agreement — the centroid placeholder — happen
// while the fleet is provably co-located and mutually whole. It does NOT
// contain the upgrade. The companion edit that tried to (1b, below) was
// withdrawn on measurement the same day: see GROUP B's banner for the numbers,
// but in one line, the upgrade is fed by completed exploration steps and a held
// robot completes none, so confining the upgrade to the hold deletes it instead
// of scheduling it. The upgrade race therefore still exists in generation 22,
// bounded only by re-derivation and re-broadcast. Do not read the hold as a fix
// for the split; it is a fix for the placeholder's agreement, and the smoke's
// Q1 still has to look for splits.
//
// THE HOLD IS THREE EDITS, NOT ONE, and two of them are in places a reader of
// the hold itself never opens:
//
//   1. THE GATE.        The WAIT_FOR_MAP -> PLAN start condition gains
//                       `hold_done` as a conjunct, so the state machine cannot
//                       leave its first state early.
//   1b. THE CONFINEMENT — WITHDRAWN 2026-09-17, and GROUP B now asserts its
//                       ABSENCE. It conjoined `within_start_hold` onto the
//                       provisional->final upgrade branch of
//                       maintainRendezvousProposal. Listed here rather than
//                       deleted because it is the obvious repair and will be
//                       proposed again; GROUP B's banner carries the measured
//                       reason it does not work.
//   1c. THE PARAMETER.  Declared, defaulted ON at 60 s, validated finite and
//                       non-negative. A NaN or negative hold compares false
//                       against every elapsed time and silently restores
//                       generation 21.
//                       THE LENGTH IS NOT LOAD-BEARING ANY MORE, and the test
//                       still pins the literal — for agreement between the two
//                       declaration sites, not because 60 is special. With 1b
//                       withdrawn the hold has one job, carrying a handshake
//                       that completes in ~21 ms on a 5 s retry, and 60 s is
//                       twelve retries of margin. The 41-77 s sweep that argued
//                       for 120 s measured the UPGRADE, which the hold no
//                       longer gates; it does not constrain this number. See
//                       mission_start_hold_sec_ in the node.
//   1d. THE HARNESS.    `-p mission_start_hold_sec:=$START_HOLD` must sit in the
//                       single unconditional per-robot argv, not in the arm-
//                       conditional EXTRA array. A hold applied to two of four
//                       arms is a startup cost masquerading as a treatment
//                       effect, which is the exact confound the all-arms design
//                       was chosen to avoid.
//
// AND A FOURTH INVARIANT THAT IS NOT AN EDIT AT ALL: heartbeatTick's rendezvous
// handshake must keep running during the hold. It does today because the
// handshake block sits ABOVE heartbeatTick's `have_active_intent_` return and
// above its state-list early return, and WAIT_FOR_MAP is in neither list. If a
// later edit hoists a state guard to the top of heartbeatTick, the hold inverts
// from a fix into a pure 60 s cost paid by all four arms for nothing, and every
// test above still passes. That is why it is tested here.
//
// It is also the invariant the whole design now rests on, since 1b's withdrawal
// leaves the hold with exactly one deliverable: the placeholder agreed under
// co-location. If the handshake does not run during the hold, generation 22
// delivers nothing at all.
//
// AND A FIFTH, ADDED 2026-09-17 WHEN A REVIEW FOUND IT: the hold must not
// destroy a diagnostic on its way past. The heartbeat beacon is state-gated
// and WAIT_FOR_MAP is a silent state, so parking every robot there for sixty
// seconds — twelve claim TTLs — made the "Heartbeat suppressed ... peers now
// read this robot as MISSING" WARN fire once on every robot of every cell of
// every arm. That WARN is the only in-node evidence separating executor
// suppression from a radio outage, and a line that always fires stops being
// evidence of anything. GROUP E guards the `state_ != State::WAIT_FOR_MAP`
// gate that fixes it, and guards the episode accounting it deliberately does
// NOT suppress. This is the class of damage a hold does that nothing in
// groups A-D would have caught: not a wrong number, a ruined instrument.
//
// GROUP F is the same class again, found the same way — by reading a log the
// hold had changed rather than by reading the diff. During the hold every
// startup precondition is satisfied, yet the fault WARN was keyed on `start`,
// which also conjoins the hold, so the log asserted "Waiting to start: map=1
// pose=1 planning_map=1" eleven times per robot per cell directly beneath the
// INFO saying the hold was progressing normally.
//
// BOTH E AND F WERE INTRODUCED BY THE HOLD AND MISSED BY THE REVIEW THAT
// APPROVED IT. Two independent Opus-5 reviews read the diff; neither found
// either, because neither is visible in the diff — the changed lines are all
// correct, and the damage is to the meaning of lines that did not change. The
// generalisation worth keeping: WHEN A CHANGE ALTERS HOW LONG A NODE SPENDS IN
// A STATE, EVERY MESSAGE CONDITIONED ON THAT STATE CHANGES MEANING, AND NO
// DIFF SHOWS IT. Read a log from the new binary, not only the patch.
//
// SOURCE SCANS, SAME REASON AS T9 AND T10. Every node invariant lives in
// explo_planner_node.cpp, which is not in explo_planner_lib's source list (an
// explicit file list in CMakeLists.txt, not a glob), so nothing links it and no
// test can call it. A scan shows a guard is PRESENT and ORDERED; it cannot show
// it is REACHED. The stated limit is the same one test_gen20_rendezvous.cpp,
// test_gen21_latched_hold.cpp and test_endpoint.cpp's GROUP C carry.
//
// MUTATION-VERIFIED on 2026-09-17, re-verified 2026-09-18 with M15 and M16.
// Sixteen deliberately broken sources were run against this binary:
//
//    M1  `hold_done` dropped from the start condition
//    M2  the hold re-keyed on state entry time instead of missionElapsed()
//    M3  the PLAN transition hoisted out of `if (start)`
//    M4  a clock term conjoined back onto the upgrade branch (the withdrawn 1b)
//    M5  the `rendezvous_held_provisional_` disjunct dropped entirely, so the
//        pair is frozen at the first derive and never upgrades
//    M6  a clock term conjoined onto the INITIAL derive
//    M7  the hold's parameter declaration removed (hard-coded)
//    M8  the hold's finiteness validation removed
//    M9  the member default zeroed (disagrees with the parameter default)
//   M10  the parameter default zeroed (disagrees with the member default)
//   M11  maintainRendezvousProposal moved below heartbeatTick's intent return
//   M12  `-p mission_start_hold_sec:=` removed from the harness argv
//   M13  the harness START_HOLD default zeroed
//   M14  the WAIT_FOR_MAP gate dropped from the suppression WARN, restoring the
//        line that fired on every robot of every cell
//   M15  the state test moved back INTO the eligibility condition, so the latch
//        is gated by it rather than consumed before it — the instant-scoped
//        first cut, which is a real regression rather than a synthetic one: it
//        shipped in generation 22 and fired in 2 of 6 smoke cells
//   M16  the startup-fault WARN re-keyed from !preconds_met back onto !start,
//        restoring "Waiting to start: map=1 pose=1 planning_map=1" for the full
//        length of the hold
//
// Sixteen mutations, sixteen failures, each in the test named for it, with both
// files restored byte-exact afterwards (sha256 re-checked). Re-run that before
// trusting any later edit: this binary reads the node and the harness at RUN
// time, so it is edit-run-restore, no rebuild.
//
// M15 IS THE ONE TO KEEP. Every other mutation here is a defect someone might
// introduce; M15 is a defect that was actually introduced, by the fix for M14,
// and survived a green suite plus a passing smoke cell. It is the standing
// evidence that "the guard is present" and "the guard holds for as long as it
// needs to" are different assertions.
//
// stripComments() is not optional here either, and this file is the worst case
// for it so far: the banner above quotes `hold_done`, `within_start_hold` and
// `-p mission_start_hold_sec:=$START_HOLD` in prose, and so do the node's own
// comment blocks. A raw-text scan would pass on a node with every guard deleted
// and the comments left behind, which is the most likely way this regresses:
// comments are what a "cleanup" edit keeps.
//
// The withdrawal of 1b makes this sharper rather than softer. GROUP B asserts
// that `within_start_hold` occurs ZERO times in maintainRendezvousProposal, and
// that function's comment now spends forty lines explaining the withdrawn
// conjunct — naming the identifier would make the test fail against correct
// code, so the node's comment deliberately spells it out longhand instead
// (`&& missionElapsed() < mission_start_hold_sec_`). A count-based assertion
// over un-stripped text would be hostage to prose; over stripped text it is a
// statement about the code. If that count ever has to tolerate a comment
// mention, the fix is a better stripper, not a looser bound.
//
// The SHELL scan below is deliberately NOT comment-stripped, because a C comment
// stripper turns `${VAR#default}` and every `//` in a path into nonsense. It is
// anchored structurally instead: the assertion is about membership of one
// backslash-continued argv, and a comment line cannot be part of one.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

/// The source with `//` and `/* */` comments removed, string literals preserved.
/// Duplicated from test_gen20_rendezvous.cpp and test_gen21_latched_hold.cpp
/// rather than shared: these are standalone source-scan binaries that link
/// nothing of their own, and a shared header between test executables whose
/// whole job is to be independently re-runnable against a mutated node would
/// make one mutation break all three.
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

/// The `case State::X:` arm of a switch, from its label to the next `case
/// State::` label. Used instead of functionBody because the start gate is a
/// switch arm inside tick(), not a function, and because the FIRST
/// `case State::WAIT_FOR_MAP:` in the file belongs to stateName().
std::string caseArm(const std::string& body, const std::string& label) {
  const size_t at = body.find(label);
  if (at == std::string::npos) return "";
  const size_t next = body.find("case State::", at + label.size());
  return body.substr(at, next == std::string::npos ? std::string::npos
                                                   : next - at);
}

/// The whole backslash-continued logical line containing `needle`, raw. Walks
/// up while the PREVIOUS physical line ends in `\` and down while THIS one
/// does. A shell comment cannot be part of such a block, which is what lets the
/// harness assertions run without a comment stripper.
std::string continuedBlockContaining(const std::string& text,
                                     const std::string& needle) {
  const size_t at = text.find(needle);
  if (at == std::string::npos) return "";
  // Start of the physical line holding the needle.
  size_t begin = text.rfind('\n', at);
  begin = (begin == std::string::npos) ? 0 : begin + 1;
  // Walk up: keep the previous line while it ends with a backslash.
  while (begin > 0) {
    const size_t prev_end = begin - 1;               // the '\n' before us
    if (prev_end == 0 || text[prev_end - 1] != '\\') break;
    size_t prev_begin = (prev_end == 0) ? std::string::npos
                                        : text.rfind('\n', prev_end - 1);
    begin = (prev_begin == std::string::npos) ? 0 : prev_begin + 1;
  }
  // Walk down: keep consuming lines while each ends with a backslash.
  size_t end = text.find('\n', at);
  while (end != std::string::npos && end > 0 && text[end - 1] == '\\') {
    end = text.find('\n', end + 1);
  }
  if (end == std::string::npos) end = text.size();
  return text.substr(begin, end - begin);
}

std::string harnessSource() { return readFile(EXPLO_PLANNER_SIM_SH); }

}  // namespace

// ===========================================================================
// GROUP A. THE GATE: the state machine cannot leave WAIT_FOR_MAP early.
// ===========================================================================

/// THE HOLD IS A CONJUNCT OF `start`, NOT A STATEMENT BEFORE IT.
///
/// The distinction is the whole test. Sequencing the hold ahead of the
/// preconditions ("wait 60 s, then wait for the map") makes the release time
/// hold + map_latency; ANDing them makes it max(hold, map_latency), which is
/// what the design says and what the campaign's wall-clock budget assumes. It
/// also fails safe in the direction that matters: a robot whose map arrives at
/// t+55 s must not get a 5 s hold.
TEST(Gen22StartHold, TheHoldIsAConjunctOfTheStartCondition) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string tick = functionBody(text, "void ExploPlannerNode::tick()");
  ASSERT_FALSE(tick.empty())
      << "the scan found no tick() definition — it has been renamed or "
         "reshaped and this test has stopped testing anything";
  const std::string arm = caseArm(tick, "case State::WAIT_FOR_MAP:");
  ASSERT_FALSE(arm.empty())
      << "tick() no longer has a WAIT_FOR_MAP arm";

  // THE LITERAL MOVED ON 2026-09-18 AND THIS TEST MOVED WITH IT. Until then
  // the whole condition was one expression, `const bool start = have_map_ &&
  // have_pose_ && (!use_planning_map_ || have_plan_map_) && hold_done;`, and
  // both assertions below read halves of that one line. Generation 23 gave the
  // preconditions their own name so the "Waiting to start" WARN could be keyed
  // on them WITHOUT the hold — see
  // TheWaitingToStartWarnFiresOnlyOnAMissingPrecondition, which owns that half.
  //
  // The rename is a pure refactor of this expression and the invariant here is
  // untouched: both terms are still conjuncts of ONE boolean, evaluated
  // unconditionally, so release is at max(hold, map latency). What this test
  // must not become is a token search — `hold_done` and `have_map_` appearing
  // somewhere in the arm would be satisfied by a sequenced wait too. The
  // literals are asserted precisely so that nothing can sit between the terms.
  EXPECT_NE(arm.find("const bool preconds_met = have_map_ && have_pose_ &&"),
            std::string::npos)
      << "the precondition set has been reshaped; re-read it before trusting "
         "any assertion below";
  EXPECT_NE(arm.find("(!use_planning_map_ || have_plan_map_);"),
            std::string::npos)
      << "the planning-map term is no longer the last conjunct of "
         "preconds_met. If use_planning_map_ is on and have_plan_map_ has "
         "dropped out, the robot starts planning against a map domain it has "
         "not received — the starvation mode that fused-map fix was for.";
  EXPECT_NE(arm.find("const bool start = preconds_met && hold_done;"),
            std::string::npos)
      << "hold_done is no longer conjoined onto the preconditions in a single "
         "expression. If it was moved to a sequenced wait above them the "
         "release time becomes hold + map latency instead of max(hold, map "
         "latency); if it was dropped altogether this is generation 21 and the "
         "fleet can disperse mid-agreement again.";
}

/// THE HOLD IS MEASURED FROM missionElapsed(), WITH `>=`.
///
/// Two claims, and the first is the load-bearing one. missionElapsed() latches
/// its baseline on the first tick with a positive clock and returns -1.0 before
/// then — which is BELOW any non-negative hold, so a robot with no /clock yet
/// holds, the safe direction. Re-keying this on state entry time or on
/// this->now() breaks that: under use_sim_time this->now() reads exactly 0
/// until the first /clock message lands, and a wall-clock delta measures the
/// launch sequence rather than the mission.
///
/// The `>=` is the smaller claim but it is asserted literally because the
/// complement of this predicate is `within_start_hold` in
/// maintainRendezvousProposal, and the two must partition the timeline exactly
/// once — see the test that pairs them.
TEST(Gen22StartHold, TheHoldIsMeasuredFromMissionElapsedWithGreaterEqual) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string tick = functionBody(text, "void ExploPlannerNode::tick()");
  ASSERT_FALSE(tick.empty()) << "the scan found no tick() definition";
  const std::string arm = caseArm(tick, "case State::WAIT_FOR_MAP:");
  ASSERT_FALSE(arm.empty()) << "tick() no longer has a WAIT_FOR_MAP arm";

  EXPECT_NE(arm.find("const double held_for = missionElapsed();"),
            std::string::npos)
      << "the hold is no longer measured from missionElapsed(). A wall clock "
         "or a state-entry delta measures the launch sequence, not the "
         "mission, and loses the -1.0-before-/clock behaviour that makes a "
         "clockless robot hold rather than start.";
  EXPECT_NE(arm.find("const bool hold_done  = held_for >= mission_start_hold_sec_;"),
            std::string::npos)
      << "hold_done is no longer `held_for >= mission_start_hold_sec_`. It is "
         "the exact complement of within_start_hold in "
         "maintainRendezvousProposal and the pair must partition the timeline.";
}

/// WAIT_FOR_MAP HAS EXACTLY ONE EXIT AND `start` GUARDS IT.
///
/// The gate is only a gate if there is nothing to walk around. This asserts
/// the arm contains a single transitionTo and that it is inside `if (start)`,
/// so a later edit that adds a second escape — a timeout, a "start anyway"
/// diagnostic path, a retry that gives up — fails here rather than in a
/// campaign six hours in.
TEST(Gen22StartHold, WaitForMapHasOneExitAndStartGuardsIt) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string tick = functionBody(text, "void ExploPlannerNode::tick()");
  ASSERT_FALSE(tick.empty()) << "the scan found no tick() definition";
  const std::string arm = caseArm(tick, "case State::WAIT_FOR_MAP:");
  ASSERT_FALSE(arm.empty()) << "tick() no longer has a WAIT_FOR_MAP arm";

  EXPECT_EQ(countOf(arm, "transitionTo("), 1)
      << "WAIT_FOR_MAP has more than one exit. The pre-mission hold guards "
         "exactly one of them; a second route out of the first state is a "
         "robot that starts exploring before the team has agreed where to "
         "meet.";
  const size_t guard = arm.find("if (start) {");
  const size_t go    = arm.find("transitionTo(State::PLAN, \"startup-preconditions-met\")");
  ASSERT_NE(guard, std::string::npos)
      << "`if (start) {` is gone; the exit is no longer guarded by the "
         "condition the hold is a conjunct of";
  ASSERT_NE(go, std::string::npos)
      << "the WAIT_FOR_MAP -> PLAN transition is gone or renamed";
  EXPECT_LT(guard, go)
      << "the PLAN transition is no longer inside `if (start)`, so the hold "
         "(and every other startup precondition) gates nothing";
}

// ===========================================================================
// GROUP B. THE DERIVE GATE MUST NOT READ THE HOLD.
//
// This group asserts the ABSENCE of an edit, which is unusual enough to say
// why. On 2026-09-17 the hold shipped with a companion conjunct confining the
// provisional->final upgrade to the hold window:
//
//     (rendezvous_held_provisional_ && within_start_hold)
//
// It is the obvious fix for the split — author the upgrade only while the team
// is co-located, and there is no partition to be on the wrong side of — and it
// was withdrawn the same day because it does not schedule the upgrade earlier,
// it prevents it entirely. Measured over every banked cell carrying
// agreed_provisional (5 campaigns, 10 cells, 28 robot-runs, 58 agreement rows):
//
//   * the step counter at the first NON-provisional commit is never below 2;
//     the observed distribution is {2, 3, 6, 7, 8} and no upgrade anywhere in
//     the corpus was authored at step 0 or step 1;
//   * on EVERY provisional commit the proposer's provenance reads
//     candidates=1, rejected_unreachable=0, rejected_excluded=0 — the pool was
//     EMPTY, not filtered, so the limit is not reachability (which co-location
//     would fix) but the absence of any tour to draw a candidate from;
//   * two banked upgrades landed at t=311.1 s and t=353.6 s, more than 250 s
//     after the provisional they replaced.
//
// A robot held in WAIT_FOR_MAP completes zero steps and so grows none of the
// EXPLORING cells the allocator builds tours over. The upgrade branch is also
// the only site that ever clears rendezvous_held_provisional_, so a window that
// closes first freezes the pair for the whole run and both scheduled arms
// quietly become "meet at the team's initial centroid" — while passing every
// unanimity check, because a fleet that unanimously agrees a placeholder is
// still unanimous. That is a silent null wearing a treatment's name, which is
// the failure mode this file exists to make loud.
//
// So the tests below fail if the conjunct comes back, in either branch.
// ===========================================================================

/// THE UPGRADE IS **NOT** TIME-CONFINED.
///
/// The gate is a PURE DISJUNCTION: derive if there is no pair, or re-derive for
/// as long as the pair is only a placeholder, whenever the team is mutually
/// whole. No clock term. See the group banner for the measurement.
///
/// PINNED ON THE SHAPE, NOT ON THE TEXT (2026-09-18). Until generation 23 this
/// read the literal string `(!rendezvous_held_.valid() ||
/// rendezvous_held_provisional_))`, and generation 23 broke it by adding a
/// third disjunct, the post-reunion re-decide (since removed: its follower
/// side was never built, so every re-decided pair was refused fleet-wide).
/// Widening the gate is legal here. Every failure this test was written to
/// catch narrows it, by conjoining a term onto the upgrade; a literal pin
/// cannot tell the two apart and fails on the safe direction while a reader
/// assumes it caught the unsafe one. So the assertion below extracts the
/// parenthesised group and demands it contain no `&&` at all: new disjuncts
/// are free, any conjunct is a failure, wherever inside the group it is
/// spelled and whatever it is named.
///
/// What bounds the split instead is stated in the derive gate's own comment and
/// is weaker than confinement, deliberately: the proposer keeps re-deriving
/// while provisional and publishTeamWorld re-broadcasts the held pair every
/// cycle, so a follower that misses an upgrade adopts it when the link returns.
/// The disagreement lasts as long as the partition, not as long as the run.
/// Two Generals says no protocol does better; this one at least converges.
TEST(Gen22StartHold, TheUpgradeIsNotConfinedToTheHold) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body = functionBody(
      text, "void ExploPlannerNode::maintainRendezvousProposal(");
  ASSERT_FALSE(body.empty())
      << "the scan found no maintainRendezvousProposal definition — it has "
         "been renamed or reshaped and this test has stopped testing anything";

  const size_t open = body.find("(!rendezvous_held_.valid() ||");
  ASSERT_NE(open, std::string::npos)
      << "the derive gate no longer opens with `(!rendezvous_held_.valid() ||`. "
         "Either the upgrade has been removed or the gate has been reshaped "
         "past what this scan can read — do not relax this, read the gate.";
  // Walk to the matching close paren. The group is the whole disjunction and
  // nothing else, so counting depth is enough; there are no string or character
  // literals inside a gate made of member names, and comments are already gone.
  size_t depth = 0, close = std::string::npos;
  for (size_t i = open; i < body.size(); ++i) {
    if (body[i] == '(') ++depth;
    else if (body[i] == ')' && --depth == 0) { close = i; break; }
  }
  ASSERT_NE(close, std::string::npos)
      << "unbalanced parentheses in the derive gate — the scan cannot read it";
  const std::string group = body.substr(open, close - open + 1);

  EXPECT_NE(group.find("rendezvous_held_provisional_"), std::string::npos)
      << "the provisional re-derive disjunct is gone from the derive gate: "
      << group
      << "\nThe upgrade branch is the only site that ever clears "
         "rendezvous_held_provisional_, so without it the pair freezes on the "
         "centroid placeholder for the whole run and both scheduled arms become "
         "\"meet at the team's initial centroid\" while passing every unanimity "
         "check.";
  EXPECT_EQ(countOf(group, "&&"), 0)
      << "a conjunct has been added to the derive gate: " << group
      << "\nIf that term is a clock, the upgrade does not happen at all: it "
         "needs the allocator to have tours, tours need completed exploration "
         "steps, and a robot inside the hold completes zero. Every banked "
         "upgrade was authored at step >= 2. The rendezvous and hybrid arms "
         "would both freeze on the centroid placeholder for the whole run and "
         "still pass every agreement check. Extra DISJUNCTS are fine; it is "
         "narrowing the gate that this test forbids.";
}

/// NEITHER BRANCH OF THE DERIVE GATE READS THE HOLD.
///
/// Enforced on the identifier rather than on the branch shape, so that a
/// re-added conjunct is caught wherever it is spelled and whichever disjunct it
/// is attached to. `within_start_hold` was the name it had; a zero count is the
/// invariant, and any clock term reintroduced under a different name will fail
/// the gate-shape assertion above instead.
///
/// The INITIAL derive must stay unconfined for a second and independent reason:
/// a run whose map arrives after the hold would otherwise leave it with no pair
/// at all, every arming would refuse, and the rendezvous arm would be a silent
/// copy of `off`.
TEST(Gen22StartHold, TheDeriveGateHasNoClockTerm) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body = functionBody(
      text, "void ExploPlannerNode::maintainRendezvousProposal(");
  ASSERT_FALSE(body.empty())
      << "the scan found no maintainRendezvousProposal definition";

  EXPECT_NE(body.find("(!rendezvous_held_.valid() ||"), std::string::npos)
      << "the initial derive is no longer a bare disjunct. If it has been "
         "conjoined with the hold, a run whose map arrives after the hold "
         "never derives a pair at all, every arming refuses, and the "
         "rendezvous arm silently becomes a copy of `off`.";
  EXPECT_EQ(countOf(body, "within_start_hold"), 0)
      << "within_start_hold is back in maintainRendezvousProposal. It was "
         "withdrawn on 2026-09-17 because confining the upgrade to the hold "
         "deletes the upgrade rather than scheduling it — see the group "
         "banner for the step-counter measurement. Do not re-add it; if the "
         "split needs closing, close it somewhere that does not depend on a "
         "stationary robot growing a map.";
}

/// THE AGREEMENT HANDSHAKE RUNS DURING THE HOLD.
///
/// Not an edit — an invariant the hold silently depends on, which is exactly
/// the kind that rots. heartbeatTick calls maintainRendezvousProposal ABOVE
/// both of its early returns (`!have_active_intent_` and the state list), and
/// WAIT_FOR_MAP is in neither list, so the handshake keeps running while the
/// robot is held. If a later edit hoists a state guard to the top of
/// heartbeatTick — a plausible "don't beat before we've started" cleanup — the
/// hold inverts from a fix into a 60 s cost paid by all four arms for nothing,
/// and every other test in this file still passes.
TEST(Gen22StartHold, TheHandshakeRunsAboveHeartbeatTicksEarlyReturns) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::heartbeatTick()");
  ASSERT_FALSE(body.empty())
      << "the scan found no heartbeatTick definition — it has been renamed or "
         "reshaped and this test has stopped testing anything";

  const size_t handshake = body.find("maintainRendezvousProposal(rzv_mutual);");
  const size_t intent    = body.find("if (!have_active_intent_) return;");
  const size_t states    = body.find("if (state_ != State::NAVIGATE &&");
  ASSERT_NE(handshake, std::string::npos)
      << "heartbeatTick no longer runs the rendezvous handshake round";
  ASSERT_NE(intent, std::string::npos)
      << "heartbeatTick's have_active_intent_ return is gone; re-read the "
         "function before trusting this ordering";
  ASSERT_NE(states, std::string::npos)
      << "heartbeatTick's state-list early return is gone; re-read the "
         "function before trusting this ordering";

  EXPECT_LT(handshake, intent)
      << "the handshake now sits below `if (!have_active_intent_) return;`. A "
         "held robot has no active intent, so it would never agree a "
         "rendezvous during the hold — the hold becomes a 60 s cost in all "
         "four arms and buys nothing.";
  EXPECT_LT(handshake, states)
      << "the handshake now sits below heartbeatTick's state-list early "
         "return, and WAIT_FOR_MAP is not in that list — so the handshake is "
         "dead for the whole duration of the hold.";
  // EVERY WAIT_FOR_MAP TEST IN THIS FUNCTION MUST SIT BELOW THE HANDSHAKE.
  //
  // This assertion was `countOf(body, "State::WAIT_FOR_MAP") == 0` until
  // 2026-09-17, and it fired — correctly — on GROUP E's suppression-WARN gate,
  // which is a WAIT_FOR_MAP test in this function that is NOT a hazard. So the
  // blanket ban is relaxed to a positional one, which is what the hazard
  // actually is: the dangerous edit is a state guard hoisted ABOVE
  // maintainRendezvousProposal ("don't beat before we've started"), because
  // that kills the agreement round for the whole hold and every other test in
  // this file still passes. A WAIT_FOR_MAP test below the handshake cannot do
  // that, by construction — the call has already returned.
  //
  // Relaxing an assertion because your own edit tripped it is the move that
  // turns a test suite into decoration, so the relaxation is deliberately the
  // SMALLEST one that admits the new gate: not "allow WAIT_FOR_MAP", but
  // "allow it only where it provably cannot skip the handshake".
  for (size_t at = body.find("State::WAIT_FOR_MAP");
       at != std::string::npos;
       at = body.find("State::WAIT_FOR_MAP", at + 1)) {
    EXPECT_GT(at, handshake)
        << "heartbeatTick tests State::WAIT_FOR_MAP at offset " << at
        << ", ABOVE the maintainRendezvousProposal call at " << handshake
        << ". The hold depends on the heartbeat being state-blind up to the "
           "handshake: a guard here means a held robot never agrees a "
           "rendezvous, so the hold becomes a 60 s cost in all four arms and "
           "buys nothing. Tests BELOW the handshake are fine — see GROUP E.";
  }
}

// ===========================================================================
// GROUP C. THE PARAMETER.
// ===========================================================================

/// THE HOLD IS A DECLARED AND VALIDATED PARAMETER.
///
/// The validation is not boilerplate. `held_for >= mission_start_hold_sec_` is
/// false for every finite held_for when the right-hand side is NaN, so a NaN
/// hold is an infinite hold — the robot never leaves WAIT_FOR_MAP and the cell
/// burns its whole wall-clock budget looking like a dead mapper. A negative
/// hold is the opposite and worse: it is true immediately, so the run is
/// generation 21 wearing a generation 22 manifest.
TEST(Gen22StartHold, TheHoldIsADeclaredAndValidatedParameter) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  EXPECT_NE(text.find("dp(\"mission_start_hold_sec\", 60.0)"), std::string::npos)
      << "mission_start_hold_sec is no longer a declared parameter. "
         "Hard-coding it means the harness knob is inert and the manifest "
         "line records a value the binary never read.";
  EXPECT_NE(text.find("if (!std::isfinite(mission_start_hold_sec_) ||\n"
                      "      mission_start_hold_sec_ < 0.0) {"),
            std::string::npos)
      << "the hold's finiteness/sign validation is gone. NaN makes the hold "
         "infinite (every comparison false); negative makes it zero-length "
         "and silently restores generation 21.";
  EXPECT_NE(text.find("throw std::runtime_error(\"mission_start_hold_sec must "
                      "be finite and >= 0\")"),
            std::string::npos)
      << "the validation no longer throws, so a bad value is a WARN in a log "
         "nobody reads rather than a cell that refuses to run";
}

/// THE MEMBER DEFAULT AND THE PARAMETER DEFAULT AGREE, AND BOTH ARE ON.
///
/// Two failure modes, one test:
///
///   * DISAGREEING defaults mean the hold depends on which construction path
///     ran. Any node built without the parameter — a bench, a unit harness, a
///     future launch file — silently gets the member's value, and "the
///     campaign got the fix, the reproduction didn't" is the hardest class of
///     result to debug because both runs report the same generation.
///   * A default of ZERO is the deliberate opposite of the two mission-return
///     knobs beside it, which default OFF so this binary reproduces banked
///     behaviour unless a campaign opts in. This one defaults ON because it
///     closes a defect that split a fleet: a campaign that forgets the knob
///     should get the fix, not the split. That asymmetry is the reason
///     generation 22 is a new generation and is not poolable with anything
///     earlier.
TEST(Gen22StartHold, TheMemberAndParameterDefaultsAgreeAndAreOn) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  EXPECT_NE(text.find("double mission_start_hold_sec_  = 60.0;"),
            std::string::npos)
      << "the member default is not 60.0. It must match the parameter default "
         "exactly, or the hold depends on which construction path ran; it must "
         "be non-zero, because this knob defaults ON by design; and it must "
         "reach past the 41-77 s window in which the banked cells make their "
         "tour-informed commit, or confining the upgrade to the hold means the "
         "upgrade never fires at all.";
  EXPECT_NE(text.find("dp(\"mission_start_hold_sec\", 60.0)"), std::string::npos)
      << "the parameter default is not 60.0; see above — the two defaults are "
         "asserted together because a mismatch is invisible at runtime";
}

// ===========================================================================
// GROUP D. THE HARNESS: all four arms, or the hold is a confound.
// ===========================================================================

/// THE HOLD IS PASSED IN THE UNCONDITIONAL PER-ROBOT ARGV.
///
/// run_explo_sim_rviz.sh has two ways to hand a parameter to the node: the one
/// `ros2 run` argv every robot gets, and the arm-conditional `EXTRA` array that
/// carries things like `rendezvous_schedule_enable:=true`. The hold must be in
/// the first. In the second it would apply to the rendezvous and hybrid arms
/// only, and then the 60 s appears in exactly the two arms under test — a
/// startup cost reported as a treatment effect, which is the confound the
/// all-four-arms design was chosen to avoid.
///
/// Asserted by membership of the backslash-continued block that carries
/// `-p reconnect_mode:=$MODE_ARG`, which is the unconditional argv by
/// definition. Raw text, no comment stripping — see the file banner.
TEST(Gen22StartHold, TheHarnessPassesTheHoldToEveryArm) {
  const std::string sh = harnessSource();
  ASSERT_FALSE(sh.empty()) << "cannot read " << EXPLO_PLANNER_SIM_SH;

  const std::string argv =
      continuedBlockContaining(sh, "-p reconnect_mode:=$MODE_ARG");
  ASSERT_FALSE(argv.empty())
      << "the harness no longer has a `-p reconnect_mode:=$MODE_ARG` line, so "
         "this test cannot locate the unconditional per-robot argv; re-anchor "
         "it before trusting anything below";

  EXPECT_NE(argv.find("-p mission_start_hold_sec:=$START_HOLD"),
            std::string::npos)
      << "mission_start_hold_sec is not in the unconditional per-robot argv. "
         "If it moved into the arm-conditional EXTRA array the hold applies to "
         "rendezvous and hybrid only, and a 60 s startup cost shows up as a "
         "treatment effect in exactly the two arms under test.";
  EXPECT_EQ(countOf(sh, "-p mission_start_hold_sec:="), 1)
      << "there is more than one launch site for mission_start_hold_sec. Two "
         "sites is how an arm-conditional override gets added without the "
         "membership assertion above ever failing.";
}

/// THE HARNESS DEFAULTS THE HOLD ON AND RECORDS IT IN THE MANIFEST.
///
/// `START_HOLD=0` is a legal and documented setting — it restores generation
/// 21 exactly, which is the only honest way to measure what the hold cost. It
/// must not be the DEFAULT, and whichever value ran must reach the manifest,
/// because a campaign whose manifest does not state its hold cannot be
/// compared against one that does.
TEST(Gen22StartHold, TheHarnessDefaultsTheHoldOnAndRecordsIt) {
  const std::string sh = harnessSource();
  ASSERT_FALSE(sh.empty()) << "cannot read " << EXPLO_PLANNER_SIM_SH;

  EXPECT_NE(sh.find("START_HOLD=\"$(flt \"${START_HOLD:-60}\")\""),
            std::string::npos)
      << "the harness default for START_HOLD is not 60. A default of 0 runs "
         "generation 21 from a generation 22 binary, and the manifest would "
         "not distinguish the two cases for anyone reading it later.";
  EXPECT_NE(sh.find("echo \"mission_start_hold_sec=$START_HOLD\""),
            std::string::npos)
      << "the manifest no longer records the hold. It is a per-campaign knob "
         "that changes the endpoint, so a cell whose manifest omits it is not "
         "comparable to one whose manifest states it.";
}

// ===========================================================================
// GROUP E. THE HOLD MUST NOT DESTROY A DIAGNOSTIC.
// ===========================================================================
//
// The heartbeat beacon is STATE-GATED: heartbeatTick() publishes an intent
// only from NAVIGATE/INTEGRATE/EXPLOIT_*/RETURN_*/PURSUE/PROXIMITY_HOLD/
// RETURN_HOME/DONE. Every other state is a silent one, and a silent robot is
// read by its peers as MISSING once coord_claim_ttl_sec (5 s) has passed —
// indistinguishable, from the receiving side, from a radio outage. The WARN in
// the !beaconing branch exists to make that distinguishable from INSIDE the
// node, and the analysis uses it to classify each peer-missing window as
// suppression rather than outage. It is the only in-node evidence of the
// difference.
//
// WAIT_FOR_MAP IS A SILENT STATE, AND GENERATION 22 PARKS EVERY ROBOT IN IT
// FOR SIXTY SECONDS. Twelve claim TTLs. Before the gate this test guards, the
// WARN therefore fired exactly once on every robot of every cell of every arm
// — an unconditional line. That is worse than a missing diagnostic, because a
// string that always appears reads as background and gets filtered, and the
// filter that hides the sixty startup lines hides the mid-run one that matters.
//
// The gate is `state_ != State::WAIT_FOR_MAP` and it is clock-free on purpose.
// WAIT_FOR_MAP is assigned once, at the member initialiser, and there is no
// transitionTo(State::WAIT_FOR_MAP) anywhere in the node — GROUP A's
// WaitForMapHasOneExitAndStartGuardsIt is the other half of that fact — so the
// state IS the predicate "has not started its mission". Writing the gate as
// `missionElapsed() < mission_start_hold_sec_` instead would have been a third
// site reading the hold's clock, and would go wrong the moment a startup
// precondition other than the hold (a late map, a late pose) keeps a robot in
// WAIT_FOR_MAP past the hold — which is a case where the WARN is just as
// uninformative and would come back.
//
// THE GATE MUST CONSUME THE LATCH, NOT SKIP THE WARN — corrected 2026-09-18
// after the first cut shipped and was caught in flight.
//
// The first version tested the state at the WARN site and did nothing else:
//     if (!hb_suppress_warned_ && held >= ttl && state_ != WAIT_FOR_MAP)
// which is wrong by roughly one second, because the state and the episode do
// not end together. The state leaves WAIT_FOR_MAP the moment the hold expires;
// the heartbeat stays suppressed until the executor next runs. In between, the
// robot is in PLAN with `held` still carrying the full sixty seconds — so a
// tick landing in that window saw a passing state guard, an unset latch and
// held >= TTL, and printed "Heartbeat suppressed 60.1 s in state PLAN": the
// precise line the gate was added to prevent.
//
// IT WAS INTERMITTENT, WHICH IS WORSE THAN ALWAYS. Measured on the gen-22
// smoke, it fired in 2 of 6 cells — so checking one clean cell was enough to
// conclude the gate worked, and the two that fired looked like genuine
// mid-run suppression at t≈60 s rather than a startup artifact.
//
// The correction sets hb_suppress_warned_ = true for the whole eligible branch
// and emits the WARN only outside WAIT_FOR_MAP. That works because the latch is
// reset when an episode BEGINS, not when one ends, so consuming it exempts this
// episode and only this episode; the next suppression clears it and warns
// normally. No clock, no second member, and no dependence on the ordering of
// two asynchronous transitions — which is what made the first cut fragile.
//
// WHY THE EPISODE ACCOUNTING IS DELIBERATELY LEFT ALONE. hb_suppressed_,
// hb_suppress_start_ and the "Heartbeat resumed after %.1f s suppressed" INFO
// are untouched, so the hold still leaves exactly one line in the log stating
// its measured length. Suppressing the episode as well would have removed the
// only direct observation of the hold from the node's own log, which is the
// opposite of what this fix is for.

/// A regression here is silent in the worst way: the campaign still runs, the
/// numbers still come out, and the suppression-vs-outage classification is
/// quietly built on a WARN that fires for everyone.
TEST(Gen22StartHold, TheSuppressionWarnIsGatedOutOfWaitForMap) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::heartbeatTick()");
  ASSERT_FALSE(body.empty()) << "the scan found no heartbeatTick() definition";

  const size_t warn = body.find("Heartbeat suppressed %.1f s in state %s");
  ASSERT_NE(warn, std::string::npos)
      << "the suppression WARN is gone entirely. It is the only in-node "
         "evidence separating executor-induced suppression from a radio "
         "outage; deleting it does not fix the noise, it removes the signal.";

  const size_t guard = body.find("state_ != State::WAIT_FOR_MAP");
  ASSERT_NE(guard, std::string::npos)
      << "the suppression WARN is no longer gated out of WAIT_FOR_MAP. With "
         "mission_start_hold_sec = 60 s and a 5 s claim TTL, every robot of "
         "every cell sits twelve TTLs in that state, so this WARN fires "
         "unconditionally and stops discriminating anything.";
  EXPECT_LT(guard, warn)
      << "`state_ != State::WAIT_FOR_MAP` appears in heartbeatTick() but "
         "AFTER the WARN text, so it is not the condition guarding it.";

  // The episode accounting must survive: the resumed-INFO is what leaves the
  // hold's measured length in the log once the robot starts beaconing.
  EXPECT_NE(body.find("Heartbeat resumed after %.1f s suppressed"),
            std::string::npos)
      << "the resumed-INFO is gone, so nothing in the node's log states how "
         "long the pre-mission hold actually held. Gate the WARN, not the "
         "episode.";
}

/// The race this guards is the one that got through the first time. The state
/// guard alone is satisfiable while still printing the WARN, so a test that
/// only checks the guard exists — as the one above did on its own — passes on
/// the broken source. This one pins the SHAPE that makes the exemption last as
/// long as the episode does.
TEST(Gen22StartHold, TheSuppressionExemptionIsEpisodeScopedNotInstantScoped) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::heartbeatTick()");
  ASSERT_FALSE(body.empty()) << "the scan found no heartbeatTick() definition";

  const size_t cond = body.find("if (!hb_suppress_warned_ && held >=");
  ASSERT_NE(cond, std::string::npos)
      << "the eligibility condition for the suppression WARN has been "
         "rewritten. It must stay `!hb_suppress_warned_ && held >= "
         "coord_claim_ttl_sec_` with the state test INSIDE, not appended to it.";

  const size_t guard = body.find("state_ != State::WAIT_FOR_MAP");
  ASSERT_NE(guard, std::string::npos) << "the WAIT_FOR_MAP gate is gone.";

  const size_t consume = body.find("hb_suppress_warned_ = true", cond);
  ASSERT_NE(consume, std::string::npos)
      << "nothing consumes the once-per-episode latch after the eligibility "
         "test, so the WARN is no longer latched at all.";

  // THE ASSERTION THAT MATTERS. The latch must be spent BEFORE the state is
  // tested. If the state test sits in the `if` condition instead, then during
  // the ~1 s in which the hold has expired but the heartbeat has not yet
  // resumed, the robot is in PLAN with an unset latch and held >= TTL, and the
  // WARN fires reporting a 60 s suppression that is really the hold. That is
  // exactly the regression observed in 2 of 6 gen-22 smoke cells.
  EXPECT_LT(consume, guard)
      << "the latch is consumed AFTER the WAIT_FOR_MAP test, which means the "
         "test is gating the assignment rather than just the WARN. Then the "
         "exemption lasts only as long as the STATE, while the suppression "
         "EPISODE outlives it by about a second — and a heartbeat tick landing "
         "in that window prints 'Heartbeat suppressed 60.1 s in state PLAN'. "
         "Consume the latch for the whole eligible branch; emit the WARN only "
         "outside WAIT_FOR_MAP.";
}

// ===========================================================================
// GROUP F. A WAIT THAT IS WORKING IS NOT A FAULT.
// ===========================================================================
//
// The WAIT_FOR_MAP arm has two quite different reasons to stay put: a
// precondition has not arrived (map, pose, planning_map), or every precondition
// HAS arrived and the pre-mission hold is still running. The first is a fault
// and wants a WARN naming the missing input. The second is the design working
// and already has two INFO lines of its own — the throttled "Pre-mission hold:
// Xs of Ys" and the one-shot "Pre-mission hold complete at t+Xs".
//
// Until 2026-09-18 the WARN was keyed on `!start`, and `start` is the
// conjunction of BOTH. So for the whole length of the hold the log carried,
// every five seconds, immediately below the INFO stating the hold was
// progressing normally:
//
//     Waiting to start: map=1 pose=1 planning_map=1 (0 = not yet received;
//     planning_map required).
//
// A line that prints three satisfied preconditions and calls itself waiting for
// them. It names no fault, suggests no next step, and directly contradicts the
// line above it. At ~11 repeats per robot per cell it was also the single most
// frequent WARN in a gen-22 log. The cost of that is not the bytes: it is that
// someone debugging a genuinely stuck startup has to first work out that the
// loudest warning in the file means nothing.
//
// The fix gates it on !preconds_met, which is the fault condition alone.

/// The failure mode is a self-contradicting log line, so the test asserts the
/// structure that makes the contradiction impossible rather than the text.
TEST(Gen22StartHold, TheWaitingToStartWarnFiresOnlyOnAMissingPrecondition) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body = functionBody(text, "void ExploPlannerNode::tick()");
  ASSERT_FALSE(body.empty()) << "the scan found no tick() definition";

  const size_t warn = body.find("Waiting to start: map=%d pose=%d");
  ASSERT_NE(warn, std::string::npos)
      << "the startup-fault WARN is gone. A stuck startup (wrong namespace, "
         "dead mapper, no TF) must stay diagnosable; the 2026-09-18 change "
         "narrows when it fires, it does not delete it.";

  // preconds_met must exist and be the preconditions WITHOUT the hold.
  const size_t decl = body.find("const bool preconds_met");
  ASSERT_NE(decl, std::string::npos)
      << "preconds_met is gone, so the WARN is almost certainly back on "
         "!start — which fires it throughout the pre-mission hold with every "
         "precondition already satisfied.";
  const size_t decl_end = body.find(';', decl);
  ASSERT_NE(decl_end, std::string::npos);
  const std::string decl_text = body.substr(decl, decl_end - decl);
  EXPECT_EQ(decl_text.find("hold_done"), std::string::npos)
      << "preconds_met reads hold_done, which collapses it back into `start` "
         "and reintroduces the contradicting line. It must be the three input "
         "preconditions and nothing else: " << decl_text;

  // start must still be the conjunction — GROUP A owns that, this is the
  // other half of it: narrowing the WARN must not have narrowed the GATE.
  const size_t start_decl = body.find("const bool start = preconds_met");
  ASSERT_NE(start_decl, std::string::npos)
      << "`start` is no longer `preconds_met && hold_done`. Splitting the "
         "preconditions out of the WARN must not weaken the hold itself.";
  const size_t start_end = body.find(';', start_decl);
  ASSERT_NE(start_end, std::string::npos);
  EXPECT_NE(body.substr(start_decl, start_end - start_decl).find("hold_done"),
            std::string::npos)
      << "`start` no longer conjoins hold_done, so the pre-mission hold does "
         "not gate the transition to PLAN at all.";

  const size_t gate = body.find("} else if (!preconds_met) {");
  ASSERT_NE(gate, std::string::npos)
      << "the WARN's branch is not gated on !preconds_met. Keyed on anything "
         "that includes the hold, it fires for the hold's full length while "
         "reporting map=1 pose=1 planning_map=1 — satisfied preconditions "
         "under a message that says it is waiting for them.";
  EXPECT_LT(gate, warn)
      << "!preconds_met appears in tick() but after the WARN text, so it is "
         "not the branch condition guarding it.";
}
