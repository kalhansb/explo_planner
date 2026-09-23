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
// Moved comments: doc/test_gen22_start_hold_notes.md

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

/// The source with line and block comments removed, string literals preserved.
/// Copied into each source-scan binary rather than shared, so one mutated node
/// cannot break every test binary. (notes: start-hold-strip-comments-copy)
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

/// hold_done is ANDed into start, not waited out ahead of the preconditions:
/// release is then at max(hold, map latency) rather than hold + map latency.
/// (notes: start-hold-conjunct-of-start)
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

  // Both terms must stay conjuncts of one boolean, evaluated unconditionally.
  // The literals are asserted, not tokens, so a sequenced wait cannot pass;
  // preconds_met is split out only for the startup WARN.
  // (notes: start-hold-conjunct-literals)
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

/// missionElapsed() returns -1.0 until the first positive clock, below any
/// non-negative hold, so a clockless robot holds. Do not re-key on state entry
/// or this->now(), which reads 0 before /clock under use_sim_time.
/// (notes: start-hold-mission-elapsed-clock)
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

/// The WAIT_FOR_MAP arm has exactly one transitionTo, inside if (start), so no
/// second escape (a timeout, a start-anyway path) can bypass the hold.
/// (notes: start-hold-single-exit)
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
// Asserts no clock term confines the provisional->final upgrade to the hold. A
// held robot completes no steps, hence builds no tours, so the upgrade, the
// only clear of rendezvous_held_provisional_, would never fire.
// (notes: start-hold-upgrade-not-confined)
// ===========================================================================

/// The derive gate is a pure disjunction with no clock term: derive if there is
/// no pair, re-derive while it is provisional. The scan rejects any && in the
/// group, so new disjuncts pass and any conjunct fails.
/// (notes: start-hold-derive-gate-disjunction)
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

/// within_start_hold must not appear in maintainRendezvousProposal, and the
/// initial derive must stay a bare disjunct: confined, a run whose map arrives
/// after the hold gets no pair and every arming refuses.
/// (notes: start-hold-derive-no-hold-read)
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

/// heartbeatTick must call maintainRendezvousProposal above both early returns
/// (no active intent, the state list); WAIT_FOR_MAP is in neither, so the
/// handshake runs while held. A state guard above it voids the hold.
/// (notes: start-hold-handshake-during-hold)
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
  // Every State::WAIT_FOR_MAP test in heartbeatTick must sit below the
  // handshake call: one above it can skip the agreement round for the whole
  // hold; one below cannot, since the call has already returned.
  // (notes: start-hold-wait-for-map-tests-below)
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

/// mission_start_hold_sec is a declared parameter, and a NaN or negative value
/// throws: NaN makes the hold infinite (held_for >= hold is always false), a
/// negative one makes it zero-length. (notes: start-hold-param-validated)
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

/// The member and parameter defaults must both be 60 s, or the hold depends on
/// which construction path ran. It defaults on, unlike the mission-return knobs
/// beside it, which default off. (notes: start-hold-defaults-agree)
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

/// run_explo_sim_rviz.sh must pass the hold once, in the unconditional
/// per-robot argv, not the arm-conditional EXTRA array, or it applies to two
/// arms only and reads as a treatment effect. Scanned raw, unstripped.
/// (notes: start-hold-harness-every-arm)
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

/// A START_HOLD of 0 is legal and disables the hold, but must not be the
/// harness default; the value that ran must be recorded in the manifest.
/// (notes: start-hold-harness-default-manifest)
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
// The suppression WARN separates beacon suppression from radio outage. It is
// skipped in WAIT_FOR_MAP, a clock-free gate since nothing re-enters that
// state; the latch is still consumed, and the resumed INFO stays.
// (notes: start-hold-suppression-warn-gate)

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

/// Pins the shape that makes the WARN exemption last the whole suppression
/// episode; a test that only checks the state guard exists passes while the
/// WARN still prints. (notes: start-hold-exemption-episode-scoped)
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

  // The latch must be consumed before the state is tested: for about a second
  // after the hold expires the robot is in PLAN with the heartbeat still
  // suppressed, and the WARN would report the hold as a suppression.
  // (notes: start-hold-latch-before-state)
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
// WAIT_FOR_MAP waits either on a missing precondition (a fault, warned with the
// missing input) or on the running hold (normal, with its own INFO lines). The
// WARN is keyed on !preconds_met, the fault alone.
// (notes: start-hold-waiting-warn-preconds)

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
