// T9. Contract tests for GENERATION 20's rendezvous invariants, read from the
// node's source.
//
// WHY THESE ARE SOURCE SCANS AND NOT BEHAVIOURAL TESTS. Every invariant below
// lives in `explo_planner_node.cpp`, which is not in explo_planner_lib's source
// list — an explicit list of files in CMakeLists.txt, not a glob — so nothing
// links it and no test can call it. That is the same wall test_endpoint.cpp's
// GROUP C hit, and the same answer: a scan that can show a guard is PRESENT and
// ORDERED, cannot show it is REACHED, and fails loudly if the scan itself stops
// matching. What it catches is the realistic regression — a later edit moving,
// renaming, splitting or deleting one of these — which is precisely how R1b
// shipped a guard that tested the wrong state for an entire generation with
// nothing noticing.
//
// THESE SCANS WERE MUTATION-VERIFIED, and for this project that is not optional
// ceremony. A scan test is a check that can quietly stop checking — this repo
// has already shipped eight guards that went inert while still printing PASSes —
// and every assertion here is of the form "X is absent", which is exactly the
// shape that passes when the scan finds nothing at all. So on 2026-09-17 each
// assertion was run against a deliberately broken node: the countdown write
// removed; a `return` inserted between the countdown and the arm; the deadline
// re-derived from interval_ms; the spent-latch release and doReturnNav's release
// re-gated on rendezvousTeamMutual; the wait cap and the settle hold re-keyed on
// appointment_armed_; a latch clear inserted into closeAppointment; the spent
// latch removed from the arming site. Nine mutations, nine failures, each in the
// test named for it. Re-run that before trusting any later edit to this file:
// the binary reads the node at RUN time, so it is edit-run-restore, no rebuild.
//
// GENERATION 29 ADDED EIGHT MORE on 2026-09-19, all killed: appointmentDue
// dropping the departure lead; dropping its no-estimate fallback; re-deciding
// the rung inside the departure trigger (the ratchet); armAppointment flooring
// on bare t_now again; armAppointment no longer reading the lateness budget;
// appointmentLeadMs dropping the scheduler's safety markup; appointmentLeadMs
// returning 0 instead of the -1 sentinel; and travelMsToCell dropping the
// target-cell bounds check that its new pre-arming caller depends on. Those
// were run against COPIES with the macro repointed rather than by editing the
// workspace file, which is the safer form of the same recipe and the only one
// available while a campaign's provenance hash is live.
//
// Two of these tests passed a first draft that was reading COMMENTS rather than
// code, in opposite directions, which is why stripComments() below exists.
//
// The PURE half of the same machinery is already covered behaviourally and is
// not repeated here: RendezvousHandshake's adopt and latch rules (the P -> R
// commit protocol) have twelve tests in test_rendezvous_scheduler.cpp, and
// teamComplete/rendezvousWaitExpired have theirs in test_planner_util.cpp.
// These tests cover only what those cannot reach — how the node WIRES them
// together.
//
// WHAT EACH GROUP EXISTS FOR
// --------------------------
//   A. THE DEPARTURE DEADLINE. Generation 19 replaced a shared meeting instant
//      with a per-robot countdown, generation 20 unified the predicate around
//      it, and GENERATION 23 PUT THE SHARED INSTANT BACK — the deadline is once
//      again the agreed schedule, now kept safe by an unbounded barrier rather
//      than by each robot having its own clock. The group kept its place in this
//      file because the hazard it guards did not change: `appointmentDue()` is a
//      bare `now >= t_meet_ms` and `RendezvousPlan::t_meet_ms` defaults to -1,
//      so an armed slot that skipped the write is due FOREVER. What changed is
//      the second test, which used to pin the countdown and now pins the agreed
//      occurrence; it is in a `Gen23AgreedSchedule` suite so that its name
//      cannot outlive the behaviour again.
//
//   B. THE SEPARATION PREDICATE. Five sites decide "the team came apart" —
//      arming, superseding, the barrier release, the confirmed release and the
//      heartbeat. A generation-20 fix made them one predicate, `teamComplete`.
//      A split between any two of them is not a cosmetic inconsistency: it is
//      the class of defect that arms a manoeuvre one site thinks is over.
//
//   C. THE APPOINTMENT-MANOEUVRE LATCH. `appointment_manoeuvre_` answers "the
//      manoeuvre I am IN was started to keep an appointment", which is a
//      different question from "an appointment stands" (`appointment_armed_`).
//      Keying the wait cap and the settle hold on the wrong one of those two is
//      the bug the latch was introduced to fix, so the keying is the assertion.

#include <gtest/gtest.h>

#include <cctype>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

/// The source with `//` comments removed, string literals preserved.
///
/// EVERY SCAN IN THIS FILE RUNS ON THIS, and the first draft of the file did
/// not, which cost it two false results in opposite directions. This node's
/// comments quote the code they are about — one of them says, verbatim,
/// "Keyed on appointment_manoeuvre_, NOT on appointment_armed_", and another
/// carries a commented-out `rendezvous_spent_ = false;` as the example of a
/// clear that broke a generation. A raw-text scan reads the first as the code
/// doing the thing it is denying and the second as the assignment itself, so a
/// correct node fails and a reverted one passes. Prose is not evidence about
/// code; strip it before asserting anything.
///
/// The quote tracking is not decoration: dropping from `//` unconditionally
/// would truncate any line holding a string with a `//` in it and silently
/// remove code from the scan's view, which is the same failure one layer down.
std::string stripComments(const std::string& text) {
  std::string out;
  out.reserve(text.size());
  bool in_str = false, in_chr = false;
  for (size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    if (in_str || in_chr) {
      out.push_back(c);
      if (c == '\\' && i + 1 < text.size()) { out.push_back(text[++i]); continue; }
      if (in_str && c == '"')  in_str = false;
      if (in_chr && c == '\'') in_chr = false;
      continue;
    }
    if (c == '"')  { in_str = true; out.push_back(c); continue; }
    if (c == '\'') { in_chr = true; out.push_back(c); continue; }
    if (c == '/' && i + 1 < text.size() && text[i + 1] == '/') {
      while (i < text.size() && text[i] != '\n') ++i;
      if (i < text.size()) out.push_back('\n');
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
///
/// Returns "" when the signature is not found, which every caller ASSERTs on: a
/// scan that silently matches nothing passes every assertion made about what it
/// did not find, and that is the failure mode this file exists to avoid.
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

/// How many times `needle` occurs in `hay`. Used for the "exactly one write"
/// assertions, where a SECOND write is the regression and a count is the only
/// way to see it — `find() != npos` is true of one write and of five.
int countOf(const std::string& hay, const std::string& needle) {
  int n = 0;
  for (size_t at = hay.find(needle); at != std::string::npos;
       at = hay.find(needle, at + needle.size())) {
    ++n;
  }
  return n;
}

/// Every byte offset at which `needle` occurs in `hay`.
std::vector<size_t> allOf(const std::string& hay, const std::string& needle) {
  std::vector<size_t> at;
  for (size_t i = hay.find(needle); i != std::string::npos;
       i = hay.find(needle, i + needle.size())) {
    at.push_back(i);
  }
  return at;
}

/// The right-hand side of the statement assigning `lhs`, or "" if `lhs` does
/// not appear or its first appearance is not an assignment. Every caller
/// ASSERTs on the empty string, for the reason functionBody does.
///
/// WHITESPACE IS WHY THIS EXISTS. The assertions below are of the form "this
/// field is fed from that one", and the node column-aligns its assignments and
/// wraps long right-hand sides — so the literal needle a reader would write
/// (`cfg.min_interval_ms = rendezvous_interval_sec_`) matches nothing, and the
/// one that does match today stops matching the first time clang-format moves a
/// line break. A test that silently stops looking is worse than no test.
std::string assignedFrom(const std::string& text, const std::string& lhs) {
  const size_t at = text.find(lhs);
  if (at == std::string::npos) return "";
  size_t j = at + lhs.size();
  while (j < text.size() && std::isspace(static_cast<unsigned char>(text[j]))) ++j;
  if (j >= text.size() || text[j] != '=') return "";
  if (j + 1 < text.size() && text[j + 1] == '=') return "";   // a comparison
  const size_t end = text.find(';', j);
  if (end == std::string::npos) return "";
  return text.substr(j + 1, end - j - 1);
}

/// Every byte offset at which `needle` is ASSIGNED, as opposed to merely read.
///
/// The one-write assertions in this file are usually about a literal, so
/// countOf is enough for them. Generation 29's flag is read in two places and
/// written in one, and WHICH of the three is the write is the entire contract —
/// a clear that migrates from the adoption up to the attempt reads identically
/// to countOf and silently drops every re-agreement whose derive came back
/// provisional.
std::vector<size_t> assignmentsOf(const std::string& hay, const std::string& needle) {
  std::vector<size_t> at;
  for (const size_t i : allOf(hay, needle)) {
    size_t j = i + needle.size();
    while (j < hay.size() && std::isspace(static_cast<unsigned char>(hay[j]))) ++j;
    if (j < hay.size() && hay[j] == '=' &&
        !(j + 1 < hay.size() && hay[j + 1] == '=')) {
      at.push_back(i);
    }
  }
  return at;
}

/// The node source with its comments removed, read once per test. Kept as a
/// function rather than a global so a read failure ASSERTs inside the test that
/// needed it.
std::string nodeSource() { return stripComments(readFile(EXPLO_PLANNER_NODE_CPP)); }

}  // namespace

// ===========================================================================
// GROUP A. THE COUNTDOWN.
// ===========================================================================

/// THE DEADLINE MUST BE WRITTEN ON EVERY PATH THAT ARMS, and this is the test
/// test_endpoint.cpp's banner says does not exist.
///
/// The hazard, stated exactly: `appointmentDue()` compares the clock against
/// `appointment_.t_meet_ms` guarded by `appointment_armed_`, and
/// `RendezvousPlan::t_meet_ms` defaults to -1. An armed slot carrying that
/// sentinel is therefore DUE FOREVER — the robot departs on every tick, for the
/// rest of the run, and the log records an appointment being kept. Generation
/// 29's departure lead makes that strictly worse rather than better: the test
/// became `now + lead >= t_meet`, so a larger lead only makes the sentinel fire
/// sooner. Nothing below depends on which of the two forms is in the file.
///
/// What is supposed to prevent it is that `armAppointment` writes the deadline
/// before it arms, with nothing in between that can skip the write and still
/// reach the arm. That is a claim about straight-line source, which is why it is
/// checked as one. Note what does NOT protect it, because a future edit will
/// reach for these first: `armed` is `ev.refused.empty() && appointment_.valid()`
/// and `RendezvousPlan::valid()` is `cell >= 0 && refused.empty()`, which never
/// looks at `t_meet_ms` at all.
TEST(Gen20DeadlineWrite, TheDeadlineIsWrittenOnEveryPathThatArms) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::armAppointment(");
  ASSERT_FALSE(body.empty())
      << "the scan found no armAppointment definition — it has been renamed or "
         "reshaped and this test has stopped testing anything";

  const size_t deadline = body.find("appointment_.t_meet_ms   =");
  ASSERT_NE(deadline, std::string::npos)
      << "armAppointment no longer assigns appointment_.t_meet_ms; an armed "
         "slot would carry the -1 default, which appointmentDue() reads as due "
         "forever";

  // Exactly one arm, so "the write precedes the arm" cannot be satisfied by the
  // first of several.
  EXPECT_EQ(countOf(body, "appointment_armed_          = true"), 1)
      << "armAppointment has more than one arming site (or the one it had has "
         "been reformatted); the ordering assertion below only constrains the "
         "first, so a second arm could skip the deadline write";
  const size_t arm = body.find("appointment_armed_          = true");
  ASSERT_NE(arm, std::string::npos)
      << "armAppointment no longer arms — this test has stopped testing "
         "anything";

  EXPECT_LT(deadline, arm)
      << "the deadline is written AFTER the appointment is armed: between "
         "those two statements the slot is armed on a -1 deadline";

  // The substantive half. An early return between the two would mean a path
  // that arms without a deadline, which is the whole hazard; `valid()` cannot
  // catch it because it does not inspect t_meet_ms.
  const std::string between = body.substr(deadline, arm - deadline);
  EXPECT_EQ(between.find("return"), std::string::npos)
      << "there is a `return` between the deadline write and the arm. Either "
         "it cannot reach the arm (in which case say so here) or there is now "
         "a path that arms on the -1 sentinel:\n"
      << between;
}

/// THE DEADLINE IS THE AGREED OCCURRENCE, NOT A PRIVATE COUNTDOWN.
///
/// This test used to assert the exact opposite, and the reversal is the whole of
/// generation 23. Generations 19-22 set `t_meet_ms` to
/// `now + rendezvous_depart_delay_sec`, a countdown from the moment THIS robot
/// noticed the team was incomplete. Every robot ran its own, so the arm labelled
/// "rendezvous" was N robots independently visiting one cell — an agreed place
/// and N different times — which is not what the experiment's rendezvous arm is
/// defined to be.
///
/// What the deadline must be now (generation 29): the first occurrence of the
/// AGREED recurring schedule this robot can still REACH within
/// rendezvous_max_lateness_sec — floored at `t_now` plus its own shortfall, and
/// nothing else. Each part is asserted, because each without the others is a
/// different bug:
///
///   * no floor at all is generation 18's shape: a robot departing for a
///     meeting that had already happened;
///   * a floor of `t_now + rendezvous_depart_delay_sec_` is generations
///     19-24: the shared notice re-applied per robot, whose per-robot sum
///     forked the ts4 N=3 cell across two occurrences (two robots kept the
///     agreed instant, the third rolled a whole interval past it and its
///     peers stood at the cell 351 and 432 s to the censor);
///   * a bare `t_now` floor is generation 25-28: correct about the lattice,
///     but every robot then signed up to occurrences it could not reach, and
///     54 of 54 kept appointments arrived late by their own drive.
///
/// THE THIRD IS NOT A RETURN OF THE SECOND, and the difference is one token.
/// `arrivalShortfallSec` clamps at zero, so a robot that can make the nearest
/// occurrence contributes EXACTLY nothing to its floor and indexes the lattice
/// from the same place as every peer that can also make it. The generation-19
/// term was unconditional, which is why a team that could all attend still
/// split. That clamp is pinned executably in test_planner_util.cpp; this scan
/// can only assert that the node asks for it.
///
/// The arithmetic itself is NOT scanned for here. It moved into
/// explo_planner::nextAgreedOccurrence precisely so it could have executable
/// tests — see test_planner_util.cpp, which pins the lattice property this scan
/// can only assert is being called.
TEST(Gen23AgreedSchedule, TheDeadlineIsTheAgreedOccurrenceNotACountdown) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::armAppointment(");
  ASSERT_FALSE(body.empty()) << "the scan found no armAppointment definition";

  const size_t deadline = body.find("appointment_.t_meet_ms   =");
  ASSERT_NE(deadline, std::string::npos)
      << "armAppointment no longer assigns appointment_.t_meet_ms";
  // The assignment expression: from the `=` to its terminating `;`.
  const size_t end = body.find(';', deadline);
  ASSERT_NE(end, std::string::npos);
  const std::string expr = body.substr(deadline, end - deadline);

  EXPECT_NE(expr.find("nextAgreedOccurrence"), std::string::npos)
      << "the departure deadline is no longer computed by "
         "nextAgreedOccurrence, so whatever it now is has no executable test "
         "behind it:\n"
      << expr;
  EXPECT_NE(expr.find("rendezvous_agreed_.t_meet_ms"), std::string::npos)
      << "the deadline is not anchored on the AGREED meeting instant. Every "
         "robot must index the same schedule from the same origin; the "
         "generation-18 failure was each robot indexing from its own, and the "
         "generation-20 failure was there being no agreed instant at all:\n"
      << expr;
  EXPECT_NE(expr.find("rendezvous_agreed_.interval_ms"), std::string::npos)
      << "the deadline does not use the agreed interval, so the appointment is "
         "a single instant rather than a standing schedule and a robot that "
         "separates after it has passed has no meeting to keep:\n"
      << expr;
  EXPECT_NE(expr.find("t_now"), std::string::npos)
      << "the deadline has no t_now floor, so a robot could depart for an "
         "occurrence in the past:\n"
      << expr;
  EXPECT_NE(expr.find("shortfall"), std::string::npos)
      << "the floor is bare t_now again. That is the generation 25-28 shape: "
         "correct about the lattice, but a robot signs up to occurrences it "
         "cannot reach and is late by its whole drive every time. The rung has "
         "to be chosen against what this robot can actually make:\n"
      << expr;
  // THE SIGN, SCANNED LITERALLY, because it is the only thing standing under
  // the "deadline already passed" diagnostic that was deleted at the head of
  // this function as unreachable-by-construction. arrivalShortfallSec clamps at
  // zero, so `t_now + shortfall` can only ever be at or after t_now; `t_now -
  // shortfall` puts the floor BEHIND now and hands back occurrences already
  // past, and every other assertion in this test passes under that flip.
  EXPECT_NE(expr.find("t_now + shortfall_sec"), std::string::npos)
      << "the floor is not `t_now + shortfall_sec`. A subtraction (or any "
         "other combination) can place the floor before now, which is the one "
         "input for which nextAgreedOccurrence returns a meeting in the past "
         "and the robot departs for an appointment that has already "
         "happened:\n"
      << expr;
  // Where that term comes from. Asserted on the function body rather than the
  // assignment because the shortfall is computed on the line above it, and
  // splitting it out is a reasonable edit that must not silently pass.
  EXPECT_NE(body.find("arrivalShortfallSec"), std::string::npos)
      << "armAppointment no longer calls arrivalShortfallSec, so whatever now "
         "builds the floor has no executable test behind its clamp at zero — "
         "and that clamp is the ONLY thing separating this from the "
         "generation-19 per-robot fork";
  EXPECT_NE(body.find("rendezvous_max_lateness_sec_"), std::string::npos)
      << "the lateness budget is no longer read at arming, so the rung choice "
         "is not the one the parameter (and the experiment record) describes";
  EXPECT_NE(body.find("appointmentLeadMs"), std::string::npos)
      << "the rung is chosen against something other than appointmentLeadMs. "
         "Arming and appointmentDue must price the same drive the same way; if "
         "they diverge a robot can sign up to a rung its own departure trigger "
         "then refuses to leave for in time";
  EXPECT_EQ(expr.find("rendezvous_depart_delay_sec_"), std::string::npos)
      << "the deadline floors on rendezvous_depart_delay_sec_ again. That is "
         "the generation 19-24 shape: the shared notice re-applied per robot, "
         "and the per-robot sum is what forked the ts4 N=3 cell across two "
         "occurrences. The governing rule (\"a place and a time which is "
         "fixed\") is the bare floor:\n"
      << expr;

  // THE COUNTDOWN MUST NOT SURVIVE ALONGSIDE IT. `t_now` is still in scope in
  // this function and the old form was a one-liner, so the realistic regression
  // is not a deletion of the call above but a second assignment beside it.
  EXPECT_EQ(countOf(body, "appointment_.t_meet_ms   ="), 1)
      << "armAppointment assigns the departure deadline more than once; the "
         "assertions above only constrain the first, so a later line could "
         "restore the private countdown and this test would not notice";
}

/// THE ROBOT LEAVES EARLY ENOUGH TO ARRIVE, AND DECIDES NOTHING ELSE.
///
/// Generation 29 splits one question into two, and both halves have to hold or
/// the arm is not what Kalhan specified ("robots should only agree to meetings
/// they can come to with a maximum delay", 2026-09-19):
///
///   * WHICH occurrence this robot keeps — decided ONCE, at arming, by the test
///     above;
///   * WHEN it leaves for it — recomputed every tick here, so a robot that
///     drifts or chases away from the cell pulls its own departure earlier and
///     the rung it signed up to stays reachable.
///
/// THE RATCHET IS WHY THE SPLIT MATTERS. If this function re-chose the rung
/// instead of just the departure, a robot exploring away from the cell would
/// grow its lead, roll to a later occurrence, explore further on the strength
/// of the extra time, and never attend at all — the meeting receding exactly as
/// fast as the robot leaves. So the scan below is in two parts: the lead must
/// be here, and the deadline must NOT be written here.
///
/// The -1 fallback is asserted for the same reason it exists. Off the snapshot
/// grid there is no travel estimate, and the robot must still depart — at
/// t_meet, late by its drive, which is the pre-29 behaviour and the right
/// degradation because it is the one that needs no estimate. Dropping the
/// branch would compare against -1 and make an armed appointment due forever.
TEST(Gen29Punctuality, TheDepartureLeadsTheMeetingAndDecidesNoRung) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::appointmentDue(");
  ASSERT_FALSE(body.empty()) << "the scan found no appointmentDue definition";

  EXPECT_NE(body.find("appointmentLeadMs"), std::string::npos)
      << "appointmentDue no longer asks for a departure lead, so t_meet is a "
         "departure instant again and every robot is late by its own drive:\n"
      << body;
  EXPECT_NE(body.find("now_ms + lead_ms >= appointment_.t_meet_ms"),
            std::string::npos)
      << "the due test is not `now + lead >= t_meet`. Whatever it now is, it "
         "is not \"leave early enough to arrive\":\n"
      << body;
  EXPECT_NE(body.find("if (lead_ms < 0) return now_ms >= appointment_.t_meet_ms"),
            std::string::npos)
      << "the no-estimate fallback is gone. A robot off the snapshot grid "
         "reads -1 and would then be compared against a negative lead — an "
         "armed appointment due forever, departing on every tick:\n"
      << body;

  // THE RATCHET GUARD. `appointment_` is a member and fully mutable from here;
  // nothing but this assertion stops a future edit from re-deciding the rung on
  // the tick that notices the robot cannot make it. Written as "no occurrence
  // of the field is followed by an assignment" rather than as a literal search,
  // because the node's own assignments are column-aligned with runs of spaces
  // and a fixed-spacing needle would miss every realistic edit.
  for (const size_t at : allOf(body, "appointment_.t_meet_ms")) {
    size_t j = at + std::string("appointment_.t_meet_ms").size();
    while (j < body.size() && std::isspace(static_cast<unsigned char>(body[j]))) ++j;
    const bool assigns =
        j < body.size() && body[j] == '=' &&
        !(j + 1 < body.size() && body[j + 1] == '=');
    EXPECT_FALSE(assigns)
        << "appointmentDue writes the deadline. The rung is chosen once, at "
           "arming: re-choosing it here ratchets, because a robot exploring "
           "away from the cell grows its lead, rolls to a later occurrence, "
           "and the meeting recedes as fast as it walks:\n"
        << body;
  }
  EXPECT_EQ(body.find("nextAgreedOccurrence"), std::string::npos)
      << "appointmentDue re-runs the occurrence search. Same ratchet as above; "
         "the departure trigger must move only the departure:\n"
      << body;
}

/// ONE MARKUP, PRICED THE SAME IN BOTH PLACES THAT READ IT.
///
/// The scheduler sizes the lattice spacing so the furthest robot's drive fits,
/// marked up by rzv_cfg_.depart_safety_milli (rendezvous_scheduler.cpp,
/// floor_ms). appointmentLeadMs prices this robot's departure with the SAME
/// constant. If the two ever diverge upward here, a robot needs more notice
/// than the spacing the team sized for it — and that is the one shortfall a
/// rung roll cannot fix, because every rung is equally too close.
///
/// Scanned rather than executed for the usual reason: these read
/// rendezvous_world_ and latest_pos_, and explo_planner_node.cpp defines
/// main(), so nothing in it links into a gtest target. The pure arithmetic that
/// COULD be lifted already was — see arrivalShortfallSec.
TEST(Gen29Punctuality, TheLeadIsTheSchedulersOwnMarkupAndBothEndsAreScreened) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  const std::string lead =
      functionBody(text, "long long ExploPlannerNode::appointmentLeadMs(");
  ASSERT_FALSE(lead.empty()) << "the scan found no appointmentLeadMs definition";
  EXPECT_NE(lead.find("rzv_cfg_.depart_safety_milli"), std::string::npos)
      << "the departure lead no longer uses the scheduler's own safety markup, "
         "so the lead and the lattice spacing it has to fit inside are priced "
         "differently:\n"
      << lead;
  EXPECT_NE(lead.find("return -1"), std::string::npos)
      << "appointmentLeadMs no longer propagates the no-estimate sentinel, so "
         "an unknown drive becomes a number and both the rung choice and the "
         "departure trigger act on it:\n"
      << lead;

  const std::string travel =
      functionBody(text, "long long ExploPlannerNode::travelMsToCell(");
  ASSERT_FALSE(travel.empty()) << "the scan found no travelMsToCell definition";
  // BOTH ENDS. The caller changed in generation 29: the rung chooser asks for a
  // cell that has not been through the arming site's bounds check yet, so the
  // screen has to be here and `here` alone is no longer enough.
  EXPECT_EQ(countOf(travel, "grid().valid("), 2)
      << "travelMsToCell does not bounds-check exactly both of its endpoints. "
         "The rung chooser calls it BEFORE the appointment is armed, so the "
         "target cell has not been screened anywhere else:\n"
      << travel;
}

// ===========================================================================
// GROUP B. THE SEPARATION PREDICATE.
// ===========================================================================

/// ONE QUESTION, ASKED THROUGH ONE FAMILY OF HELPERS, AT EVERY SITE THAT ACTS
/// ON IT.
///
/// Before generation 20 the arming path and the release path asked different
/// questions about the same fact, so a robot could arm a manoeuvre for a team
/// the barrier already considered whole, and hold at the meeting point for a
/// reunion the arming site had not noticed. Unifying them was the fix; this
/// pins that the unification survives, function by function, because a single
/// site drifting out of it is invisible at the call site and catastrophic at
/// run time.
///
/// WHAT GENERATION 23 CHANGED, AND WHY THIS TEST HAD TO MOVE WITH IT. Through
/// generation 22 the unification was the literal token `teamComplete(` at all
/// five sites, and this test asserted exactly that. Contagious arming split the
/// one predicate into a rooted family:
///
///     teamSettled(live)              = teamComplete(live, expected)
///                                      && !peerReportsTeamBreak()
///     manoeuvreReleaseEligible(live) = appointment_manoeuvre_
///                                          ? teamSettled(live) ||
///                                            teamComplete(reachable, expected)
///                                          : teamComplete(live, expected)
///
/// (schematic: the appointment branch also carries generation 23's
/// !peerInboundToAppointment() term with its generation-27 one-hop
/// peer-report companion, and `reachable` is generation 27's closure count —
/// the extensions still root in the same two primitives)
///
/// so three of the five sites legitimately stopped naming `teamComplete`
/// directly. Asserting the old token here did not catch a defect — it just went
/// red against the new design, which is the worst state a guard can be in.
///
/// THE DIVISION OF LABOUR WITH test_gen23_contagion.cpp MATTERS. That file's
/// Gen23ContagionFiveSites pins WHICH member of the family each individual site
/// must ask, site by site, and is where a `teamSettled` -> bare `teamComplete`
/// drift fails. Repeating that here would add nothing. What this test owns is
/// the weaker but differently-shaped claim the per-site tests cannot make: that
/// no site escapes the family ALTOGETHER, and that the family is still rooted in
/// one primitive. A site that hand-rolls `live >= rendezvous_expected_peers_`,
/// or a `teamSettled` rewritten to stop consulting `teamComplete`, satisfies
/// every per-site assertion and is precisely the generation-18/19 failure
/// reappearing one layer down.
///
/// It asserts PRESENCE, not exclusivity: `rendezvousTeamMutual()` is still the
/// right predicate for the two quantities that genuinely need both directions
/// of every link (the anchor stamp and the derive gate), so a blanket ban would
/// be wrong.
TEST(Gen20SeparationPredicate, EveryDecisionSiteAsksTheSameQuestion) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  // The family, and the root it must reduce to. Checked first: every assertion
  // below is only worth making if asking a family member still amounts to
  // asking the one primitive.
  const char* kRoot = "teamComplete(";
  const std::vector<const char*> kFamily = {
      "teamComplete(", "teamSettled(", "manoeuvreReleaseEligible("};

  for (const char* sig : {"bool ExploPlannerNode::teamSettled(",
                          "bool ExploPlannerNode::manoeuvreReleaseEligible("}) {
    const std::string body = functionBody(text, sig);
    ASSERT_FALSE(body.empty())
        << "the scan found no definition for '" << sig
        << "' — the generation-23 predicate family has been reshaped and this "
           "test has stopped testing that the sites below share a root";
    EXPECT_NE(body.find(kRoot), std::string::npos)
        << sig
        << " no longer reduces to teamComplete(). The five decision sites are "
           "unified by delegating to this family, so a member that stops "
           "consulting the primitive silently un-unifies every site that asks "
           "it — with no site itself changing:\n"
        << body;
  }

  struct Site {
    const char* signature;
    const char* what;
  };
  const std::vector<Site> sites = {
      {"void ExploPlannerNode::doPlan(",
       "arming and superseding a reconnect manoeuvre"},
      {"void ExploPlannerNode::transitionTo(",
       "classifying how a manoeuvre ended"},
      {"void ExploPlannerNode::doReturnSync(",
       "releasing the barrier at the meeting point"},
      {"void ExploPlannerNode::doReturnNav(",
       "releasing a robot still driving to the meeting point"},
  };
  for (const Site& s : sites) {
    const std::string body = functionBody(text, s.signature);
    ASSERT_FALSE(body.empty())
        << "the scan found no definition for '" << s.signature
        << "' — it has been renamed or reshaped and this test has stopped "
           "testing " << s.what;
    bool asks = false;
    for (const char* member : kFamily) {
      if (body.find(member) != std::string::npos) { asks = true; break; }
    }
    EXPECT_TRUE(asks)
        << s.signature << " (" << s.what
        << ") asks none of teamComplete() / teamSettled() / "
           "manoeuvreReleaseEligible(). Every site that acts on \"has the team "
           "come apart\" goes through that family precisely so that no two of "
           "them can disagree about it; a site deciding for itself is the "
           "generation-18 and generation-19 defect, both of which were exactly "
           "this";
  }

  // The fifth site is the heartbeat, which is not its own function — it is the
  // block that clears rendezvous_spent_ — so it is scanned by its own landmark.
  //
  // TWO THINGS MATCH THAT LANDMARK AND ONLY ONE OF THEM IS THIS SITE. The
  // member's own declaration is `bool rendezvous_spent_ = false;`, so a plain
  // find() lands on it roughly eleven thousand lines early, reads whatever
  // happens to precede the declaration, and reports on nothing. Take every
  // occurrence, drop the declaration, and require the rest — because a SECOND
  // clear added later is exactly the regression worth failing on, and it would
  // be invisible to a scan that stopped at the first hit.
  std::vector<size_t> clears;
  for (size_t at : allOf(text, "rendezvous_spent_ = false")) {
    const size_t line = text.rfind('\n', at);
    const std::string prefix =
        text.substr(line == std::string::npos ? 0 : line, at - (line == std::string::npos ? 0 : line));
    if (prefix.find("bool ") != std::string::npos) continue;  // the declaration
    clears.push_back(at);
  }
  ASSERT_FALSE(clears.empty())
      << "nothing clears rendezvous_spent_ any more, so an appointment kept "
         "once can never be kept again in the same run";
  for (size_t at : clears) {
    const size_t guard = text.rfind("if (", at);
    ASSERT_NE(guard, std::string::npos)
        << "a rendezvous_spent_ clear is not inside any conditional at all";
    std::string cond = text.substr(guard, at - guard);
    // ONE LEVEL OF ALIAS RESOLUTION (generation 23). The guard may name a local
    // bool rather than call the helper inline — it does today, because the
    // predicate is now wrapped in a flicker dwell whose window has to be
    // stepped OUTSIDE this `if` to avoid freezing, so the call sits on its own
    // statement a few lines above and the condition reads `!appointment_armed_
    // && team_back_dwelt`. Scanning only the condition text would then report
    // "a guard of its own invention" against code that is asking the family
    // question correctly.
    //
    // Resolve, do not relax. Every `identifier` in the condition that this
    // function also DECLARES (`const bool <name> =`) is replaced by its
    // initialiser, once. A genuinely invented guard still has no family member
    // anywhere in its definition and still fails. What this cannot follow is a
    // chain of two aliases, which is the honest limit of a source scan and is
    // why it is one level and says so.
    const size_t fn_start = text.rfind("\n\nvoid ExploPlannerNode::", guard);
    if (fn_start != std::string::npos) {
      for (size_t decl : allOf(text, "  const bool ")) {
        if (decl < fn_start || decl > at) continue;
        const size_t eq = text.find(" =", decl);
        const size_t semi = text.find(';', decl);
        if (eq == std::string::npos || semi == std::string::npos || eq > semi) {
          continue;
        }
        const std::string name =
            text.substr(decl + 12, eq - (decl + 12));
        if (name.empty() || cond.find(name) == std::string::npos) continue;
        cond += "\n/*resolved " + name + "*/ " +
                text.substr(eq + 2, semi - (eq + 2));
      }
    }
    bool asks = false;
    for (const char* member : kFamily) {
      if (cond.find(member) != std::string::npos) { asks = true; break; }
    }
    EXPECT_TRUE(asks)
        << "a rendezvous_spent_ release is gated on none of teamComplete() / "
           "teamSettled() / manoeuvreReleaseEligible(). It is the fifth site of "
           "the unification and the only one that decides whether the mechanism "
           "can fire a SECOND time, so a guard of its own invention here is the "
           "generation-19 defect (armed strict, released weak) with the arm and "
           "the release swapped:\n"
        << cond;
  }
}

// ===========================================================================
// GROUP C. THE APPOINTMENT-MANOEUVRE LATCH.
// ===========================================================================

/// SET ONCE, CLEARED ONCE, AND DELIBERATELY NOT CLEARED IN closeAppointment.
///
/// `appointment_manoeuvre_` records that the manoeuvre currently in flight was
/// started to keep an appointment. It is set in `startReturnTo` from the action
/// string and cleared at the END of the manoeuvre in `transitionTo`.
///
/// It must NOT be cleared in `closeAppointment`, and that is the subtle half.
/// closeAppointment runs when the APPOINTMENT is resolved, which happens on the
/// tick the robot arrives — while the manoeuvre it started is still running,
/// still holding at the meeting point, and still needing every decision keyed on
/// this latch to read true. A clear there is the bug the latch was introduced to
/// fix, arriving by the other door.
TEST(Gen20AppointmentManoeuvre, SetOnceClearedOnceAndNotInCloseAppointment) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  const std::string start =
      functionBody(text, "void ExploPlannerNode::startReturnTo(");
  ASSERT_FALSE(start.empty()) << "the scan found no startReturnTo definition";
  EXPECT_EQ(countOf(start, "appointment_manoeuvre_ ="), 1)
      << "startReturnTo does not assign appointment_manoeuvre_ exactly once; "
         "the latch has to be (re)decided on every manoeuvre start, because a "
         "latch that is only ever SET would make every later anchor_return "
         "look like an appointment";
  EXPECT_NE(start.find("std::strcmp(what, \"appointment\")"), std::string::npos)
      << "startReturnTo no longer derives appointment_manoeuvre_ from the "
         "action string, so the latch and the logged action can now disagree "
         "about what the manoeuvre is";

  const std::string trans =
      functionBody(text, "void ExploPlannerNode::transitionTo(");
  ASSERT_FALSE(trans.empty()) << "the scan found no transitionTo definition";
  EXPECT_NE(trans.find("appointment_manoeuvre_ = false"), std::string::npos)
      << "the manoeuvre latch is never cleared at the end of a manoeuvre, so "
         "the first appointment of a run makes every subsequent manoeuvre read "
         "as one";

  const std::string close =
      functionBody(text, "void ExploPlannerNode::closeAppointment(");
  ASSERT_FALSE(close.empty()) << "the scan found no closeAppointment definition";
  EXPECT_EQ(close.find("appointment_manoeuvre_"), std::string::npos)
      << "closeAppointment touches appointment_manoeuvre_. It runs on the tick "
         "the robot ARRIVES, while the manoeuvre is still in flight and still "
         "holding at the meeting point — clearing the latch there drops the "
         "settle hold and re-keys the wait cap mid-hold, which is the exact "
         "defect the latch exists to prevent";
}

/// THE WAIT CAP AND THE SETTLE HOLD ARE KEYED ON THE MANOEUVRE, NOT THE FLAG.
///
/// `appointment_armed_` answers "does an appointment stand?" and goes false the
/// moment the appointment is closed — which is the tick the robot arrives. A
/// wait cap keyed on it therefore changes value underneath a robot that is
/// standing at the meeting point waiting for its peers, and the settle hold that
/// lets the maps merge disappears at the instant it becomes relevant.
TEST(Gen20AppointmentManoeuvre, TheWaitCapAndSettleHoldAreKeyedOnTheManoeuvre) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::doReturnSync(");
  ASSERT_FALSE(body.empty()) << "the scan found no doReturnSync definition";

  const size_t cap = body.find("rendezvous_appointment_wait_sec_");
  ASSERT_NE(cap, std::string::npos)
      << "doReturnSync no longer selects a wait cap for the appointment case; "
         "the mid-run cap would then apply to a robot keeping an appointment";
  // The selection expression around it must test the manoeuvre latch, not the
  // arm flag. Scanned over the enclosing statement rather than the whole
  // function, so an unrelated appointment_armed_ elsewhere cannot satisfy it.
  const size_t stmt_begin = body.rfind(';', cap);
  const size_t stmt_end   = body.find(';', cap);
  ASSERT_NE(stmt_end, std::string::npos);
  const std::string stmt = body.substr(
      stmt_begin == std::string::npos ? 0 : stmt_begin,
      stmt_end - (stmt_begin == std::string::npos ? 0 : stmt_begin));
  EXPECT_NE(stmt.find("appointment_manoeuvre_"), std::string::npos)
      << "the wait cap is not keyed on appointment_manoeuvre_:\n"
      << stmt;
  EXPECT_EQ(stmt.find("appointment_armed_"), std::string::npos)
      << "the wait cap is keyed on appointment_armed_, which goes false on the "
         "tick the robot arrives — so the cap changes value underneath a robot "
         "that is standing at the meeting point waiting for its peers:\n"
      << stmt;

  const size_t settle = body.find("rendezvous_settle_sec_");
  ASSERT_NE(settle, std::string::npos)
      << "doReturnSync no longer holds for the settle period, so the team "
         "leaves the meeting point before the maps have merged — the second "
         "half of the governing rule (\"wait for a while more so that map is "
         "updated then begin explore\")";
  const size_t settle_stmt = body.rfind("if (", settle);
  ASSERT_NE(settle_stmt, std::string::npos);
  const std::string settle_cond = body.substr(settle_stmt, settle - settle_stmt);
  EXPECT_NE(settle_cond.find("appointment_manoeuvre_"), std::string::npos)
      << "the settle hold is not gated on appointment_manoeuvre_, so it either "
         "fires on manoeuvres that are not appointments or stops firing on the "
         "tick the appointment closes:\n"
      << settle_cond;
}

/// THE SPENT LATCH IS SET WHERE THE APPOINTMENT ARMS.
///
/// `rendezvous_spent_` is what stops one agreed pair being kept twice in a run.
/// It has to be set at the arming site — not at departure, not at arrival —
/// because every later site that could set it is on a path a robot may never
/// take, and a latch that is only set on the happy path is not a latch.
TEST(Gen20SpentLatch, IsSetWhereTheAppointmentArms) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::armAppointment(");
  ASSERT_FALSE(body.empty()) << "the scan found no armAppointment definition";

  const size_t spent = body.find("rendezvous_spent_           = true");
  ASSERT_NE(spent, std::string::npos)
      << "armAppointment no longer marks the agreed pair as spent, so one "
         "agreement can be armed repeatedly within a run";
  const size_t arm = body.find("appointment_armed_          = true");
  ASSERT_NE(arm, std::string::npos) << "armAppointment no longer arms";
  const std::string between =
      body.substr(std::min(arm, spent),
                  std::max(arm, spent) - std::min(arm, spent));
  EXPECT_EQ(between.find("return"), std::string::npos)
      << "there is a `return` between the arm and the spent latch, so an "
         "appointment can be armed without being marked spent:\n"
      << between;
}

// ===========================================================================
// GROUP F. THE AGREEMENT ITSELF (generation 23).
// ===========================================================================
//
// Kalhan's rule for this arm, verbatim: "the robots have a predefined time and
// place to meet at the start of the mission. if any of the robot is
// disconnected, all the robots go to this place at this time. then they share
// maps then after they share the maps they decide on the next place and time to
// meet. then they start exploring again."
//
// Two of its clauses are pinned mechanisms here: the time is shared, not
// private (GROUP A above), and the commit requires every robot (the scan
// below). The third — "after they share the maps they decide on the next place
// and time to meet" — is generation 29's, and is GROUP G.
//
// WHY SCANS AND NOT BEHAVIOUR: same wall as the rest of this file — the logic
// is in explo_planner_node.cpp, which defines main(). The one piece that could
// be lifted out was, and has executable tests (nextAgreedOccurrence, in
// test_planner_util.cpp). What remains is wiring, and wiring is what a scan can
// legitimately check.

/// EVERY COMMIT NEEDS EVERY ECHO, AT EVERY N.
///
/// This has been reverted twice and restored twice, so the reason it is safe NOW
/// is recorded at the code site and must be read before it is weakened a third
/// time. The short form: both measured splits were between a robot holding the
/// provisional placeholder and a robot holding the real triple, and both were
/// harmful only because the placeholder carried no meeting time. It carries the
/// full agreed schedule now, so a commit that fails to reach everyone degrades
/// to "the team keeps the agreement it already has" — which is the rule, not a
/// violation of it.
///
/// The regression this catches is the exact shape of the previous two reverts:
/// re-attaching the unanimity requirement to `rendezvous_held_provisional_`, so
/// that a FINAL triple commits on a partial echo count.
TEST(Gen23AgreedSchedule, EveryCommitRequiresEveryEcho) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::maintainRendezvousProposal(");
  ASSERT_FALSE(body.empty())
      << "the scan found no maintainRendezvousProposal definition — the "
         "agreement protocol has been renamed or split and this test has "
         "stopped testing anything";

  const size_t unanimity = body.find("if (on_pair != fleet_.size() - 1) return;");
  ASSERT_NE(unanimity, std::string::npos)
      << "the unanimity return is gone. A triple that some robots never echoed "
         "can now be committed, which is two robots driving to two places:\n"
      << body.substr(0, 400);

  // THE SHAPE OF BOTH PREVIOUS REVERTS. Conditioning the requirement on the
  // provisional flag is what let a final triple commit on a partial count.
  EXPECT_EQ(body.find("rendezvous_held_provisional_ && on_pair"),
            std::string::npos)
      << "the unanimity requirement has been re-attached to the provisional "
         "flag. That is the generation-22 rule, under which a FINAL triple "
         "commits without every echo";

  // ORDERING IS THE WHOLE ASSERTION: the return must precede the commit, or it
  // guards nothing. The commit is identified by the log line the campaign gate
  // greps for, which is inside a string literal and therefore survives
  // stripComments.
  const size_t commit = body.find("Rendezvous AGREED by all");
  ASSERT_NE(commit, std::string::npos)
      << "the commit log line is gone from maintainRendezvousProposal; either "
         "the commit moved or sim/rendezvous_agreement.py has lost its only "
         "evidence that a cell ever agreed anything";
  EXPECT_LT(unanimity, commit)
      << "the unanimity return sits AFTER the commit, so the commit happens "
         "first and the guard is decorative";
}

// ===========================================================================
// GROUP G. THE NEXT PLACE AND TIME (generation 29).
// ===========================================================================
//
// Kalhan's requirement, verbatim: "we should make the meeting times more
// separate let's say after 300s but robots should come together no matter what
// others wait until all are there, and the time to meet is updated before going
// to explore again" (2026-09-19).
//
// Three of those four clauses are wiring, and wiring is what this file checks:
// the 300 s spacing has to be a knob of its own rather than the proposal
// period's side effect, the update has to be REQUESTED by the meeting, and the
// proposer has to be allowed to derive a second time. The fourth — "come
// together no matter what" — was already true and is GROUP B's: arming is
// unconditional on the separation predicate, including the contagion case where
// the robot itself can still hear everyone.
//
// The arithmetic these tests do not do is in test_rendezvous_scheduler.cpp,
// which executes the solver against the campaign's own two numbers. Everything
// here is about which member feeds which, because that is what cannot be
// executed from a translation unit that defines main().

/// THE LATTICE AND THE BARRIER WAIT ARE THEIR OWN KNOBS, and before generation
/// 29 neither was.
///
/// The spacing used to be rendezvous_proposal_period_sec — the same 30 s that
/// paces the derive attempts and bounds the snapshot's staleness, sharing a
/// number with the timetable for no reason beyond both being periods. That is
/// what this pins apart, and it is not cosmetic: the departure test is
/// `now + lead >= t_meet`, a late robot rolls to the next rung, and hybrid
/// chases only while its appointment is not yet due — so the chase window is
/// about `interval` minus the lateness budget. On the banked generation-28
/// armings the lattice came out at 30 s against a 60 s budget, a NEGATIVE
/// window on 180 of 211 armings, and hybrid never chased once. Putting the
/// spacing back on the proposal period restores exactly that arm collapse while
/// every log line still reads correctly.
///
/// The cap is the same shape of mistake one field over. It used to be fed from
/// reconnect_midrun_max_wait_sec — how long a MID-RUN reconnect attempt waits —
/// which is not the wait a robot keeping an appointment spends. That is
/// rendezvous_appointment_wait_sec, the barrier, and it is the only one whose
/// exhaustion can make a meeting unfindable.
TEST(Gen29Timetable, TheLatticeAndTheBarrierAreSeparateKnobs) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "ExploPlannerNode::deriveRendezvousProposal()");
  ASSERT_FALSE(body.empty())
      << "the scan found no deriveRendezvousProposal definition";

  const std::string floor_rhs = assignedFrom(body, "cfg.min_interval_ms");
  ASSERT_FALSE(floor_rhs.empty())
      << "deriveRendezvousProposal no longer assigns cfg.min_interval_ms, so "
         "the lattice spacing is whatever the tours happened to ask for and "
         "the 300 s requirement is unenforced:\n"
      << body;
  EXPECT_NE(floor_rhs.find("rendezvous_interval_sec_"), std::string::npos)
      << "the lattice is not fed from rendezvous_interval_sec_. It is fed "
         "from `" << floor_rhs << "`, and if that is the proposal period "
         "again the spacing is back to 30 s, which leaves hybrid no chase "
         "window at the 60 s lateness budget";
  EXPECT_EQ(floor_rhs.find("rendezvous_proposal_period_sec_"), std::string::npos)
      << "the lattice is back on the proposal period — the exact coupling "
         "generation 29 split, and the one that collapsed hybrid into "
         "rendezvous on 180 of 211 generation-28 armings";

  const std::string cap_rhs = assignedFrom(body, "cfg.max_interval_ms");
  ASSERT_FALSE(cap_rhs.empty())
      << "deriveRendezvousProposal no longer assigns cfg.max_interval_ms:\n"
      << body;
  EXPECT_NE(cap_rhs.find("rendezvous_appointment_wait_sec_"), std::string::npos)
      << "the findability cap is not fed from the appointment barrier. It is "
         "fed from `" << cap_rhs << "`";
  EXPECT_EQ(cap_rhs.find("reconnect_midrun_max_wait_sec_"), std::string::npos)
      << "the cap is back on the mid-run reconnect wait, which is not the wait "
         "a robot keeping an appointment spends and so cannot bound whether "
         "the appointment is findable";
}

/// THE MEETING ASKS FOR THE NEXT ONE, AND ONLY AFTER THE MAPS HAVE MOVED.
///
/// "the time to meet is updated before going to explore again" is a statement
/// about ORDER, so this test is mostly about order. The release site holds the
/// gathered team for rendezvous_settle_sec while the peer voxels cross the
/// emulator, and the request has to be raised downstream of that hold — raised
/// above it, a team that reached the cell and was still exchanging would ask
/// for its next meeting on the map it arrived with, which is the map the
/// standing pair was already derived from.
///
/// The flag is a request, not the update, and it is raised on every robot while
/// only the proposer reads it — see the member's own comment for why that is
/// inert rather than a bug. What matters here is that there is exactly ONE
/// place that raises it. A second raiser is a meeting the team never had.
///
/// THE MERGE COUNTER IS CHECKED HERE because it is the release's only
/// peer-exclusive evidence: voxels move when the robot's own sensor sweeps the
/// room it is standing in, and the census hash moves on any status change, but
/// `applied` counts cells whose local status a PEER changed. Accumulating it
/// anywhere other than the single mergeWire call site would double-count, and
/// the "moved nothing at all" WARN would then stop firing on exactly the
/// exchanges it exists to catch.
TEST(Gen29ReAgreement, TheMeetingAsksForTheNextOne) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::doReturnSync(");
  ASSERT_FALSE(body.empty()) << "the scan found no doReturnSync definition";

  // TWO RAISERS, BOTH IN HERE. One at the end of the settle, where an
  // appointment asks and then stands there for the answer; one in the release
  // tail, which is all a barrier with no settle stage (a pursuit reunion, or
  // rendezvous_settle_sec 0) can do. Counted as assignments rather than as a
  // literal so the count cannot be defeated by whitespace.
  const std::vector<size_t> raises = assignmentsOf(body, "rendezvous_reagree_due_");
  ASSERT_EQ(raises.size(), 2u)
      << "doReturnSync raises the re-agreement request " << raises.size()
      << " times rather than twice. An extra raiser re-sites the meeting on a "
         "gathering that did not happen; a missing one leaves some barrier "
         "releasing on the pair the team derived before its first outage:\n"
      << body;
  // Five over the whole file: the member's own default initialiser, which
  // assignmentsOf cannot tell from a statement, the two raises above, and ONE
  // CLEAR PER ROLE — the proposer's, where it adopts its own derive, and the
  // follower's, where it adopts the proposer's. Both clears are required and
  // neither is reachable from the other's branch, so this is five and not four.
  EXPECT_EQ(assignmentsOf(text, "rendezvous_reagree_due_").size(), 5u)
      << "the request is written outside doReturnSync and the two adopt sites, "
         "so either something other than a meeting can ask for a new pair, or "
         "one of the two roles no longer spends the request when it adopts";

  const size_t request = raises.front();

  // ORDER IS THE ASSERTION. The settle hold returns early while the exchange is
  // still running; the request must sit past that return, on the path that has
  // the merged map.
  const size_t settling = body.find("settled_sec < rendezvous_settle_sec_");
  ASSERT_NE(settling, std::string::npos)
      << "the settle hold is gone from doReturnSync, so there is no map "
         "exchange for the re-agreement to be downstream OF:\n"
      << body;
  EXPECT_LT(settling, request)
      << "the request is raised BEFORE the settle hold, so a team that has "
         "arrived but not yet exchanged anything asks for its next meeting on "
         "the map it walked in with — which is the map the pair it is standing "
         "on was already derived from";

  const std::string drain =
      functionBody(text, "void ExploPlannerNode::drainTeamWorld(");
  ASSERT_FALSE(drain.empty()) << "the scan found no drainTeamWorld definition";
  EXPECT_EQ(countOf(text, "team_merge_applied_total_ +="), 1)
      << "the peer-applied cell count is accumulated in more than one place, "
         "so the release's merged_cells delta counts some merges twice and the "
         "\"moved nothing at all\" WARN stops firing on silent exchanges";
  EXPECT_EQ(countOf(drain, "team_merge_applied_total_ +="), 1)
      << "the accumulation has moved out of drainTeamWorld, away from the "
         "mergeWire call site whose MergeStats it is summing";
}

// A ZERO-SPACED LATTICE IS NOT A SCHEDULE. nextAgreedOccurrence has no next
// occurrence to roll a late robot to when the interval is zero, so it hands
// back the agreed instant unchanged and the appointment is due the moment it
// arms. armAppointment's deleted "the deadline already passed" diagnostic is
// justified in comments by "while the committed interval is positive" — this
// is the line that makes that a fact rather than an expectation.
TEST(Gen29Timetable, AZeroIntervalIsNotAValidPair) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  EXPECT_NE(text.find("return cell >= 0 && interval_ms > 0 && t_meet_ms >= 0;"),
            std::string::npos)
      << "RendezvousProposal::valid() no longer requires a positive interval, "
         "so a zero-spaced pair can be committed and every robot holding it "
         "arms an appointment that is already overdue";
}

// "... AND ONLY THEN RESUME EXPLORING" is the clause this pins. Asking for the
// next pair and driving away in the same tick satisfies the letter of
// TheMeetingAsksForTheNextOne above and none of the intent: the handshake then
// runs while the fleet disperses, and the one thing that can lose it — a link
// break — is the thing dispersing causes. The team stays on the cell until the
// commit lands.
TEST(Gen29ReAgreement, TheTeamStandsOnTheCellUntilTheNextPairCommits) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::doReturnSync(");
  ASSERT_FALSE(body.empty()) << "the scan found no doReturnSync definition";

  const size_t enter = body.find("rendezvous_reagree_waiting_ = true");
  ASSERT_NE(enter, std::string::npos)
      << "nothing enters the re-agreement wait, so the release and the request "
         "land on the same tick and the team re-agrees while it drives away:\n"
      << body;

  // THE RELEASE TEST IS THE COMMIT RULE'S OWN OUTPUT. rendezvous_agreed_ only
  // moves when every peer has echoed the same triple, and it reads the same on
  // a follower as on the proposer — unlike rendezvous_reagree_due_, which only
  // the proposer ever clears and which would therefore hold every follower on
  // the cell until the bound expired.
  const size_t test = body.find("rendezvous_agreed_ == rendezvous_reagree_from_");
  ASSERT_NE(test, std::string::npos)
      << "the wait does not release on rendezvous_agreed_ moving off the pair "
         "the robot arrived on, so it is waiting for something other than the "
         "team committing a new one:\n"
      << body;
  EXPECT_LT(enter, test) << "the wait is tested before the pair it compares "
                            "against has been stamped";

  // BOUNDED. An unbounded wait turns a handshake that cannot land -- a peer
  // that goes quiet mid-commit -- into an appointment arm that never explores
  // again, which is a worse failure than leaving on a staler pair: a schedule
  // rolls forward rather than expiring, so the team that leaves still has a
  // place and a time.
  EXPECT_NE(body.find("kRendezvousReagreeWaitSec"), std::string::npos)
      << "the re-agreement wait has no bound in doReturnSync:\n"
      << body;

  // VOID WITH THE SETTLE. The wait is that hold's second stage and a commit
  // needs every peer's echo, so a team that came apart cannot finish one —
  // leaving the flag set would hold the robot on a stale `from` when the team
  // next gathers, and skip the wait it should have run.
  EXPECT_EQ(assignmentsOf(body, "rendezvous_reagree_waiting_").size(), 3u)
      << "the wait flag is not written exactly three times in doReturnSync "
         "(entered once, cleared on the release and on the incomplete-team "
         "path), so one of the two exits leaks it:\n"
      << body;
  EXPECT_EQ(countOf(body, "rendezvous_reagree_waiting_ = false"), 2)
      << "the wait flag is cleared on fewer than both exits";
}

/// THE REQUEST REOPENS THE DERIVE, AND IS SPENT BY THE ADOPTION — NOT BY THE
/// ATTEMPT.
///
/// One pair per MEETING, which is the generation-29 rule and reads one word
/// away from the generation-23 rule it replaces (one pair per RUN). The derive
/// branch is shut between meetings; the three things that open it are no pair
/// at all, a provisional pair that can be upgraded, and a meeting the team has
/// just kept. Dropping the third from the gate is a silent revert: everything
/// still compiles, the release still raises the flag, the log still says the
/// team met, and the pair simply never changes again.
///
/// THE CLEAR IS THE SUBTLE HALF. A release that lands on a momentarily empty
/// allocator derives a centroid-fallback pair, which `!now_provisional`
/// correctly declines — and if the flag were cleared at the attempt rather than
/// at the adoption, that declined attempt would consume the request and the
/// team would go back out on the pair it had just kept, with nothing left to
/// retry. So the clear has to sit past the commit of the new pair, and the
/// provisional guard has to be in the adopt condition rather than only in the
/// gate. Both are checked by position, because both regressions are single-line
/// moves that no other assertion in this file would notice.
TEST(Gen29ReAgreement, TheProposerReopensTheDeriveAndSpendsItOnAdoptionOnly) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::maintainRendezvousProposal(");
  ASSERT_FALSE(body.empty())
      << "the scan found no maintainRendezvousProposal definition";

  const size_t gate = body.find("rendezvous_reagree_due_");
  ASSERT_NE(gate, std::string::npos)
      << "maintainRendezvousProposal never mentions the re-agreement request, "
         "so the proposer derives once per RUN again and the meeting the team "
         "just kept is the only one it will ever keep:\n"
      << body.substr(0, 400);

  const std::string adopt = assignedFrom(body, "const bool adopt");
  ASSERT_FALSE(adopt.empty())
      << "the adopt decision is no longer a named condition, so nothing below "
         "can say what it is made of:\n"
      << body;
  EXPECT_LT(gate, body.find("const bool adopt"))
      << "the request is first mentioned at or after the adopt decision, which "
         "means it is not in the derive branch condition — the derive never "
         "reopens and the adoption it guards is unreachable";
  EXPECT_NE(adopt.find("rendezvous_reagree_due_"), std::string::npos)
      << "the adopt condition does not consider the re-agreement request, so a "
         "reopened derive produces a pair and then declines to take it. The "
         "condition is `" << adopt << "`";
  EXPECT_NE(adopt.find("!now_provisional"), std::string::npos)
      << "the provisional guard is gone from the adopt condition. A barrier "
         "release that catches the allocator empty would then replace a "
         "tour-informed pair with the team's own centroid and send everyone "
         "back out aimed at it. The condition is `" << adopt << "`";

  // TWO CLEARS, one per role. The proposer spends the request when it adopts
  // its own derive; the follower spends it when it adopts the proposer's. They
  // are different robots on different branches and neither runs the other's, so
  // a single clear would leave whichever role lacks it asking forever — and on
  // the follower "asking forever" means accepting every later generation
  // unconditionally, which is the fleet split the handshake exists to refuse.
  const std::vector<size_t> cleared =
      assignmentsOf(body, "rendezvous_reagree_due_");
  ASSERT_EQ(cleared.size(), 2u)
      << "the request is written " << cleared.size()
      << " times in maintainRendezvousProposal, not twice (once on the "
         "proposer's adoption, once on the follower's)";
  const size_t commit = body.find("rendezvous_held_provisional_ = now_provisional");
  ASSERT_NE(commit, std::string::npos)
      << "the provisional flag is no longer committed alongside the adopted "
         "pair, so there is no landmark for where the adoption happens:\n"
      << body;
  EXPECT_GT(cleared.front(), commit)
      << "the request is cleared BEFORE the new pair is committed — at the "
         "attempt rather than at the adoption. A derive that comes back "
         "provisional is correctly declined and the re-agreement is then lost "
         "with nothing to retry it: the team explores on having asked for a "
         "new meeting and never got one";
}

TEST(Gen29ReAgreement, TheFollowerCanActuallyTakeTheReplacement) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::maintainRendezvousProposal(");
  ASSERT_FALSE(body.empty())
      << "the scan found no maintainRendezvousProposal definition";

  // THE DECISION MUST BE TOLD. RendezvousHandshake::adopt refuses final-over-
  // final by default, and a re-agreement is exactly that shape, so a follower
  // whose call omits the request flag cannot ever take the replacement. The
  // proposer still adopts its own, the commit gate never reaches unanimity, and
  // both appointment arms silently re-meet at the t=0 cell for the whole run.
  const size_t call = body.find("RendezvousHandshake::adopt(");
  ASSERT_NE(call, std::string::npos)
      << "the follower no longer delegates the adopt decision:\n" << body;
  const size_t call_end = body.find(");", call);
  ASSERT_NE(call_end, std::string::npos) << "unterminated adopt call";
  const std::string args = body.substr(call, call_end - call);
  EXPECT_NE(args.find("rendezvous_reagree_due_"), std::string::npos)
      << "the follower's adopt call does not pass the re-agreement request, so "
         "the replacement lands as kConflict and the team never moves off the "
         "meeting it agreed at t=0. The call is `" << args << "`";

  // AND THE ANSWER MUST BE HANDLED. Passing the flag without a branch for the
  // new answer is the same defect wearing a different hat: the decision comes
  // back kReagree, matches neither the take/upgrade branch nor the conflict
  // branch, and the follower silently does nothing at all.
  EXPECT_NE(body.find("Adopt::kReagree"), std::string::npos)
      << "maintainRendezvousProposal never mentions kReagree, so the follower "
         "asks the right question and then drops the answer on the floor";

  // AND IT MUST BE SPENT. Cleared inside the branch that took the replacement,
  // not at the call: a follower that leaves it set has stopped refusing second
  // generations for the rest of the run.
  const size_t reagree = body.find("Adopt::kReagree");
  const std::vector<size_t> cleared =
      assignmentsOf(body, "rendezvous_reagree_due_");
  ASSERT_EQ(cleared.size(), 2u) << "expected one clear per role";
  EXPECT_GT(cleared.back(), reagree)
      << "the follower's clear is not inside the kReagree handling. Left set, "
         "the flag turns adopt() into 'take any replacement forever' on this "
         "robot, which is precisely the split the refusals exist to prevent";
}
