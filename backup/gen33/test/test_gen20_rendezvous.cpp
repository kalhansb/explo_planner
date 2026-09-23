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
// Moved comments: doc/test_gen20_rendezvous_notes.md

#include <gtest/gtest.h>

#include <cctype>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

/// The source with comments removed, string literals preserved. Every scan here
/// runs on it because the node's comments quote code; quote tracking keeps a //
/// inside a string from truncating code. (notes: scan-strip-comments)
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
/// Returns an empty string when not found; every caller must ASSERT on that.
/// (notes: scan-function-body)
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

/// The right-hand side of the statement assigning lhs, or an empty string if
/// lhs is absent or its first appearance is not an assignment (callers ASSERT).
/// Skips whitespace, so aligned or wrapped assignments match.
/// (notes: scan-assigned-from)
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

/// Every byte offset at which needle is assigned (followed by = but not ==), as
/// opposed to merely read. Use it where which occurrence is the write matters,
/// not just the count. (notes: scan-assignments-of)
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

/// armAppointment must write appointment_.t_meet_ms before it arms, with no
/// return between. RendezvousPlan::t_meet_ms defaults to -1 and valid() never
/// checks it, so an armed slot on -1 is due forever.
/// (notes: deadline-written-before-arm)
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

/// The deadline is the first agreed occurrence this robot can reach within
/// rendezvous_max_lateness_sec, floored at t_now plus its own shortfall
/// (clamped at zero). The arithmetic is in nextAgreedOccurrence.
/// (notes: deadline-agreed-occurrence)
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
  // The sign is scanned literally: arrivalShortfallSec clamps at zero, so t_now
  // + shortfall_sec never floors before now. A subtraction would pass every
  // other assertion here. (notes: deadline-floor-sign)
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

/// The rung is chosen once, at arming. appointmentDue recomputes only when to
/// leave (now + lead >= t_meet); re-choosing the rung there would ratchet the
/// meeting away. With no travel estimate (-1) it leaves at t_meet.
/// (notes: departure-lead-no-rung)
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

  // Nothing else stops appointmentDue from writing appointment_.t_meet_ms.
  // Checked as any occurrence followed by an assignment, because the node
  // column-aligns its assignments. (notes: departure-ratchet-guard)
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

/// appointmentLeadMs must price departure with the scheduler's own markup,
/// rzv_cfg_.depart_safety_milli (floor_ms in rendezvous_scheduler.cpp). A lead
/// above the sized spacing is a shortfall no rung roll fixes.
/// (notes: departure-lead-shared-markup)
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

/// Every site acting on team separation must ask teamComplete, teamSettled or
/// manoeuvreReleaseEligible, which must root in teamComplete. Presence only:
/// rendezvousTeamMutual stays right for the anchor stamp and derive gate.
/// (notes: separation-predicate-family)
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

  // The fifth site, the heartbeat, is the block that clears rendezvous_spent_.
  // Every clear is checked except the member's own declaration, so a second
  // clear added later is caught too. (notes: separation-heartbeat-site-scan)
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
    // A local const bool named in the guard has its initialiser added to the
    // scanned condition, once, so a dwell-wrapped alias still counts as asking
    // the family. A chain of two aliases is not followed.
    // (notes: separation-alias-resolution)
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

/// appointment_manoeuvre_ is set in startReturnTo from the action string and
/// cleared in transitionTo when the manoeuvre ends. It must not be cleared in
/// closeAppointment, which runs on arrival while the manoeuvre holds.
/// (notes: manoeuvre-latch-set-and-clear)
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

/// The wait cap and settle hold in doReturnSync must key on
/// appointment_manoeuvre_, not appointment_armed_, which goes false on the tick
/// the robot arrives. (notes: manoeuvre-latch-keys-wait-cap)
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

/// rendezvous_spent_ latches an agreed pair as used once it arms. It must be
/// set at the arming site, not at departure or arrival, which a robot may never
/// reach. (notes: spent-latch-at-arming)
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
// Pins that the meeting time is shared, not private (GROUP A), and that the
// commit requires every robot. Scans, because the logic is in
// explo_planner_node.cpp, which defines main(). (notes: agreement-group-scope)

/// Every commit needs every echo, at every N. Do not re-attach the unanimity
/// requirement to rendezvous_held_provisional_: that lets a final triple commit
/// on a partial echo count. (notes: agreement-commit-every-echo)
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
// Pins the wiring: the meeting spacing is its own knob, the meeting requests
// the update, and the proposer may derive again. The solver arithmetic is
// tested in test_rendezvous_scheduler.cpp. (notes: reagree-group-scope)

/// Lattice spacing comes from rendezvous_interval_sec_, not the proposal
/// period; the cap from rendezvous_appointment_wait_sec_, not
/// reconnect_midrun_max_wait_sec_. The chase window is about interval minus
/// lateness budget. (notes: timetable-separate-knobs)
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

/// The re-agreement request must be raised after the settle hold, on the path
/// with the merged map. team_merge_applied_total_ is summed only at the
/// mergeWire call site, or the moved-nothing WARN stops firing.
/// (notes: reagree-after-settle)
TEST(Gen29ReAgreement, TheMeetingAsksForTheNextOne) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::doReturnSync(");
  ASSERT_FALSE(body.empty()) << "the scan found no doReturnSync definition";

  // Two raisers, both in doReturnSync: one at the end of the settle, one in the
  // release tail for barriers with no settle stage (pursuit reunion,
  // rendezvous_settle_sec 0). Counted as assignments.
  // (notes: reagree-two-raisers)
  const std::vector<size_t> raises = assignmentsOf(body, "rendezvous_reagree_due_");
  ASSERT_EQ(raises.size(), 2u)
      << "doReturnSync raises the re-agreement request " << raises.size()
      << " times rather than twice. An extra raiser re-sites the meeting on a "
         "gathering that did not happen; a missing one leaves some barrier "
         "releasing on the pair the team derived before its first outage:\n"
      << body;
  // assignmentsOf finds writes, not values: a raiser flipped to false keeps the
  // count and the ordering and silently ends re-agreement.
  // (notes: reagree-raisers-write-true)
  for (const size_t at : raises) {
    const std::string rhs =
        assignedFrom(body.substr(at), "rendezvous_reagree_due_");
    EXPECT_NE(rhs.find("true"), std::string::npos)
        << "a raiser writes `" << rhs << "` rather than true, so a meeting the "
           "team kept does not ask for the next one";
  }
  // Five writes over the file: the member's default initialiser, the two
  // raises, and one clear per role (the proposer on adopting its derive, the
  // follower on adopting the proposer's). (notes: reagree-five-writes)
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

  // The release-tail raiser sits under the rendezvous_reagree_waiting_ guard,
  // so only barriers with no settle stage raise there. Re-raising after an
  // appointment's commit re-derives and slides the agreed t_meet.
  // (notes: reagree-release-tail-guard)
  const std::string guard = "if (!rendezvous_reagree_waiting_) {";
  const size_t guarded = body.rfind(guard, raises.back());
  ASSERT_NE(guarded, std::string::npos)
      << "the release-tail re-agreement raiser has no "
         "rendezvous_reagree_waiting_ guard before it:\n"
      << body;
  EXPECT_EQ(body.find_first_not_of(
                " \t\r\n", guarded + guard.size()), raises.back())
      << "the release-tail raiser is not the first statement of its "
         "rendezvous_reagree_waiting_ guard, so either it is unguarded — an "
         "appointment re-derives after its wait already committed the next "
         "pair, and t_meet slides — or the guard now covers other writes too";

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

// RendezvousProposal::valid() must require interval_ms > 0: with a zero
// interval nextAgreedOccurrence cannot roll a late robot forward, so the
// appointment is due the moment it arms.
// (notes: timetable-zero-interval-invalid)
TEST(Gen29Timetable, AZeroIntervalIsNotAValidPair) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  EXPECT_NE(text.find("return cell >= 0 && interval_ms > 0 && t_meet_ms >= 0;"),
            std::string::npos)
      << "RendezvousProposal::valid() no longer requires a positive interval, "
         "so a zero-spaced pair can be committed and every robot holding it "
         "arms an appointment that is already overdue";
}

// The team stays on the cell until the next pair commits: re-agreeing while
// dispersing risks the link break that loses the handshake.
// (notes: reagree-stand-until-commit)
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

  // The wait releases when rendezvous_agreed_ moves off the pair the robot
  // arrived on. It moves only on a unanimous commit and reads the same on every
  // robot, so followers release too. (notes: reagree-release-on-agreed)
  const size_t test = body.find("rendezvous_agreed_ == rendezvous_reagree_from_");
  ASSERT_NE(test, std::string::npos)
      << "the wait does not release on rendezvous_agreed_ moving off the pair "
         "the robot arrived on, so it is waiting for something other than the "
         "team committing a new one:\n"
      << body;
  EXPECT_LT(enter, test) << "the wait is tested before the pair it compares "
                            "against has been stamped";

  // The wait is bounded (kRendezvousReagreeWaitSec): a handshake that cannot
  // land must not stop the arm exploring. The old schedule rolls forward, so
  // the team still has a place and a time. (notes: reagree-wait-bounded)
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

/// The derive reopens on no pair, a provisional pair, or
/// rendezvous_reagree_due_. The request is cleared only after the new pair
/// commits, and !now_provisional sits in the adopt condition.
/// (notes: reagree-reopen-and-spend)
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

  // Two clears, one per role: the proposer when it adopts its own derive, the
  // follower when it adopts the proposer's. A role without its clear asks
  // forever; a follower then accepts every later generation.
  // (notes: reagree-clear-per-role)
  const std::vector<size_t> cleared =
      assignmentsOf(body, "rendezvous_reagree_due_");
  ASSERT_EQ(cleared.size(), 2u)
      << "the request is written " << cleared.size()
      << " times in maintainRendezvousProposal, not twice (once on the "
         "proposer's adoption, once on the follower's)";
  // Both writes must be false: an inverted clear sits where a clear belongs,
  // passes every offset check, and leaves the role asking forever.
  // (notes: reagree-clears-write-false)
  for (const size_t at : cleared) {
    const std::string rhs =
        assignedFrom(body.substr(at), "rendezvous_reagree_due_");
    EXPECT_NE(rhs.find("false"), std::string::npos)
        << "an adopt site writes `" << rhs << "` rather than false, so the "
           "request outlives the adoption that was supposed to spend it";
  }
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

  // RendezvousHandshake::adopt refuses final-over-final by default, so the
  // follower's call must pass rendezvous_reagree_due_ or it can never take the
  // replacement. (notes: reagree-follower-passes-flag)
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
