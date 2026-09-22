// T10. Contract tests for GENERATION 21's at-the-rendezvous hold.
//
// WHAT GENERATION 21 CHANGED, in one sentence: a robot that latches exploration
// complete while standing on the agreed rendezvous cell no longer walks away
// from the meeting. Before this, maybeLatchCoverageDone ran straight into its
// ending (park, coast, or drive home) from inside RETURN_SYNC, and the smoke20
// N=3 rendezvous cell recorded the consequence exactly: all three robots logged
// `no-show arrived=True` while all three were standing on cell 45 together.
//
// THE HOLD IS FOUR EDITS, NOT ONE, and that is why this file exists. Three of
// the four are guards on paths that were previously unconditional, in functions
// a reader of the hold itself never opens:
//
//   1. THE HOLD.            maybeLatchCoverageDone returns false instead of
//                           ending, after recording the endpoint.
//   1b. THE RELEASE PATH.   doReturnSync's barrier release went to PLAN
//                           unconditionally. A latched robot sent back to PLAN
//                           explores a finished map to max_steps_ and emits a
//                           SECOND exploration_complete (the stamp is keyed on
//                           step_, not once per run), in the rendezvous and
//                           hybrid arms only, which corrupts the primary
//                           endpoint asymmetrically across arms.
//   1c. THE CAP.            The appointment patience is unbounded by default
//                           and is meant to be, so the hold needs its own cap
//                           or a finished robot can stand still until the
//                           harness wall clock ends the cell.
//   1d. THE EXPIRY PATH.    `if (!reconnect_terminal_)` is the DEFAULT state on
//                           a mid-run manoeuvre (16/16 gen-20 dispatch+end rows
//                           carry terminal=False), and it also dumps a latched
//                           robot into PLAN.
//
// Each of 1b/1c/1d is individually sufficient to reintroduce the defect, and
// none of them is near the hold in the source. A later edit that "simplifies"
// any one back to its pre-gen-21 form leaves the hold present, apparently
// working, and silently endpoint-corrupting in two of four arms. That is what
// these scans are for.
//
// SOURCE SCANS, SAME REASON AS T9. Every invariant lives in
// explo_planner_node.cpp, which is not in explo_planner_lib's source list (an
// explicit file list in CMakeLists.txt, not a glob), so nothing links it and no
// test can call it. A scan shows a guard is PRESENT and ORDERED; it cannot show
// it is REACHED. The stated limit is the same one test_gen20_rendezvous.cpp and
// test_endpoint.cpp's GROUP C carry.
//
// MUTATION-VERIFIED on 2026-09-17, because this repo has already shipped eight
// guards that went inert while still printing PASSes and every assertion of the
// form "X is absent" passes when the scan finds nothing at all. Fourteen
// deliberately broken nodes were run against this binary:
//
//    M1  the hold's `return false` deleted (falls through into the endings)
//    M2  the hold's arrival test dropped (manoeuvre alone)
//    M3  the hold's RETURN_SYNC test dropped
//    M4  the teardown flag moved ABOVE the hold
//    M5  the barrier-release `coverage_latched_` guard disabled
//    M6  that guard kept, but neither finishing nor homing (skips PLAN only)
//    M7  the latched-hold cap disabled
//    M8  the cap re-keyed on state entry instead of coverage_latch_hold_start_sec_
//    M9  the cap ordered below rendezvousWaitExpired
//   M10  `&& !coverage_latched_` dropped from the mid-run give-up
//   M11  "run-ended" reordered below "no-show"
//   M12  the classifier keyed on coverage_latched_ instead of the teardown flag
//   M13  the cap's parameter declaration removed
//   M14  the cap's finiteness validation removed
//
// Fourteen mutations, fourteen failures, each in the test named for it, with the
// node restored byte-exact afterwards (sha256 re-checked). Re-run that before
// trusting any later edit: the binary reads the node at RUN time, so it is
// edit-run-restore, no rebuild.
//
// GENERATION 32 WIDENED WHO THE HOLD CATCHES, so GROUP E is here rather than in
// a file of its own. A robot that finishes exploring with a standing
// appointment now KEEPS it (keepAppointmentOnFinish) instead of closing it and
// driving home, which means the three guards above are no longer the only ways
// into a latched robot at a meeting — and it added a fourth door of its own,
// the appointment leg's terminal rung, with the same PLAN exit and the same
// second exploration_complete behind it.
//
// It also decayed 1c, which is the reason these groups belong together. The cap
// test anchored FORWARDS on `if (coverage_latched_ &&`; generation 32 added a
// second conditional of that shape above the cap, so the anchor moved to the
// wrong block, M9 stopped being caught, and the suite stayed green. The anchor
// now searches BACK from the cap's own ending string, and M9 was re-run against
// the repair on 2026-09-21 and fails again. A scan that names a pattern rather
// than a landmark is one edit away from testing something else.
//
// MUTATION STATUS: M1-M14 are the gen-21 record above; GROUP E (M15-M22) and
// GROUP F (M23-M26) carry their own below. One sequence for the whole file.
//
// stripComments() is not optional here either. The hold's own comment block
// quotes the code it is about ("Returning false does NOT un-finish anything"),
// the release path's comment names `finishOrRendezvous("step-budget")`, and the
// expiry path's comment quotes `coverage_latched_` in prose. A raw-text scan
// would pass on a node with every guard deleted and the comments left behind —
// which is the most likely way this regresses, since comments are what a
// "cleanup" edit keeps.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

/// The source with `//` and `/* */` comments removed, string literals preserved.
/// Duplicated from test_gen20_rendezvous.cpp rather than shared: these are
/// standalone source-scan binaries that link nothing of their own, and a shared
/// header between two test executables whose whole job is to be independently
/// re-runnable against a mutated node would make one mutation break both.
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

}  // namespace

// ===========================================================================
// GROUP A. THE HOLD ITSELF.
// ===========================================================================

/// THE HOLD SITS BETWEEN THE ENDPOINT STAMP AND EVERY ENDING.
///
/// Both halves of that sentence are the assertion, and they fail differently:
///
///   * ABOVE THE ENDINGS. There are three of them (startReturnHome, the park,
///     the done_seek coast) and the first is `mission_return_enabled_ &&
///     have_home_`, which is TRUE in every campaign config. A hold placed after
///     it is dead code that reads as live — the robot drives home and the test
///     suite is green.
///   * BELOW recordExplorationComplete. The exploration ENDPOINT must not move
///     because of the hold. The robot finished when the map saturated, not when
///     the meeting resolved, and stamping it later would make the rendezvous
///     and hybrid arms pay their own hold on the primary metric.
TEST(Gen21LatchedHold, TheHoldIsBelowTheEndpointStampAndAboveEveryEnding) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::maybeLatchCoverageDone(");
  ASSERT_FALSE(body.empty())
      << "the scan found no maybeLatchCoverageDone definition — it has been "
         "renamed or reshaped and this test has stopped testing anything";

  const size_t stamp = body.find("recordExplorationComplete(\"coverage-latched\")");
  ASSERT_NE(stamp, std::string::npos)
      << "maybeLatchCoverageDone no longer stamps the exploration endpoint";

  const size_t hold = body.find("appointment_arrived_");
  ASSERT_NE(hold, std::string::npos)
      << "the at-the-rendezvous hold is gone: nothing in maybeLatchCoverageDone "
         "asks whether this robot is standing on the agreed cell, so a robot "
         "that saturates its map at the meeting walks away from it and the "
         "appointment is logged as a no-show at a meeting that happened";

  const size_t ending = body.find("mission_return_enabled_");
  ASSERT_NE(ending, std::string::npos)
      << "maybeLatchCoverageDone no longer has the mission-return ending; the "
         "ordering assertion below has nothing to order against";

  EXPECT_LT(stamp, hold)
      << "the hold is checked BEFORE the exploration endpoint is recorded, so "
         "a held robot's completion time would be stamped when the meeting "
         "resolved rather than when its map saturated — the rendezvous and "
         "hybrid arms would pay their own hold on the primary metric";
  EXPECT_LT(hold, ending)
      << "the hold is below the mission-return ending. That branch is "
         "`mission_return_enabled_ && have_home_`, which is true in every "
         "campaign config, so the hold is unreachable and the robot drives "
         "home from the meeting exactly as it did before generation 21";

  // The hold must EXIT the function, not fall through into the endings it was
  // placed above. `return false` specifically: coverage_latched_ is already
  // true, so the guard at the top of the function makes every re-entry a no-op
  // and the manifest still reports finished. `return true` would tell the
  // caller the run ended, which is the thing the hold is refusing to do.
  const size_t hold_stmt_end = body.find('}', hold);
  ASSERT_NE(hold_stmt_end, std::string::npos);
  const std::string hold_block = body.substr(hold, hold_stmt_end - hold);
  EXPECT_NE(hold_block.find("return false"), std::string::npos)
      << "the at-the-rendezvous hold does not return false, so control falls "
         "through into the endings it was placed above and the hold does "
         "nothing at all:\n"
      << hold_block;
  EXPECT_NE(hold_block.find("coverage_latch_hold_start_sec_"), std::string::npos)
      << "the hold does not stamp coverage_latch_hold_start_sec_, so the "
         "latched-hold cap (which is measured from it and is gated on it being "
         ">= 0) can never fire and the hold is unbounded:\n"
      << hold_block;
}

/// THE HOLD IS GATED ON HAVING ARRIVED, NOT ON MERELY BEING ON AN APPOINTMENT.
///
/// `appointment_manoeuvre_` is true for the whole drive, and the return-budget
/// and no-progress paths also enter RETURN_SYNC on an appointment manoeuvre with
/// arrived=false. Those robots are not standing on any agreed place, so holding
/// them is a hold in an arbitrary spot — the pre-gen-21 ending is the right one
/// for them and this gate is what keeps it.
TEST(Gen21LatchedHold, TheHoldRequiresArrivalAndTheSyncState) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::maybeLatchCoverageDone(");
  ASSERT_FALSE(body.empty())
      << "the scan found no maybeLatchCoverageDone definition";

  const size_t hold = body.find("appointment_arrived_");
  ASSERT_NE(hold, std::string::npos) << "the at-the-rendezvous hold is gone";
  const size_t guard = body.rfind("if (", hold);
  ASSERT_NE(guard, std::string::npos)
      << "the hold is not inside any conditional at all, so it would fire on "
         "every latch in every state and in every arm";
  const std::string cond = body.substr(guard, hold - guard);

  EXPECT_NE(cond.find("State::RETURN_SYNC"), std::string::npos)
      << "the hold is not gated on State::RETURN_SYNC, so a robot that latches "
         "while still DRIVING to the meeting would be held wherever it "
         "happened to be:\n"
      << cond;
  EXPECT_NE(cond.find("appointment_manoeuvre_"), std::string::npos)
      << "the hold is not gated on appointment_manoeuvre_, so it would fire on "
         "a pursuit or terminal barrier — neither of which has an agreed place "
         "to hold:\n"
      << cond;
}

/// EVERY ENDING BELOW THE HOLD IS FLAGGED AS A TEARDOWN.
///
/// `coverage_latch_teardown_` is what stops the appointment outcome classifier
/// calling a coverage-latch ending a no-show. It must be set BELOW the hold and
/// ABOVE the endings: above the hold it would fire on the held robot too, which
/// is the one case that is NOT a teardown — that robot is still keeping its
/// appointment and its outcome is still open.
TEST(Gen21LatchedHold, TheTeardownFlagIsSetBelowTheHoldAndAboveTheEndings) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::maybeLatchCoverageDone(");
  ASSERT_FALSE(body.empty())
      << "the scan found no maybeLatchCoverageDone definition";

  const size_t teardown = body.find("coverage_latch_teardown_ = true");
  ASSERT_NE(teardown, std::string::npos)
      << "maybeLatchCoverageDone no longer marks its endings as coverage-latch "
         "teardowns, so the appointment outcome classifier reads every one of "
         "them as a no-show — which is the sign reversal generation 21 exists "
         "to fix";
  EXPECT_EQ(countOf(body, "coverage_latch_teardown_ = true"), 1)
      << "maybeLatchCoverageDone sets the teardown flag more than once; the "
         "ordering assertions below only constrain the first, so a second set "
         "could sit above the hold";

  const size_t hold = body.find("appointment_arrived_");
  ASSERT_NE(hold, std::string::npos) << "the at-the-rendezvous hold is gone";
  const size_t ending = body.find("mission_return_enabled_");
  ASSERT_NE(ending, std::string::npos)
      << "maybeLatchCoverageDone no longer has the mission-return ending";

  EXPECT_LT(hold, teardown)
      << "the teardown flag is set ABOVE the hold, so a robot that is HELD at "
         "the meeting — whose appointment is still open and may yet be kept — "
         "is marked as having torn the appointment down. Its eventual outcome "
         "would read `run-ended` even when the team arrives and it should read "
         "`reconnected`";
  EXPECT_LT(teardown, ending)
      << "the teardown flag is set BELOW the first ending, so a robot that "
         "drives home from the latch reaches transitionTo with the flag still "
         "false and is classified `no-show` at a meeting it was standing on";
}

// ===========================================================================
// GROUP B. THE THREE GUARDED EXITS.
// ===========================================================================

/// 1b. THE BARRIER RELEASE MUST NOT HAND A LATCHED ROBOT BACK TO PLAN.
///
/// This release is unconditional in every generation before 21, and it was safe
/// only because a robot in RETURN_SYNC could never be finished. The hold breaks
/// that premise. A latched robot sent to PLAN explores a finished map to
/// max_steps_ and calls finishOrRendezvous("step-budget") ->
/// recordExplorationComplete a second time; that stamp is keyed on `step_`, not
/// once per run, so the cell carries TWO exploration_complete rows at very
/// different t_sim, in the rendezvous and hybrid arms only, and the readers
/// disagree about which to keep (ts1b_cells.py takes the last, n23.py the
/// first). An asymmetric corruption of the primary endpoint is strictly worse
/// than the no-show miscount the hold was added to fix.
TEST(Gen21GuardedExits, TheBarrierReleaseEndsTheRunForALatchedRobot) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::doReturnSync(");
  ASSERT_FALSE(body.empty())
      << "the scan found no doReturnSync definition — it has been renamed or "
         "reshaped and this test has stopped testing anything";

  const size_t plan = body.find("transitionTo(State::PLAN, \"barrier-released\")");
  ASSERT_NE(plan, std::string::npos)
      << "doReturnSync no longer releases the barrier to PLAN; this test has "
         "stopped testing anything";

  const size_t guard = body.find("if (coverage_latched_)");
  ASSERT_NE(guard, std::string::npos)
      << "the barrier release is unconditional again. A robot held at the "
         "meeting by the generation-21 hold is finished, and releasing it to "
         "PLAN makes it explore a saturated map to the step budget and emit a "
         "SECOND exploration_complete — corrupting the primary endpoint in the "
         "rendezvous and hybrid arms only";
  EXPECT_LT(guard, plan)
      << "the coverage_latched_ guard is BELOW the release to PLAN, so it is "
         "unreachable and the latched robot goes back to exploring";

  // The guard must END the run, not merely skip the PLAN transition: falling
  // out of doReturnSync would leave the robot in RETURN_SYNC on the next tick,
  // where the team is still complete, and it would arrive here again forever.
  const std::string guarded = body.substr(guard, plan - guard);
  EXPECT_TRUE(guarded.find("finishNow(") != std::string::npos ||
              guarded.find("startReturnHome(") != std::string::npos)
      << "the coverage_latched_ release path neither finishes nor starts the "
         "homing traverse, so it is not an ending — the robot stays in "
         "RETURN_SYNC with a complete team and re-enters this branch on every "
         "tick until the harness wall clock ends the cell:\n"
      << guarded;
  EXPECT_NE(guarded.find("coverage_latch_teardown_ = true"), std::string::npos)
      << "the coverage_latched_ release path does not flag the teardown, so "
         "the appointment it just kept SUCCESSFULLY is classified by whatever "
         "the flag happened to hold:\n"
      << guarded;
}

/// 1c. THE HOLD HAS ITS OWN CAP, CHECKED BEFORE THE ORDINARY PATIENCE.
///
/// The ordinary appointment patience is `rendezvous_appointment_wait_sec`, which
/// is 0 = unbounded by default and is MEANT to be — that patience belongs to a
/// robot with exploring left to trade against it. A finished robot has none, so
/// an unbounded hold is a cell that runs to the harness wall clock with one
/// robot standing still. Because the ordinary cap is unbounded, rendezvousWaitExpired
/// never fires on an appointment manoeuvre, so a cap placed after it is dead.
TEST(Gen21GuardedExits, TheLatchedHoldIsCappedIndependentlyOfTheWaitCap) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::doReturnSync(");
  ASSERT_FALSE(body.empty()) << "the scan found no doReturnSync definition";

  // ANCHORED FROM THE ENDING REASON BACKWARDS, and this direction is the whole
  // point. Anchoring on the member name alone would pass with the cap deleted —
  // the release path's log line names it thirty lines earlier, also above
  // rendezvousWaitExpired. Anchoring FORWARDS on the conditional was the fix for
  // that and it decayed in generation 32, which added a second
  // `if (coverage_latched_ && ...)` — the lazy hold stamp — ABOVE the cap: the
  // forward find then resolved to the stamp, every assertion here passed against
  // the wrong block, and moving the cap below rendezvousWaitExpired stopped
  // being caught while the suite still reported green. Search back from
  // "latched-hold-expired" instead. That string is the cap's own ending and
  // nothing else emits it, so the conditional immediately above it is the cap by
  // construction no matter how many siblings are added later.
  const size_t cap_end = body.find("latched-hold-expired");
  ASSERT_NE(cap_end, std::string::npos)
      << "the latched-hold cap does not reach its own ending reason string; it "
         "has been reshaped and this test has stopped testing anything";
  const size_t cap_test = body.rfind("if (coverage_latched_ &&", cap_end);
  ASSERT_NE(cap_test, std::string::npos)
      << "the latched hold has no cap gated on coverage_latched_. The "
         "appointment patience it runs under is unbounded by default, so a "
         "finished robot whose team never arrives stands on the agreed cell "
         "until the harness kills the cell";
  EXPECT_NE(body.find("rendezvous_latched_hold_sec_", cap_test),
            std::string::npos)
      << "the cap's conditional does not consult rendezvous_latched_hold_sec_, "
         "so the hold is bounded by something other than its own knob";

  const size_t expired = body.find("rendezvousWaitExpired(");
  ASSERT_NE(expired, std::string::npos)
      << "doReturnSync no longer tests rendezvousWaitExpired; the ordering "
         "assertion below has nothing to order against";
  EXPECT_LT(cap_test, expired)
      << "the latched-hold cap is checked AFTER rendezvousWaitExpired. On an "
         "appointment manoeuvre the wait cap is rendezvous_appointment_wait_sec "
         "= 0 (unbounded), so rendezvousWaitExpired is false forever and the "
         "cap below it is unreachable";

  // Measured from the LATCH, not from state entry. `waited` starts when the
  // robot entered RETURN_SYNC, which is before it finished; charging the hold
  // for that time cuts it short by however long the robot waited while it was
  // still exploring, and on a long drive-and-wait that is most of the hold.
  const std::string cap_block = body.substr(cap_test, cap_end - cap_test);
  EXPECT_NE(cap_block.find("coverage_latch_hold_start_sec_"), std::string::npos)
      << "the latched hold is not measured from coverage_latch_hold_start_sec_:\n"
      << cap_block;
  EXPECT_EQ(cap_block.find("state_enter_time_"), std::string::npos)
      << "the latched hold is measured from state entry rather than from the "
         "latch. The robot entered RETURN_SYNC before it finished, so the hold "
         "is charged for time it spent waiting while still exploring and is cut "
         "short by exactly that much:\n"
      << cap_block;
}

/// 1d. THE MID-RUN GIVE-UP MUST NOT HAND A LATCHED ROBOT BACK TO PLAN EITHER.
///
/// This is the same defect as 1b through a different door, and it is the easier
/// one to miss: `reconnect_terminal_` reads as an exceptional state but its
/// value on a mid-run manoeuvre is FALSE by construction — all 16 gen-20
/// dispatch+end row pairs carry terminal=False — so this branch is the DEFAULT
/// path out of an expired barrier, not a corner of it.
TEST(Gen21GuardedExits, TheMidRunGiveUpIsDisqualifiedByTheLatch) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::doReturnSync(");
  ASSERT_FALSE(body.empty()) << "the scan found no doReturnSync definition";

  const size_t resume =
      body.find("transitionTo(State::PLAN, \"midrun-barrier-expired\")");
  ASSERT_NE(resume, std::string::npos)
      << "doReturnSync no longer resumes exploration after a mid-run barrier "
         "expiry; this test has stopped testing anything";

  const size_t guard = body.rfind("if (!reconnect_terminal_", resume);
  ASSERT_NE(guard, std::string::npos)
      << "the mid-run give-up is no longer gated on !reconnect_terminal_; the "
         "branch has been reshaped and this test has stopped testing anything";
  const std::string cond = body.substr(guard, resume - guard);
  const size_t cond_end = cond.find(')');
  ASSERT_NE(cond_end, std::string::npos);
  const std::string predicate = cond.substr(0, cond_end);

  EXPECT_NE(predicate.find("coverage_latched_"), std::string::npos)
      << "the mid-run give-up does not exclude a latched robot. Its own "
         "premise — \"the map is not saturated\" — is exactly what the "
         "generation-21 hold falsifies, since the hold keeps a SATURATED robot "
         "inside a mid-run manoeuvre on purpose. A latched robot sent back to "
         "PLAN here re-explores a finished map and emits a second "
         "exploration_complete at the step budget:\n"
      << predicate;
}

// ===========================================================================
// GROUP C. THE OUTCOME CLASSIFIER.
// ===========================================================================

/// `run-ended` ANSWERS A PRIOR QUESTION TO arrived/unreachable, AND A LATER ONE
/// THAN team_back.
///
/// The ordering IS the semantics, and both neighbours matter:
///
///   * BELOW team_back. If the team is back, the meeting succeeded — regardless
///     of what ended this robot's run. Putting run-ended above it would relabel
///     successful reunions.
///   * ABOVE the arrived/unreachable split. Whether this robot reached the cell
///     says how the appointment was GOING; it says nothing once the run ended
///     underneath it. A teardown mid-drive is no more "unreachable" than a
///     teardown on the cell is a "no-show".
///
/// The defect this fixes is a sign reversal, not a miscount: in the smoke20 N=3
/// rendezvous cell the arm's headline failure count was made of its successes.
TEST(Gen21OutcomeClassifier, RunEndedSitsBelowTeamBackAndAboveTheArrivalSplit) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "void ExploPlannerNode::transitionTo(");
  ASSERT_FALSE(body.empty())
      << "the scan found no transitionTo definition — it has been renamed or "
         "reshaped and this test has stopped testing anything";

  const size_t reconnected = body.find("\"reconnected\"");
  const size_t run_ended   = body.find("\"run-ended\"");
  const size_t no_show     = body.find("\"no-show\"");
  const size_t unreachable = body.find("\"unreachable\"");

  ASSERT_NE(reconnected, std::string::npos)
      << "the appointment outcome classifier no longer emits `reconnected`";
  ASSERT_NE(run_ended, std::string::npos)
      << "the appointment outcome classifier no longer emits `run-ended`, so a "
         "coverage-latch teardown at the meeting is logged as a no-show. That "
         "is the smoke20 N=3 reading where all three robots recorded `no-show "
         "arrived=True` while standing on cell 45 together";
  ASSERT_NE(no_show, std::string::npos)
      << "the appointment outcome classifier no longer emits `no-show`";
  ASSERT_NE(unreachable, std::string::npos)
      << "the appointment outcome classifier no longer emits `unreachable`";

  EXPECT_LT(reconnected, run_ended)
      << "`run-ended` is tested BEFORE the team-back case, so a meeting that "
         "SUCCEEDED and then ended the run is logged as a failure";
  EXPECT_LT(run_ended, no_show)
      << "`run-ended` is tested AFTER `no-show`, so it can never be reached "
         "for a robot that arrived — which is the whole case it was added for";
  EXPECT_LT(run_ended, unreachable)
      << "`run-ended` is tested AFTER `unreachable`, so a teardown mid-drive "
         "is still classified as a navigation failure";

  // It must key on the teardown flag, not on coverage_latched_ itself: a HELD
  // robot is latched and its appointment is still open, so keying on the latch
  // would relabel a meeting that is still going to happen.
  const size_t stmt_end = body.find(';', unreachable);
  ASSERT_NE(stmt_end, std::string::npos);
  const size_t stmt_begin = std::min(reconnected, run_ended);
  const std::string stmt = body.substr(stmt_begin, stmt_end - stmt_begin);
  EXPECT_NE(stmt.find("coverage_latch_teardown_"), std::string::npos)
      << "the `run-ended` arm is not keyed on coverage_latch_teardown_:\n"
      << stmt;
  EXPECT_EQ(stmt.find("coverage_latched_ ?"), std::string::npos)
      << "the classifier keys on coverage_latched_ rather than on the teardown "
         "flag. A robot HELD at the meeting is latched too, and its "
         "appointment is still open — keying on the latch relabels a meeting "
         "that is still going to happen:\n"
      << stmt;
}

// ===========================================================================
// GROUP D. THE KNOB.
// ===========================================================================

/// THE CAP IS A DECLARED PARAMETER WITH A VALIDATED DEFAULT.
///
/// A hard-coded hold would be untunable from the harness, and the harness is
/// the only place the value is ever chosen (`RDV_LATCHED_HOLD`). The validation
/// matters for the same reason every other duration in this node validates: a
/// NaN or a negative from a typo'd override would make `held >= cap` false
/// forever, which is silently the unbounded hold this cap exists to prevent.
TEST(Gen21LatchedHold, TheCapIsADeclaredAndValidatedParameter) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  EXPECT_NE(text.find("dp(\"rendezvous_latched_hold_sec\""), std::string::npos)
      << "rendezvous_latched_hold_sec is not declared as a parameter, so the "
         "harness knob RDV_LATCHED_HOLD is inert and the cap is whatever the "
         "member initialiser says on every run";
  EXPECT_NE(text.find("std::isfinite(rendezvous_latched_hold_sec_)"),
            std::string::npos)
      << "rendezvous_latched_hold_sec_ is not checked for finiteness. A NaN "
         "makes `held >= cap` false forever, which is exactly the unbounded "
         "hold the cap exists to prevent — and it fails silently";

  // NON-POSITIVE, not merely negative. Every guard on this value reads
  // `> 0.0`, so zero DISABLES the cap rather than setting it to nothing, and
  // since generation 32 the cap is the only bound a latched keeper has: it is
  // non-terminal by construction, so rendezvous_max_wait_sec cannot end it and
  // rendezvous_appointment_wait_sec is itself 0 = forever. A `< 0.0` validator
  // admits the one value that reproduces the censored cell, through a
  // parameter that reads like a disable switch.
  const size_t read = text.find("dp(\"rendezvous_latched_hold_sec\"");
  ASSERT_NE(read, std::string::npos);
  const size_t reset = text.find("rendezvous_latched_hold_sec_ = 300.0", read);
  ASSERT_NE(reset, std::string::npos)
      << "rendezvous_latched_hold_sec_ has no fallback assignment after its "
         "read, so an unusable override is warned about and then kept";
  const std::string validator = text.substr(read, reset - read);
  EXPECT_NE(validator.find("rendezvous_latched_hold_sec_ <= 0.0"),
            std::string::npos)
      << "the cap's validator does not reject a non-positive value. Zero "
         "passes every `> 0.0` guard as 'no cap at all', so a finished robot "
         "keeping an appointment waits the barrier out to the harness wall "
         "clock — the exact censored cell generation 32 was built to remove:\n"
      << validator;
}

// ===========================================================================
// GROUP E. GENERATION 32 — THE APPOINTMENT OUTRANKS THE HOMING TRAVERSE.
//
// MUTATION-VERIFIED on 2026-09-21 by the same edit-run-restore method, node
// sha256 re-checked after each:
//
//    M15  the latch's keep changed back to `return false` (the authored defect
//         this group was written after — doPlan plans on under the dispatch)
//    M16  the keep dropped from finishOrRendezvous
//    M17  the keep moved BELOW mission_return_enabled_ in the latch
//    M18  the unplaceable decline deleted
//    M19  reconnect_terminal_/appointment_departed_ moved below startReturnTo
//    M20  the terminal rung's coverage_latched_ guard deleted
//    M21  that guard kept, but neither finishing nor homing
//    M22  the cap validator relaxed from `<= 0.0` back to `< 0.0`
// ===========================================================================

/// THE KEEP IS CALLED FROM BOTH ENDINGS, AND ABOVE MISSION RETURN IN EACH.
///
/// Both halves fail differently and both are silent:
///
///   * ONE CALLER ONLY. The two endings catch different robots — the coverage
///     latch catches one that saturates mid-tick, finishOrRendezvous catches
///     the step budget and the peer-announced paths — and a robot that reaches
///     the ending this call is missing from abandons its meeting exactly as it
///     did in generation 31.
///   * BELOW `mission_return_enabled_`. That branch is true in every campaign
///     config, so a keep placed under it never runs. This is not hypothetical:
///     it is precisely why dispatchReconnect's own "exploration is over, go to
///     the appointment now" branch was unreachable for eleven generations
///     while reading as live.
TEST(Gen32KeepAppointment, BothEndingsKeepItAheadOfMissionReturn) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  const char* kEndings[] = {"bool ExploPlannerNode::maybeLatchCoverageDone(",
                            "bool ExploPlannerNode::finishOrRendezvous("};
  for (const char* sig : kEndings) {
    const std::string body = functionBody(text, sig);
    ASSERT_FALSE(body.empty()) << "the scan found no definition for " << sig;

    const size_t keep = body.find("keepAppointmentOnFinish(");
    ASSERT_NE(keep, std::string::npos)
        << sig
        << " does not keep a standing appointment. A robot that finishes "
           "exploring there closes the meeting its peer is driving to and "
           "goes home instead, and the peer can only discover that by waiting "
           "its barrier out — ts4_31 rendezvous seed 6, censored at the "
           "duration cap";
    const size_t ending = body.find("mission_return_enabled_");
    ASSERT_NE(ending, std::string::npos)
        << sig << " no longer has a mission-return ending to order against";
    EXPECT_LT(keep, ending)
        << sig
        << " keeps the appointment BELOW the mission-return branch. That "
           "branch is `mission_return_enabled_ && have_home_`, true in every "
           "campaign arm, so the keep is unreachable and this generation "
           "changes nothing";
  }
}

/// THE LATCH'S KEEP RETURNS TRUE, AND THE DISTINCTION IS THE SAFETY ARGUMENT.
///
/// maybeLatchCoverageDone's return value means "the caller must stop this
/// tick", and doPlan is the only caller that acts on it. The gen-21 hold above
/// can afford `false` because it is gated on State::RETURN_SYNC, which doPlan
/// is never in; this call is not, because a robot saturates from PLAN. `false`
/// here lets doPlan run on past a dispatch that has already published the
/// meeting goal, select a frontier, publish it over that goal and enter
/// NAVIGATE — at which point the classifier closes the appointment
/// `unreachable` on a fabricated navigation failure, and with the coverage
/// ending spent there is nothing left to end the run but max_steps_.
TEST(Gen32KeepAppointment, TheLatchStopsThePlanTickWhenItKeeps) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::maybeLatchCoverageDone(");
  ASSERT_FALSE(body.empty())
      << "the scan found no maybeLatchCoverageDone definition";

  const size_t keep = body.find("keepAppointmentOnFinish(");
  ASSERT_NE(keep, std::string::npos)
      << "the coverage latch does not keep a standing appointment";
  const size_t stmt_end = body.find(';', keep);
  ASSERT_NE(stmt_end, std::string::npos);
  const std::string stmt = body.substr(keep, stmt_end - keep);

  EXPECT_NE(stmt.find("return true"), std::string::npos)
      << "the latch's keep does not return true, so doPlan carries on planning "
         "underneath a dispatched appointment drive and overwrites the meeting "
         "goal with a frontier on the same tick:\n"
      << stmt;
  EXPECT_EQ(stmt.find("return false"), std::string::npos)
      << "the latch's keep returns false. doPlan reads that as 'not finished, "
         "keep planning' and publishes a frontier over the meeting goal:\n"
      << stmt;
}

/// AN UNPLACEABLE CELL IS DECLINED BEFORE ANY STATE MOVES.
///
/// appointmentPoint() has a branch that cannot solve the agreed cell on either
/// grid: it latches `appointment_unplaceable_` and answers the robot's OWN
/// position. Departing for that "arrives" on the next tick, so the keeper would
/// hold a barrier at its own feet for the full latched cap over a meeting no
/// travel of its could reach. The check must be above `appointment_departed_`,
/// because declining after that flag is set closes the record as a departure
/// that never happened.
TEST(Gen32KeepAppointment, AnUnplaceableCellIsDeclinedBeforeDeparture) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::keepAppointmentOnFinish(");
  ASSERT_FALSE(body.empty())
      << "the scan found no keepAppointmentOnFinish definition — it has been "
         "renamed or reshaped and this test has stopped testing anything";

  const size_t unplaceable = body.find("appointment_unplaceable_");
  ASSERT_NE(unplaceable, std::string::npos)
      << "the keep does not consult appointment_unplaceable_, so a robot that "
         "cannot place the agreed cell drives to its own feet, records an "
         "arrival, and holds the barrier there for the whole latched cap";
  const size_t departed = body.find("appointment_departed_ = true");
  ASSERT_NE(departed, std::string::npos)
      << "the keep no longer marks the appointment as departed, so the "
         "classifier calls the drive it just started `superseded`";
  const size_t dispatch = body.find("startReturnTo(");
  ASSERT_NE(dispatch, std::string::npos)
      << "the keep no longer dispatches the drive to the agreed cell";

  EXPECT_LT(unplaceable, departed)
      << "the unplaceable check sits BELOW appointment_departed_, so the "
         "decline leaves a departure flag set for a drive that was never made";
  EXPECT_LT(unplaceable, dispatch)
      << "the unplaceable check sits BELOW the dispatch, so the goal at the "
         "robot's own feet has already been published by the time it fires";

  // Both flags above the dispatch, for the reason the appointment-due
  // departure states: the classifier and the barrier's wait cap are read
  // inside the transition startReturnTo tails into, so assigning after it
  // labels the manoeuvre as the `superseded` this path exists to stop.
  const size_t terminal = body.find("reconnect_terminal_");
  ASSERT_NE(terminal, std::string::npos)
      << "the keep does not set reconnect_terminal_, so a keeper that is NOT "
         "coverage-latched takes the unbounded appointment patience with no "
         "latched-hold cap to end it — an unbounded wait with nothing left to "
         "stop it, which is the failure this path exists to remove";
  EXPECT_LT(terminal, dispatch)
      << "reconnect_terminal_ is assigned after the dispatch, so the barrier "
         "cap is chosen from the previous manoeuvre's value";
  EXPECT_LT(departed, dispatch)
      << "appointment_departed_ is assigned after the dispatch, so the "
         "classifier sees a keeper that never left";
}

/// THE APPOINTMENT LEG'S LAST RUNG MUST NOT HAND A LATCHED ROBOT BACK TO PLAN.
///
/// The same defect as 1b and 1d through a third door, and this one opened only
/// in generation 32: before the keep, no drive to an agreed cell could be
/// carrying a finished robot, so every robot this rung returned to PLAN still
/// had its ending in front of it. One that has spent the coverage ending does
/// not — first-touch latching will not give it another — so the exit leaves
/// nothing but max_steps_ and a SECOND exploration_complete, in the treated
/// arms only.
TEST(Gen32KeepAppointment, TheUnreachedLegDoesNotResumeALatchedRobot) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::appointmentLegWatchdog(");
  ASSERT_FALSE(body.empty())
      << "the scan found no appointmentLegWatchdog definition";

  const size_t resume =
      body.find("transitionTo(State::PLAN, \"appointment-unreached\")");
  ASSERT_NE(resume, std::string::npos)
      << "the appointment leg no longer returns to PLAN when it runs out of "
         "rungs; this test has stopped testing anything";
  const size_t guard = body.rfind("if (coverage_latched_)", resume);
  ASSERT_NE(guard, std::string::npos)
      << "the terminal rung of the appointment leg is not guarded against a "
         "latched robot. It hands one back to PLAN with its coverage ending "
         "already spent, which re-explores a finished map to the step budget "
         "and emits a second exploration_complete in the rendezvous and hybrid "
         "arms only";
  const std::string guarded = body.substr(guard, resume - guard);

  EXPECT_TRUE(guarded.find("finishNow(") != std::string::npos &&
              guarded.find("startReturnHome(") != std::string::npos)
      << "the latched branch of the terminal rung neither finishes nor starts "
         "the homing traverse, so it is not an ending and control falls into "
         "the PLAN exit below it anyway:\n"
      << guarded;
  EXPECT_NE(guarded.find("coverage_latch_teardown_ = true"), std::string::npos)
      << "the latched branch does not flag the teardown, so the appointment it "
         "is closing is classified by whatever the flag happened to hold — "
         "`unreachable`, a navigation failure, rather than the run ending "
         "underneath the drive:\n"
      << guarded;
}

// ===========================================================================
// GROUP F. THE CAP'S CLOCK STARTS NO EARLIER THAN THE MEETING.
// ===========================================================================

/// BOTH STAMPS FLOOR THE HOLD'S START AT t_meet.
///
/// Until 2026-09-22 both wrote the bare mission clock, on the strength of a
/// note in doReturnSync asserting that "t_meet is the DEPARTURE time, so
/// arrival is necessarily at or after it". That was true of the bare
/// `now >= t_meet` test that lived between 2026-09-18 and 2026-09-19.
/// appointmentDue() has departed at t_meet minus appointmentLeadMs ever since,
/// precisely so the ARRIVAL lands on t_meet — which makes an early arrival the
/// designed case rather than an impossible one.
///
/// WHAT AN EARLY START COSTS IS THE CAP'S SIZING MARGIN, not tidiness.
/// run_explo_sim_rviz.sh derives RDV_LATCHED_HOLD as interval + max_lateness +
/// 60 s so one rolled rung is always covered, and that arithmetic measures from
/// t_meet, because the teammate it is sized against rolled to t_meet plus a
/// whole interval. On a deadline departure the lead cannot exceed the time
/// remaining to the rung, so the early margin reaches interval/6 — 50 s of the
/// 60 s the derivation has. A KEEPER SPENDS FAR MORE: keepAppointmentOnFinish
/// leaves the instant the map saturates and never asks appointmentDue(), so a
/// robot that finishes early stands out most of the countdown and unclamped
/// would reach the cap before the meeting fell due. Either way the finished
/// robot walks off the cell as the robot it was waiting for drives onto it,
/// which is the one outcome the hold exists to forbid — and nothing in the node
/// checks the relation (run_explo_sim_rviz.sh says so in as many words), so
/// this scan is the check.
///
/// TWO STAMPS, TWO ASSERTIONS, and only the pair covers the hold: the latch
/// stamps a robot that saturated while already standing at the cell,
/// doReturnSync stamps one that finished on the road and stopped here later.
/// Flooring either alone leaves the other starting early.
///
/// MUTATION-VERIFIED 2026-09-22, same edit-run-restore as M1-M14 above.
/// NUMBERED FROM 23 because GROUP E already holds M15-M22; the ledger is one
/// sequence for the whole file and a reused id makes two different mutations
/// read as one re-run.
///
///   M23  the latch's floor dropped (bare mission clock)
///   M24  doReturnSync's floor dropped, the latch's kept
///   M25  the floor written as a ceiling (min, not max)
///   M26  the floor applied without appointment_.valid(), so a closed
///        appointment contributes t_meet_ms = -1 as a meeting instant
///
/// Four mutations, four failures, node restored byte-exact (sha256 re-checked).
TEST(Gen33HoldClock, BothStampsFloorTheCapAtTheMeetingInstant) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  const std::string latch =
      functionBody(text, "bool ExploPlannerNode::maybeLatchCoverageDone(");
  ASSERT_FALSE(latch.empty())
      << "the scan found no maybeLatchCoverageDone definition";
  const std::string sync =
      functionBody(text, "void ExploPlannerNode::doReturnSync(");
  ASSERT_FALSE(sync.empty()) << "the scan found no doReturnSync definition";

  // Anchored BACK from the write, not forward from a pattern — the lesson
  // GROUP E's decayed anchor taught. A landmark survives an edit above it.
  const std::pair<std::string, std::string> stamps[] = {
      {"maybeLatchCoverageDone", latch}, {"doReturnSync", sync}};
  for (const auto& fn : stamps) {
    const size_t write =
        fn.second.find("coverage_latch_hold_start_sec_ = t_hold;");
    ASSERT_NE(write, std::string::npos)
        << fn.first
        << " no longer stamps the latched hold's clock through a floored "
           "value, so the cap starts wherever this function happens to write "
           "and an early arrival spends its sizing margin before the team is "
           "due";
    const size_t clock = fn.second.rfind("missionElapsed()", write);
    ASSERT_NE(clock, std::string::npos)
        << fn.first << " stamps the hold without reading the mission clock";
    const std::string window = fn.second.substr(clock, write - clock);

    EXPECT_NE(window.find("appointment_.t_meet_ms"), std::string::npos)
        << fn.first
        << " starts the latched hold's cap on the bare mission clock. A robot "
           "that arrives early — the designed case since 2026-09-19, not an "
           "edge one — burns up to interval/6 of the cap before the meeting is "
           "due, against a derivation that carries 60 s of margin:\n"
        << window;
    EXPECT_NE(window.find("std::max"), std::string::npos)
        << fn.first
        << " reads t_meet without taking the later of the two, so the floor is "
           "not a floor and a LATE arrival would have its cap rewound instead:\n"
        << window;
    EXPECT_NE(window.find("appointment_.valid()"), std::string::npos)
        << fn.first
        << " floors on t_meet_ms without screening the appointment, so a closed "
           "one contributes its -1 sentinel as a meeting instant:\n"
        << window;
  }
}

// ===========================================================================
// GROUP G. GENERATION 33 — WHAT ENDS THE MAP-EXCHANGE HOLD.
// ===========================================================================

/// THE DRAIN PREDICATE IS A LEVEL, THEN A RATE, OVER EVERY PEER IT CAN READ.
///
/// This is test-plan item 8's structural half. The behavioural half cannot be
/// a gtest at all: the predicate lives in explo_planner_node.cpp, which is not
/// in explo_planner_lib's source list, so nothing links it and a scan is the
/// only check there is.
///
/// THREE INDEPENDENT WAYS THIS RELEASES ON NOTHING, and each is an assertion:
///
///   * DROP THE LEVEL. A peer whose bytes are not crossing the radio and a peer
///     that has sent everything it has BOTH present a rate of zero over the
///     window. Reduced to a rate test the release fires FASTEST in exactly the
///     blackout the hold exists to sit through — which is how the defect
///     entered the design in the first place, not a hypothetical.
///   * DROP THE RATE. The complement: "the counter stopped moving" over-holds,
///     measured, in 10 of 31 long gen-32 meetings that were still gaining
///     voxels at departure.
///   * READ NOBODY. Every skip in the loop is a peer this robot could not read,
///     and with all of them taken the loop leaves `drained` at the true it was
///     initialised to. The hold opens on max(active, reachablePeerCount()),
///     which counts peers the loop may decline — so a relay-only team would
///     open a hold and release it in the same breath, having examined no one.
///     The direct-only filter this replaces made that reachable by an ordinary
///     N=3 topology rather than an exotic one.
///
/// Relayed peers are IN. §4.1 of the design settles it on measurement: relayed
/// rows applied a merge 15.5% of the time against 0.9% overall, so excluding
/// them drops the most productive exchange channel on the team while claiming
/// the exchange finished.
///
/// MUTATION-VERIFIED 2026-09-22, edit-run-restore on the node source, no
/// rebuild (nothing links it):
///
///   M27  the presence filter narrowed back to `!p.direct` alone
///   M28  `if (examined == 0) drained = false;` deleted
///   M29  clause 1 deleted, leaving the rate test alone
///   M30  clause 2 deleted, leaving the level test alone
///   M31  `bool drained = measurable;` initialised to bare `true`
///
/// Five mutations, five failures, node restored byte-exact (sha256 re-checked).
TEST(Gen33DrainRelease, ThePredicateIsALevelThenARateOverEveryReadablePeer) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  const std::string sync =
      functionBody(text, "void ExploPlannerNode::doReturnSync(");
  ASSERT_FALSE(sync.empty()) << "the scan found no doReturnSync definition";

  // Anchored on the accumulator's declaration and closed on its first read,
  // so the window is the loop and its verdict and nothing else. Same
  // anchor-back-from-the-landmark discipline as GROUP F.
  const size_t begin = sync.find("bool drained = measurable;");
  ASSERT_NE(begin, std::string::npos)
      << "the drain verdict is no longer seeded from `measurable`, so a "
         "meeting with no per-peer counters to difference can release on the "
         "absence of evidence instead of holding to the cap";
  const size_t end = sync.find("if (!drained)", begin);
  ASSERT_NE(end, std::string::npos)
      << "the drain verdict is never consumed after the loop";
  const std::string window = sync.substr(begin, end - begin);

  EXPECT_NE(window.find("!p.direct && !p.via_relay"), std::string::npos)
      << "the exchange loop no longer reads relayed peers. A two-hop partner "
         "still delivers — dscovox credits the robot that SENSED the voxels, "
         "not the one that bridged them — and gen 32 measured relay as the "
         "most productive channel per row, so skipping it lets the busiest "
         "stream on the meeting keep arriving while this test calls the "
         "exchange finished:\n"
      << window;
  EXPECT_NE(window.find("if (examined == 0) drained = false;"),
            std::string::npos)
      << "the loop can now fall out having examined no peer at all and still "
         "report drained, which releases the hold on nothing:\n"
      << window;
  EXPECT_NE(window.find("rendezvous_exchange_.peer_deltas[k]"),
            std::string::npos)
      << "CLAUSE 1 IS GONE. Without the level against the hold-start baseline "
         "a silent peer and a finished peer are the same observation, and the "
         "release fires fastest in a blackout:\n"
      << window;
  EXPECT_NE(window.find("rendezvous_drain_rate_vox_sec_"), std::string::npos)
      << "CLAUSE 2 IS GONE. A pure zero-test over-holds: a third of the long "
         "gen-32 meetings were still gaining when the robot departed:\n"
      << window;
}

/// R AND W ARE REFUSED, NOT DEFAULTED — AND R = 0 IS A REFUSAL TOO.
///
/// The knobs decide when a robot stops waiting for its partner's map. A
/// substituted default would run, log a plausible arm name, and be a different
/// experiment from the one the label claims.
///
/// R = 0 IS NOT THE PERMISSIVE END OF THE RANGE, WHICH IS WHY THE BOUND IS
/// STRICT. The test is `rate >= R -> not drained` and `rate` is clamped at zero
/// by construction, so R = 0 is true for every peer on every window: the
/// release can never fire, every meeting runs to the cap, and the arm reports
/// UNFINISHED EXCHANGE for exchanges that finished. That is the treatment
/// silently not running under its own name — indistinguishable in the tables
/// from a mechanism that does nothing.
///
/// MUTATION-VERIFIED 2026-09-22, same edit-run-restore:
///
///   M32  the rate bound relaxed from `<= 0.0` back to `< 0.0`
///   M33  the window bound relaxed from `<= 0.0` to `< 0.0`
///   M34  the whole refusal block deleted — the `if` through its closing
///        brace, RCLCPP_ERROR and throw with it, so the parameters are read
///        and then simply used
///
/// Three mutations, three failures, node restored byte-exact.
///
/// M34 IS SPELLED OUT BECAUSE THE OBVIOUS CHEAP VERSION OF IT IS NOT A TEST.
/// Reword the throw's message and this test fails too, but only because the
/// message is what it searches for — that kills the anchor, not the guard, and
/// it would pass just as well if the guard had been reduced to a no-op that
/// still threw the same string. The mutation has to remove the refusal.
TEST(Gen33DrainRelease, TheThresholdsAreRefusedAndZeroIsRefusedWithThem) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;

  const size_t begin = text.find("dp(\"rendezvous_drain_rate_vox_sec\"");
  ASSERT_NE(begin, std::string::npos)
      << "rendezvous_drain_rate_vox_sec is no longer a declared parameter";
  const size_t end =
      text.find("rendezvous_drain_release requires", begin);
  ASSERT_NE(end, std::string::npos)
      << "the thresholds are read but never refused, so the arm can run on "
         "whatever the yaml omitted";
  const std::string window = text.substr(begin, end - begin);

  EXPECT_EQ(countOf(window, "<= 0.0"), 2)
      << "both thresholds must be refused at zero, not just below it. R = 0 "
         "makes `rate >= R` true for every peer on every window, so the drain "
         "never releases and the arm reports an unfinished exchange for every "
         "meeting; W = 0 divides the window into a rate with no interval:\n"
      << window;
}
