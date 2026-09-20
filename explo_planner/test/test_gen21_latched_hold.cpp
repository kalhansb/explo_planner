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

  // ANCHORED ON THE CAP'S OWN CONDITIONAL, not on the member name. The member
  // is also named in the release path's log line thirty lines earlier, which is
  // above rendezvousWaitExpired — so a scan keyed on the name alone would keep
  // passing the ordering assertion below with the entire cap block deleted.
  const size_t cap_test = body.find("if (coverage_latched_ &&");
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
  const size_t cap_end = body.find("latched-hold-expired", cap_test);
  ASSERT_NE(cap_end, std::string::npos)
      << "the latched-hold cap does not reach its own ending reason string; it "
         "has been reshaped and this test has stopped testing anything";
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
}
