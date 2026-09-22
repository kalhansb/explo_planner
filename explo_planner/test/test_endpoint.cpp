// T5. Contract tests for THE ENDPOINT — the three events that decide when a
// robot's run is over.
//
// It used to also carry the arithmetic rule that decided when a robot left a
// task to keep an appointment. That rule is deleted (generation 19) and so is
// the group of tests that pinned it; see the GROUP A marker below for what they
// asserted and which of it still matters.
//
// WHY THIS FILE EXISTS, AND WHY IT TESTS WHAT IT DOES. The primary metric of
// every campaign in this repo is a stamp on one of these rows. Two separate
// defects have already been found in them AFTER a 240-cell campaign had been
// analysed:
//
//   * ts1b shipped 16 robot-runs carrying TWO `mission_complete` rows, from a
//     DONE -> RETURN_HOME re-entry. Every homing metric in those runs was
//     double-counted, and finding it required auditing 240 cells by hand
//     precisely because the duplicate rows were indistinguishable from each
//     other. R1b added a re-entry guard and T5 added
//     `MissionCompleteEvent::occurrence` so the NEXT one would be a grep —
//     but R1b's guard tested `state_ == RETURN_HOME`, and the re-entry that
//     was measured arrives from DONE, so it could not fire on the very case
//     it was written for. Generation 9 (F1) adds the guard that can:
//     `mission_return_done_`, latched when the return RESOLVES rather than
//     while the leg is in flight, plus an idempotence latch in the writer.
//
//   * A refused appointment re-arm left the node with `appointment_armed_ ==
//     true` and `t_meet_ms == -1`, which made the departure test false forever
//     AND suppressed the mid-run trigger — reconnection dead for the rest of the
//     run, with nothing in the log saying so (see the note at
//     explo_planner_node.cpp's appointment re-arm). The refusal is correct; what
//     was missing was a test asserting that the -1 refuses even when the
//     appointment reads as long overdue, which is the exact state that arose.
//
//     THE HAZARD OUTLIVED ITS TEST. The departure test is gone and the tests
//     that pinned it went with it, but appointmentDue() is now a bare
//     `now >= t_meet` and a -1 t_meet on an armed slot reads as due FOREVER —
//     the same silent death by the same door. What stands in for the deleted
//     assertion is armAppointment clearing appointment_armed_ on every refusal
//     path, which is a node-level invariant this suite cannot reach.
//
// WHAT THIS FILE CANNOT TEST, STATED SO ITS ABSENCE IS NOT MISREAD AS COVERAGE.
// The endpoint's STATE MACHINE lives in explo_planner_node.cpp, which is not in
// explo_planner_lib's source list — an explicit list of files in CMakeLists.txt,
// not a glob — so there is no node-level test of "DONE is entered once" here
// or anywhere. `reconnect_terminal_`, the three departure sites, and the
// RETURN_HOME re-entry guard are all verified by reading, not by this suite.
// That is the reason `occurrence` exists: the thing the tests cannot reach has
// to leave evidence in the artifact instead.
//
// And none of it is a self-reporting PASS. A field that a guard prevents from
// ever reaching 2 would be another check that stopped checking, so: the node
// increments `mission_complete_count_` unconditionally at the call site; the
// writer writes whatever it is handed (`OccurrenceIsWrittenNotAssumed` hands it
// a 3 on a single emission, so a writer that hardcoded 1 would fail); the
// writer's own idempotence latch COUNTS what it suppresses instead of silently
// dropping it; and both counters — the node's refused re-entries and the
// writer's suppressed rows — ride out on `run_end` and are written even when
// they are zero (`DuplicateEndpointCountersAreWrittenEvenAtZero`), so a zero is
// a reading rather than an absent field. The offline gate decides.
//
// Generation 8 made the opposite call here — it let the duplicate ROW through
// so the duplicate would be visible — and that is why 16 robot-runs reached
// analysis with two endpoint rows and their cells were dropped whole. Keeping
// the evidence and keeping the defect are separable, and this is the split.

#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "explo_planner/experiment_log.hpp"
#include "explo_planner/rendezvous_scheduler.hpp"

using namespace explo_planner;

namespace {

// ---------------------------------------------------------------------------
// Group B helpers — the endpoint rows.
// ---------------------------------------------------------------------------

/// Per-test, per-process path, removed on destruction. Same rationale as
/// test_experiment_log.cpp's: `colcon test` runs these binaries concurrently
/// and a shared /tmp name would let one test truncate another's file
/// mid-write, which is the worst way for a contract test to fail.
struct TempLogPath {
  explicit TempLogPath(const char* stem)
      : path(std::string("/tmp/explo_endpoint_") + stem + "_" +
             std::to_string(static_cast<long>(::getpid())) + ".jsonl") {}
  ~TempLogPath() { std::remove(path.c_str()); }
  std::string path;
};

/// Every line of the file, in write order.
///
/// Returns RAW LINES rather than a parsed structure on purpose: two of the
/// defects below are about a field being absent or present-but-wrong, and both
/// survive a lenient parser. The analysis scripts see bytes, so the assertions
/// are made on bytes.
std::vector<std::string> readLines(const std::string& path) {
  std::vector<std::string> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty()) out.push_back(line);
  }
  return out;
}

/// Index of the first line whose `"event"` is `kind`, or -1.
int indexOfEvent(const std::vector<std::string>& lines, const char* kind) {
  const std::string needle = std::string("\"event\":\"") + kind + "\"";
  for (size_t i = 0; i < lines.size(); ++i) {
    if (lines[i].find(needle) != std::string::npos) return static_cast<int>(i);
  }
  return -1;
}

int countEvent(const std::vector<std::string>& lines, const char* kind) {
  const std::string needle = std::string("\"event\":\"") + kind + "\"";
  int n = 0;
  for (const auto& l : lines) {
    if (l.find(needle) != std::string::npos) ++n;
  }
  return n;
}

ExperimentContext ctxAt(double sim_sec, const char* state, int step) {
  ExperimentContext ctx;
  ctx.sim_time_sec = sim_sec;
  ctx.state = state;
  ctx.step = step;
  return ctx;
}

ExplorationCompleteEvent exploreDone(int occurrence) {
  ExplorationCompleteEvent e;
  e.reason = "coverage-saturated";
  e.unknown_fraction = 0.61;
  e.coverage_source = "scovox";
  e.steps = 42;
  e.distance_m = 311.5;
  e.occurrence = occurrence;
  return e;
}

MissionCompleteEvent missionDone(const char* result, const char* reason,
                                 int occurrence) {
  MissionCompleteEvent e;
  e.result = result;
  e.reason = reason;
  e.home_x = 1.0; e.home_y = 2.0;
  e.final_x = 1.1; e.final_y = 2.2;
  e.dist_to_home_m = 0.223607;
  e.homing_duration_sec = 88.0;
  e.homing_distance_m = 97.25;
  e.latched = true;
  e.occurrence = occurrence;
  return e;
}

/// The body of a function definition, by brace matching from its signature.
///
/// Used only by the node-source scans below. Returns "" when the signature is
/// not found, which the callers ASSERT on: a scan that silently matches nothing
/// passes every assertion made about what it did not find, and that is the
/// failure mode these tests exist to avoid in the first place.
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

/// Drop comments before scanning, so an assertion is about CODE.
///
/// The node's comments quote the very identifiers these scans look for —
/// several of the paragraphs around the endpoint explain the defect by naming
/// the call that caused it — so an unstripped scan can be satisfied by prose
/// describing a bug that is still present, and a reverted node would pass. The
/// quote tracking is not decoration: dropping from `//` unconditionally would
/// truncate any line holding a string literal containing `//` and silently
/// remove real code from the scan's view, which is the same failure one layer
/// down. Copied deliberately rather than shared: these scan helpers are
/// per-file by convention in this suite, and a header would make one file's
/// tightening everyone else's surprise.
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

std::string nodeSource() { return stripComments(readFile(EXPLO_PLANNER_NODE_CPP)); }

RunEndEvent runEnd() {
  RunEndEvent e;
  e.reason = "coverage-done";
  e.steps = 42;
  e.distance_m = 408.75;
  e.unknown_fraction = 0.61;
  e.coverage_source = "scovox";
  return e;
}

}  // namespace

// ===========================================================================
// GROUP A IS GONE (generation 19).
//
// Nine tests stood here pinning the edge arithmetic of
// RendezvousScheduler::shouldDepart — the `<=` boundary, zero travel as a real
// estimate rather than a refusal, t_meet == 0 as a valid appointment, negative
// safety and negative margin clamping instead of inverting, zero safety leaving
// the margin as the whole lead time, both -1 sentinels refusing even when long
// overdue, and the truncation direction.
//
// Every one of them was a correct statement about a function that no longer
// exists. THE IDEA CAME BACK IN GENERATION 29 AND THE FUNCTION DID NOT: a robot
// signs up to the first agreed occurrence it can still ARRIVE at, then leaves
// when `now + appointmentLeadMs >= t_meet_ms`. That is a live lead against an
// instant the team agreed, not a privately computed deadline, so there is still
// no `remaining <= need` comparison for these tests to guard.
//
// THE SENTINEL CASE IS THE ONE WORTH RE-READING BEFORE ANY FUTURE EDIT. It
// pinned that an un-armed slot (t_meet_ms == -1) must not read as "long
// overdue" and send a robot to cell -1. appointmentDue() still inherits exactly
// that hazard, and the lead makes it strictly worse rather than better: a -1
// t_meet on an armed slot is overdue by the bare comparison AND by the
// lead-adjusted one.
//
// WHAT PROTECTS IT IS CO-LOCATION, AND NOTHING ELSE. There is exactly one
// `appointment_armed_ = true` in the node, and it sits in the same branch that
// assigned `t_meet_ms` unconditionally from
// `nextAgreedOccurrence(agreed.t_meet_ms, agreed.interval_ms, <a floor at or
// after t_now>)`, under `rendezvous_agreed_.valid()` — so an armed slot cannot
// carry the sentinel without someone separating those two writes. Keep them
// together. The floor gained a per-robot arrival shortfall in generation 29;
// the invariant is indifferent to what the floor is, only to the two writes
// staying in one branch.
//
// Two things that LOOK like the guard and are not, because a future edit will
// reach for them first:
//
//   * armAppointment clearing the flag on its refusal path. Real, but it fires
//     where the flag is already false — its own comment says so — so it
//     restores an invariant rather than establishing one. It protects the
//     PLAN/FLAG pair against a future edit to a distant guard; it does not
//     protect t_meet.
//   * the `appointment_.valid()` conjunct in the arming condition.
//     RendezvousPlan::valid() is `cell >= 0 && refused.empty()` and does not
//     look at t_meet_ms at all.
// ===========================================================================

// ===========================================================================
// GROUP B. THE ENDPOINT ROWS.
// ===========================================================================

/// The endpoint trio must appear in causal order in the FILE, because every
/// analysis script reads the file forward and several take "the last X before
/// Y". Homing that resolves before exploration completes, or a run_end before
/// the mission resolved, is a defect that changes which stamp is the metric.
TEST(EndpointOrdering, ExplorationThenMissionThenRunEnd) {
  TempLogPath tmp("order");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open()) << "could not open " << tmp.path;
    log.startRun(ctxAt(100.0, "PLAN", 0), {});
    log.logExplorationComplete(ctxAt(900.0, "PLAN", 42), exploreDone(1));
    log.logMissionComplete(ctxAt(988.0, "RETURN_HOME", 42),
                           missionDone("arrived", "coverage-latched", 1));
    log.logRunEnd(ctxAt(989.0, "DONE", 42), runEnd());
  }
  const auto lines = readLines(tmp.path);
  const int i_start = indexOfEvent(lines, "run_start");
  const int i_exp   = indexOfEvent(lines, "exploration_complete");
  const int i_mis   = indexOfEvent(lines, "mission_complete");
  const int i_end   = indexOfEvent(lines, "run_end");
  ASSERT_GE(i_start, 0); ASSERT_GE(i_exp, 0);
  ASSERT_GE(i_mis, 0);   ASSERT_GE(i_end, 0);
  EXPECT_LT(i_start, i_exp);
  EXPECT_LT(i_exp, i_mis);
  EXPECT_LT(i_mis, i_end);
  // And exactly one of each: this is the shape a healthy run has, and it is the
  // baseline the duplicate test below is measured against.
  EXPECT_EQ(countEvent(lines, "exploration_complete"), 1);
  EXPECT_EQ(countEvent(lines, "mission_complete"), 1);
  EXPECT_EQ(countEvent(lines, "run_end"), 1);
}

/// `result` and `reason` answer different questions and must not collapse into
/// each other: `reason` is WHY the robot went home (the terminal exploration
/// ending) and `result` is HOW the homing leg resolved. A run whose homing
/// timed out after a coverage latch reads result=timeout, reason=coverage-
/// latched, and conflating them would make every timeout look like a
/// step-budget ending.
TEST(MissionCompleteRow, ResultAndReasonAreDistinctFields) {
  TempLogPath tmp("result_reason");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), {});
    log.logMissionComplete(ctxAt(500.0, "RETURN_HOME", 9),
                           missionDone("timeout", "coverage-latched", 1));
  }
  const auto lines = readLines(tmp.path);
  const int i = indexOfEvent(lines, "mission_complete");
  ASSERT_GE(i, 0);
  const std::string& row = lines[static_cast<size_t>(i)];
  EXPECT_NE(row.find("\"result\":\"timeout\""), std::string::npos) << row;
  EXPECT_NE(row.find("\"reason\":\"coverage-latched\""), std::string::npos)
      << row;
  EXPECT_NE(row.find("\"latched\":true"), std::string::npos) << row;
  EXPECT_NE(row.find("\"occurrence\":1"), std::string::npos) << row;
}

/// THE DUPLICATE, BOTH HALVES: refused in the file, recorded in the run.
///
/// This replaces generation 8's `OccurrenceCountsPastOne`, which asserted the
/// opposite contract — two emissions, two rows — on the reasoning that a latch
/// here would erase the only offline evidence a duplicate had happened. The
/// reasoning was sound and the conclusion was not: it left the artifact with
/// two endpoint rows and every consumer silently choosing one, which is exactly
/// how 16 ts1b robot-runs were double-counted and how `event_log.py` then
/// dropped their cells whole — the slow tail of the treatment arm.
///
/// The file now holds one mission_complete the way it holds one run_end. The
/// evidence is not lost, it MOVED: to `dupMissionCompletes()` in-process, to
/// `mission_completes_suppressed` on the run_end row, and to a WARN on the
/// first suppression. All three are asserted here, because a latch whose only
/// witness is a log line a human might read is a latch that can go inert.
TEST(MissionCompleteRow, DuplicateIsSuppressedAndCounted) {
  TempLogPath tmp("dupe");
  long long dupes = -1;
  long long dropped = -1;
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), {});
    log.logMissionComplete(ctxAt(500.0, "RETURN_HOME", 9),
                           missionDone("arrived", "barrier-gave-up", 1));
    EXPECT_EQ(log.dupMissionCompletes(), 0)
        << "the first mission_complete is not a duplicate";
    // The ts1b shape exactly: the second arrives 140 s later carrying a
    // DIFFERENT result and reason, which is what made the two rows impossible
    // to reconcile offline.
    log.logMissionComplete(ctxAt(640.0, "RETURN_HOME", 9),
                           missionDone("timeout", "coverage-latched", 2));
    log.logMissionComplete(ctxAt(700.0, "RETURN_HOME", 9),
                           missionDone("budget", "coverage-latched", 3));
    dupes = log.dupMissionCompletes();
    dropped = log.droppedBeforeStart();
    log.logRunEnd(ctxAt(900.0, "DONE", 9), runEnd());
  }
  EXPECT_EQ(dupes, 2);
  // A suppressed duplicate is not a pre-start drop. Two counters, two
  // questions; collapsing them would make either one unreadable.
  EXPECT_EQ(dropped, 0);

  const auto lines = readLines(tmp.path);
  ASSERT_EQ(countEvent(lines, "mission_complete"), 1);
  const int i = indexOfEvent(lines, "mission_complete");
  ASSERT_GE(i, 0);
  const std::string& row = lines[static_cast<size_t>(i)];
  // The row that survives is the FIRST — the leg the robot actually flew.
  EXPECT_NE(row.find("\"result\":\"arrived\""), std::string::npos) << row;
  EXPECT_NE(row.find("\"reason\":\"barrier-gave-up\""), std::string::npos) << row;
  EXPECT_NE(row.find("\"occurrence\":1"), std::string::npos) << row;
  EXPECT_EQ(row.find("coverage-latched"), std::string::npos)
      << "the later attempt's reason leaked into the kept row: " << row;

  // And the suppression is in the artifact, not only in the process.
  const int r = indexOfEvent(lines, "run_end");
  ASSERT_GE(r, 0);
  EXPECT_NE(lines[static_cast<size_t>(r)].find(
                "\"mission_completes_suppressed\":2"),
            std::string::npos)
      << "the run_end row does not carry the suppression count, so the latch "
         "is invisible to every offline reader: "
      << lines[static_cast<size_t>(r)];
}

/// The hardcoding check the old duplicate-row test also served, kept alive
/// without the duplicate row. ONE emission, carrying occurrence 3 — a writer
/// that emitted a literal 1, or that derived the number from its own row count,
/// fails here. The node's counter is the authority on this field and the writer
/// is a transcriber.
TEST(MissionCompleteRow, OccurrenceIsWrittenNotAssumed) {
  TempLogPath tmp("occ_verbatim");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), {});
    log.logMissionComplete(ctxAt(500.0, "RETURN_HOME", 9),
                           missionDone("arrived", "coverage-latched", 3));
  }
  const auto lines = readLines(tmp.path);
  const int i = indexOfEvent(lines, "mission_complete");
  ASSERT_GE(i, 0);
  EXPECT_NE(lines[static_cast<size_t>(i)].find("\"occurrence\":3"),
            std::string::npos)
      << lines[static_cast<size_t>(i)];
}

/// The opposite contract on the same-shaped field: a second
/// `exploration_complete` is LEGITIMATE (a merged map can deliver new frontiers
/// after a reconnect), so the pair of completion stamps has to keep both ends.
/// `explore_done_first_sim_sec` latches the first declaration while
/// `explore_done_sim_sec` advances to the latest — they answer "when did this
/// robot finish its own map" and "when did it stop trying", and a run is not
/// entitled to only one of them.
TEST(ExplorationCompleteRow, SecondDeclarationAdvancesLastAndLatchesFirst) {
  TempLogPath tmp("explore_twice");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), {});
    log.logExplorationComplete(ctxAt(900.0, "PLAN", 42), exploreDone(1));
    log.logExplorationComplete(ctxAt(1'250.0, "PLAN", 71), exploreDone(2));
    log.logRunEnd(ctxAt(1'251.0, "DONE", 71), runEnd());
  }
  const auto lines = readLines(tmp.path);
  ASSERT_EQ(countEvent(lines, "exploration_complete"), 2);

  // Second row: last advanced to 1250, first stayed at 900.
  int i_last = -1;
  for (size_t i = 0; i < lines.size(); ++i) {
    if (lines[i].find("\"event\":\"exploration_complete\"") !=
        std::string::npos) {
      i_last = static_cast<int>(i);
    }
  }
  ASSERT_GE(i_last, 0);
  const std::string& second = lines[static_cast<size_t>(i_last)];
  EXPECT_NE(second.find("\"explore_done_sim_sec\":1250.000000"),
            std::string::npos) << second;
  EXPECT_NE(second.find("\"explore_done_first_sim_sec\":900.000000"),
            std::string::npos) << second;
  EXPECT_NE(second.find("\"occurrence\":2"), std::string::npos) << second;

  // And run_end reports the resume delta rather than folding it into either
  // stamp: 1250 - 900 == 350.
  const int i_end = indexOfEvent(lines, "run_end");
  ASSERT_GE(i_end, 0);
  const std::string& end = lines[static_cast<size_t>(i_end)];
  EXPECT_NE(end.find("\"explore_done_resume_delta_sec\":350.000000"),
            std::string::npos) << end;
}

// ---------------------------------------------------------------------------
// Group C — `post_latch`, the coverage-milestone attribution flag.
//
// C1 added `post_latch`/`sec_since_explore_done` to `coverage_milestone` to
// answer one question: was this rung crossed AFTER the robot stopped
// exploring, i.e. is it post-stop map merging rather than exploration? A rung
// crossed post-declaration is a selection artifact, because WHETHER a run
// reaches such a rung at all correlates with the arm.
//
// Through generation 8 the flag was keyed on `explore_done_first_sec_`, a
// one-way latch, and that inverted its purpose. A robot that declares, is
// pulled into a reconnect manoeuvre, comes back with a merged map and explores
// on was "post-latch" for the rest of the run — so every late rung of genuine
// resumed exploration was labelled post-stop merging. Manoeuvres happen only
// in the reconnecting arms, so a flag added to detect an arm-correlated
// artifact was itself arm-correlated, in the direction that would have thrown
// away the treated arms' real coverage.
//
// Generation 9 keys it on the LAST declaration plus a resume flag that
// `logStep` clears. These four tests pin both halves, and they are a probe
// calibration as a set: the flag is observed BOTH true (2, 4) and false (1, 3)
// on the same binary, so neither reading can be the flag being hardcoded,
// absent, or wired to the wrong member. The interval is asserted by value, not
// merely by sign, because -1 is the "not applicable" sentinel and a measured
// interval that happened to be negative would be indistinguishable from it.
// ---------------------------------------------------------------------------

namespace {

/// A four-rung ladder. Every case below crosses exactly one rung per
/// `noteCoverage` call, so each `coverage_milestone` row in these tests is
/// attributable to one call at one stamp.
const std::vector<double>& ladder4() {
  static const std::vector<double> l{0.8, 0.7, 0.6, 0.5};
  return l;
}

StepEvent stepAt(double unknown_fraction) {
  StepEvent e;
  e.unknown_fraction = unknown_fraction;
  e.coverage_source = "scovox";
  e.distance_m = 10.0;
  return e;
}

/// The last `coverage_milestone` line, or "" — ASSERTed by every caller, since
/// a scan that matched nothing would pass every assertion made about what it
/// does not contain.
std::string lastMilestone(const std::vector<std::string>& lines) {
  std::string out;
  for (const auto& l : lines) {
    if (l.find("\"event\":\"coverage_milestone\"") != std::string::npos) {
      out = l;
    }
  }
  return out;
}

}  // namespace

/// Nothing declared yet: the flag is false and the interval is the sentinel,
/// NOT a measured zero and not an interval counted from `run_start`.
TEST(CoverageMilestonePostLatch, BeforeAnyDeclarationTheFlagIsFalse) {
  TempLogPath tmp("postlatch_before");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), ladder4());
    log.noteCoverage(ctxAt(200.0, "PLAN", 5), 0.75, "scovox", 12.0, 1.0, 2.0);
    log.logRunEnd(ctxAt(300.0, "DONE", 5), runEnd());
  }
  const auto lines = readLines(tmp.path);
  ASSERT_EQ(countEvent(lines, "coverage_milestone"), 1);
  const std::string row = lastMilestone(lines);
  ASSERT_FALSE(row.empty());
  EXPECT_NE(row.find("\"post_latch\":false"), std::string::npos) << row;
  EXPECT_NE(row.find("\"sec_since_explore_done\":-1.000000"),
            std::string::npos)
      << "'never declared' must write the sentinel, not 0 and not the time "
         "since run_start: " << row;
}

/// Declared and never resumed: the flag is true and the interval is measured
/// from the declaration. This is the case the field was added for.
TEST(CoverageMilestonePostLatch, AfterADeclarationWithNoResumeTheFlagIsTrue) {
  TempLogPath tmp("postlatch_after");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), ladder4());
    log.logExplorationComplete(ctxAt(900.0, "PLAN", 42), exploreDone(1));
    log.noteCoverage(ctxAt(1'000.0, "RETURN_HOME", 42), 0.75, "scovox", 300.0,
                     1.0, 2.0);
    log.logRunEnd(ctxAt(1'100.0, "DONE", 42), runEnd());
  }
  const auto lines = readLines(tmp.path);
  ASSERT_EQ(countEvent(lines, "coverage_milestone"), 1);
  const std::string row = lastMilestone(lines);
  ASSERT_FALSE(row.empty());
  EXPECT_NE(row.find("\"post_latch\":true"), std::string::npos) << row;
  EXPECT_NE(row.find("\"sec_since_explore_done\":100.000000"),
            std::string::npos)
      << "the interval runs from the declaration (900), not from run_start "
         "(100): " << row;
}

/// THE GENERATION-8 DEFECT, as a test. A `step` after the declaration means
/// exploration resumed — the step counter is frozen for the whole of a
/// manoeuvre and the whole homing leg, so a step event can only arrive when
/// the robot is planning again — and every rung crossed from then on is
/// exploration, not post-stop merging.
///
/// Under generation 8 this row read `post_latch:true` with an interval of 100,
/// which is exactly the mislabelling that would have discarded the
/// reconnecting arms' resumed coverage.
TEST(CoverageMilestonePostLatch, AStepAfterTheDeclarationClearsTheFlag) {
  TempLogPath tmp("postlatch_resume");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), ladder4());
    log.logExplorationComplete(ctxAt(900.0, "PLAN", 42), exploreDone(1));
    log.logStep(ctxAt(950.0, "PLAN", 43), stepAt(0.76));
    log.noteCoverage(ctxAt(1'000.0, "PLAN", 43), 0.75, "scovox", 300.0, 1.0,
                     2.0);
    log.logRunEnd(ctxAt(1'100.0, "DONE", 43), runEnd());
  }
  const auto lines = readLines(tmp.path);
  ASSERT_EQ(countEvent(lines, "coverage_milestone"), 1);
  const std::string row = lastMilestone(lines);
  ASSERT_FALSE(row.empty());
  EXPECT_NE(row.find("\"post_latch\":false"), std::string::npos)
      << "a step after the declaration means the robot is exploring again; "
         "this rung is coverage, not merging: " << row;
  EXPECT_NE(row.find("\"sec_since_explore_done\":-1.000000"),
            std::string::npos)
      << "a cleared flag must write the sentinel, not the interval it would "
         "have written: " << row;
}

/// Declare, resume, declare again. The second declaration re-arms the flag,
/// and the interval is measured from the LAST declaration — 50 s, not the 350
/// s a first-declaration latch would report. Both halves matter: the re-arm is
/// what keeps the flag from being one-shot, and `explore_done_last_sec_` is
/// what keeps the interval honest.
TEST(CoverageMilestonePostLatch, ASecondDeclarationReArmsFromTheLastStamp) {
  TempLogPath tmp("postlatch_rearm");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), ladder4());
    log.logExplorationComplete(ctxAt(900.0, "PLAN", 42), exploreDone(1));
    log.logStep(ctxAt(950.0, "PLAN", 43), stepAt(0.76));
    // Rung 0 (0.8) falls here, while the robot is exploring again.
    log.noteCoverage(ctxAt(1'000.0, "PLAN", 43), 0.75, "scovox", 300.0, 1.0,
                     2.0);
    log.logExplorationComplete(ctxAt(1'200.0, "PLAN", 71), exploreDone(2));
    // Rung 1 (0.7) falls here, after the second declaration and with no step
    // in between.
    log.noteCoverage(ctxAt(1'250.0, "RETURN_HOME", 71), 0.65, "scovox", 400.0,
                     3.0, 4.0);
    log.logRunEnd(ctxAt(1'300.0, "DONE", 71), runEnd());
  }
  const auto lines = readLines(tmp.path);
  ASSERT_EQ(countEvent(lines, "coverage_milestone"), 2);
  const std::string row = lastMilestone(lines);
  ASSERT_FALSE(row.empty());
  EXPECT_NE(row.find("\"rung\":1"), std::string::npos)
      << "the assertions below are about the SECOND rung: " << row;
  EXPECT_NE(row.find("\"post_latch\":true"), std::string::npos)
      << "the second declaration must re-arm a flag that a step had cleared: "
      << row;
  EXPECT_NE(row.find("\"sec_since_explore_done\":50.000000"),
            std::string::npos)
      << "the interval runs from the LAST declaration (1200), not the first "
         "(900), which would read 350: " << row;
}

/// A CENSORED run — the duration cap killed it before it ever declared
/// exhaustion — must write `explore_done_sim_sec` as NULL on run_end, not as a
/// number. This is the single most dangerous field in the schema: a reader that
/// takes "max sim time in the file" as the completion time converts every
/// censored run into a fast one, biasing the primary metric in the direction
/// that flatters whichever arm times out most.
TEST(RunEndRow, CensoredRunWritesNullCompletionNotAStamp) {
  TempLogPath tmp("censored");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), {});
    RunEndEvent e = runEnd();
    e.reason = "duration-cap";
    log.logRunEnd(ctxAt(1'900.0, "NAVIGATE", 210), e);
  }
  const auto lines = readLines(tmp.path);
  const int i = indexOfEvent(lines, "run_end");
  ASSERT_GE(i, 0);
  const std::string& row = lines[static_cast<size_t>(i)];
  EXPECT_NE(row.find("\"explore_done_sim_sec\":null"), std::string::npos)
      << row;
  EXPECT_NE(row.find("\"explore_done_first_sim_sec\":null"), std::string::npos)
      << row;
  EXPECT_NE(row.find("\"explore_done_resume_delta_sec\":null"),
            std::string::npos) << row;
  // log_span_sim_sec is a property of when the PROCESS stopped and is always a
  // number — which is precisely why it must never be read as a completion, and
  // why it is named so it cannot be.
  EXPECT_NE(row.find("\"log_span_sim_sec\":1800.000000"), std::string::npos)
      << row;
}

/// Endpoints emitted before `run_start` are DROPPED AND COUNTED, never
/// buffered. The node calls startRun on its first tick with a live clock, so a
/// run whose clock never comes up, or that ends inside that first tick, loses
/// its endpoints — and `events_dropped_before_start` on run_end is the only
/// evidence that anything was lost. A zero there must mean "nothing was
/// dropped", so this asserts the counter actually counts.
TEST(EndpointOrdering, EventsBeforeRunStartAreDroppedAndCounted) {
  TempLogPath tmp("before_start");
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    EXPECT_FALSE(log.started());
    log.logExplorationComplete(ctxAt(50.0, "PLAN", 1), exploreDone(1));
    log.logMissionComplete(ctxAt(60.0, "RETURN_HOME", 1),
                           missionDone("arrived", "coverage-latched", 1));
    log.logRunEnd(ctxAt(70.0, "DONE", 1), runEnd());
    // Now the clock is live.
    log.startRun(ctxAt(100.0, "PLAN", 0), {});
    log.logRunEnd(ctxAt(900.0, "DONE", 42), runEnd());
  }
  const auto lines = readLines(tmp.path);
  EXPECT_EQ(countEvent(lines, "exploration_complete"), 0);
  EXPECT_EQ(countEvent(lines, "mission_complete"), 0);
  ASSERT_EQ(countEvent(lines, "run_end"), 1) << "the pre-start run_end must "
                                                "have been dropped too";
  const int i = indexOfEvent(lines, "run_end");
  ASSERT_GE(i, 0);
  // 3, counting the dropped run_end itself. It is counted here because it used
  // not to be: logRunEnd's guard was a single collapsed `if` that returned
  // without touching the counter, so this one event kind escaped the logger's
  // own accounting — in exactly the scenario where the accounting is the only
  // surviving evidence.
  EXPECT_NE(lines[static_cast<size_t>(i)].find(
                "\"events_dropped_before_start\":3"),
            std::string::npos)
      << lines[static_cast<size_t>(i)];
}

/// A SECOND run_end is suppressed — the file must keep exactly one last line,
/// because every "read the tail" consumer would otherwise get a coin flip — but
/// the suppression is COUNTED and warned, not silent. A guard that absorbs a
/// duplicate endpoint indefinitely with nothing recorded is the same shape as
/// the 16 ts1b runs whose double mission_complete rows took a 240-cell audit to
/// find.
TEST(RunEndRow, DuplicateRunEndIsSuppressedAndCounted) {
  TempLogPath tmp("dup_run_end");
  long long dupes = -1;
  long long dropped = -1;
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_endpoint"));
    ASSERT_TRUE(log.open());
    log.startRun(ctxAt(100.0, "PLAN", 0), {});
    log.logRunEnd(ctxAt(900.0, "DONE", 42), runEnd());
    EXPECT_EQ(log.dupRunEnds(), 0) << "the first run_end is not a duplicate";
    RunEndEvent second = runEnd();
    second.reason = "done-re-entry";
    log.logRunEnd(ctxAt(961.0, "DONE", 42), second);
    log.logRunEnd(ctxAt(1'002.0, "DONE", 42), second);
    dupes = log.dupRunEnds();
    dropped = log.droppedBeforeStart();
  }
  EXPECT_EQ(dupes, 2);
  // A suppressed duplicate is NOT a pre-start drop. The two counters answer
  // different questions and collapsing them would make either one unreadable.
  EXPECT_EQ(dropped, 0);
  const auto lines = readLines(tmp.path);
  EXPECT_EQ(countEvent(lines, "run_end"), 1);
  // The row that survived is the FIRST one, stamp and reason both.
  const int i = indexOfEvent(lines, "run_end");
  ASSERT_GE(i, 0);
  EXPECT_NE(lines[static_cast<size_t>(i)].find("\"reason\":\"coverage-done\""),
            std::string::npos) << lines[static_cast<size_t>(i)];
  EXPECT_EQ(lines[static_cast<size_t>(i)].find("done-re-entry"),
            std::string::npos) << lines[static_cast<size_t>(i)];
}

/// PROBE CALIBRATION for the two duplicate-endpoint counters.
///
/// Both are meant to read 0 for the rest of this project's life, and a field
/// that is only ever observed at 0 is indistinguishable from a field that is
/// absent, hardcoded, or wired to the wrong member. So: one run with nothing
/// wrong must WRITE both keys at 0, and one run handed non-zero values must
/// write those values back verbatim. Without the second half, "0 re-entries
/// across the campaign" would be a claim about the gate rather than about the
/// runs.
///
/// `mission_return_reentries` is the NODE's counter (startReturnHome refusing a
/// second homing leg) and can only arrive through RunEndEvent, so it is handed
/// in. `mission_completes_suppressed` is the WRITER's own and is read off the
/// latch — which is why it is driven here by actually emitting a duplicate
/// rather than by setting a field.
TEST(RunEndRow, DuplicateEndpointCountersAreWrittenEvenAtZero) {
  {
    TempLogPath tmp("dup_counters_zero");
    {
      ExperimentLog log(tmp.path, "testbot",
                        rclcpp::get_logger("test_endpoint"));
      ASSERT_TRUE(log.open());
      log.startRun(ctxAt(100.0, "PLAN", 0), {});
      log.logMissionComplete(ctxAt(500.0, "RETURN_HOME", 9),
                             missionDone("arrived", "coverage-latched", 1));
      log.logRunEnd(ctxAt(900.0, "DONE", 42), runEnd());
    }
    const auto lines = readLines(tmp.path);
    const int r = indexOfEvent(lines, "run_end");
    ASSERT_GE(r, 0);
    const std::string& row = lines[static_cast<size_t>(r)];
    EXPECT_NE(row.find("\"mission_return_reentries\":0"), std::string::npos)
        << row;
    EXPECT_NE(row.find("\"mission_completes_suppressed\":0"), std::string::npos)
        << row;
  }
  {
    TempLogPath tmp("dup_counters_nonzero");
    {
      ExperimentLog log(tmp.path, "testbot",
                        rclcpp::get_logger("test_endpoint"));
      ASSERT_TRUE(log.open());
      log.startRun(ctxAt(100.0, "PLAN", 0), {});
      log.logMissionComplete(ctxAt(500.0, "RETURN_HOME", 9),
                             missionDone("arrived", "barrier-gave-up", 1));
      log.logMissionComplete(ctxAt(640.0, "RETURN_HOME", 9),
                             missionDone("arrived", "coverage-latched", 2));
      RunEndEvent e = runEnd();
      // 4, not 1: a count that happened to equal the suppressed-row count
      // could be either member read twice.
      e.mission_return_reentries = 4;
      log.logRunEnd(ctxAt(900.0, "DONE", 42), e);
    }
    const auto lines = readLines(tmp.path);
    const int r = indexOfEvent(lines, "run_end");
    ASSERT_GE(r, 0);
    const std::string& row = lines[static_cast<size_t>(r)];
    EXPECT_NE(row.find("\"mission_return_reentries\":4"), std::string::npos)
        << row;
    EXPECT_NE(row.find("\"mission_completes_suppressed\":1"), std::string::npos)
        << row;
  }
}

// ===========================================================================
// GROUP C. THE ONCE-PER-RUN MISSION RETURN, read from the node's source.
//
// Everything above this line tests the WRITER. The guard that actually stops
// the ts1b defect is in the node — startReturnHome refusing a second homing leg
// — and `explo_planner_node.cpp` is not in explo_planner_lib's source list (an
// explicit file list, not a glob), so nothing links it and no test can call it.
// The choice is therefore between a source scan and no coverage at all, and the
// file header's promise that the node-level guard is "verified by reading, not
// by this suite" is precisely how R1b shipped a guard that tested the wrong
// state for a whole generation without anything noticing.
//
// So these read the source, exactly as test_experiment_log's
// DeclaredKindsMatchTheWriter does, and they are honest about their limits:
// they can show the latch is present and ORDERED correctly, and they cannot
// show it is reached. What they catch is the realistic regression — a later
// edit moving, renaming, or deleting it — and they fail loudly if the scan
// itself stops matching, which is the way a check of this kind usually dies.
// ===========================================================================

/// The latch must be READ before any of startReturnHome's resets run. Every
/// assignment in that function is a reset: a guard placed after even one of
/// them refills the escape budget and zeroes the approach window on a request
/// it then refuses, which is worse than no guard, because the damage happens
/// and the log says the request was denied.
TEST(MissionReturnGuard, TheLatchIsReadBeforeAnyReset) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::startReturnHome(");
  ASSERT_FALSE(body.empty())
      << "the scan found no startReturnHome definition — it has been renamed "
         "or reshaped and this test has stopped testing anything";

  const size_t guard = body.find("if (mission_return_done_)");
  ASSERT_NE(guard, std::string::npos)
      << "startReturnHome has no once-per-run guard; the DONE -> RETURN_HOME "
         "re-entry that produced 16 doubled ts1b endpoints is back";

  // The counter has to be inside the guard, not somewhere later: a refusal
  // that is not counted leaves run_end.mission_return_reentries reading 0 in
  // a run where the guard fired, i.e. a check that reports its own failure as
  // a pass.
  const size_t counted = body.find("++mission_return_reentries_");
  ASSERT_NE(counted, std::string::npos)
      << "the guard refuses silently — nothing offline can tell a run where "
         "it fired from a run where it never had to";
  EXPECT_GT(counted, guard) << "the re-entry counter is incremented outside "
                               "the guard it is meant to count";

  // Three resets, spread across the function, each independently damaging.
  for (const char* reset : {"return_home_dist_at_start_ =",
                            "home_escapes_used_",
                            "home_mode_"}) {
    const size_t at = body.find(reset);
    ASSERT_NE(at, std::string::npos)
        << "the reset '" << reset << "' is gone from startReturnHome, so this "
           "ordering assertion no longer constrains anything";
    EXPECT_LT(guard, at)
        << "'" << reset << "' runs before the once-per-run guard: a refused "
           "re-entry still damages the homing leg already in progress";
  }

  // And the leg-scoped guard is still there beside it. They answer different
  // questions (resolved vs in flight) and neither subsumes the other.
  EXPECT_NE(body.find("if (state_ == State::RETURN_HOME)"), std::string::npos)
      << "the in-flight re-entry guard has been dropped";
}

/// The latch must be SET before the run can end, and unconditionally — not
/// inside the `if (exp_log_)` block. The damage a re-entry does (a second
/// mission_return_max_sec budget, a refilled escape ladder) is behavioural and
/// lands identically in a build with no event log.
TEST(MissionReturnGuard, TheLatchIsSetBeforeTheEndingCommits) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::finishMissionReturn(");
  ASSERT_FALSE(body.empty())
      << "the scan found no finishMissionReturn definition — it has been "
         "renamed or reshaped and this test has stopped testing anything";

  const size_t set = body.find("mission_return_done_  = true;");
  ASSERT_NE(set, std::string::npos)
      << "finishMissionReturn never latches the return as spent, so "
         "startReturnHome's guard can never fire";

  const size_t log_gate = body.find("if (exp_log_)");
  ASSERT_NE(log_gate, std::string::npos);
  EXPECT_LT(set, log_gate)
      << "the latch is set inside or after the event-log block — a build "
         "without an event log would still take a second homing leg";

  const size_t finish = body.find("finishNow(");
  ASSERT_NE(finish, std::string::npos);
  EXPECT_LT(set, finish) << "the run ends before the return is marked spent";
}

// ===========================================================================
// GROUP E. THE TERMINAL DISPATCH ALWAYS DISPOSES OF THE RUN.
// ===========================================================================
//
// finishOrRendezvous returns a bool the callers read as "was the run disposed
// of". `false` means DEFERRED — ask again on the next tick — and doLogStep acts
// on it by sending the node back to PLAN. That contract is only safe while
// every `false` it can return is BOUNDED, and exactly one is: the
// reconnect_confirm_sec confirmation window, which resolves either into a
// manoeuvre or into DONE because team_last_complete_time_ only advances while
// the team is whole.
//
// WHAT THIS GROUP EXISTS TO CATCH (2026-09-18). For two days the function also
// returned dispatchReconnect's value straight through. On a terminal dispatch
// reconnect_terminal_ is set, so may_defer is false by construction, and the
// only `false` dispatchReconnect can then produce is the RENDEZVOUS/HYBRID tail
// declining outright — armAppointment refused and the 2026-09-16 removal left
// that arm with no unscheduled fallback to drive to. Nothing about that
// resolves: the agreed pair is frozen for the duration of the outage, step_ is
// only incremented by doLogStep, and PLAN's budget test re-fires on the frozen
// count. The node spins in PLAN at the 10 Hz tick rate forever, AFTER
// recordExplorationComplete has stamped the endpoint — which is also what
// disarms the harness's stall gate, so the runner waits out the full wall
// clock with nothing in the log saying the run was over.
//
// It never reached a campaign: every campaign sets mission_return_enabled,
// which returns from finishOrRendezvous above the dispatch, and all 196 planner
// logs written since the change end on [latch] / [coverage-latched] with zero
// [step-budget]. Generation 32 put a SECOND early return above them both —
// keepAppointmentOnFinish, which takes any robot that finishes with a standing
// appointment and sends it to the meeting — so the terminal dispatch's own
// appointment branch is now shadowed twice over. The scans below are unmoved by
// that: what they constrain is the shape of the path once it IS reached, and
// both shadows are config-and-state accidents rather than deletions. Every one of the 39 appointment refusals in them carries the
// mid-run reason `peer-lost`, where a `false` is correct and the caller simply
// keeps planning. That is a config accident, not a property of the code, and
// `--mission-return 0` is a supported flag.
//
// These are source scans, with the limits every scan in this file has: they
// show the shape is present, not that it is reached.

/// A terminal dispatch that declines must end the run, not report a deferral.
TEST(TerminalDispatch, ADeclinedManoeuvreStillEndsTheRun) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::finishOrRendezvous(");
  ASSERT_FALSE(body.empty())
      << "the scan found no finishOrRendezvous definition — it has been "
         "renamed or reshaped and this test has stopped testing anything";

  const size_t terminal = body.find("reconnect_terminal_ = true");
  ASSERT_NE(terminal, std::string::npos)
      << "finishOrRendezvous no longer marks its dispatch terminal, so there "
         "is no longer a distinction between the bounded confirmation-window "
         "deferral and an outright decline — which is the distinction this "
         "whole group is about";

  EXPECT_EQ(body.find("return dispatchReconnect("), std::string::npos)
      << "finishOrRendezvous propagates dispatchReconnect's value directly. "
         "On the terminal path that value is `false` exactly when the arm "
         "declined outright and nothing will change its mind, and the caller "
         "reads `false` as \"ask again next tick\" — a 10 Hz spin in PLAN with "
         "step_ frozen and the endpoint already stamped";

  const size_t dispatch = body.find("dispatchReconnect(", terminal);
  ASSERT_NE(dispatch, std::string::npos)
      << "nothing dispatches a manoeuvre after the terminal marker is set";
  const size_t finish = body.find("finishNow(", dispatch);
  EXPECT_NE(finish, std::string::npos)
      << "there is no ending after the terminal dispatch. A decline has to "
         "land on finishNow: the robot has finished exploring and has nowhere "
         "agreed to be, so DONE is the honest answer and the only one that "
         "terminates";
}

/// The dispatch is where an unbounded `false` can originate, so pin that the
/// terminal flag really does rule out the deferral branch inside it. If
/// may_defer stopped consulting reconnect_terminal_, a terminal dispatch could
/// take the "keep exploring until the deadline" path — which is a deferral with
/// no caller willing to wait for it, and the test above would keep passing.
TEST(TerminalDispatch, TheDeferralIsRuledOutByTheTerminalFlag) {
  const std::string text = nodeSource();
  ASSERT_FALSE(text.empty()) << "cannot read " << EXPLO_PLANNER_NODE_CPP;
  const std::string body =
      functionBody(text, "bool ExploPlannerNode::dispatchReconnect(");
  ASSERT_FALSE(body.empty())
      << "the scan found no dispatchReconnect definition — it has been renamed "
         "or reshaped and this test has stopped testing anything";

  const size_t may_defer = body.find("may_defer =");
  ASSERT_NE(may_defer, std::string::npos)
      << "dispatchReconnect no longer computes may_defer, so whether a "
         "terminal dispatch can defer is decided somewhere this test cannot "
         "see";
  const size_t eol = body.find('\n', may_defer);
  const std::string expr = body.substr(may_defer, eol - may_defer);
  EXPECT_NE(expr.find("reconnect_terminal_"), std::string::npos)
      << "may_defer no longer excludes the terminal case:\n"
      << expr
      << "\nA robot whose exploration is already over cannot \"keep exploring "
         "until the departure deadline\", and no caller on that path is "
         "willing to wait for one";
}
