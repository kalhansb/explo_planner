// T5. Contract tests for THE ENDPOINT — the three events that decide when a
// robot's run is over.
//
// The endpoint state machine is in explo_planner_node.cpp, outside
// explo_planner_lib's source list, so no test links it. Duplicate endpoints
// leave evidence instead: both duplicate counters ride on run_end, even at
// zero. (notes: endpoint-tests-header-history)
// Moved comments: doc/test_endpoint_notes.md

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

/// The file's non-empty lines in write order, as raw text: an absent or
/// present-but-wrong field survives a lenient parser, so assertions are made on
/// bytes. (notes: endpoint-readlines-raw)
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
/// Returns "" when not found; callers must ASSERT on that, since a scan that
/// matches nothing passes every assertion. (notes: endpoint-functionbody-scan)
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

/// Drops comments so scans assert on code, not on prose quoting the same
/// identifiers. Tracks string and char literals so a // inside a literal is not
/// cut. A per-file copy by convention; do not share it.
/// (notes: endpoint-strip-comments)
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
// An armed slot with t_meet_ms == -1 reads as overdue forever. Only keeping the
// arming of appointment_armed_ with the t_meet_ms assignment prevents it;
// neither appointment_.valid() nor armAppointment's refusal clear does.
// (notes: endpoint-group-a-sentinel)
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

/// result (how the homing leg resolved) and reason (why the robot went home)
/// are distinct fields and must not collapse into each other.
/// (notes: endpoint-result-vs-reason)
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

/// A duplicate mission_complete is suppressed: the file keeps only the first
/// row, and the suppression is counted in dupMissionCompletes() and in
/// mission_completes_suppressed on run_end.
/// (notes: endpoint-duplicate-mission-complete)
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

/// One emission carrying occurrence 3: the writer transcribes the node's count
/// and must not write a literal 1 or derive it from its own rows.
/// (notes: endpoint-occurrence-transcribed)
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

/// A second exploration_complete is legitimate (a merged map can bring new
/// frontiers): explore_done_first_sim_sec latches the first declaration,
/// explore_done_sim_sec advances to the latest.
/// (notes: endpoint-second-exploration-complete)
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
// post_latch marks a rung crossed after the last declaration with no step
// since: post-stop merging, not exploration. logStep clears it.
// sec_since_explore_done is -1 when not applicable, so it is asserted by value.
// (notes: postlatch-flag-purpose)
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

/// A step after the declaration means exploration resumed (the step counter is
/// frozen through a manoeuvre and the homing leg), so every rung crossed after
/// it is exploration: post_latch false. (notes: postlatch-step-clears-flag)
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

/// Declare, resume, declare again: the second declaration re-arms post_latch,
/// and the interval runs from the last declaration (50 s here, not 350 s).
/// (notes: postlatch-rearm-last-declaration)
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

/// A censored run (duration cap before any declaration) writes
/// explore_done_sim_sec as null on run_end, never a number: reading max sim
/// time as completion would turn censored runs into fast ones.
/// (notes: runend-censored-null-completion)
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

/// Endpoints emitted before run_start are dropped and counted, never buffered;
/// events_dropped_before_start on run_end is the only evidence of the loss, so
/// the counter must count. (notes: endpoint-dropped-before-start)
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
  // 3, counting the dropped run_end itself.
  // (notes: runend-dropped-counts-itself)
  EXPECT_NE(lines[static_cast<size_t>(i)].find(
                "\"events_dropped_before_start\":3"),
            std::string::npos)
      << lines[static_cast<size_t>(i)];
}

/// A second run_end is suppressed so the file keeps exactly one last line; the
/// suppression is counted and warned, never silent.
/// (notes: runend-duplicate-suppressed)
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

/// Both duplicate-endpoint counters must be written at 0 on a clean run and
/// echo non-zero values verbatim. mission_return_reentries is the node's, via
/// RunEndEvent; mission_completes_suppressed is the writer's own.
/// (notes: endpoint-dup-counters-calibration)
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
// The re-entry guard is in startReturnHome, in explo_planner_node.cpp, which no
// test links, so these scan the node's source: they show the latch present and
// ordered, not reached, and fail if the scan stops matching.
// (notes: return-guard-source-scan)
// ===========================================================================

/// startReturnHome must read the mission_return_done_ latch before any of its
/// resets: a guard after even one reset refills the escape budget and zeroes
/// the approach window on a request it then refuses.
/// (notes: return-guard-before-resets)
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
// finishOrRendezvous returns whether the run was disposed of; false means
// deferred to the next tick, safe only while bounded (the reconnect_confirm_sec
// window). A terminal dispatch that declines must end the run.
// (notes: terminal-dispatch-disposes-run)

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

/// Pins that may_defer in dispatchReconnect excludes reconnect_terminal_, so a
/// terminal dispatch cannot take the deferral path; the test above would not
/// catch that. (notes: terminal-dispatch-may-defer)
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
