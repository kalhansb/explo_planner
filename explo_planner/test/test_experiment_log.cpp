// Contract tests for the `home_watchdog` event row.
//
// These exist because of a generation-7 defect that no test could have caught,
// since no test covered this writer at all: the fire rows recorded `metric_m`
// (an INSTANTANEOUS remaining distance) while the quantity the detector
// actually compared — the movement over the window — was recorded nowhere, and
// the field comment described metric_m as movement "over window_sec". On the
// seven banked g6pilot fires metric_m stood 4.1x to 162x above the delta that
// fired the detector, and one approach fire had the robot RECEDING (-0.03 m)
// against a logged 3.69. Anyone scoring the detector on the documented contract
// would have called every fire spurious.
//
// That classification is load-bearing: these rows are the only basis for
// deciding whether a `home-gave-up` park is a genuine stall or a detector
// artefact, and that decision feeds censoring in the mission-completion
// analysis. So the fired inequality is asserted here, at the byte level of the
// emitted JSONL, rather than trusted.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "explo_planner/experiment_log.hpp"

using namespace explo_planner;

namespace {

/// Writes to a unique path under the build dir's temp area and removes it on
/// destruction. Not a fixture member so each test names its own file — a
/// shared path across tests would let one test's truncation race another's.
struct TempLogPath {
  explicit TempLogPath(const char* stem)
      : path(std::string("/tmp/explo_test_") + stem + ".jsonl") {}
  ~TempLogPath() { std::remove(path.c_str()); }
  std::string path;
};

/// Emits one home_watchdog row and returns it verbatim.
///
/// Deliberately returns the RAW LINE rather than a parsed structure: the defect
/// being guarded against is a field that is present-but-wrong or absent, and
/// both of those survive a lenient parser. Substring assertions on the emitted
/// bytes are what the analysis scripts actually see.
std::string emitWatchdogRow(const std::string& path, const char* kind,
                            double dist_home_m, double metric_m,
                            double window_sec, double test_delta_m,
                            double test_threshold_m) {
  {
    ExperimentLog log(path, "testbot", rclcpp::get_logger("test_experiment_log"));
    EXPECT_TRUE(log.open()) << "could not open " << path;
    ExperimentContext ctx;
    ctx.sim_time_sec = 100.0;
    ctx.state = "RETURN_HOME";
    ctx.step = 7;
    log.startRun(ctx, {});
    ctx.sim_time_sec = 142.5;
    log.logHomeWatchdog(ctx, kind, "direct", "resend", dist_home_m, metric_m,
                        window_sec, /*escapes_used=*/0, test_delta_m,
                        test_threshold_m);
  }  // destructor flushes and closes

  std::ifstream in(path);
  std::string line, found;
  while (std::getline(in, line)) {
    if (line.find("\"home_watchdog\"") != std::string::npos) found = line;
  }
  return found;
}

}  // namespace

// The frozen detector's canonical fire: a robot that moved EXACTLY nothing.
// 0.0 is a real, common measurement here, which is why absence — not a
// numeric sentinel — has to be what marks a row with no inequality.
TEST(ExperimentLogHomeWatchdog, FrozenFireCarriesTheTestedInequality) {
  TempLogPath tmp("frozen");
  const std::string row = emitWatchdogRow(tmp.path, "frozen",
                                          /*dist_home_m=*/18.4,
                                          /*metric_m=*/18.4,
                                          /*window_sec=*/20.0,
                                          /*test_delta_m=*/0.0,
                                          /*test_threshold_m=*/0.5);
  ASSERT_FALSE(row.empty()) << "no home_watchdog row was written at all";
  EXPECT_NE(row.find("\"test_delta_m\":0.000000"), std::string::npos) << row;
  EXPECT_NE(row.find("\"test_threshold_m\":0.500000"), std::string::npos) << row;
  // metric_m must survive alongside it. The two answer different questions
  // ("how far is left" vs "how far it moved") and the diagnosis needs both;
  // collapsing them is what produced the generation-7 ambiguity.
  EXPECT_NE(row.find("\"metric_m\":18.400000"), std::string::npos) << row;
}

// The sign case. A receding robot fired the approach detector at -0.03 m in
// g6pilot_hybrid_seed103 while the row read 3.69 — the field did not merely
// disagree in magnitude, it disagreed in DIRECTION. If a future change routes
// this through a magnitude or clamps at zero, this fails.
TEST(ExperimentLogHomeWatchdog, ApproachFirePreservesNegativeDelta) {
  TempLogPath tmp("approach");
  const std::string row = emitWatchdogRow(tmp.path, "approach",
                                          /*dist_home_m=*/12.0,
                                          /*metric_m=*/3.69,
                                          /*window_sec=*/30.0,
                                          /*test_delta_m=*/-0.03,
                                          /*test_threshold_m=*/1.0);
  ASSERT_FALSE(row.empty());
  EXPECT_NE(row.find("\"test_delta_m\":-0.030000"), std::string::npos) << row;
  EXPECT_NE(row.find("\"test_threshold_m\":1.000000"), std::string::npos) << row;
  // The fired inequality must be re-derivable from the row ALONE, without
  // joining to run_start params.
  EXPECT_LT(-0.03, 1.0);
}

// escape-end is a leg termination, not a detector fire: it evaluates no
// inequality. The fields must be ABSENT, not zero-filled — a zero here would
// be indistinguishable from the frozen fire asserted above.
TEST(ExperimentLogHomeWatchdog, EscapeEndOmitsTheTestFieldsEntirely) {
  TempLogPath tmp("escape_end");
  const std::string row = emitWatchdogRow(tmp.path, "escape-end",
                                          /*dist_home_m=*/9.1,
                                          /*metric_m=*/9.1,
                                          /*window_sec=*/0.0,
                                          /*test_delta_m=*/0.0,
                                          /*test_threshold_m=*/0.0);
  ASSERT_FALSE(row.empty()) << "escape-end row must still be written";
  EXPECT_EQ(row.find("test_delta_m"), std::string::npos)
      << "escape-end evaluates no inequality; the field must be absent: " << row;
  EXPECT_EQ(row.find("test_threshold_m"), std::string::npos) << row;
}

// Guards the writer against a silent regression to the generation-7 default
// arguments, where a caller that passed nothing still produced a row that
// LOOKED complete (0.0 / 0.0) and read as a frozen fire at a zero threshold —
// an inequality that is false for every possible delta.
TEST(ExperimentLogHomeWatchdog, DefaultedCallStillEmitsBothFields) {
  TempLogPath tmp("defaulted");
  std::string row;
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_experiment_log"));
    ASSERT_TRUE(log.open());
    ExperimentContext ctx;
    ctx.sim_time_sec = 100.0;
    ctx.state = "RETURN_HOME";
    log.startRun(ctx, {});
    ctx.sim_time_sec = 130.0;
    log.logHomeWatchdog(ctx, "frozen", "direct", "resend", 5.0, 5.0, 20.0, 0);
  }
  std::ifstream in(tmp.path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.find("\"home_watchdog\"") != std::string::npos) row = line;
  }
  ASSERT_FALSE(row.empty());
  // Present, so a stale caller is visible in the data as an impossible
  // threshold rather than invisible as a missing field.
  EXPECT_NE(row.find("\"test_delta_m\""), std::string::npos) << row;
  EXPECT_NE(row.find("\"test_threshold_m\":0.000000"), std::string::npos) << row;
}
