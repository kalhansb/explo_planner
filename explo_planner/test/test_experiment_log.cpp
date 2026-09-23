// Contract tests for the `home_watchdog` event row.
//
// These rows are the only basis for telling a genuine home-gave-up stall from a
// detector artefact, which feeds censoring, so the fired inequality is asserted
// on the emitted JSONL bytes. (notes: watchdog-tests-header-history)
// Moved comments: doc/test_experiment_log_notes.md

#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "explo_planner/experiment_log.hpp"

using namespace explo_planner;

namespace {

/// A per-test, per-process path under /tmp, removed on destruction: colcon test
/// runs test binaries in parallel, and a shared path would let concurrent
/// writes interleave. (notes: explog-temp-log-path)
struct TempLogPath {
  explicit TempLogPath(const char* stem)
      : path(std::string("/tmp/explo_test_") + stem + "_" +
             std::to_string(static_cast<long>(::getpid())) + ".jsonl") {}
  ~TempLogPath() { std::remove(path.c_str()); }
  std::string path;
};

/// Reads one numeric field back out of a JSONL row, so assertions are made
/// against what the writer emitted, not the test's own literals.
/// (notes: explog-readnum)
bool readNum(const std::string& row, const char* name, double* out) {
  const std::string key = std::string("\"") + name + "\":";
  const size_t at = row.find(key);
  if (at == std::string::npos) return false;
  return std::sscanf(row.c_str() + at + key.size(), "%lf", out) == 1;
}

/// Emits one home_watchdog row and returns it as the raw line: an absent or
/// present-but-wrong field survives a lenient parser, so assertions are on the
/// bytes. (notes: explog-emit-watchdog-row)
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

// The sign case: test_delta_m must keep its sign and can disagree with metric_m
// in direction, not just magnitude. Routing it through a magnitude or clamping
// at zero fails this. (notes: watchdog-negative-delta)
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
  // joining to run_start params — so it is re-derived here from the row's own
  // bytes rather than restated over the literals passed in above.
  double td = 0.0, tt = 0.0;
  ASSERT_TRUE(readNum(row, "test_delta_m", &td)) << row;
  ASSERT_TRUE(readNum(row, "test_threshold_m", &tt)) << row;
  EXPECT_LT(td, tt) << "the row does not carry a fired inequality: " << row;
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

// A frozen fire during an escape leg is its own kind and carries the
// inequality. Pins the writer's round-trip only, not what homeWatchdogFire
// passes. The kind must differ from escape-frozen, the escape-end response.
// (notes: watchdog-frozen-in-escape)
TEST(ExperimentLogHomeWatchdog, FrozenInEscapeIsAFireAndDoesNotAliasEscapeEnd) {
  TempLogPath tmp("frozen_in_escape");
  const std::string row = emitWatchdogRow(tmp.path, "frozen-in-escape",
                                          /*dist_home_m=*/22.7,
                                          /*metric_m=*/22.7,
                                          /*window_sec=*/6.0,
                                          /*test_delta_m=*/0.11,
                                          /*test_threshold_m=*/0.20);
  ASSERT_FALSE(row.empty()) << "no home_watchdog row was written at all";
  // It is a FIRE: the omit-case must not swallow it.
  double td = 0.0, tt = 0.0;
  ASSERT_TRUE(readNum(row, "test_delta_m", &td)) << row;
  ASSERT_TRUE(readNum(row, "test_threshold_m", &tt)) << row;
  EXPECT_LT(td, tt) << "the row does not carry a fired inequality: " << row;
  // window_sec is the DETECTOR's window on a fire, never the escape leg's
  // duration — the leg duration convention belongs to escape-end alone.
  EXPECT_NE(row.find("\"window_sec\":6.000000"), std::string::npos) << row;
  // And the kind must stay distinct from the escape-end response token.
  EXPECT_NE(row.find("\"kind\":\"frozen-in-escape\""), std::string::npos) << row;
  EXPECT_EQ(row.find("\"kind\":\"escape-frozen\""), std::string::npos)
      << "kind aliases the escape-end response token: " << row;
}

// A call that omits the test arguments still writes both test fields, at their
// 0.0 defaults. (notes: watchdog-defaulted-call)
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

// --- cell_census (schema v4) ---
// P1's gate reads covered_fraction and roi_unknown_fraction off this one row.
// Keep both on it: joining cell_census to coverage_milestone by timestamp
// compares two different ticks. (notes: census-coverage-pairing)
TEST(ExperimentLogCellCensus, CarriesBothCoverageMeasuresOnOneRow) {
  TempLogPath tmp("census");
  std::string row;
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_experiment_log"));
    ASSERT_TRUE(log.open()) << "could not open " << tmp.path;
    ExperimentContext ctx;
    ctx.sim_time_sec = 10.0;
    ctx.state = "EXPLORE";
    ctx.step = 3;
    log.startRun(ctx, {});
    ctx.sim_time_sec = 55.0;

    CellCensusEvent e;
    e.cells_total = 9;
    e.unseen = 2; e.exploring = 3; e.covered = 4;
    e.exploring_by_others = 0; e.covered_by_others = 0;
    e.covered_fraction = 4.0 / 9.0;
    e.roi_unknown_fraction = 0.375;
    e.coverage_source = "scovox";
    e.changed = 1;
    e.commits_total = 12;
    e.cell_size_m = 10.0;
    e.nx = 3; e.ny = 3;
    e.grid_hash = 0xdeadbeefu;
    e.edges_enabled = 15; e.edges_total = 20;
    e.cells_measured = 7; e.cells_frontier_ok = 5;
    e.cell_unknown_min = 0.08;
    e.cell_unknown_p10 = 0.11;
    e.cell_unknown_median = 0.42;
    e.cell_frontier_frac_min = 0.01;
    e.cell_frontier_frac_median = 0.07;
    e.cell_frontier_frac_at_best_unknown = 0.03;
    log.logCellCensus(ctx, e);
  }
  std::ifstream in(tmp.path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.find("\"cell_census\"") != std::string::npos) row = line;
  }
  ASSERT_FALSE(row.empty()) << "no cell_census row was written at all";

  // The pairing the gate depends on, read back from the emitted bytes.
  double covered_fraction = -99.0, roi_unknown = -99.0;
  ASSERT_TRUE(readNum(row, "covered_fraction", &covered_fraction)) << row;
  ASSERT_TRUE(readNum(row, "roi_unknown_fraction", &roi_unknown)) << row;
  EXPECT_NEAR(covered_fraction, 4.0 / 9.0, 1e-6);
  EXPECT_NEAR(roi_unknown, 0.375, 1e-6);
  EXPECT_NE(row.find("\"coverage_source\":\"scovox\""), std::string::npos)
      << row;

  // The five statuses stay five fields. Collapsing the second-hand pair into
  // the first-hand counts would erase the only record of how much of this
  // robot's belief it never observed itself.
  double v = -99.0;
  for (const char* f : {"unseen", "exploring", "covered",
                        "exploring_by_others", "covered_by_others"}) {
    EXPECT_TRUE(readNum(row, f, &v)) << "missing '" << f << "' in " << row;
  }

  // The flap detector. Zero here on a run whose statuses moved would mean the
  // commit counter was never wired, and the hysteresis band would then look
  // healthy in every run regardless of how hard the cells were oscillating.
  ASSERT_TRUE(readNum(row, "commits_total", &v)) << row;
  EXPECT_DOUBLE_EQ(v, 12.0);

  // Geometry: without grid_hash a cell id in this file means nothing across
  // robots, and a hash written as a quoted hex string reads as text to half
  // the tools that open the file.
  ASSERT_TRUE(readNum(row, "grid_hash", &v)) << row;
  EXPECT_DOUBLE_EQ(v, 3735928559.0);

  // Reachability of the COVERED threshold, checked by value: without these a
  // census with zero COVERED cells looks broken, and a distribution over the
  // wrong cells still writes all five fields.
  // (notes: census-covered-reachability)
  ASSERT_TRUE(readNum(row, "cells_measured", &v)) << row;
  EXPECT_DOUBLE_EQ(v, 7.0);
  ASSERT_TRUE(readNum(row, "cells_frontier_ok", &v)) << row;
  EXPECT_DOUBLE_EQ(v, 5.0);
  ASSERT_TRUE(readNum(row, "cell_unknown_min", &v)) << row;
  EXPECT_NEAR(v, 0.08, 1e-6);
  ASSERT_TRUE(readNum(row, "cell_unknown_p10", &v)) << row;
  EXPECT_NEAR(v, 0.11, 1e-6);
  ASSERT_TRUE(readNum(row, "cell_unknown_median", &v)) << row;
  EXPECT_NEAR(v, 0.42, 1e-6);

  // The frontier veto's distribution. cell_frontier_frac_at_best_unknown is the
  // joint reading, so it is pinned to a value distinct from both marginals;
  // wired to either, it would still pass.
  // (notes: census-frontier-joint-reading)
  ASSERT_TRUE(readNum(row, "cell_frontier_frac_min", &v)) << row;
  EXPECT_NEAR(v, 0.01, 1e-6);
  ASSERT_TRUE(readNum(row, "cell_frontier_frac_median", &v)) << row;
  EXPECT_NEAR(v, 0.07, 1e-6);
  ASSERT_TRUE(readNum(row, "cell_frontier_frac_at_best_unknown", &v)) << row;
  EXPECT_NEAR(v, 0.03, 1e-6);
}

// --- appointment_leg (schema v11) ---
// leg_sec (escape-end only) and rolled_to_sec (unreached only) are written on
// every row, -1.0 when not applicable, never omitted. Asserted on a kind that
// owns neither column. (notes: apptleg-conditional-columns)
TEST(ExperimentLogAppointmentLeg, CarriesBothConditionalColumnsOnEveryKind) {
  TempLogPath tmp("appointment_leg");
  std::string row;
  {
    ExperimentLog log(tmp.path, "testbot",
                      rclcpp::get_logger("test_experiment_log"));
    ASSERT_TRUE(log.open()) << "could not open " << tmp.path;
    ExperimentContext ctx;
    ctx.sim_time_sec = 10.0;
    ctx.state = "RETURN_NAV";
    log.startRun(ctx, {});
    ctx.sim_time_sec = 256.7;
    AppointmentLegEvent e;
    e.kind         = "escape";
    e.cause        = "no-progress";
    e.dist_m       = 39.97;
    e.dest_x       = 15.0;
    e.dest_y       = -5.0;
    e.cell         = 12;
    e.escapes_used = 1;
    e.escapes_max  = 3;
    log.logAppointmentLeg(ctx, e);
  }
  std::ifstream in(tmp.path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.find("\"appointment_leg\"") != std::string::npos) row = line;
  }
  ASSERT_FALSE(row.empty()) << "no appointment_leg row was written at all";

  double v = 0.0;
  ASSERT_TRUE(readNum(row, "leg_sec", &v)) << row;
  EXPECT_NEAR(v, -1.0, 1e-6);
  ASSERT_TRUE(readNum(row, "rolled_to_sec", &v)) << row;
  EXPECT_NEAR(v, -1.0, 1e-6);

  // The cap travels with the count for the same reason: normalising "1" against
  // a limit fetched from a param row is the step a reader skips.
  ASSERT_TRUE(readNum(row, "escapes_used", &v)) << row;
  EXPECT_NEAR(v, 1.0, 1e-6);
  ASSERT_TRUE(readNum(row, "escapes_max", &v)) << row;
  EXPECT_NEAR(v, 3.0, 1e-6);

  EXPECT_NE(row.find("\"kind\":\"escape\""), std::string::npos) << row;
  EXPECT_NE(row.find("\"cause\":\"no-progress\""), std::string::npos) << row;
}

// --- The declared event vocabulary (schema v4) ---
// kEventKinds must match the kinds the writer emits: sim/equiv_gate.py scores
// new kinds against it and reads opt-in kinds from its tail. The test scans the
// writer's source, at a path CMake sets. (notes: schema-declared-event-kinds)
TEST(ExperimentLogSchema, DeclaredKindsMatchTheWriter) {
  std::ifstream src(EXPERIMENT_LOG_CPP);
  ASSERT_TRUE(src.good()) << "cannot read " << EXPERIMENT_LOG_CPP;
  std::stringstream buf;
  buf << src.rdbuf();
  const std::string text = buf.str();

  // Every `begin("<kind>"` in the writer.
  std::vector<std::string> emitted;
  const std::string needle = "begin(\"";
  for (size_t p = text.find(needle); p != std::string::npos;
       p = text.find(needle, p + 1)) {
    const size_t s = p + needle.size();
    const size_t e = text.find('"', s);
    ASSERT_NE(e, std::string::npos);
    const std::string kind = text.substr(s, e - s);
    if (std::find(emitted.begin(), emitted.end(), kind) == emitted.end()) {
      emitted.push_back(kind);
    }
  }
  ASSERT_FALSE(emitted.empty()) << "scanned " << EXPERIMENT_LOG_CPP
                                << " and found no begin(\"...\") call — the "
                                   "scan itself has stopped working";

  std::vector<std::string> declared(
      ExperimentLog::kEventKinds,
      ExperimentLog::kEventKinds + ExperimentLog::kEventKindCount);

  for (const std::string& k : emitted) {
    EXPECT_NE(std::find(declared.begin(), declared.end(), k), declared.end())
        << "the writer emits '" << k
        << "' but kEventKinds does not declare it; sim/equiv_gate.py would "
           "score it as a legacy kind and never notice it appearing at "
           "defaults";
  }
  for (const std::string& k : declared) {
    if (std::find(emitted.begin(), emitted.end(), k) != emitted.end()) continue;
    // A declared-but-unemitted kind is legal only while its phase is unwritten,
    // and only in the v4 block. One in the legacy block means a writer was
    // deleted without the vocabulary following it.
    const size_t idx = static_cast<size_t>(
        std::find(declared.begin(), declared.end(), k) - declared.begin());
    EXPECT_GE(idx, ExperimentLog::kFirstV4EventKind)
        << "kEventKinds declares legacy kind '" << k
        << "' that no writer emits any more";
  }

  // The boundary itself. If someone appends a v4 kind ABOVE the marker, the
  // gate reads it as legacy — so pin that everything below the marker is a
  // kind the writer already has, and the marker sits where v3 ended.
  ASSERT_LE(ExperimentLog::kFirstV4EventKind, ExperimentLog::kEventKindCount);
  for (size_t i = 0; i < ExperimentLog::kFirstV4EventKind; ++i) {
    EXPECT_NE(std::find(emitted.begin(), emitted.end(),
                        std::string(ExperimentLog::kEventKinds[i])),
              emitted.end())
        << "kEventKinds[" << i << "] = '" << ExperimentLog::kEventKinds[i]
        << "' sits in the pre-v4 block but the writer does not emit it";
  }
}
