// Contract tests for the `home_watchdog` event row.
//
// These exist because of a generation-7 defect that no test could have caught,
// since no test covered this writer at all: the fire rows recorded `metric_m`
// (an INSTANTANEOUS remaining distance) while the quantity the detector
// actually compared — the movement over the window — was recorded nowhere, and
// the field comment described metric_m as movement "over window_sec".
//
// The banked evidence, re-measured rather than recalled: 7 non-escape-end
// home_watchdog rows exist across the g6pilot cells (6 in hybrid_seed103's
// bestla, 1 in off_seed102's atlas), all of kind `approach`; no frozen fire was
// ever banked. In 4 of the 7 metric_m and dist_home_m disagree (8.06 vs 4.00,
// and so on), which is the only reason the conflation was visible at all. The
// tested delta itself appears in NEITHER the jsonl NOR the plaintext of any
// banked cell — grep finds zero lines carrying it — so any statement about how
// far it sat from metric_m is a RECONSTRUCTION from the CSV pose track, not a
// reading. That unrecoverability is the finding: on the documented contract a
// scorer would have called every fire spurious, and had no logged quantity to
// check that against.
//
// That classification is load-bearing: these rows are the only basis for
// deciding whether a `home-gave-up` park is a genuine stall or a detector
// artefact, and that decision feeds censoring in the mission-completion
// analysis. So the fired inequality is asserted here, at the byte level of the
// emitted JSONL, rather than trusted.

#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "explo_planner/experiment_log.hpp"

using namespace explo_planner;

namespace {

/// Writes to a per-test, per-process path under /tmp and removes it on
/// destruction. Not a fixture member so each test names its own file — a
/// shared path across tests would let one test's truncation race another's.
/// The pid is in the name for the same reason one level up: `colcon test`
/// runs test executables in parallel, and two concurrent runs of this binary
/// sharing /tmp/explo_test_frozen.jsonl would interleave their writes and
/// fail intermittently, which is the worst way for a contract test to fail.
struct TempLogPath {
  explicit TempLogPath(const char* stem)
      : path(std::string("/tmp/explo_test_") + stem + "_" +
             std::to_string(static_cast<long>(::getpid())) + ".jsonl") {}
  ~TempLogPath() { std::remove(path.c_str()); }
  std::string path;
};

/// Reads one `"name":<number>` field back out of a JSONL row.
///
/// Exists so the inequality assertions below are made against what the WRITER
/// emitted, not against the literals the test passed in. Asserting
/// `EXPECT_LT(-0.03, 1.0)` on two constants is a tautology that holds for
/// every possible writer, including one that emits nothing at all.
bool readNum(const std::string& row, const char* name, double* out) {
  const std::string key = std::string("\"") + name + "\":";
  const size_t at = row.find(key);
  if (at == std::string::npos) return false;
  return std::sscanf(row.c_str() + at + key.size(), "%lf", out) == 1;
}

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

// The sign case. metric_m 3.69 is a real banked row (g6pilot_hybrid_seed103,
// bestla); the -0.03 beside it is reconstructed from that run's pose track, not
// logged — the point being that a receding robot and an advancing one produced
// indistinguishable rows. The field must therefore be able to disagree with
// metric_m in DIRECTION, not just in magnitude. If a future change routes this
// through a magnitude or clamps at zero, this fails.
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

// The abort-an-escape fire. This kind exists because a frozen detector firing
// DURING an escape leg used to be recorded only by the escape-end row that
// followed it, and escape-end is the writer's omit-case — so the one fire on
// that path was written down carrying no inequality at all.
//
// SCOPE, because two of the assertions below are weaker than they look.
// emitWatchdogRow() calls logHomeWatchdog() with the test's own literals, so
// this file can only pin the WRITER's round-trip: given this kind and this
// window, does the row come out carrying them, and does the omit-case leave
// the inequality alone. It cannot see what homeWatchdogFire actually passes.
//
// So the `window_sec` and the not-`escape-frozen` assertions would keep
// passing if the production call site regressed to the leg duration or to the
// aliased token. That is the whole failure they are named after, and it is
// covered at scoring time — by gate_g8.py, over the emitted cells — not here.
// Do not read a green run of this file as proof the call site is right.
//
// Why the name matters at all: `escape-frozen` is the `response` on the
// escape-end row this abort emits immediately afterwards, so if the kind ever
// becomes that same token, one abort puts the string in two columns of two
// consecutive rows and a reader grepping the token rather than the column
// counts one abort as two.
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
