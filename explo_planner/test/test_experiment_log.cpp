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

// --- cell_census (schema v4) ---
//
// P1's gate is read off this row and nothing else: it asks whether the coarse
// census converges to COVERED as the ROI's continuous unknown fraction falls.
// Both numbers therefore have to be ON THE SAME ROW — the sim is nondetermin-
// istic enough run-to-run that joining `cell_census` to `coverage_milestone`
// on a timestamp would be comparing two ticks and reporting the difference as
// a disagreement between the measures. If a future edit moves either number
// off this event, the gate silently degrades into that join, so the pairing is
// asserted at the byte level rather than assumed.
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

  // Reachability of the COVERED threshold. Without these, a census reporting
  // zero COVERED cells is indistinguishable from a census that is broken, and
  // P1's first run produced exactly that row: an entire run to the completion
  // criterion with not one promotion and nothing to say why. They are checked
  // by VALUE, not just presence — a distribution wired to the wrong cells (the
  // whole grid rather than the measured ones) still writes all five fields,
  // and reads as a world where nothing is mappable.
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

  // The frontier veto's own distribution. `_at_best_unknown` is the joint
  // reading and the only one that can answer whether the two thresholds are
  // simultaneously satisfiable, so it is pinned to a value DISTINCT from both
  // marginals — wiring it to either of them would otherwise pass this test
  // while silently answering a different question.
  ASSERT_TRUE(readNum(row, "cell_frontier_frac_min", &v)) << row;
  EXPECT_NEAR(v, 0.01, 1e-6);
  ASSERT_TRUE(readNum(row, "cell_frontier_frac_median", &v)) << row;
  EXPECT_NEAR(v, 0.07, 1e-6);
  ASSERT_TRUE(readNum(row, "cell_frontier_frac_at_best_unknown", &v)) << row;
  EXPECT_NEAR(v, 0.03, 1e-6);
}

// --- appointment_leg (schema v11) ---
//
// Two of this row's columns belong to one kind each — `leg_sec` to escape-end,
// `rolled_to_sec` to unreached — and the writer carries both on every row at
// -1.0 rather than omitting them off their own kind. That choice is what the
// reader depends on, and it is invisible in the data if it breaks: an omitted
// key and a sentinel one look identical to any `.get(k, -1)` reader, so the
// give-up that HAD no appointment left to roll and the one whose roll was never
// written would fold into the same count. Asserted at the byte level, on the
// kind that owns NEITHER column, because that is the row where a writer that
// omitted them would still look right on both of the others.
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
//
// kEventKinds is what sim/equiv_gate.py scores "did a new event kind appear at
// defaults?" against, and a declared universe that has drifted from the writer
// answers that question wrongly in the silent direction: a kind missing from
// the list reads as "new" on every run that emits it (noise, which gets the
// gate loosened), and — worse — the gate's notion of which kinds are opt-in
// comes from the list's tail, so an unlisted v4 kind is scored as legacy and
// its appearance at defaults never fails anything.
//
// Nothing in C++ can enumerate the writer's calls, so the test reads the
// writer's SOURCE, exactly as test_failed_goal_blacklist reads the shipped
// YAML rather than a copy of its values. The path comes from CMake, so this
// cannot silently pass by scanning a stale installed tree.
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
