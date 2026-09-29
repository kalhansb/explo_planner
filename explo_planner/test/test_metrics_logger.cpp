#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "explo_planner/metrics_logger.hpp"

using namespace explo_planner;

namespace {

// A private directory per test, removed on scope exit. mkdtemp (rather than a
// fixed name) keeps concurrent runs and other users off each other's paths: a
// fixed /tmp name owned by another user would make remove_all throw out of the
// test body as an uncaught exception rather than a clean failure.
class TempDir {
 public:
  TempDir() {
    std::string pattern =
        (std::filesystem::temp_directory_path() / "explo_metrics_XXXXXX")
            .string();
    std::vector<char> buf(pattern.begin(), pattern.end());
    buf.push_back('\0');
    const char* made = ::mkdtemp(buf.data());
    if (made == nullptr) throw std::runtime_error("mkdtemp failed");
    path_ = made;
  }
  ~TempDir() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);  // never throw from a destructor
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;

  std::filesystem::path file(const std::string& name) const {
    return path_ / name;
  }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

std::vector<std::string> readLines(const std::filesystem::path& p) {
  std::vector<std::string> lines;
  std::ifstream in(p);
  std::string line;
  while (std::getline(in, line))
    if (!line.empty()) lines.push_back(line);
  return lines;
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string f;
  while (std::getline(ss, f, ',')) fields.push_back(f);
  return fields;
}

// Value of a named column in a header+row pair, by header name.
std::string valueOf(const std::vector<std::string>& names,
                    const std::vector<std::string>& values,
                    const std::string& column) {
  for (size_t i = 0; i < names.size() && i < values.size(); ++i)
    if (names[i] == column) return values[i];
  return "<missing column>";
}

}  // namespace

// The header and the row must agree on how many columns there are. Adding a
// StepMetrics field to logStep() but not to writeHeader() (or the reverse)
// shifts every column after it, which silently corrupts the analysis of a
// finished run rather than failing it.
TEST(MetricsLogger, HeaderAndRowHaveSameColumnCount) {
  TempDir dir;
  auto csv = dir.file("m.csv");
  {
    MetricsLogger log(csv.string());
    log.logStep(StepMetrics{});
  }
  auto lines = readLines(csv);
  ASSERT_EQ(lines.size(), 2u);  // header + one row
  EXPECT_EQ(splitCsv(lines[0]).size(), splitCsv(lines[1]).size());
  // Pin the width too: a field added to StepMetrics but wired into NEITHER
  // writeHeader nor logStep is invisible to the comparison above.
  EXPECT_EQ(splitCsv(lines[0]).size(), 26u)
      << "column count changed — update writeHeader, logStep, and the "
         "expected list in ColumnOrderMatchesHeaderNames together";
}

// Column ORDER, not just count: every column gets a distinct value, so a value
// written into the wrong position is caught by name. Numbers are compared
// numerically so this test stays about the schema and does not also pin the
// stream's number formatting (that is EpochTimeKeepsMillisecondResolution's
// job). sim_time_sec deliberately carries a realistic wall-clock epoch.
TEST(MetricsLogger, ColumnOrderMatchesHeaderNames) {
  constexpr double kEpoch = 1758912345.678;  // ~2025-09-26, as on hardware

  StepMetrics m;
  m.step                    = 1;
  m.sim_time_sec            = kEpoch;
  m.total_observed_voxels   = 3;
  m.frontier_voxels         = 4;
  m.distance_traveled       = 5;
  m.selected_score          = 6;
  m.plan_time_ms            = 7;
  m.mean_eig                = 8;
  m.mean_entropy            = 9;
  m.mean_variance           = 10;
  m.mean_info_gain          = 11;
  m.mean_path_cost          = 12;
  m.selected_info_gain      = 13;
  m.selected_path_cost      = 14;
  m.selected_utility        = 15;
  m.coord_active_peers      = 16;
  m.rejected_by_minpos      = 17;
  m.rejected_by_unreachable = 18;
  m.phase                   = "exploit";
  m.target_id               = 20;
  m.vantage_index           = 21;
  m.n_vantages_valid        = 22;
  m.vantage_los_clear       = 23;
  m.dwell_sec               = 24;
  m.prox_hold_count         = 25;
  m.prox_hold_total_sec     = 26;

  TempDir dir;
  auto csv = dir.file("m.csv");
  {
    MetricsLogger log(csv.string());
    log.logStep(m);
  }
  auto lines = readLines(csv);
  ASSERT_EQ(lines.size(), 2u);

  const std::vector<std::pair<std::string, double>> expected = {
      {"step", 1},
      {"sim_time_sec", kEpoch},
      {"total_observed_voxels", 3},
      {"frontier_voxels", 4},
      {"distance_traveled", 5},
      {"selected_score", 6},
      {"plan_time_ms", 7},
      {"mean_eig", 8},
      {"mean_entropy", 9},
      {"mean_variance", 10},
      {"mean_info_gain", 11},
      {"mean_path_cost", 12},
      {"selected_info_gain", 13},
      {"selected_path_cost", 14},
      {"selected_utility", 15},
      {"coord_active_peers", 16},
      {"rejected_by_minpos", 17},
      {"rejected_by_unreachable", 18},
      {"phase", 0},  // string column, value checked separately below
      {"target_id", 20},
      {"vantage_index", 21},
      {"n_vantages_valid", 22},
      {"vantage_los_clear", 23},
      {"dwell_sec", 24},
      {"prox_hold_count", 25},
      {"prox_hold_total_sec", 26},
  };

  auto names  = splitCsv(lines[0]);
  auto values = splitCsv(lines[1]);
  ASSERT_EQ(names.size(), expected.size());
  ASSERT_EQ(values.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(names[i], expected[i].first) << "header column " << i;
    if (expected[i].first == "phase") {
      EXPECT_EQ(values[i], "exploit") << "value for column " << i;
      continue;
    }
    // Millisecond tolerance: enough to catch a swapped or shifted column,
    // loose enough not to depend on the exact text formatting.
    EXPECT_NEAR(std::stod(values[i]), expected[i].second, 1e-3)
        << "value for column " << i << " (" << names[i] << ")";
  }
}

// sim_time_sec comes from this->now().seconds(), which with the shipped
// use_sim_time: false is a wall-clock epoch (~1.76e9). At the stream's default
// 6 significant digits every row of a run collapses to the same text
// (1.75891e+09, ~1000 s resolution), destroying the only time reference a
// finished experiment has. Two rows a few seconds apart must stay distinct and
// round-trip to millisecond accuracy.
TEST(MetricsLogger, EpochTimeKeepsMillisecondResolution) {
  constexpr double kT0 = 1758912345.678;
  constexpr double kDt = 3.234;

  TempDir dir;
  auto csv = dir.file("m.csv");
  {
    MetricsLogger log(csv.string());
    StepMetrics a;
    a.step = 1;
    a.sim_time_sec = kT0;
    log.logStep(a);
    StepMetrics b;
    b.step = 2;
    b.sim_time_sec = kT0 + kDt;
    log.logStep(b);
  }
  auto lines = readLines(csv);
  ASSERT_EQ(lines.size(), 3u);
  auto names = splitCsv(lines[0]);
  const std::string t0_text = valueOf(names, splitCsv(lines[1]), "sim_time_sec");
  const std::string t1_text = valueOf(names, splitCsv(lines[2]), "sim_time_sec");

  EXPECT_NE(t0_text, t1_text)
      << "both rows wrote the same timestamp text (" << t0_text
      << "); every row of a real run would share one time";
  EXPECT_NEAR(std::stod(t0_text), kT0, 1e-3);
  EXPECT_NEAR(std::stod(t1_text), kT0 + kDt, 1e-3);
  EXPECT_NEAR(std::stod(t1_text) - std::stod(t0_text), kDt, 2e-3);
}

// An explore-phase step leaves every exploitation and proximity column at the
// defaults metrics_logger.hpp documents, so a CSV row always attributes itself
// to a phase.
TEST(MetricsLogger, ExploreRowCarriesDocumentedDefaults) {
  TempDir dir;
  auto csv = dir.file("m.csv");
  {
    MetricsLogger log(csv.string());
    log.logStep(StepMetrics{});
  }
  auto lines = readLines(csv);
  ASSERT_EQ(lines.size(), 2u);
  auto names  = splitCsv(lines[0]);
  auto values = splitCsv(lines[1]);
  EXPECT_EQ(valueOf(names, values, "phase"), "explore");
  EXPECT_EQ(valueOf(names, values, "target_id"), "-1");
  EXPECT_EQ(valueOf(names, values, "vantage_index"), "-1");
  EXPECT_EQ(valueOf(names, values, "n_vantages_valid"), "0");
  EXPECT_EQ(valueOf(names, values, "vantage_los_clear"), "0");
  EXPECT_EQ(valueOf(names, values, "dwell_sec"), "0");
  EXPECT_EQ(valueOf(names, values, "prox_hold_count"), "0");
  EXPECT_EQ(valueOf(names, values, "prox_hold_total_sec"), "0");
}

// The header is written once, on the first row — not per row.
TEST(MetricsLogger, HeaderWrittenOnceAcrossManyRows) {
  TempDir dir;
  auto csv = dir.file("m.csv");
  {
    MetricsLogger log(csv.string());
    for (int i = 0; i < 3; ++i) {
      StepMetrics m;
      m.step = i;
      log.logStep(m);
    }
  }
  auto lines = readLines(csv);
  ASSERT_EQ(lines.size(), 4u);  // 1 header + 3 rows
  EXPECT_EQ(splitCsv(lines[0])[0], "step");
  EXPECT_EQ(splitCsv(lines[1])[0], "0");
  EXPECT_EQ(splitCsv(lines[2])[0], "1");
  EXPECT_EQ(splitCsv(lines[3])[0], "2");
}

// Header emission is lazy: a run that never completes a step leaves an empty
// file, not a header-only one that reads as a valid zero-row experiment.
TEST(MetricsLogger, NoStepsLeavesFileEmpty) {
  TempDir dir;
  auto csv = dir.file("m.csv");
  { MetricsLogger log(csv.string()); }
  EXPECT_TRUE(std::filesystem::exists(csv));
  EXPECT_EQ(std::filesystem::file_size(csv), 0u);
}

// Every row is flushed as it is logged, so a run killed mid-experiment keeps
// the steps it already completed.
TEST(MetricsLogger, RowsAreFlushedBeforeDestruction) {
  TempDir dir;
  auto csv = dir.file("m.csv");
  MetricsLogger log(csv.string());  // deliberately still alive below
  StepMetrics m;
  m.step = 7;
  log.logStep(m);
  auto lines = readLines(csv);  // read while the logger holds the file open
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(splitCsv(lines[1])[0], "7");
}

// A re-run truncates: the CSV holds only the new run, never new rows appended
// under a previous run's header.
TEST(MetricsLogger, ReopenTruncatesPreviousRun) {
  TempDir dir;
  auto csv = dir.file("m.csv");
  {
    MetricsLogger log(csv.string());
    StepMetrics m;
    m.step = 111;
    log.logStep(m);
  }
  {
    MetricsLogger log(csv.string());
    StepMetrics m;
    m.step = 222;
    log.logStep(m);
  }
  auto lines = readLines(csv);
  ASSERT_EQ(lines.size(), 2u);  // not 3, and not 4
  EXPECT_EQ(splitCsv(lines[1])[0], "222");
}

// An unwritable path must throw at construction rather than silently no-op
// every subsequent write and produce an empty experiment.
TEST(MetricsLogger, UnwritablePathThrows) {
  TempDir dir;
  auto missing = dir.path() / "no_such_subdir" / "m.csv";
  EXPECT_THROW(MetricsLogger log(missing.string()), std::runtime_error);
}
