/// @file test_metrics_logger.cpp
/// @brief Schema tests for the planner CSV.
///
/// A CSV writer fails in a way that no runtime check catches: the header and
/// the row are two separate lists of the same thing, written 70 lines apart,
/// and nothing ties them together. Append a field to one and forget the other
/// and every campaign after it is silently misaligned — a positional reader
/// returns the wrong column and a by-name reader returns the wrong value under
/// the right name, which is worse. The run still completes, the gate still
/// passes, and the damage is only visible months later in an archive.
///
/// So these tests do not test behaviour; they pin the correspondence.

#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "explo_planner/metrics_logger.hpp"

using explo_planner::MetricsLogger;
using explo_planner::StepMetrics;

namespace {

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(tok);
  return out;
}

/// Write one row and return (header fields, row fields).
std::pair<std::vector<std::string>, std::vector<std::string>>
writeOne(const StepMetrics& m, const std::string& tag) {
  const std::string path = std::string(testing::TempDir()) + "/mlog_" + tag + ".csv";
  {
    MetricsLogger log(path);
    log.logStep(m);
  }
  std::ifstream in(path);
  std::string header, row;
  std::getline(in, header);
  std::getline(in, row);
  return {split(header), split(row)};
}

/// Header name -> value, which is how every reader in this tree resolves
/// columns. Building it is itself the ordering check: a duplicate name or a
/// count mismatch shows up here.
std::map<std::string, std::string> byName(
    const std::vector<std::string>& hdr, const std::vector<std::string>& row) {
  std::map<std::string, std::string> out;
  for (size_t i = 0; i < hdr.size() && i < row.size(); ++i) out[hdr[i]] = row[i];
  return out;
}

} // namespace

/// The drift guard. Everything else in this file assumes it holds.
TEST(MetricsLoggerSchema, HeaderAndRowHaveTheSameColumnCount) {
  StepMetrics m;
  auto [hdr, row] = writeOne(m, "count");
  EXPECT_EQ(hdr.size(), row.size())
      << "header and row disagree: a field was appended to one and not the "
         "other. Header has " << hdr.size() << ", row has " << row.size();
  EXPECT_GT(hdr.size(), 30u) << "header looks truncated";
}

/// No duplicate column names. A duplicate silently shadows: every by-name
/// reader takes one of the two and there is no error to notice.
TEST(MetricsLoggerSchema, ColumnNamesAreUnique) {
  StepMetrics m;
  auto [hdr, row] = writeOne(m, "unique");
  std::map<std::string, int> seen;
  for (const auto& h : hdr) ++seen[h];
  for (const auto& [name, n] : seen)
    EXPECT_EQ(n, 1) << "duplicate column name: " << name;
}

/// The ordering check a column-count test cannot do.
///
/// Every new field gets a DISTINCT value, so swapping any two of them in the
/// writer — plan_rej_map for plan_rej_unreach, say — fails here. That specific
/// swap is the one that matters: it mislabels the cause of a starvation, which
/// is the entire quantity these columns were added to record, and both columns
/// are ints so nothing else would ever flag it.
TEST(MetricsLoggerSchema, EachPlanColumnCarriesItsOwnValue) {
  StepMetrics m;
  m.plan_cand_total    = 178;
  m.plan_rej_close     = 11;
  m.plan_rej_map       = 82;
  m.plan_rej_unreach   = 94;
  m.plan_rej_blacklist = 13;
  m.plan_rej_minpos    = 55;
  m.plan_stall_ticks   = 28;

  auto [hdr, row] = writeOne(m, "plan");
  ASSERT_EQ(hdr.size(), row.size());
  auto v = byName(hdr, row);

  EXPECT_EQ(v["plan_cand_total"],    "178");
  EXPECT_EQ(v["plan_rej_close"],     "11");
  EXPECT_EQ(v["plan_rej_map"],       "82");
  EXPECT_EQ(v["plan_rej_unreach"],   "94");
  EXPECT_EQ(v["plan_rej_blacklist"], "13");
  EXPECT_EQ(v["plan_rej_minpos"],    "55");
  EXPECT_EQ(v["plan_stall_ticks"],   "28");
}

/// -1 must survive to the file as -1.
///
/// These columns are read as "no planning attempt has happened yet". If the
/// default were ever changed to 0, or clamped anywhere on the way out, a row
/// from before the first plan would be indistinguishable from a row recording
/// a genuine zero-rejection tick. That ambiguity is exactly what made
/// rejected_by_minpos useless for a whole campaign.
TEST(MetricsLoggerSchema, UnattemptedPlanColumnsWriteMinusOneNotZero) {
  StepMetrics m;  // untouched: no planning attempt
  auto [hdr, row] = writeOne(m, "unattempted");
  auto v = byName(hdr, row);

  for (const char* c : {"plan_cand_total", "plan_rej_close", "plan_rej_map",
                        "plan_rej_unreach", "plan_rej_blacklist",
                        "plan_rej_minpos", "plan_stall_ticks"}) {
    EXPECT_EQ(v[c], "-1") << c << " must distinguish 'never attempted' from a "
                                  "measured zero";
  }
}

/// The old columns keep their old names and their old meaning. Archived
/// campaigns are read with scripts that predate this change; renaming or
/// repurposing either of these would change the answer those scripts give
/// without changing the question they ask.
TEST(MetricsLoggerSchema, LegacyRejectionColumnsAreUnchanged) {
  StepMetrics m;
  m.rejected_by_minpos      = 7;
  m.rejected_by_unreachable = 9;
  auto [hdr, row] = writeOne(m, "legacy");
  auto v = byName(hdr, row);

  EXPECT_EQ(v["rejected_by_minpos"], "7");
  EXPECT_EQ(v["rejected_by_unreachable"], "9");
  // Still defaulting to 0, not -1: their semantics are LOG_STEP-only and are
  // deliberately not being migrated.
  StepMetrics d;
  auto [h2, r2] = writeOne(d, "legacy_default");
  auto v2 = byName(h2, r2);
  EXPECT_EQ(v2["rejected_by_minpos"], "0");
  EXPECT_EQ(v2["rejected_by_unreachable"], "0");
}

/// The new block sits at the right-hand end, which is the schema rule the
/// header comment states: readers outside this tree may resolve positionally,
/// and an old file read against a new schema must stay aligned up to the point
/// where it simply runs out of columns.
TEST(MetricsLoggerSchema, NewColumnsAreAppendedAtTheEnd) {
  StepMetrics m;
  auto [hdr, row] = writeOne(m, "append");
  ASSERT_GE(hdr.size(), 7u);
  const std::vector<std::string> tail(hdr.end() - 7, hdr.end());
  EXPECT_EQ(tail, (std::vector<std::string>{
                      "plan_cand_total", "plan_rej_close", "plan_rej_map",
                      "plan_rej_unreach", "plan_rej_blacklist",
                      "plan_rej_minpos", "plan_stall_ticks"}));
}
