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
/// Moved comments: doc/explo_planner_code_notes.md

#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>  // std::pair, returned by the reader helper
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

/// The ordering check a column-count test cannot do: every plan column gets a
/// distinct value, so swapping any two in the writer (e.g. plan_rej_map and
/// plan_rej_unreach) fails here. (notes: mlog-test-plan-column-order)
TEST(MetricsLoggerSchema, EachPlanColumnCarriesItsOwnValue) {
  StepMetrics m;
  m.plan_cand_total    = 178;
  m.plan_rej_close     = 11;
  m.plan_rej_map       = 82;
  m.plan_rej_unreach   = 94;
  m.plan_rej_blacklist = 13;
  m.plan_rej_visited   = 4;
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
  // Deliberately NOT equal to plan_rej_blacklist, and deliberately smaller:
  // visited is a subset of the union, and the whole reason the column exists
  // is that a reader cannot recover it from the union. If the writer ever
  // aliased the two, every other assertion here would still pass.
  EXPECT_EQ(v["plan_rej_visited"],   "4");
  EXPECT_EQ(v["plan_rej_minpos"],    "55");
  EXPECT_EQ(v["plan_stall_ticks"],   "28");
}

/// -1 must survive to the file as -1: it means no planning attempt yet. A 0
/// default or an output clamp would make it indistinguishable from a measured
/// zero-rejection tick. (notes: mlog-test-plan-minus-one)
TEST(MetricsLoggerSchema, UnattemptedPlanColumnsWriteMinusOneNotZero) {
  StepMetrics m;  // untouched: no planning attempt
  auto [hdr, row] = writeOne(m, "unattempted");
  auto v = byName(hdr, row);

  for (const char* c : {"plan_cand_total", "plan_rej_close", "plan_rej_map",
                        "plan_rej_unreach", "plan_rej_blacklist",
                        "plan_rej_visited", "plan_rej_minpos",
                        "plan_stall_ticks"}) {
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

/// Columns are appended at the end, never inserted, because readers may
/// resolve positionally; each append updates this tail list. Do not move a
/// column (e.g. plan_rej_visited) beside its relatives.
/// (notes: mlog-test-append-only-tail)
TEST(MetricsLoggerSchema, NewColumnsAreAppendedAtTheEnd) {
  StepMetrics m;
  auto [hdr, row] = writeOne(m, "append");
  ASSERT_GE(hdr.size(), 11u);
  const std::vector<std::string> tail(hdr.end() - 11, hdr.end());
  EXPECT_EQ(tail, (std::vector<std::string>{
                      "plan_cand_total", "plan_rej_close", "plan_rej_map",
                      "plan_rej_unreach", "plan_rej_blacklist",
                      "plan_rej_minpos", "plan_stall_ticks",
                      "pursue_peer", "pursue_quarry_live", "team_complete",
                      "plan_rej_visited"}));
}

/// R5. The §3.4 columns: defaults are the "not applicable" sentinel, not a
/// measured false, and a peer id that contains the field separator must not be
/// able to shift the row.
TEST(MetricsLoggerSchema, PursuitColumnsDefaultToNotApplicable) {
  StepMetrics m;  // untouched: no coordination, no chase
  auto [hdr, row] = writeOne(m, "pursuit_default");
  auto v = byName(hdr, row);

  // Also the only reachable test of sanitizeField on empty input, since that
  // function is TU-local: an empty pursue_peer must be written as an empty
  // field. (notes: mlog-test-sanitize-empty)
  EXPECT_EQ(v["pursue_peer"], "")
      << "no chase must write an empty id, not a placeholder that could be "
         "mistaken for a robot name";
  EXPECT_EQ(v["pursue_quarry_live"], "-1")
      << "'not pursuing' must not read as 'quarry was not heard'";
  EXPECT_EQ(v["team_complete"], "-1")
      << "'no peer table' must not read as 'team was incomplete'";
}

/// The case §3.4 is about: the quarry came back, the rest of the team did not.
/// Both facts have to survive to the file independently, because the single
/// `outcome` label collapses them.
TEST(MetricsLoggerSchema, PursuitColumnsRecordTheDisagreementCase) {
  StepMetrics m;
  m.pursue_peer        = "bestla";
  m.pursue_quarry_live = 1;
  m.team_complete      = 0;
  auto [hdr, row] = writeOne(m, "pursuit_disagree");
  auto v = byName(hdr, row);

  EXPECT_EQ(v["pursue_peer"], "bestla");
  EXPECT_EQ(v["pursue_quarry_live"], "1");
  EXPECT_EQ(v["team_complete"], "0");
}

/// A robot id carrying a separator would shift every column to its right on
/// that row and nowhere else, and the file would still parse. Substitution,
/// not quoting: the awk header-scanner in run_explo_sim_rviz.sh does not
/// implement quoting, so a correctly quoted field would break a live reader.
TEST(MetricsLoggerSchema, PursuitPeerIdCannotShiftTheRow) {
  StepMetrics ref;
  auto [ref_hdr, ref_row] = writeOne(ref, "pursuit_width_ref");

  StepMetrics m;
  m.pursue_peer = "bad,name\"with\nbreaks";
  auto [hdr, row] = writeOne(m, "pursuit_sanitize");

  EXPECT_EQ(row.size(), ref_row.size())
      << "a peer id with a separator must not change the column count";
  EXPECT_EQ(hdr.size(), row.size());
  auto v = byName(hdr, row);
  EXPECT_EQ(v["pursue_peer"], "bad_name_with_breaks");
}
