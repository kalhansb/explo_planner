// Moved comments: doc/explo_planner_code_notes.md
#include "explo_planner/metrics_logger.hpp"

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <stdexcept>

namespace explo_planner {

MetricsLogger::MetricsLogger(const std::string& csv_path)
    : path_(csv_path), file_(csv_path, std::ios::out | std::ios::trunc) {
  // Throws if the CSV cannot be opened: std::ofstream does not throw, so writes
  // would silently no-op. A clean open does not cover later writes;
  // noteStreamState() checks those. (notes: metrics-csv-open-fails-loudly)
  if (!file_.is_open()) {
    throw std::runtime_error(
        "MetricsLogger: cannot open output CSV '" + csv_path + "': " +
        std::strerror(errno));
  }
}

void MetricsLogger::noteStreamState(const char* where) {
  // Sticky and first-wins: once failbit or badbit is set the stream stays
  // failed, so only the first failure is recorded. Tests fail(), not bad(): a
  // formatting failure loses the row too. (notes: metrics-stream-state-sticky)
  if (!error_.empty()) return;
  if (!file_.fail()) return;
  error_ = std::string("MetricsLogger: ") + where + " write to '" + path_ +
           "' failed (" + std::strerror(errno) +
           "); CSV is truncated from this point on";
}

MetricsLogger::~MetricsLogger() {
  if (file_.is_open()) file_.close();
}

namespace {
/// Replaces anything that would change the column count. An empty string stays
/// empty: there is no dash sentinel for not pursuing.
/// (notes: metrics-sanitize-field)
std::string sanitizeField(const std::string& s) {
  std::string out = s;
  for (char& ch : out)
    if (ch == ',' || ch == '\n' || ch == '\r' || ch == '"') ch = '_';
  return out;
}
}  // namespace

void MetricsLogger::writeHeader() {
  file_ << "step,sim_time_sec,total_observed_voxels,frontier_voxels,"
        << "distance_traveled,selected_score,plan_time_ms,"
        << "mean_eig,mean_entropy,mean_variance,"
        << "mean_info_gain,mean_path_cost,"
        << "selected_info_gain,selected_path_cost,selected_utility,"
        << "coord_active_peers,rejected_by_minpos,rejected_by_unreachable,"
        << "phase,target_id,vantage_index,n_vantages_valid,"
        << "vantage_los_clear,dwell_sec,"
        << "prox_hold_count,prox_hold_total_sec,"
        // Columns are appended, never inserted. In-tree readers resolve columns
        // by header name, but a positional reader of an archived CSV would
        // silently read the wrong column. (notes: metrics-append-only-schema)
        << "state,reconnect_range_to_goal_m,reconnect_elapsed_sec,"
        << "unknown_fraction,coverage_source,"
        // Same rule: belongs beside mean_info_gain by meaning, but goes here
        // because the schema only ever grows at the right-hand end.
        << "info_gain_std,"
        // Same rule again (separation.hpp). These belong beside
        // rejected_by_minpos — they are the manipulation checks for the
        // mechanism that replaces it — and they are here instead.
        << "sep_peer_dist_m,sep_discount,sep_reordered,sep_eligible_peers,"
        // Same append-only rule. These belong next to rejected_by_unreachable
        // by meaning and are here instead. Unlike every other block in this
        // header they are populated on timer rows too — see the struct.
        << "plan_cand_total,plan_rej_close,plan_rej_map,plan_rej_unreach,"
        << "plan_rej_blacklist,plan_rej_minpos,plan_stall_ticks,"
        // R5, and the same append-only rule one more time: these belong beside
        // the reconnect_* block by meaning and are here instead.
        << "pursue_peer,pursue_quarry_live,team_complete,"
        // Belongs after plan_rej_blacklist by meaning (its visited half) but is
        // appended here under the append-only rule.
        // (notes: metrics-plan-rej-visited-placement)
        << "plan_rej_visited\n";
  header_written_ = true;
  // A header that did not reach the file is the one failure a by-name reader
  // cannot recover from at all -- the rows below are then a nameless matrix of
  // numbers -- so it is checked separately from the rows rather than waiting for
  // the first logStep().
  noteStreamState("header");
}

void MetricsLogger::logStep(const StepMetrics& m) {
  if (!header_written_) writeHeader();
  file_ << m.step << ","
        // Fixed-point, not the default 6 SIGNIFICANT digits: on hardware
        // sim_time_sec is a Unix epoch stamp (~1.7e9), where 6 sig figs is
        // ±1000 s — every row of an hour-long run collapses to the same
        // value and dt-based post-processing divides by zero.
        << std::fixed << std::setprecision(6) << m.sim_time_sec
        << std::defaultfloat << ","
        << m.total_observed_voxels << ","
        << m.frontier_voxels << ","
        << m.distance_traveled << ","
        << m.selected_score << ","
        << m.plan_time_ms << ","
        << m.mean_eig << ","
        << m.mean_entropy << ","
        << m.mean_variance << ","
        << m.mean_info_gain << ","
        << m.mean_path_cost << ","
        << m.selected_info_gain << ","
        << m.selected_path_cost << ","
        << m.selected_utility << ","
        << m.coord_active_peers << ","
        << m.rejected_by_minpos << ","
        << m.rejected_by_unreachable << ","
        << m.phase << ","
        << m.target_id << ","
        << m.vantage_index << ","
        << m.n_vantages_valid << ","
        << m.vantage_los_clear << ","
        << m.dwell_sec << ","
        << m.prox_hold_count << ","
        << m.prox_hold_total_sec << ","
        << m.state << ","
        << m.reconnect_range_to_goal_m << ","
        << m.reconnect_elapsed_sec << ","
        << m.unknown_fraction << ","
        << m.coverage_source << ","
        << m.info_gain_std << ","
        << m.sep_peer_dist_m << ","
        << m.sep_discount << ","
        << m.sep_reordered << ","
        << m.sep_eligible_peers << ","
        << m.plan_cand_total << ","
        << m.plan_rej_close << ","
        << m.plan_rej_map << ","
        << m.plan_rej_unreach << ","
        << m.plan_rej_blacklist << ","
        << m.plan_rej_minpos << ","
        << m.plan_stall_ticks << ","
        // pursue_peer is the only config-sourced free-text field, so a comma in
        // it would silently shift columns. Substituted, not quoted: the awk
        // header-scanner in run_explo_sim_rviz.sh does not handle quoting.
        // (notes: metrics-pursue-peer-sanitised)
        << sanitizeField(m.pursue_peer) << ","
        << m.pursue_quarry_live << ","
        << m.team_complete << ","
        << m.plan_rej_visited << "\n";
  file_.flush();
  // Checked after the flush: a row can sit whole in the buffer and fail only on
  // the way to the device (ENOSPC). Flushing per row is this class's contract,
  // so a killed run keeps its steps. (notes: metrics-check-after-flush)
  noteStreamState("row");
}

} // namespace explo_planner
