#include "explo_planner/metrics_logger.hpp"

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <stdexcept>

namespace explo_planner {

MetricsLogger::MetricsLogger(const std::string& csv_path)
    : file_(csv_path, std::ios::out | std::ios::trunc) {
  // Fail loudly. std::ofstream does not throw by default, so an unwritable
  // output_csv (missing parent directory, read-only mount, bad permissions)
  // used to leave every subsequent `file_ << ...` a silent no-op: the run
  // completed, the node logged "Step N logged" for every step, and the
  // experiment produced no data at all. There is nothing to salvage from a
  // metrics run whose metrics cannot be written, so refuse to start.
  if (!file_.is_open()) {
    throw std::runtime_error(
        "MetricsLogger: cannot open output CSV '" + csv_path + "': " +
        std::strerror(errno));
  }
}

MetricsLogger::~MetricsLogger() {
  if (file_.is_open()) file_.close();
}

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
        // Appended, never inserted. Every reader in this tree resolves columns
        // by header name — csv.DictReader in the python, and a header-scanning
        // awk in run_explo_sim_rviz.sh — so inserting would not in fact break
        // any of them; a comment here used to claim otherwise and was wrong.
        // The rule is kept anyway, for the readers that are NOT in this tree.
        // A campaign archive outlives the binary that wrote it and gets opened
        // by whatever is to hand months later, and a positional read of an
        // old file against a new schema does not fail, it silently returns the
        // wrong column. Appending costs one out-of-place block of names; that
        // is the whole price, and it is paid below three times over.
        << "state,reconnect_range_to_goal_m,reconnect_elapsed_sec,"
        << "unknown_fraction,coverage_source,"
        // Same rule: belongs beside mean_info_gain by meaning, but goes here
        // because the schema only ever grows at the right-hand end.
        << "info_gain_std,"
        // Same rule again (separation.hpp). These belong beside
        // rejected_by_minpos — they are the manipulation checks for the
        // mechanism that replaces it — and they are here instead.
        << "sep_peer_dist_m,sep_discount,sep_reordered,sep_eligible_peers\n";
  header_written_ = true;
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
        << m.sep_eligible_peers << "\n";
  file_.flush();
}

} // namespace explo_planner
