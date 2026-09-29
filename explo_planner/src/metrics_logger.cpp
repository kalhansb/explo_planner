#include "explo_planner/metrics_logger.hpp"

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace explo_planner {

namespace {

// sim_time_sec is this->now().seconds(), which with the shipped
// use_sim_time: false is a wall-clock epoch (~1.76e9). At the stream's default
// 6 significant digits every row of a run prints as "1.75891e+09" — ~1000 s of
// effective resolution, i.e. one identical timestamp for the whole experiment.
// Format it here instead of touching the stream's precision, which would then
// apply to every float column after it.
std::string epochSeconds(double t) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3) << t;
  return ss.str();
}

}  // namespace

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
        << "prox_hold_count,prox_hold_total_sec\n";
  header_written_ = true;
}

void MetricsLogger::logStep(const StepMetrics& m) {
  if (!header_written_) writeHeader();
  file_ << m.step << ","
        << epochSeconds(m.sim_time_sec) << ","
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
        << m.prox_hold_total_sec << "\n";
  file_.flush();
}

} // namespace explo_planner
