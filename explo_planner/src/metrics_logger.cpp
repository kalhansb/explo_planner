#include "explo_planner/metrics_logger.hpp"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

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

// Creates the missing folders above csv_path, so a new output folder works on
// first use. A failure here is left to the open, which reports it.
const std::string& withParentDirs(const std::string& csv_path) {
  const auto parent = std::filesystem::path(csv_path).parent_path();
  std::error_code ec;
  if (!parent.empty()) std::filesystem::create_directories(parent, ec);
  return csv_path;
}

}  // namespace

MetricsLogger::MetricsLogger(const std::string& csv_path)
    : file_(withParentDirs(csv_path), std::ios::out | std::ios::trunc) {
  // Fail loudly. std::ofstream does not throw by default, so an unwritable
  // output_csv (read-only mount, bad permissions, a parent that is a file)
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
        << "goal_outcome,nav_elapsed_sec,goal_path_length_m,"
        << "nav2_aborts,nav2_recoveries,nav2_error_code,"
        << "pick_fallback\n";
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
        << m.prox_hold_total_sec << ","
        << m.goal_outcome << ","
        << m.nav_elapsed_sec << ","
        << m.goal_path_length_m << ","
        << m.nav2_aborts << ","
        << m.nav2_recoveries << ","
        << m.nav2_error_code << ","
        << m.pick_fallback << "\n";
  file_.flush();
}

} // namespace explo_planner
