#include "explo_planner/metrics_logger.hpp"

namespace explo_planner {

MetricsLogger::MetricsLogger(const std::string& csv_path)
    : file_(csv_path, std::ios::out | std::ios::trunc) {}

MetricsLogger::~MetricsLogger() {
  if (file_.is_open()) file_.close();
}

void MetricsLogger::writeHeader() {
  file_ << "step,sim_time_sec,total_observed_voxels,frontier_voxels,"
        << "distance_traveled,selected_score,plan_time_ms,"
        << "mean_eig,mean_entropy,mean_variance,"
        << "mean_info_gain,mean_path_cost,"
        << "selected_info_gain,selected_path_cost,selected_utility,"
        << "coord_active_peers,rejected_by_minpos,rejected_by_unreachable\n";
  header_written_ = true;
}

void MetricsLogger::logStep(const StepMetrics& m) {
  if (!header_written_) writeHeader();
  file_ << m.step << ","
        << m.sim_time_sec << ","
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
        << m.rejected_by_unreachable << "\n";
  file_.flush();
}

} // namespace explo_planner
