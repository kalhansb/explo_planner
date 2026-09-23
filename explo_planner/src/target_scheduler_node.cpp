/// @file target_scheduler_node.cpp
/// @brief Time-based tree-target publisher for perceptive exploitation.
///
/// The experiment uses a *preselected* list of tree targets released on a time
/// schedule. This node reads that list as parallel ROS parameter arrays and
/// publishes each as a TreeTarget on the shared targets topic when its release
/// time (relative to node start, in sim time) elapses.
///
/// A LIVE DETECTOR DOES EXIST — tree_detector_node.cpp, built by the same
/// package — and it publishes the identical message on the same topic, so the
/// explo_planner cannot tell them apart. The two are mutually exclusive
/// alternatives chosen by the `use_detector` launch argument, which defaults to
/// false. It is kept deliberately, not for want of a detector: the schedule is
/// an INPUT the experiment controls, so every arm and every seed meets the same
/// targets at the same times, which is what makes an exploitation contrast
/// attributable to the arm rather than to what each robot's own map happened to
/// segment.
///
/// NO CAMPAIGN RUNS EITHER ONE TODAY, and this said "this scheduler is what the
/// campaigns actually run" until 2026-09-18. That was true of the exploitation
/// campaigns and is false of every reconnection/comms campaign since: the
/// matrix requires pure exploration, so run_campaign.sh launches each cell with
/// EXPLOIT=0, and run_explo_sim_rviz.sh then sets exploitation_enabled:=false
/// and starts no target_scheduler at all (see the EXPLOIT block there for why
/// leaving it on would contaminate the endpoint — target detours of
/// arm-dependent length, and a /exploration/targets bus the radio emulator does
/// not relay). This node's own default is unchanged; what changed is which
/// experiment is being run. Read the cell's manifest, not this sentence.
///
/// It is C++ (not a Python one-shot) so use_sim_time / bag playback time is
/// handled the same way the planner handles it: release times are measured on
/// the node clock, which is sim time during bag playback.
///
/// Parameters (all arrays are indexed in parallel by target):
///   targets_topic        (string)        publish topic (default /exploration/targets)
///   frame_id             (string)        TreeTarget.header.frame_id (default "map")
///   target_ids           (int[])         stable per-target ids (required, drives count)
///   target_x/y/z         (double[])      trunk centre in frame_id (m)
///   target_radius        (double[])      trunk radius (m); <=0 => default standoff
///   target_height        (double[])      optional trunk extent (m); 0 if omitted
///   target_release_sec   (double[])      release time relative to start (s)
/// Moved comments: doc/explo_planner_code_notes.md

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <explo_planner_msgs/msg/tree_target.hpp>

namespace explo_planner {

class TargetSchedulerNode : public rclcpp::Node {
public:
  TargetSchedulerNode() : Node("target_scheduler") {
    topic_    = declare_parameter<std::string>("targets_topic",
                                               "/exploration/targets");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    ids_     = declare_parameter<std::vector<int64_t>>("target_ids",
                                                       std::vector<int64_t>{});
    xs_      = declare_parameter<std::vector<double>>("target_x",
                                                      std::vector<double>{});
    ys_      = declare_parameter<std::vector<double>>("target_y",
                                                      std::vector<double>{});
    zs_      = declare_parameter<std::vector<double>>("target_z",
                                                      std::vector<double>{});
    radii_   = declare_parameter<std::vector<double>>("target_radius",
                                                      std::vector<double>{});
    heights_ = declare_parameter<std::vector<double>>("target_height",
                                                      std::vector<double>{});
    release_ = declare_parameter<std::vector<double>>("target_release_sec",
                                                      std::vector<double>{});

    const size_t n = ids_.size();
    // Every parallel array must match the id array length; bail loudly rather
    // than publish a half-specified target.
    auto bad = [&](const char* name, size_t sz) {
      if (sz != n) {
        RCLCPP_ERROR(get_logger(),
            "%s has %zu entries but target_ids has %zu — disabling scheduler.",
            name, sz, n);
        return true;
      }
      return false;
    };
    if (n == 0) {
      RCLCPP_WARN(get_logger(),
          "No target_ids configured — scheduler will publish nothing.");
    }
    valid_ = !(bad("target_x", xs_.size()) || bad("target_y", ys_.size()) ||
               bad("target_z", zs_.size()) ||
               bad("target_radius", radii_.size()) ||
               bad("target_release_sec", release_.size()));
    // target_height is optional; default to zeros if omitted.
    if (valid_ && heights_.empty()) heights_.assign(n, 0.0);
    if (valid_ && heights_.size() != n) {
      RCLCPP_ERROR(get_logger(),
          "target_height has %zu entries but target_ids has %zu — disabling.",
          heights_.size(), n);
      valid_ = false;
    }

    published_.assign(n, false);

    // Latched + deep history so a planner that subscribes after several targets
    // were already released still receives them all. Matches the planner QoS.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local();
    pub_ = create_publisher<explo_planner_msgs::msg::TreeTarget>(topic_, qos);

    // start_time_ is latched in tick() on the first valid-clock tick, not here:
    // under use_sim_time the clock reads 0 before /clock arrives, which would
    // release every target at once. (notes: target-sched-start-latch)
    timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::milliseconds(500),
        [this] { tick(); });

    RCLCPP_INFO(get_logger(),
        "Target scheduler ready: %zu target(s) on %s (frame=%s).",
        n, topic_.c_str(), frame_id_.c_str());
  }

private:
  void tick() {
    if (!valid_) return;
    const rclcpp::Time now = this->now();
    // Latch the schedule origin on the first tick with a valid (non-zero) clock,
    // so release times are measured from when playback actually started — not
    // from the sim-time-0 epoch captured before /clock arrived.
    if (!start_latched_) {
      if (now.nanoseconds() <= 0) return;  // clock not up yet
      start_time_ = now;
      start_latched_ = true;
      RCLCPP_INFO(get_logger(),
          "Schedule origin latched at t=%.2fs; releasing on schedule.",
          now.seconds());
    }
    const double elapsed = (now - start_time_).seconds();
    for (size_t i = 0; i < ids_.size(); ++i) {
      if (published_[i]) continue;
      if (elapsed < release_[i]) continue;

      explo_planner_msgs::msg::TreeTarget msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      msg.target_id = static_cast<uint32_t>(ids_[i]);
      msg.center.x = xs_[i];
      msg.center.y = ys_[i];
      msg.center.z = zs_[i];
      msg.radius = static_cast<float>(radii_[i]);
      msg.height = static_cast<float>(heights_[i]);
      msg.discovered_by = "schedule";
      msg.status = explo_planner_msgs::msg::TreeTarget::STATUS_PENDING;
      pub_->publish(msg);
      published_[i] = true;

      RCLCPP_INFO(get_logger(),
          "Released target id=%u at (%.2f, %.2f, %.2f) r=%.2f "
          "(release=%.1fs, t=%.1fs).",
          msg.target_id, xs_[i], ys_[i], zs_[i], radii_[i], release_[i],
          elapsed);
    }
  }

  std::string topic_;
  std::string frame_id_;
  std::vector<int64_t> ids_;
  std::vector<double> xs_, ys_, zs_, radii_, heights_, release_;
  std::vector<bool> published_;
  bool valid_ = true;

  rclcpp::Time start_time_;
  bool start_latched_ = false;  // schedule origin latched on first valid-clock tick
  rclcpp::Publisher<explo_planner_msgs::msg::TreeTarget>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace explo_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explo_planner::TargetSchedulerNode>());
  rclcpp::shutdown();
  return 0;
}
