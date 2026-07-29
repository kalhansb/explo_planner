/// @file tree_detector_node.cpp
/// @brief Live tree detector for perceptive exploitation. Drop-in replacement
///        for target_scheduler_node: instead of releasing a preselected list on
///        a timer, it segments trees out of the fused dscovox map, scores how
///        well each has been observed, and publishes the UNDER-INFORMED ones as
///        TreeTarget on the same targets topic. The explo_planner is unchanged —
///        it ingests the identical message.
///
/// "Not enough info" is angular-coverage-primary (see tree_detector.hpp): a
/// trunk the map has only seen from one side scores a high info_deficit and is
/// emitted; once exploitation circles it the far-side sectors fill, the deficit
/// drops below threshold, and the detector stops nominating it — a self-closing
/// loop, since the detector's predicate is the very quantity the vantage circle
/// improves.
///
/// Stability for a live feed into the TargetQueue:
///   * stable ids     — the target id is derived from the trunk's quantised
///                      map-frame position (cellId), so the SAME tree gets the
///                      SAME id on every robot (all detectors run on the shared
///                      fused map) and across re-detections. The planner's
///                      id-keyed dedup and team dwell-credit therefore refer to
///                      one tree fleet-wide; a per-node counter would collide
///                      (two robots both starting at 1 for different trees). A
///                      persistent track table (nearest match within
///                      track_match_radius_m) still drives confirmation +
///                      emit-once;
///   * confirmation    — a tree must read under-informed for confirm_ticks
///                      consecutive scans before it is emitted; a well-observed
///                      read resets the streak (the map grows as the robot
///                      moves, so a half-built trunk must not fire);
///   * emit-once       — each track publishes one TreeTarget when confirmed
///                      (latched/transient_local QoS covers late-joining
///                      planners); it re-arms only after being lost for a while.
///
/// Parameters:
///   robot_name            (string) namespace for the default map topic.
///   dscovox_topic         (string) fused ScovoxMap in (default
///                                  /<robot>/dscovox_node/scovox).
///   targets_topic         (string) TreeTarget out (default /exploration/targets).
///   frame_id              (string) TreeTarget.header.frame_id (default "map").
///   scan_period_sec       (double) detector cadence (default 2.0).
///   confirm_ticks         (int)    consecutive under-informed scans to emit.
///   track_match_radius_m  (double) XY radius to match a detection to a track.
///   track_timeout_sec     (double) drop a track unseen this long (re-arms id).
///   id_cell_m             (double) XY grid (m) the position-derived id snaps to.
///   use_semantics         (bool)   false => geometric mode for LiDAR-only maps
///                                  (empty semantic records): terrain removal +
///                                  stem-shape gates replace the class gate.
///   targets_qos_depth     (int)    latched history depth on the targets topic;
///                                  must exceed the targets a run can emit.
///   plus the TreeDetectorConfig knobs (veg_class, occ_thresh, deficit_thresh…
///   and, in geometric mode, terrain_cell_m, ground_margin_m, stem_slice_lo/hi,
///   attach_radius_m, min_linearity, max_tilt_deg).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <scovox_msgs/msg/scovox_map.hpp>
#include <explo_planner_msgs/msg/tree_target.hpp>

#include "explo_planner/tree_detector.hpp"

namespace explo_planner {

class TreeDetectorNode : public rclcpp::Node {
public:
  TreeDetectorNode() : Node("tree_detector"), det_(loadConfig()) {
    robot_name_    = get_parameter("robot_name").as_string();
    targets_topic_ = declare_parameter<std::string>("targets_topic",
                                                     "/exploration/targets");
    frame_id_      = declare_parameter<std::string>("frame_id", "map");

    std::string map_topic = declare_parameter<std::string>("dscovox_topic", "");
    if (map_topic.empty())
      map_topic = "/" + robot_name_ + "/dscovox_node/scovox";

    const double scan_period = declare_parameter<double>("scan_period_sec", 2.0);
    confirm_ticks_       = declare_parameter<int>("confirm_ticks", 2);
    match_radius_        = declare_parameter<double>("track_match_radius_m", 1.5);
    track_timeout_       = declare_parameter<double>("track_timeout_sec", 30.0);
    id_cell_m_           = declare_parameter<double>("id_cell_m", 1.0);

    // Match the merger's latched publisher QoS so the current map is delivered
    // immediately on connect (same as the planner's own subscription).
    map_sub_ = create_subscription<scovox_msgs::msg::ScovoxMap>(
        map_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](scovox_msgs::msg::ScovoxMap::SharedPtr msg) { latest_map_ = msg; });

    // Latched + deep history so a planner that subscribes after the first
    // targets were emitted still receives them all. The depth must exceed the
    // number of targets a run can emit: geometric mode nominates every
    // vertical structure in the map, not just a preselected handful, so the
    // old fixed KeepLast(50) could silently drop early targets for
    // late-joining planners.
    const int qos_depth = static_cast<int>(
        declare_parameter<int>("targets_qos_depth", 500));
    auto qos = rclcpp::QoS(rclcpp::KeepLast(std::max(1, qos_depth)))
                   .reliable().transient_local();
    pub_ = create_publisher<explo_planner_msgs::msg::TreeTarget>(targets_topic_, qos);

    timer_ = rclcpp::create_timer(
        this, get_clock(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(scan_period)),
        [this] { scan(); });

    RCLCPP_INFO(get_logger(),
        "Tree detector ready (%s mode): map=%s targets=%s (frame=%s), "
        "deficit_thresh=%.2f confirm_ticks=%d.",
        det_.config().use_semantics ? "semantic" : "geometric",
        map_topic.c_str(), targets_topic_.c_str(), frame_id_.c_str(),
        det_.config().deficit_thresh, confirm_ticks_);
  }

private:
  struct Track {
    uint32_t id;
    Eigen::Vector3f center;
    float radius;
    float height;
    int   confirm = 0;        ///< consecutive under-informed scans.
    bool  emitted = false;
    rclcpp::Time last_seen;
  };

  TreeDetectorConfig loadConfig() {
    // robot_name is declared here (first param access) so the ctor can read it.
    robot_name_ = declare_parameter<std::string>("robot_name", "robot1");
    TreeDetectorConfig c;
    c.voxel_size      = declare_parameter<double>("voxel_size", c.voxel_size);
    c.veg_class       = static_cast<uint16_t>(
        declare_parameter<int>("veg_class", c.veg_class));
    c.occ_thresh      = declare_parameter<double>("occ_thresh", c.occ_thresh);
    c.min_class_conf  = declare_parameter<double>("min_class_conf", c.min_class_conf);
    c.cluster_tol_m   = declare_parameter<double>("cluster_tol_m", c.cluster_tol_m);
    c.trunk_band_lo   = declare_parameter<double>("trunk_band_lo", c.trunk_band_lo);
    c.trunk_band_hi   = declare_parameter<double>("trunk_band_hi", c.trunk_band_hi);
    c.min_trunk_voxels = declare_parameter<int>("min_trunk_voxels", c.min_trunk_voxels);
    c.min_height      = declare_parameter<double>("min_height", c.min_height);
    c.max_radius      = declare_parameter<double>("max_radius", c.max_radius);
    c.n_azimuth_bins  = declare_parameter<int>("n_azimuth_bins", c.n_azimuth_bins);
    c.n_height_bins   = declare_parameter<int>("n_height_bins", c.n_height_bins);
    c.w_coverage      = declare_parameter<double>("w_coverage", c.w_coverage);
    c.w_entropy       = declare_parameter<double>("w_entropy", c.w_entropy);
    c.w_vertical      = declare_parameter<double>("w_vertical", c.w_vertical);
    c.deficit_thresh  = declare_parameter<double>("deficit_thresh", c.deficit_thresh);
    c.use_semantics   = declare_parameter<bool>("use_semantics", c.use_semantics);
    c.terrain_cell_m  = declare_parameter<double>("terrain_cell_m", c.terrain_cell_m);
    c.ground_margin_m = declare_parameter<double>("ground_margin_m", c.ground_margin_m);
    c.stem_slice_lo   = declare_parameter<double>("stem_slice_lo", c.stem_slice_lo);
    c.stem_slice_hi   = declare_parameter<double>("stem_slice_hi", c.stem_slice_hi);
    c.attach_radius_m = declare_parameter<double>("attach_radius_m", c.attach_radius_m);
    c.min_linearity   = declare_parameter<double>("min_linearity", c.min_linearity);
    c.max_tilt_deg    = declare_parameter<double>("max_tilt_deg", c.max_tilt_deg);
    return c;
  }

  /// Convert the latest ScovoxMap into the detector's SemVoxel input. Semantic
  /// mode forwards only vegetation-class voxels (the dominant reduction) and
  /// the detector applies the occupancy / confidence gates. Geometric mode
  /// (LiDAR-only maps: semantic records empty, so the veg pre-filter would
  /// silently drop everything) forwards every likely-occupied voxel instead —
  /// the detector's terrain removal + shape gates do the reduction there.
  std::vector<SemVoxel> buildInput(const scovox_msgs::msg::ScovoxMap& m) const {
    std::vector<SemVoxel> in;
    in.reserve(m.voxels.size() / 4 + 1);

    if (!det_.config().use_semantics) {
      for (const auto& v : m.voxels) {
        const float N = v.a_occ + v.a_free;
        const float p = (N > 0.0f) ? v.a_occ / N : 0.5f;
        if (p < det_.config().occ_thresh) continue;  // skip the free-space bulk
        SemVoxel sv;
        sv.pos = Eigen::Vector3f(v.position.x, v.position.y, v.position.z);
        sv.p_occ = p;
        sv.evidence = N;
        in.push_back(sv);  // best_class / class_conf stay 0: unused in this mode
      }
      return in;
    }

    const uint16_t veg = det_.config().veg_class;
    for (const auto& v : m.voxels) {
      // Best semantic class = argmax evidence; confidence includes a_unk in the
      // denominator so a mostly-unknown voxel reads low.
      float sum_ev = v.a_unk;
      float best_ev = -1.0f;
      uint16_t best = 0;
      for (const auto& se : v.semantic_evidence) {
        sum_ev += se.evidence_count;
        if (se.evidence_count > best_ev) {
          best_ev = se.evidence_count;
          best = se.class_id;
        }
      }
      if (best != veg) continue;  // pre-filter to trees

      const float N = v.a_occ + v.a_free;
      SemVoxel sv;
      sv.pos = Eigen::Vector3f(v.position.x, v.position.y, v.position.z);
      sv.p_occ = (N > 0.0f) ? v.a_occ / N : 0.5f;
      sv.evidence = N;
      sv.best_class = best;
      sv.class_conf = (sum_ev > 0.0f) ? std::max(best_ev, 0.0f) / sum_ev : 0.0f;
      in.push_back(sv);
    }
    return in;
  }

  void scan() {
    if (!latest_map_ || latest_map_->voxels.empty()) return;
    const rclcpp::Time now = this->now();
    if (now.nanoseconds() <= 0) return;  // clock not up yet (sim time)

    // Track the merger's resolution so hashing matches the source grid.
    if (latest_map_->resolution > 0.0f) {
      TreeDetectorConfig c = det_.config();
      c.voxel_size = latest_map_->resolution;
      det_ = TreeDetector(c);
    }

    const auto input = buildInput(*latest_map_);
    const auto dets = det_.detect(input);

    // Heartbeat so a mode/topic misconfiguration (e.g. semantic mode on a
    // LiDAR-only map, where every voxel is dropped) is visible instead of the
    // node just never emitting anything.
    size_t needy = 0;
    for (const auto& d : dets) needy += d.under_informed ? 1 : 0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
        "scan (%s mode): %zu map voxels -> %zu candidate voxels -> %zu trees "
        "(%zu under-informed), %zu tracks.",
        det_.config().use_semantics ? "semantic" : "geometric",
        latest_map_->voxels.size(), input.size(), dets.size(), needy,
        tracks_.size());

    // Age out stale tracks (re-arms their id for a genuinely new tree later).
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if ((now - it->last_seen).seconds() > track_timeout_)
        it = tracks_.erase(it);
      else
        ++it;
    }

    // At most one detection may claim a given track per scan. Without this,
    // two trunks closer together than match_radius_ both resolved to the SAME
    // nearest track: `confirm` incremented twice inside a single scan (so
    // confirm_ticks=2 was satisfied after one scan rather than two in a row,
    // defeating the consecutive-confirmation requirement) and the second trunk
    // was silently absorbed into the first track's centre/radius estimate
    // instead of getting its own track and its own emitted target. Index-
    // aligned with tracks_; push_back below keeps both in step.
    std::vector<char> claimed(tracks_.size(), 0);

    for (const auto& d : dets) {
      // Match to the nearest unclaimed track within match_radius_ (XY). Done
      // for well-observed detections too, so a track whose tree now reads
      // adequately covered can have its consecutive-under-informed streak
      // reset below.
      Track* best = nullptr;
      size_t best_i = 0;
      float best_d2 = static_cast<float>(match_radius_ * match_radius_);
      for (size_t i = 0; i < tracks_.size(); ++i) {
        if (claimed[i]) continue;
        const float dx = tracks_[i].center.x() - d.center.x();
        const float dy = tracks_[i].center.y() - d.center.y();
        const float d2 = dx * dx + dy * dy;
        if (d2 <= best_d2) {
          best_d2 = d2;
          best = &tracks_[i];
          best_i = i;
        }
      }
      if (best) claimed[best_i] = 1;

      if (!d.under_informed) {
        // A well-observed read breaks the confirmation streak: reset the
        // consecutive counter so emission requires confirm_ticks scans IN A ROW,
        // not confirm_ticks reads scattered across the track's lifetime. Keep
        // the track alive (refresh last_seen) but never emit on this branch.
        if (best) {
          best->center = d.center;
          best->radius = d.radius;
          best->height = d.height;
          best->last_seen = now;
          best->confirm = 0;
        }
        continue;
      }

      if (!best) {
        tracks_.push_back(Track{cellId(d.center), d.center, d.radius, d.height, 1,
                                false, now});
        claimed.push_back(1);  // brand-new track: already taken this scan
        continue;
      }

      // Update the track with the freshest estimate + advance confirmation.
      best->center = d.center;
      best->radius = d.radius;
      best->height = d.height;
      best->last_seen = now;
      if (best->confirm < confirm_ticks_) ++best->confirm;

      if (!best->emitted && best->confirm >= confirm_ticks_) {
        publishTarget(*best, d, now);
        best->emitted = true;
      }
    }
  }

  // Deterministic target id from the trunk's map-frame XY, quantised to an
  // id_cell_m grid. Every robot's detector runs on the SAME fused dscovox map,
  // so a trunk at a given position hashes to the SAME id fleet-wide — which is
  // what the planner's id-keyed dedup and team dwell-credit need (a per-node
  // counter collides: two robots both start at 1 for different trees). Coarse
  // quantisation (≈ the planner's target_dedup_radius_m) absorbs small per-robot
  // centre-estimate differences; if two estimates still straddle a cell edge the
  // ids differ and credit just isn't shared for that trunk (safe degradation to
  // independent coverage — never a cross-tree mis-merge).
  uint32_t cellId(const Eigen::Vector3f& center) const {
    const double cell = (id_cell_m_ > 0.0) ? id_cell_m_ : 1.0;
    const int64_t cx = static_cast<int64_t>(std::llround(center.x() / cell));
    const int64_t cy = static_cast<int64_t>(std::llround(center.y() / cell));
    uint64_t h = static_cast<uint64_t>(cx) * 73856093ull ^
                 static_cast<uint64_t>(cy) * 19349663ull;
    return static_cast<uint32_t>(h ^ (h >> 32));
  }

  void publishTarget(const Track& t, const TreeDetection& d,
                     const rclcpp::Time& now) {
    explo_planner_msgs::msg::TreeTarget msg;
    msg.header.stamp = now;
    msg.header.frame_id = frame_id_;
    msg.target_id = t.id;
    msg.center.x = t.center.x();
    msg.center.y = t.center.y();
    msg.center.z = t.center.z();
    msg.radius = t.radius;
    msg.height = t.height;
    msg.discovered_by = robot_name_;
    msg.status = explo_planner_msgs::msg::TreeTarget::STATUS_PENDING;
    pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
        "Emitted target id=%u at (%.2f, %.2f, %.2f) r=%.2f h=%.1f "
        "[deficit=%.2f cov=%.2f ent=%.2f vert=%.2f, %d trunk voxels].",
        t.id, t.center.x(), t.center.y(), t.center.z(), t.radius, t.height,
        d.info_deficit, d.angular_coverage, d.mean_entropy,
        d.vertical_completeness, d.trunk_voxels);
  }

  // Params / wiring.
  std::string robot_name_;
  std::string targets_topic_;
  std::string frame_id_;
  int    confirm_ticks_ = 2;
  double match_radius_  = 1.5;
  double track_timeout_ = 30.0;
  double id_cell_m_     = 1.0;

  TreeDetector det_;
  std::vector<Track> tracks_;

  scovox_msgs::msg::ScovoxMap::SharedPtr latest_map_;
  rclcpp::Subscription<scovox_msgs::msg::ScovoxMap>::SharedPtr map_sub_;
  rclcpp::Publisher<explo_planner_msgs::msg::TreeTarget>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace explo_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explo_planner::TreeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
