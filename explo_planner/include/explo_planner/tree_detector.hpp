#pragma once
/// @file tree_detector.hpp
/// @brief Pure (no-ROS) tree-instance detector over a semantic-occupancy voxel
///        set. Segments vegetation trunks out of the dscovox map and scores how
///        well each has been observed, flagging the ones that lack enough
///        angular surface coverage to be considered "done". The exploitation
///        node turns each under-informed trunk into a TreeTarget, so this is the
///        real detector the time-based target_scheduler_node stands in for.
///
/// Deliberately ROS-free: it consumes a flat vector of SemVoxel (the node builds
/// those from a ScovoxMap) and returns TreeDetection structs, so the whole
/// segment + score pipeline is unit-tested against synthetic voxels
/// (test_tree_detector.cpp), matching the pure-core pattern of target_queue /
/// vantage_planner. Spatial queries use an internal integer-coord hash; no
/// Bonxai / MapCache dependency, because MapCache's UnifiedVoxel drops the
/// semantic class this detector keys on.

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace explo_planner {

/// One occupied voxel with the semantics the detector needs. The node builds
/// these from a ScovoxMap voxel: best_class = argmax over semantic_evidence,
/// class_conf = best_evidence / (sum_evidence + a_unk), evidence = a_occ+a_free,
/// p_occ = a_occ / (a_occ + a_free).
struct SemVoxel {
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();  ///< voxel centre, map frame.
  float    p_occ      = 0.5f;   ///< a_occ / (a_occ + a_free).
  float    evidence   = 2.0f;   ///< a_occ + a_free (occupancy evidence; prior 2).
  uint16_t best_class = 0;      ///< argmax semantic class id (5 = vegetation).
  float    class_conf = 0.0f;   ///< best-class evidence share in [0, 1].
};

/// A segmented tree instance plus its information-sufficiency assessment. All
/// three info metrics are in [0, 1]; info_deficit combines them so that higher
/// means "less observed / needs circling".
struct TreeDetection {
  Eigen::Vector3f center = Eigen::Vector3f::Zero();  ///< trunk axis XY, base z.
  float radius = 0.0f;   ///< estimated trunk radius (m).
  float height = 0.0f;   ///< observed vertical extent (m).
  int   trunk_voxels = 0;///< occupied vegetation voxels in the trunk band.

  float angular_coverage      = 0.0f;  ///< filled azimuth sectors / K.
  float mean_entropy          = 0.0f;  ///< mean occupancy entropy, /ln2.
  float vertical_completeness = 0.0f;  ///< filled height bins / total.
  float info_deficit          = 0.0f;  ///< combined "not enough info" score.

  bool  under_informed = false;        ///< info_deficit > cfg.deficit_thresh.
};

struct TreeDetectorConfig {
  double voxel_size = 0.15;   ///< grid resolution (m); neighbour-hash quantum.

  // --- Segmentation / gating ---
  uint16_t veg_class      = 5;      ///< vegetation/tree semantic id (palette).
  float    occ_thresh     = 0.6f;   ///< p_occ >= this => an occupied surface voxel.
  float    min_class_conf = 0.3f;   ///< voxel must be this confidently vegetation.
  float    cluster_tol_m  = 0.3f;   ///< connect cells within this range (m);
                                    ///< >1 voxel bridges thin-surface gaps.
  float    trunk_band_lo  = 0.5f;   ///< trunk band start above cluster base (m).
  float    trunk_band_hi  = 2.5f;   ///< trunk band end above cluster base (m).
  int      min_trunk_voxels = 8;    ///< reject clusters thinner than this.
  float    min_height     = 1.5f;   ///< reject clusters shorter than this (m).
  float    max_radius     = 1.0f;   ///< reject fat blobs (walls / hedges) (m).

  // --- Coverage / information ---
  // 8 sectors (45 deg). Kept coarse on purpose: a thin trunk only presents
  // ~2*pi*r/voxel distinct surface voxels around its circumference, so too many
  // bins can never all fill and a fully-circled trunk would keep a high deficit,
  // breaking the self-closing loop. 8 is fillable at typical trunk r / map res.
  int   n_azimuth_bins = 8;     ///< K sectors for angular coverage.
  int   n_height_bins  = 6;     ///< bins for vertical completeness.
  float w_coverage     = 0.60f; ///< info_deficit weights (auto-normalised).
  float w_entropy      = 0.25f;
  float w_vertical     = 0.15f;
  float deficit_thresh = 0.35f; ///< under_informed when info_deficit > this.
};

class TreeDetector {
public:
  explicit TreeDetector(const TreeDetectorConfig& cfg) : cfg_(cfg) {}

  /// Segment tree instances from `voxels` and score each. Returns every tree
  /// that passes the geometric gates (not just under-informed ones), sorted by
  /// descending info_deficit, so the caller can emit the neediest first and log
  /// the rest. `under_informed` is set per cfg.deficit_thresh.
  std::vector<TreeDetection> detect(const std::vector<SemVoxel>& voxels) const;

  const TreeDetectorConfig& config() const { return cfg_; }

private:
  TreeDetectorConfig cfg_;
};

}  // namespace explo_planner
