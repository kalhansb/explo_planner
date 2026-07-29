#pragma once
/// @file tree_detector.hpp
/// @brief Pure (no-ROS) tree-instance detector over a semantic-occupancy voxel
///        set. Segments vegetation trunks out of the dscovox map and scores how
///        well each has been observed, flagging the ones that lack enough
///        angular surface coverage to be considered "done". The exploitation
///        node turns each under-informed trunk into a TreeTarget, so this is the
///        real detector the time-based target_scheduler_node stands in for.
///
/// Two segmentation front-ends share one fit + scoring back-end:
///   * semantic (use_semantics = true, fused LiDAR+RGB-D maps): voxels are
///     gated on the vegetation class, so ground/walls/rocks never enter;
///   * geometric (use_semantics = false, LiDAR-only maps whose semantic
///     records are empty): terrain is estimated and removed (else the ground
///     plane connects every tree into one component), stems are clustered in a
///     height slice above local ground (trees separate at stem level even when
///     canopies touch), and the class gate is replaced by trunk-shape gates
///     (PCA linearity + verticality). Thin man-made verticals (posts, poles)
///     can pass the shape gates — geometric mode is a fallback for maps with
///     no semantics, not a replacement.
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
/// p_occ = a_occ / (a_occ + a_free). In geometric mode (LiDAR-only maps) the
/// semantic records are empty: best_class / class_conf stay 0 and are ignored.
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
  Eigen::Vector3f center = Eigen::Vector3f::Zero();  ///< trunk axis XY, base z
                         ///< (geometric mode: >= terrain + ground_margin_m, so
                         ///< ~0.4 m above the true base at defaults).
  float radius = 0.0f;   ///< estimated trunk radius (m).
  float height = 0.0f;   ///< observed vertical extent above base z (m); in
                         ///< geometric mode the stripped ground margin is not
                         ///< included, so min_height effectively gates
                         ///< min_height + ground_margin_m of true tree height.
  int   trunk_voxels = 0;///< occupied voxels in the trunk band (semantic mode:
                         ///< vegetation-class) / stem slice (geometric mode).

  float angular_coverage      = 0.0f;  ///< filled azimuth sectors / K.
  float mean_entropy          = 0.0f;  ///< mean occupancy entropy, /ln2.
  float vertical_completeness = 0.0f;  ///< filled height bins / total.
  float info_deficit          = 0.0f;  ///< combined "not enough info" score.

  bool  under_informed = false;        ///< info_deficit > cfg.deficit_thresh.
};

struct TreeDetectorConfig {
  double voxel_size = 0.15;   ///< grid resolution (m); neighbour-hash quantum.

  // --- Mode ---
  // true  => semantic front-end (veg_class / min_class_conf gates, trunk band
  //          relative to each cluster's base).
  // false => geometric front-end for LiDAR-only maps: occupancy-only gate plus
  //          the "Geometric mode" knobs below; veg_class / min_class_conf /
  //          trunk_band_* are ignored. Scoring and the info-deficit predicate
  //          are identical in both modes.
  bool use_semantics = true;

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

  // --- Geometric mode (use_semantics == false) only ---
  // Terrain: min occupied z per XY cell, median-filtered over the 3x3 cell
  // neighbourhood, bilinearly interpolated at query positions. Known
  // limitations: (a) grades beyond ~2*ground_margin_m/terrain_cell_m (~38 deg
  // at defaults) still leak ground voxels past the margin gate; (b) cells that
  // only ever saw canopy (never the ground under it) over-estimate terrain and
  // may cost the tree its lowest voxels; (c) two stems whose surfaces fall
  // within cluster_tol_m merge into one slice cluster that can fail the
  // linearity gate, dropping both (semantic mode returns one merged detection
  // instead); (d) a tree fragment separated from its stem by a map gap wider
  // than cluster_tol_m is not attached and goes uncounted.
  float terrain_cell_m  = 1.0f;   ///< XY cell of the min-z terrain grid (m).
  float ground_margin_m = 0.4f;   ///< drop voxels closer than this to terrain (m).
  float stem_slice_lo   = 0.5f;   ///< stem clustering slice above terrain (m);
  float stem_slice_hi   = 3.0f;   ///< clustering only here splits touching canopies.
  float attach_radius_m = 2.0f;   ///< canopy/lower-trunk voxels join a stem of
                                  ///< their connected component within this XY
                                  ///< radius of its axis.
  float min_linearity   = 0.55f;  ///< PCA (l_max-l_mid)/l_max: rejects isotropic
                                  ///< blobs (bushes). NOT what rejects walls — a
                                  ///< wall longer than its in-slice height reads
                                  ///< linear along its length; the tilt gate
                                  ///< below is what catches it.
  float max_tilt_deg    = 30.0f;  ///< stem principal axis max tilt from vertical
                                  ///< (this gate rejects walls, logs, ground
                                  ///< ribbons: their axis is horizontal).

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
