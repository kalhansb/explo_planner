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
/// Moved comments: doc/explo_planner_code_notes.md

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace explo_planner {

/// One occupied voxel as the detector needs it, built by the node from a
/// ScovoxMap voxel; class_conf = best_evidence / (sum_evidence + a_unk). In
/// geometric mode best_class and class_conf stay 0 and are ignored.
/// (notes: tree-semvoxel)
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

  float angular_coverage      = 0.0f;  ///< filled azimuth sectors / K, measured
                         ///< about the FITTED axis (see axis_fitted). This is a
                         ///< surface-geometry proxy for "seen from all sides";
                         ///< the node can override it with the far more direct
                         ///< robot-bearing history (use_bearing_coverage).
  float mean_entropy          = 0.0f;  ///< mean occupancy entropy, /ln2.
  float vertical_completeness = 0.0f;  ///< filled height bins / total. NOTE:
                         /// the bins span the cluster's own [base_z, top_z], so
                         /// the end bins are always filled and only an interior
                         /// gap registers. Diagnostic only; w_vertical defaults
                         /// to 0. (notes: tree-vertical-completeness)
  float info_deficit          = 0.0f;  ///< combined "not enough info" score.

  bool  under_informed = false;        ///< info_deficit > cfg.deficit_thresh.
  bool  axis_fitted    = false;        ///< true => centre/radius came from the
                         ///< circle fit; false => the fit was degenerate and the
                         ///< biased median fallback was used (see fitCrossSection
                         ///< in tree_detector.cpp), so angular_coverage on this
                         ///< detection is unreliable.
};

struct TreeDetectorConfig {
  double voxel_size = 0.15;   ///< grid resolution (m); neighbour-hash quantum.

  // --- Mode ---
  // true: semantic front-end (class gates, trunk band relative to each
  // cluster's base). false: geometric front-end for LiDAR-only maps; veg_class,
  // min_class_conf and trunk_band_* are ignored. Scoring is the same in both.
  // (notes: tree-detector-mode)
  bool use_semantics = true;

  // --- Segmentation / gating ---
  uint16_t veg_class      = 5;      ///< vegetation/tree semantic id (palette).
  float    occ_thresh     = 0.6f;   ///< p_occ >= this => an occupied surface voxel.
  float    min_class_conf = 0.3f;   ///< voxel must be this confidently vegetation.
  float    cluster_tol_m  = 0.3f;   ///< connect cells within this range (m);
                                    ///< >1 voxel bridges thin-surface gaps.
  float    trunk_band_lo  = 0.5f;   ///< trunk band start above cluster base (m).
  float    trunk_band_hi  = 2.5f;   ///< trunk band end above cluster base (m).
  int      min_trunk_voxels = 60;   ///< reject clusters thinner than this. The
                                    /// gate counts voxels, not metres: scale it
                                    /// down for coarser maps or smaller stems.
                                    /// (notes: tree-min-trunk-voxels)
  float    min_height     = 1.5f;   ///< reject clusters shorter than this (m).
  float    max_radius     = 1.0f;   ///< reject fat blobs (walls / hedges) (m).

  // --- Geometric mode (use_semantics == false) only ---
  // Terrain is the min occupied z per XY cell, 3x3 median-filtered, bilinearly
  // interpolated. Known limits: steep grades leak ground, canopy-only cells
  // overestimate terrain, and stems closer than cluster_tol_m merge.
  // (notes: tree-geometric-terrain)
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
  // Keep n_azimuth_bins coarse (8 x 45 deg): a thin trunk cannot fill many
  // bins, so a circled trunk would keep a high deficit. Map-geometry coverage
  // is only a proxy; prefer the node's use_bearing_coverage when a pose exists.
  // (notes: tree-azimuth-bins-coarse)
  int   n_azimuth_bins = 8;     ///< K sectors for angular coverage.
  int   n_height_bins  = 6;     ///< bins for vertical completeness.
  float w_coverage     = 0.75f; ///< info_deficit weights (auto-normalised).
  float w_entropy      = 0.25f;
  float w_vertical     = 0.00f; ///< 0 by design: vertical_completeness is
                                ///< structurally pinned at 1.0 (see the field
                                ///< doc on TreeDetection). Leaving it weighted
                                ///< only shrank the reachable deficit range.
  float deficit_thresh = 0.35f; ///< under_informed when info_deficit > this.
};

/// Combine the three information terms with cfg's auto-normalised weights
/// (result in [0, 1]). Shared with tree_detector_node, which re-scores with
/// bearing coverage, so both verdicts stay on one scale.
/// (notes: tree-info-deficit-shared)
float infoDeficit(const TreeDetectorConfig& cfg, float coverage,
                  float mean_entropy, float vertical);

// ===================================================================
// Emission gate (bearing coverage)
// ===================================================================
// Per-tree state behind tree_detector_node's TreeTarget publishing: one gate
// per tracked tree, fed exactly one observation per scan. Emits only once the
// bearing history stops growing and the tree is still under-covered.
// (notes: tree-emit-gate)

/// Per-tree emission bookkeeping. Default-constructed state means "seen from
/// nowhere, never confirmed, never published".
struct EmitGate {
  uint32_t bearing_mask = 0;  ///< azimuth sectors the ROBOT has viewed this
                              ///< tree from, one bit per n_azimuth_bins (<=32).
                              ///< Per-run state: unlike the map-geometry score
                              ///< it cannot be recomputed from a map snapshot.
  int  confirm = 0;           ///< consecutive under-informed scans.
  int  settle  = 0;           ///< consecutive scans bearing_mask did NOT grow.
  bool emitted = false;       ///< a TreeTarget has been published for this tree.
};

/// Bit for the azimuth sector the robot sits in as seen FROM the trunk. A trunk
/// viewed from the north and from the south lights two opposite bits; one seen
/// on a single pass lights the one or two sectors that pass swept.
uint32_t bearingBit(const Eigen::Vector2f& center, const Eigen::Vector2f& robot,
                    int n_bins);

/// Fraction of the n_bins azimuth sectors present in `mask`, in [0, 1].
float bearingCoverage(uint32_t mask, int n_bins);

/// Fold one scan's viewing bearing into `g` and return the resulting coverage.
/// Also advances the settle counter: reset to 0 when `bit` is a sector the tree
/// had not been viewed from before, incremented otherwise. Call exactly once
/// per scan per tracked tree, BEFORE stepEmitGate (which reads the counter).
float observeBearing(EmitGate& g, uint32_t bit, int n_bins);

/// Returns true once per tree, setting g.emitted, when under_informed has held
/// for confirm_ticks consecutive scans and, if has_bearing and settle_ticks >
/// 0, the bearing mask has not grown for settle_ticks scans.
/// (notes: tree-step-emit-gate)
bool stepEmitGate(EmitGate& g, bool under_informed, bool has_bearing,
                  int confirm_ticks, int settle_ticks);

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
