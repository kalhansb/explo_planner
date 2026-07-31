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

  float angular_coverage      = 0.0f;  ///< filled azimuth sectors / K, measured
                         ///< about the FITTED axis (see axis_fitted). This is a
                         ///< surface-geometry proxy for "seen from all sides";
                         ///< the node can override it with the far more direct
                         ///< robot-bearing history (use_bearing_coverage).
  float mean_entropy          = 0.0f;  ///< mean occupancy entropy, /ln2.
  float vertical_completeness = 0.0f;  ///< filled height bins / total. NOTE:
                         ///< the bins span [base_z, top_z], and those bounds are
                         ///< themselves the min/max of the cluster, so the first
                         ///< and last bin are always filled and the span is
                         ///< self-normalising. It therefore only ever detects an
                         ///< INTERIOR gap (>= 1/n_height_bins of the height with
                         ///< no voxels at all), which a continuous trunk never
                         ///< has -- it read exactly 1.00 on all 17 trunks of the
                         ///< map-test-2 bag. It is reported for diagnostics but
                         ///< w_vertical defaults to 0: it is not a usable
                         ///< occlusion signal in its current form.
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
  int      min_trunk_voxels = 60;   ///< reject clusters thinner than this. The
                                    ///< old default of 8 admitted noise: on the
                                    ///< map-test-2 bag every real trunk carried
                                    ///< 112-717 trunk voxels while every false
                                    ///< positive carried 9-33, so this single
                                    ///< gate separates them cleanly. Scale it
                                    ///< down for coarser maps / smaller stems
                                    ///< (it counts voxels, not metres).
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
  //
  // Coverage measured from map geometry is only ever a PROXY, and a fragile one
  // -- it depends entirely on the axis estimate being the true axis rather than
  // the centroid of whatever happens to have been observed (see fitCrossSection).
  // The node's use_bearing_coverage measures the same quantity directly, from
  // the bearings the robot actually viewed the trunk from, and should be
  // preferred whenever a pose is available.
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

/// Combine the three information terms into the deficit using cfg's weights
/// (auto-normalised, so the result stays in [0, 1] however they are set).
/// Exposed because tree_detector_node re-scores a detection against a coverage
/// value the map alone cannot supply: the set of robot bearings a trunk has
/// actually been viewed from. Keeping one implementation means the node's
/// verdict and the detector's stay on the same scale.
float infoDeficit(const TreeDetectorConfig& cfg, float coverage,
                  float mean_entropy, float vertical);

// ===================================================================
// Emission gate (bearing coverage)
// ===================================================================
// Per-tree state behind tree_detector_node's decision to publish a TreeTarget.
// Lives here rather than in the node so the state machine is unit-testable
// without a ROS graph -- the node owns one gate per tracked tree and feeds it
// exactly one observation per scan.
//
// The gate answers a different question from the detector's `under_informed`.
// The detector scores a SNAPSHOT ("is this trunk under-observed right now?"),
// which under bearing coverage is trivially yes for every tree at first sight:
// a track's bearing history starts empty, so coverage reads 1/n_azimuth_bins
// and the deficit clears any sane threshold. Emitting there means "nominate
// every tree the instant it is detected", which makes deficit_thresh
// decorative. The gate instead waits until the bearing history has STOPPED
// GROWING -- the robot has finished passing this tree -- and only then asks
// whether it is still under-covered. A tree the robot happened to walk around
// closes itself and is never nominated.

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

/// Advance confirmation and decide whether to publish this tree now. Returns
/// true exactly once per tree, on the scan where all of these first hold:
///   * `under_informed` (on the coverage the caller scored the tree with);
///   * confirm_ticks consecutive under-informed scans -- a well-observed read
///     resets the streak, so a half-built trunk cannot fire on a fluke;
///   * the bearing history has been static for settle_ticks scans, i.e. the
///     robot has finished passing the tree. Skipped when `has_bearing` is false
///     (no pose, so the verdict came from map geometry and there is no bearing
///     history to settle) or settle_ticks <= 0 (deferral disabled).
/// Sets g.emitted on the true return, so emit-once needs no caller bookkeeping.
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
