/// @file tree_detector.cpp
/// @brief Implementation of the pure tree-instance detector. Two segmentation
///        front-ends (semantic for fused maps, geometric for LiDAR-only maps)
///        feed one shared trunk-fit + information-scoring back-end. See the
///        header for the "not enough info" rationale (angular coverage primary)
///        and the geometric-mode design.
/// Moved comments: doc/tree_detector_notes.md

#include "explo_planner/tree_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigenvalues>

namespace explo_planner {
namespace {

/// Integer voxel coordinate, used as the clustering hash key.
struct Coord {
  int32_t x, y, z;
  bool operator==(const Coord& o) const {
    return x == o.x && y == o.y && z == o.z;
  }
};
struct CoordHash {
  size_t operator()(const Coord& c) const noexcept {
    // Cheap spatial hash; the three large odd multipliers spread adjacent
    // coords across buckets well enough for the modest voxel counts here.
    uint64_t h = static_cast<uint32_t>(c.x) * 73856093u ^
                 static_cast<uint32_t>(c.y) * 19349663u ^
                 static_cast<uint32_t>(c.z) * 83492791u;
    return static_cast<size_t>(h);
  }
};

/// Voxel size to divide by, with the same 0.15 fallback fitAndScore() uses. A
/// non-positive resolution would make the int32 casts undefined; the node does
/// not clamp voxel_size. (notes: tree-safe-res)
double safeRes(double res) { return (res > 0.0) ? res : 0.15; }

Coord toCoord(const Eigen::Vector3f& p, double res) {
  // floor, not round: a regular voxel lattice maps to consecutive integers for
  // any grid phase offset, with no gaps or collisions. lround would open a
  // one-cell gap at the zero crossing (…-1, [no 0], 1…), splitting a straddling
  // object into two clusters and corrupting the radius fit.
  const double inv = 1.0 / safeRes(res);
  return Coord{static_cast<int32_t>(std::floor(p.x() * inv)),
               static_cast<int32_t>(std::floor(p.y() * inv)),
               static_cast<int32_t>(std::floor(p.z() * inv))};
}

/// Binary occupancy entropy of p normalised to [0, 1], p clamped off 0 and 1.
/// Non-finite p reads 1.0: a NaN would reach info_deficit, invert
/// under_informed and break the sort ordering. (notes: tree-norm-entropy-nan)
float normEntropy(float p) {
  if (!std::isfinite(p)) return 1.0f;
  p = std::min(std::max(p, 1e-4f), 1.0f - 1e-4f);
  const float h = -p * std::log(p) - (1.0f - p) * std::log(1.0f - p);
  return h / static_cast<float>(M_LN2);
}

/// Result of the trunk cross-section fit. `offset` moves a per-layer median
/// reference onto the true axis; `radius` is the fitted circle radius.
struct AxisFit {
  bool ok = false;
  Eigen::Vector2f offset = Eigen::Vector2f::Zero();
  float radius = 0.0f;
};

/// Taubin circle fit over residuals pooled across z-layers, each relative to
/// its own layer median; a one-sided arc's centre lands behind the arc.
/// ok=false on a collinear or runaway fit; the caller uses the median.
/// (notes: tree-cross-section-fit)
AxisFit fitCrossSection(const TreeDetectorConfig& cfg,
                        const std::vector<Eigen::Vector2f>& res) {
  AxisFit f;
  const size_t n = res.size();
  if (n < 6) return f;

  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& p : res) mean += Eigen::Vector2d(p.x(), p.y());
  mean /= static_cast<double>(n);

  // Taubin, not Kasa: Kasa pulls the centre toward a partial arc and shrinks
  // the radius, and a one-sided trunk is a partial arc.
  // (notes: tree-taubin-not-kasa)
  double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;
  for (const auto& p : res) {
    const double u = static_cast<double>(p.x()) - mean.x();
    const double v = static_cast<double>(p.y()) - mean.y();
    const double zz = u * u + v * v;
    Mxx += u * u;  Myy += v * v;  Mxy += u * v;
    Mxz += u * zz; Myz += v * zz; Mzz += zz * zz;
  }
  const double dn = static_cast<double>(n);
  Mxx /= dn;  Myy /= dn;  Mxy /= dn;  Mxz /= dn;  Myz /= dn;  Mzz /= dn;

  const double Mz = Mxx + Myy;
  const double cov_xy = Mxx * Myy - Mxy * Mxy;
  const double var_z = Mzz - Mz * Mz;

  // Scale-aware degeneracy test: a collinear residual set (a flat wall patch)
  // has cov_xy ~ 0 and no recoverable centre. Comparing against Mz^2 keeps the
  // test invariant to the point count and to the units of the map.
  if (!(std::abs(cov_xy) > 1e-6 * Mz * Mz) || Mz <= 0.0) return f;

  const double A3 = 4.0 * Mz;
  const double A2 = -3.0 * Mz * Mz - Mzz;
  const double A1 = var_z * Mz + 4.0 * cov_xy * Mz - Mxz * Mxz - Myz * Myz;
  const double A0 = Mxz * (Mxz * Myy - Myz * Mxy) +
                    Myz * (Myz * Mxx - Mxz * Mxy) - var_z * cov_xy;
  const double A22 = A2 + A2;
  const double A33 = A3 + A3 + A3;

  // Newton from x = 0 on the characteristic polynomial; converges in a handful
  // of steps for any well-posed set, and the guards below catch the rest.
  double x = 0.0, y = A0;
  for (int it = 0; it < 100; ++it) {
    const double dy = A1 + x * (A22 + A33 * x);
    if (dy == 0.0) break;
    const double xn = x - y / dy;
    if (xn == x || !std::isfinite(xn)) break;
    const double yn = A0 + xn * (A1 + xn * (A2 + xn * A3));
    if (std::abs(yn) >= std::abs(y)) break;
    x = xn;
    y = yn;
  }

  const double det = x * x - x * Mz + cov_xy;
  if (!std::isfinite(det) || std::abs(det) < 1e-12) return f;
  const double uc = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.0;
  const double vc = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.0;

  const double r2 = uc * uc + vc * vc + Mz;
  if (!std::isfinite(r2) || r2 <= 0.0) return f;
  const double r = std::sqrt(r2);

  const Eigen::Vector2f off(static_cast<float>(mean.x() + uc),
                            static_cast<float>(mean.y() + vc));
  // Both bounds mean "the fit ran away" rather than "the tree is too fat" --
  // the max_radius rejection is the caller's job, on whichever estimate wins.
  const float lim = 2.0f * cfg.max_radius;
  if (!std::isfinite(r) || static_cast<float>(r) > lim) return f;
  if (!off.allFinite() || off.norm() > lim) return f;

  f.ok = true;
  f.offset = off;
  f.radius = static_cast<float>(r);
  return f;
}

float medianOf(std::vector<float>& v) {
  if (v.empty()) return 0.0f;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

/// Half-width in cells of the candidate cube the flood-fill scans: a prefilter
/// only, connectionRadiusSqCells() decides joins. The cube always encloses that
/// ball, so prefiltering loses nothing. (notes: tree-connection-cube)
int connectionRadius(const TreeDetectorConfig& cfg) {
  return std::max(1, static_cast<int>(std::lround(
                         cfg.cluster_tol_m / safeRes(cfg.voxel_size))));
}

/// Squared join radius in cells: the larger of the cluster_tol_m ball and the
/// cube's inscribed ball, so the merge distance is the same on every bearing.
/// Floored at 3 so R = 1 keeps 26-connectivity.
/// (notes: tree-connection-radius-euclidean)
float connectionRadiusSqCells(const TreeDetectorConfig& cfg) {
  const double r = cfg.cluster_tol_m / safeRes(cfg.voxel_size);
  const double cube = static_cast<double>(connectionRadius(cfg));
  return static_cast<float>(std::max({r * r, cube * cube, 3.0}));
}

/// Flood-fill labelling of sel, joining cells within squared radius r2_cells
/// (in cells); the (2R+1)^3 window only enumerates candidates. sel must hold at
/// most one voxel per cell. (notes: tree-cluster-voxels)
std::vector<std::vector<int>> clusterVoxels(const std::vector<SemVoxel>& voxels,
                                            const std::vector<int>& sel,
                                            double res, int R,
                                            float r2_cells) {
  std::unordered_map<Coord, int, CoordHash> grid;  // coord -> position in sel
  grid.reserve(sel.size() * 2);
  for (size_t k = 0; k < sel.size(); ++k)
    grid.emplace(toCoord(voxels[sel[k]].pos, res), static_cast<int>(k));

  std::vector<int> label(sel.size(), -1);
  int n_clusters = 0;
  std::vector<int> stack;
  for (size_t seed = 0; seed < sel.size(); ++seed) {
    if (label[seed] != -1) continue;
    const int cid = n_clusters++;
    stack.clear();
    stack.push_back(static_cast<int>(seed));
    label[seed] = cid;
    while (!stack.empty()) {
      const int cur = stack.back();
      stack.pop_back();
      const Coord c = toCoord(voxels[sel[cur]].pos, res);
      for (int dx = -R; dx <= R; ++dx)
        for (int dy = -R; dy <= R; ++dy)
          for (int dz = -R; dz <= R; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            // The cube enumerates candidates; this Euclidean test decides. It
            // stays before the hash lookup so rejected corners cost no find().
            // (notes: tree-euclidean-join-test)
            const float d2 = static_cast<float>(dx * dx + dy * dy + dz * dz);
            if (d2 > r2_cells) continue;
            auto it = grid.find(Coord{c.x + dx, c.y + dy, c.z + dz});
            if (it == grid.end()) continue;
            const int nb = it->second;
            if (label[nb] != -1) continue;
            label[nb] = cid;
            stack.push_back(nb);
          }
    }
  }

  std::vector<std::vector<int>> clusters(n_clusters);
  for (size_t k = 0; k < sel.size(); ++k)
    clusters[label[k]].push_back(sel[k]);
  return clusters;
}

/// Shared back-end: trunk axis/radius fit + the three information metrics.
/// `trunk` drives the axis, radius, angular coverage and entropy; `members`
/// (trunk ⊆ members) drives base z, height and vertical completeness. Returns
/// nullopt when a geometric gate (trunk size, height, radius) rejects.
std::optional<TreeDetection> fitAndScore(const TreeDetectorConfig& cfg,
                                         const std::vector<SemVoxel>& voxels,
                                         const std::vector<int>& trunk,
                                         const std::vector<int>& members) {
  if (static_cast<int>(trunk.size()) < cfg.min_trunk_voxels)
    return std::nullopt;

  float base_z = std::numeric_limits<float>::max();
  float top_z  = std::numeric_limits<float>::lowest();
  for (int idx : members) {
    base_z = std::min(base_z, voxels[idx].pos.z());
    top_z  = std::max(top_z, voxels[idx].pos.z());
  }
  const float height = top_z - base_z;
  if (height < cfg.min_height) return std::nullopt;

  // --- Trunk axis + radius -------------------------------------------------
  // Each z-layer is referenced to its OWN median XY, so superimposing the
  // layers cancels axis tilt or bend and the pooled residuals form a single
  // cross-section that fitCrossSection can fit a circle to.
  const double vs = (cfg.voxel_size > 0.0) ? cfg.voxel_size : 0.15;
  std::unordered_map<int32_t, std::vector<float>> layer_x, layer_y;
  for (int idx : trunk) {
    const int32_t k = static_cast<int32_t>(std::floor(voxels[idx].pos.z() / vs));
    layer_x[k].push_back(voxels[idx].pos.x());
    layer_y[k].push_back(voxels[idx].pos.y());
  }
  std::unordered_map<int32_t, Eigen::Vector2f> layer_ref;
  layer_ref.reserve(layer_x.size());
  std::vector<float> ref_xs, ref_ys;
  ref_xs.reserve(layer_x.size());
  ref_ys.reserve(layer_x.size());
  for (auto& kv : layer_x) {
    const float mx = medianOf(kv.second);
    const float my = medianOf(layer_y[kv.first]);
    layer_ref.emplace(kv.first, Eigen::Vector2f(mx, my));
    ref_xs.push_back(mx);
    ref_ys.push_back(my);
  }

  std::vector<Eigen::Vector2f> residuals;
  residuals.reserve(trunk.size());
  for (int idx : trunk) {
    const int32_t k = static_cast<int32_t>(std::floor(voxels[idx].pos.z() / vs));
    const Eigen::Vector2f& r = layer_ref[k];
    residuals.emplace_back(voxels[idx].pos.x() - r.x(),
                           voxels[idx].pos.y() - r.y());
  }
  const AxisFit fit = fitCrossSection(cfg, residuals);

  float cx, cy, radius;
  if (fit.ok) {
    // Median layer reference shifted onto the fitted axis.
    cx = medianOf(ref_xs) + fit.offset.x();
    cy = medianOf(ref_ys) + fit.offset.y();

    // Centre from the fit, radius from the median distance to it, not
    // fit.radius, which clutter in the cluster inflates. The planner's vantage
    // standoff is radius + vantage_standoff_m. (notes: tree-radius-from-median)
    std::vector<float> dists;
    dists.reserve(trunk.size());
    for (int idx : trunk) {
      const int32_t k =
          static_cast<int32_t>(std::floor(voxels[idx].pos.z() / vs));
      const Eigen::Vector2f& r = layer_ref[k];
      const float dx = voxels[idx].pos.x() - (r.x() + fit.offset.x());
      const float dy = voxels[idx].pos.y() - (r.y() + fit.offset.y());
      dists.push_back(std::sqrt(dx * dx + dy * dy));
    }
    radius = medianOf(dists);
  } else {
    // Degenerate fit: fall back to the per-component median centre and
    // median-distance radius. That is biased for a one-sided trunk, and so is
    // angular_coverage, hence axis_fitted=false. (notes: tree-median-fallback)
    std::vector<float> xs, ys;
    xs.reserve(trunk.size());
    ys.reserve(trunk.size());
    for (int idx : trunk) {
      xs.push_back(voxels[idx].pos.x());
      ys.push_back(voxels[idx].pos.y());
    }
    cx = medianOf(xs);
    cy = medianOf(ys);

    std::vector<float> dists;
    dists.reserve(trunk.size());
    for (int idx : trunk) {
      const float dx = voxels[idx].pos.x() - cx;
      const float dy = voxels[idx].pos.y() - cy;
      dists.push_back(std::sqrt(dx * dx + dy * dy));
    }
    radius = medianOf(dists);
  }
  // Reject fat blobs (walls, hedges) that reach this far only because they are
  // not trees.
  radius = std::max(radius, 0.5f * static_cast<float>(cfg.voxel_size));
  if (radius > cfg.max_radius) return std::nullopt;

  const float two_pi = 2.0f * static_cast<float>(M_PI);

  // --- Angular coverage (primary info metric) ---
  // Bin observed trunk voxels by azimuth around the axis; coverage is the
  // fraction of sectors that hold at least one voxel. A trunk seen from one
  // side fills only the sectors on that arc; the occluded far side is absent
  // and reads as empty sectors -- exactly what the vantage circle then fills.
  //
  // n_az is floored at 1 locally, as bearingBit() does: zero bins would index
  // out of bounds and make coverage NaN, which inverts under_informed and
  // breaks the sort in detect(). (notes: tree-azimuth-bins-floor)
  const int n_az = std::max(cfg.n_azimuth_bins, 1);
  std::vector<char> az_hit(n_az, 0);
  float entropy_sum = 0.0f;
  for (int idx : trunk) {
    // Bin about the axis AT THIS VOXEL'S HEIGHT (layer reference + the fitted
    // offset), so a leaning trunk does not smear its own azimuths across
    // sectors it was never seen from. Without a fit there is only the one
    // biased centre to bin about.
    float ax = cx, ay = cy;
    if (fit.ok) {
      const int32_t k =
          static_cast<int32_t>(std::floor(voxels[idx].pos.z() / vs));
      const Eigen::Vector2f& r = layer_ref[k];
      ax = r.x() + fit.offset.x();
      ay = r.y() + fit.offset.y();
    }
    const float dx = voxels[idx].pos.x() - ax;
    const float dy = voxels[idx].pos.y() - ay;
    entropy_sum += normEntropy(voxels[idx].p_occ);

    // A voxel sitting near the axis has no meaningful bearing from it: a few
    // centimetres of noise swing its azimuth through a half-turn, so it lights
    // an arbitrary sector and inflates coverage. Only voxels out on the surface
    // carry direction information.
    if (dx * dx + dy * dy < (0.35f * radius) * (0.35f * radius)) continue;

    float a = std::atan2(dy, dx);
    if (a < 0.0f) a += two_pi;
    int b = static_cast<int>(a / two_pi * n_az);
    if (b >= n_az) b = n_az - 1;
    az_hit[b] = 1;
  }
  int filled = 0;
  for (char h : az_hit) filled += h ? 1 : 0;
  const float coverage = static_cast<float>(filled) / static_cast<float>(n_az);

  // --- Occupancy entropy (secondary) --- thin / freshly-seen surface sits
  // near the Beta prior (p~0.5) and reads high; a well-hit surface reads ~0.
  //
  // An empty trunk (possible when min_trunk_voxels is 0) gets mean_entropy 0
  // instead of a 0/0 NaN that would reach under_informed and the sort.
  // (notes: tree-empty-trunk-entropy)
  const float mean_entropy =
      trunk.empty() ? 0.0f
                    : entropy_sum / static_cast<float>(trunk.size());

  // --- Vertical completeness (secondary) --- gaps between base and canopy
  // (e.g. mid-trunk occluded) show up as empty height bins.
  // n_z is floored at 1 for the same reasons as n_az.
  // (notes: tree-height-bins-floor)
  const int n_z = std::max(cfg.n_height_bins, 1);
  std::vector<char> z_hit(n_z, 0);
  const float z_span = std::max(height, 1e-3f);
  for (int idx : members) {
    int b = static_cast<int>((voxels[idx].pos.z() - base_z) / z_span * n_z);
    if (b < 0) b = 0;
    if (b >= n_z) b = n_z - 1;
    z_hit[b] = 1;
  }
  int z_filled = 0;
  for (char h : z_hit) z_filled += h ? 1 : 0;
  const float vertical = static_cast<float>(z_filled) / static_cast<float>(n_z);

  const float deficit = infoDeficit(cfg, coverage, mean_entropy, vertical);

  TreeDetection d;
  d.axis_fitted = fit.ok;
  d.center = Eigen::Vector3f(cx, cy, base_z);
  d.radius = radius;
  d.height = height;
  d.trunk_voxels = static_cast<int>(trunk.size());
  d.angular_coverage = coverage;
  d.mean_entropy = mean_entropy;
  d.vertical_completeness = vertical;
  d.info_deficit = deficit;
  d.under_informed = deficit > cfg.deficit_thresh;
  return d;
}

/// Semantic front-end (fused LiDAR+RGB-D maps): gate on the vegetation class,
/// cluster everything that survives, and take the trunk as a band above each
/// cluster's base.
std::vector<TreeDetection> detectSemantic(const TreeDetectorConfig& cfg,
                                          const std::vector<SemVoxel>& voxels) {
  std::vector<TreeDetection> out;

  // 1. Keep only occupied, confidently-vegetation voxels. These are the trunk +
  //    canopy surface points; unseen (e.g. occluded far-side) space is simply
  //    absent from the map, which is exactly what the coverage metric reads.
  std::vector<int> veg;  // indices into `voxels`
  veg.reserve(voxels.size());
  std::unordered_map<Coord, char, CoordHash> seen;
  seen.reserve(voxels.size() * 2);
  for (int i = 0; i < static_cast<int>(voxels.size()); ++i) {
    const SemVoxel& v = voxels[i];
    if (v.best_class != cfg.veg_class) continue;
    if (v.p_occ < cfg.occ_thresh) continue;
    if (v.class_conf < cfg.min_class_conf) continue;
    // One representative voxel per cell: a real ScovoxMap holds a single voxel
    // per coord, but keeping this explicit stops any aliased duplicates from
    // seeding phantom singleton clusters.
    auto [it, inserted] = seen.emplace(toCoord(v.pos, cfg.voxel_size), 1);
    (void)it;
    if (!inserted) continue;
    veg.push_back(i);
  }
  if (veg.empty()) return out;

  // 2. Connected-component labelling over the vegetation voxels; one component
  // is about one tree. Touching canopies can merge two trees into one component
  // (a known limitation of this mode). (notes: tree-semantic-components)
  const auto clusters =
      clusterVoxels(voxels, veg, cfg.voxel_size, connectionRadius(cfg),
                    connectionRadiusSqCells(cfg));

  // 3. Fit + score each cluster.
  for (const auto& members : clusters) {
    if (static_cast<int>(members.size()) < cfg.min_trunk_voxels) continue;

    float base_z = std::numeric_limits<float>::max();
    for (int idx : members) base_z = std::min(base_z, voxels[idx].pos.z());

    // Trunk band: voxels just above the cluster base. Isolates the narrow trunk
    // from the wide canopy so the axis/radius fit is not blown out by foliage.
    // Falls back to the whole cluster if the band is empty (stubby vegetation).
    const float band_lo = base_z + cfg.trunk_band_lo;
    const float band_hi = base_z + cfg.trunk_band_hi;
    std::vector<int> trunk;
    for (int idx : members) {
      const float z = voxels[idx].pos.z();
      if (z >= band_lo && z <= band_hi) trunk.push_back(idx);
    }
    if (static_cast<int>(trunk.size()) < cfg.min_trunk_voxels) trunk = members;

    if (auto d = fitAndScore(cfg, voxels, trunk, members)) out.push_back(*d);
  }
  return out;
}

/// Geometric front-end (LiDAR-only maps): remove terrain (else the ground joins
/// everything), cluster stems in a slice above local ground, shape-gate them by
/// PCA, then attach the rest of each tree. (notes: tree-geometric-front-end)
std::vector<TreeDetection> detectGeometric(const TreeDetectorConfig& cfg,
                                           const std::vector<SemVoxel>& voxels) {
  std::vector<TreeDetection> out;

  // 1. Occupancy gate + one-voxel-per-cell dedup (class fields are ignored —
  //    a LiDAR-only map leaves them 0).
  std::vector<int> occ;  // indices into `voxels`
  occ.reserve(voxels.size());
  std::unordered_map<Coord, char, CoordHash> seen;
  seen.reserve(voxels.size() * 2);
  for (int i = 0; i < static_cast<int>(voxels.size()); ++i) {
    if (voxels[i].p_occ < cfg.occ_thresh) continue;
    auto [it, inserted] =
        seen.emplace(toCoord(voxels[i].pos, cfg.voxel_size), 1);
    (void)it;
    if (!inserted) continue;
    occ.push_back(i);
  }
  if (occ.empty()) return out;

  // 2. Terrain per XY cell = min occupied z, median-filtered over the 3x3 cell
  //    neighbourhood so a single mid-air voxel (canopy edge, noise) cannot
  //    punch a false valley or peak into the ground estimate.
  const double cell = std::max(static_cast<double>(cfg.terrain_cell_m),
                               cfg.voxel_size);
  const auto cellKey = [](int32_t cx, int32_t cy) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
           static_cast<uint64_t>(static_cast<uint32_t>(cy));
  };
  const auto cellOf = [&](const Eigen::Vector3f& p) {
    return cellKey(static_cast<int32_t>(std::floor(p.x() / cell)),
                   static_cast<int32_t>(std::floor(p.y() / cell)));
  };
  std::unordered_map<uint64_t, float> min_z;
  min_z.reserve(occ.size());
  for (int idx : occ) {
    auto [it, inserted] = min_z.emplace(cellOf(voxels[idx].pos),
                                        voxels[idx].pos.z());
    if (!inserted) it->second = std::min(it->second, voxels[idx].pos.z());
  }
  std::unordered_map<uint64_t, float> terrain;
  terrain.reserve(min_z.size());
  for (const auto& [key, mz] : min_z) {
    const int32_t cx = static_cast<int32_t>(key >> 32);
    const int32_t cy = static_cast<int32_t>(key & 0xffffffffu);
    std::vector<float> nb;
    nb.reserve(9);
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy) {
        auto it = min_z.find(cellKey(cx + dx, cy + dy));
        if (it != min_z.end()) nb.push_back(it->second);
      }
    terrain.emplace(key, medianOf(nb));
    (void)mz;
  }

  // Terrain height at (x, y), bilinear between filtered cell-centre samples; a
  // piecewise-constant lookup leaks ground through the margin gate on slopes.
  // Missing neighbours use the query's own cell. (notes: tree-terrain-bilinear)
  const auto terrainAt = [&](float x, float y) {
    const double gx = static_cast<double>(x) / cell - 0.5;
    const double gy = static_cast<double>(y) / cell - 0.5;
    const int32_t x0 = static_cast<int32_t>(std::floor(gx));
    const int32_t y0 = static_cast<int32_t>(std::floor(gy));
    const float fx = static_cast<float>(gx - x0);
    const float fy = static_cast<float>(gy - y0);
    const float own = terrain.at(
        cellKey(static_cast<int32_t>(std::floor(x / cell)),
                static_cast<int32_t>(std::floor(y / cell))));
    const auto sample = [&](int32_t cx, int32_t cy) {
      auto it = terrain.find(cellKey(cx, cy));
      return it != terrain.end() ? it->second : own;
    };
    return (1.0f - fx) * (1.0f - fy) * sample(x0, y0) +
           fx * (1.0f - fy) * sample(x0 + 1, y0) +
           (1.0f - fx) * fy * sample(x0, y0 + 1) +
           fx * fy * sample(x0 + 1, y0 + 1);
  };

  // 3. Split above-ground voxels from terrain, and pick out the stem slice.
  //    Clustering ONLY the slice separates trees at stem level even when their
  //    canopies touch; the canopy re-joins its tree in the attach step below.
  std::vector<int> above;     // all voxels clear of the ground
  std::vector<int> stem_sel;  // the subset inside the stem slice
  above.reserve(occ.size());
  for (int idx : occ) {
    const float h = voxels[idx].pos.z() -
                    terrainAt(voxels[idx].pos.x(), voxels[idx].pos.y());
    if (h < cfg.ground_margin_m) continue;
    above.push_back(idx);
    if (h >= cfg.stem_slice_lo && h <= cfg.stem_slice_hi)
      stem_sel.push_back(idx);
  }
  if (stem_sel.empty()) return out;

  // 4. Cluster the stem slice and shape-gate each cluster: PCA linearity
  // rejects isotropic bushes; the verticality gate (max_tilt_deg) rejects
  // walls, logs and ground ribbons, which linearity passes.
  // (notes: tree-stem-shape-gates)
  const auto clusters =
      clusterVoxels(voxels, stem_sel, cfg.voxel_size, connectionRadius(cfg),
                    connectionRadiusSqCells(cfg));
  const float cos_tilt = std::cos(cfg.max_tilt_deg *
                                  static_cast<float>(M_PI) / 180.0f);
  struct Stem {
    std::vector<int> trunk;
    float cx, cy;
  };
  std::vector<Stem> stems;
  for (const auto& cl : clusters) {
    if (static_cast<int>(cl.size()) < cfg.min_trunk_voxels) continue;

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int idx : cl) mean += voxels[idx].pos;
    mean /= static_cast<float>(cl.size());
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int idx : cl) {
      const Eigen::Vector3f d = voxels[idx].pos - mean;
      cov += d * d.transpose();
    }
    cov /= static_cast<float>(cl.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
    if (es.info() != Eigen::Success) continue;
    const Eigen::Vector3f evals = es.eigenvalues();  // ascending
    if (evals(2) <= 1e-9f) continue;                 // degenerate (single cell)
    const float linearity = (evals(2) - evals(1)) / evals(2);
    if (linearity < cfg.min_linearity) continue;
    if (std::abs(es.eigenvectors().col(2).z()) < cos_tilt) continue;

    // Axis = median XY, same robust estimate the shared back-end refits.
    std::vector<float> xs, ys;
    xs.reserve(cl.size());
    ys.reserve(cl.size());
    for (int idx : cl) {
      xs.push_back(voxels[idx].pos.x());
      ys.push_back(voxels[idx].pos.y());
    }
    stems.push_back(Stem{cl, medianOf(xs), medianOf(ys)});
  }
  if (stems.empty()) return out;

  // Deterministic output independent of map iteration order: ties in the
  // nearest-axis arbitration below resolve by stem order, so fix that order
  // geometrically.
  std::sort(stems.begin(), stems.end(), [](const Stem& a, const Stem& b) {
    return a.cx != b.cx ? a.cx < b.cx : a.cy < b.cy;
  });

  // 5. Attach the rest of each tree: a stem keeps its own slice voxels
  // (fitAndScore needs trunk within members); any other voxel joins the nearest
  // stem axis within attach_radius_m in its own component, else is dropped.
  // (notes: tree-attach-to-stem)
  const auto comps =
      clusterVoxels(voxels, above, cfg.voxel_size, connectionRadius(cfg),
                    connectionRadiusSqCells(cfg));
  std::unordered_map<int, int> vox_comp;  // voxel index -> component id
  vox_comp.reserve(above.size() * 2);
  for (size_t c = 0; c < comps.size(); ++c)
    for (int idx : comps[c]) vox_comp.emplace(idx, static_cast<int>(c));

  std::unordered_map<int, int> owner;  // slice-voxel index -> stem id
  std::vector<int> stem_comp(stems.size());
  for (size_t s = 0; s < stems.size(); ++s) {
    for (int idx : stems[s].trunk) owner.emplace(idx, static_cast<int>(s));
    // A stem's voxels are grid-connected, so any one identifies its component.
    stem_comp[s] = vox_comp.at(stems[s].trunk.front());
  }

  // 2D stem hash on attach_radius_m cells: per-voxel candidates are the stems
  // in the 3x3 neighbourhood, keeping this step O(|above| * local stem
  // density) instead of O(|above| * |stems|) — the latter ate most of the
  // scan period on large maps.
  const double acell = std::max(static_cast<double>(cfg.attach_radius_m),
                                cfg.voxel_size);
  std::unordered_map<uint64_t, std::vector<int>> stem2d;
  for (size_t s = 0; s < stems.size(); ++s) {
    const uint64_t k =
        cellKey(static_cast<int32_t>(std::floor(stems[s].cx / acell)),
                static_cast<int32_t>(std::floor(stems[s].cy / acell)));
    stem2d[k].push_back(static_cast<int>(s));
  }

  std::vector<std::vector<int>> members(stems.size());
  const float att2 = cfg.attach_radius_m * cfg.attach_radius_m;
  for (int idx : above) {
    const auto own_it = owner.find(idx);
    if (own_it != owner.end()) {  // slice voxel: its stem keeps it
      members[own_it->second].push_back(idx);
      continue;
    }
    const int comp = vox_comp.at(idx);
    const int32_t ax =
        static_cast<int32_t>(std::floor(voxels[idx].pos.x() / acell));
    const int32_t ay =
        static_cast<int32_t>(std::floor(voxels[idx].pos.y() / acell));
    int best = -1;
    float best_d2 = att2;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy) {
        auto it = stem2d.find(cellKey(ax + dx, ay + dy));
        if (it == stem2d.end()) continue;
        for (int s : it->second) {
          if (stem_comp[s] != comp) continue;
          const float ddx = voxels[idx].pos.x() - stems[s].cx;
          const float ddy = voxels[idx].pos.y() - stems[s].cy;
          const float d2 = ddx * ddx + ddy * ddy;
          if (d2 <= best_d2) {
            best_d2 = d2;
            best = s;
          }
        }
      }
    if (best >= 0) members[best].push_back(idx);
  }

  // 6. Shared fit + score; the stem-slice cluster plays the trunk role.
  for (size_t s = 0; s < stems.size(); ++s) {
    if (auto d = fitAndScore(cfg, voxels, stems[s].trunk, members[s]))
      out.push_back(*d);
  }
  return out;
}

}  // namespace

float infoDeficit(const TreeDetectorConfig& cfg, float coverage,
                  float mean_entropy, float vertical) {
  // Weights auto-normalised so the score stays in [0, 1] regardless of how the
  // caller sets them -- note this also means zeroing a weight (w_vertical
  // defaults to 0) redistributes its share rather than shrinking the range.
  const float wsum = cfg.w_coverage + cfg.w_entropy + cfg.w_vertical;
  const float w = (wsum > 0.0f) ? wsum : 1.0f;
  return (cfg.w_coverage * (1.0f - coverage) +
          cfg.w_entropy * mean_entropy +
          cfg.w_vertical * (1.0f - vertical)) / w;
}

// ===================================================================
// Emission gate (bearing coverage)
// ===================================================================

uint32_t bearingBit(const Eigen::Vector2f& center, const Eigen::Vector2f& robot,
                    int n_bins) {
  const int n = std::min(std::max(n_bins, 1), 32);
  float a = std::atan2(robot.y() - center.y(), robot.x() - center.x());
  if (a < 0.0f) a += 2.0f * static_cast<float>(M_PI);
  int b = static_cast<int>(a / (2.0f * static_cast<float>(M_PI)) * n);
  if (b >= n) b = n - 1;  // guards a == 2*pi from the float divide
  return 1u << b;
}

float bearingCoverage(uint32_t mask, int n_bins) {
  const int n = std::min(std::max(n_bins, 1), 32);
  int filled = 0;
  for (int i = 0; i < n; ++i) filled += (mask >> i) & 1u;
  return static_cast<float>(filled) / static_cast<float>(n);
}

float observeBearing(EmitGate& g, uint32_t bit, int n_bins) {
  const uint32_t grown = g.bearing_mask | bit;
  // A new sector means the robot is still discovering viewing angles on this
  // tree, so the observation is not finished: restart the settle countdown.
  g.settle = (grown != g.bearing_mask) ? 0 : g.settle + 1;
  g.bearing_mask = grown;
  return bearingCoverage(g.bearing_mask, n_bins);
}

bool stepEmitGate(EmitGate& g, bool under_informed, bool has_bearing,
                  int confirm_ticks, int settle_ticks) {
  if (!under_informed) {
    // Break the streak rather than just skipping the emit: emission requires
    // confirm_ticks scans IN A ROW, not that many reads scattered over the
    // track's life. The map grows as the robot moves, so a trunk that reads
    // adequately covered even once must restart the count.
    g.confirm = 0;
    return false;
  }
  if (g.confirm < confirm_ticks) ++g.confirm;

  const bool settled =
      !has_bearing || settle_ticks <= 0 || g.settle >= settle_ticks;
  const bool emit = !g.emitted && g.confirm >= confirm_ticks && settled;
  if (emit) g.emitted = true;
  return emit;
}

std::vector<TreeDetection> TreeDetector::detect(
    const std::vector<SemVoxel>& voxels) const {
  std::vector<TreeDetection> out = cfg_.use_semantics
                                       ? detectSemantic(cfg_, voxels)
                                       : detectGeometric(cfg_, voxels);

  // Neediest first, tie-broken on center for determinism. The comparator is a
  // strict weak ordering only while info_deficit is finite; any new
  // infoDeficit() term must carry its own NaN screen.
  // (notes: tree-sort-strict-weak-order)
  std::sort(out.begin(), out.end(),
            [](const TreeDetection& a, const TreeDetection& b) {
              if (a.info_deficit != b.info_deficit)
                return a.info_deficit > b.info_deficit;
              if (a.center.x() != b.center.x())
                return a.center.x() < b.center.x();
              return a.center.y() < b.center.y();
            });
  return out;
}

}  // namespace explo_planner
