/// @file tree_detector.cpp
/// @brief Implementation of the pure tree-instance detector. See the header for
///        the "not enough info" rationale (angular coverage primary).

#include "explo_planner/tree_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

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

Coord toCoord(const Eigen::Vector3f& p, double res) {
  // floor, not round: a regular voxel lattice maps to consecutive integers for
  // any grid phase offset, with no gaps or collisions. lround would open a
  // one-cell gap at the zero crossing (…-1, [no 0], 1…), splitting a straddling
  // object into two clusters and corrupting the radius fit.
  const double inv = 1.0 / res;
  return Coord{static_cast<int32_t>(std::floor(p.x() * inv)),
               static_cast<int32_t>(std::floor(p.y() * inv)),
               static_cast<int32_t>(std::floor(p.z() * inv))};
}

/// Binary occupancy entropy of p, normalised to [0, 1] (H / ln2). p is clamped
/// off the {0,1} rails so the log is finite; a fully-decided voxel reads ~0.
float normEntropy(float p) {
  p = std::min(std::max(p, 1e-4f), 1.0f - 1e-4f);
  const float h = -p * std::log(p) - (1.0f - p) * std::log(1.0f - p);
  return h / static_cast<float>(M_LN2);
}

float medianOf(std::vector<float>& v) {
  if (v.empty()) return 0.0f;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

}  // namespace

std::vector<TreeDetection> TreeDetector::detect(
    const std::vector<SemVoxel>& voxels) const {
  std::vector<TreeDetection> out;

  // 1. Keep only occupied, confidently-vegetation voxels. These are the trunk +
  //    canopy surface points; unseen (e.g. occluded far-side) space is simply
  //    absent from the map, which is exactly what the coverage metric reads.
  std::vector<int> veg;  // indices into `voxels`
  veg.reserve(voxels.size());
  std::unordered_map<Coord, int, CoordHash> grid;  // coord -> position in `veg`
  grid.reserve(voxels.size() * 2);
  for (int i = 0; i < static_cast<int>(voxels.size()); ++i) {
    const SemVoxel& v = voxels[i];
    if (v.best_class != cfg_.veg_class) continue;
    if (v.p_occ < cfg_.occ_thresh) continue;
    if (v.class_conf < cfg_.min_class_conf) continue;
    // One representative voxel per cell: a real ScovoxMap holds a single voxel
    // per coord, but keeping this explicit stops any aliased duplicates from
    // seeding phantom singleton clusters.
    auto [it, inserted] =
        grid.emplace(toCoord(v.pos, cfg_.voxel_size), static_cast<int>(veg.size()));
    (void)it;
    if (!inserted) continue;
    veg.push_back(i);
  }
  if (veg.empty()) return out;

  // 2. Connected-component labelling over the vegetation voxels (26-neighbour).
  //    One component ~ one tree; well-separated trunks land in distinct
  //    components. (Touching canopies can merge two trees into one component —
  //    an accepted v1 limitation; the trunk-band fit below still recovers a
  //    single dominant axis.)
  // Connection window: two occupied cells join if within cluster_tol_m. A
  // window > 1 voxel bridges the 1-2 cell gaps a thin/sparse trunk surface
  // leaves under strict 26-connectivity, without merging trees metres apart.
  const int R = std::max(1, static_cast<int>(std::lround(
                                cfg_.cluster_tol_m / cfg_.voxel_size)));
  std::vector<int> label(veg.size(), -1);
  int n_clusters = 0;
  std::vector<int> stack;
  for (size_t seed = 0; seed < veg.size(); ++seed) {
    if (label[seed] != -1) continue;
    const int cid = n_clusters++;
    stack.clear();
    stack.push_back(static_cast<int>(seed));
    label[seed] = cid;
    while (!stack.empty()) {
      const int cur = stack.back();
      stack.pop_back();
      const Coord c = toCoord(voxels[veg[cur]].pos, cfg_.voxel_size);
      for (int dx = -R; dx <= R; ++dx)
        for (int dy = -R; dy <= R; ++dy)
          for (int dz = -R; dz <= R; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            auto it = grid.find(Coord{c.x + dx, c.y + dy, c.z + dz});
            if (it == grid.end()) continue;
            const int nb = it->second;
            if (label[nb] != -1) continue;
            label[nb] = cid;
            stack.push_back(nb);
          }
    }
  }

  // Bucket voxel indices by cluster.
  std::vector<std::vector<int>> clusters(n_clusters);
  for (size_t k = 0; k < veg.size(); ++k)
    clusters[label[k]].push_back(veg[k]);

  const float two_pi = 2.0f * static_cast<float>(M_PI);

  // 3. Fit + score each cluster.
  for (const auto& members : clusters) {
    if (static_cast<int>(members.size()) < cfg_.min_trunk_voxels) continue;

    float base_z = std::numeric_limits<float>::max();
    float top_z  = std::numeric_limits<float>::lowest();
    for (int idx : members) {
      base_z = std::min(base_z, voxels[idx].pos.z());
      top_z  = std::max(top_z, voxels[idx].pos.z());
    }
    const float height = top_z - base_z;
    if (height < cfg_.min_height) continue;

    // Trunk band: voxels just above the cluster base. Isolates the narrow trunk
    // from the wide canopy so the axis/radius fit is not blown out by foliage.
    // Falls back to the whole cluster if the band is empty (stubby vegetation).
    const float band_lo = base_z + cfg_.trunk_band_lo;
    const float band_hi = base_z + cfg_.trunk_band_hi;
    std::vector<int> trunk;
    for (int idx : members) {
      const float z = voxels[idx].pos.z();
      if (z >= band_lo && z <= band_hi) trunk.push_back(idx);
    }
    if (static_cast<int>(trunk.size()) < cfg_.min_trunk_voxels) trunk = members;

    // Robust axis = per-component median of trunk XY (immune to a leaning
    // canopy or a one-sided observation skewing the mean).
    std::vector<float> xs, ys;
    xs.reserve(trunk.size());
    ys.reserve(trunk.size());
    for (int idx : trunk) {
      xs.push_back(voxels[idx].pos.x());
      ys.push_back(voxels[idx].pos.y());
    }
    const float cx = medianOf(xs);
    const float cy = medianOf(ys);

    // Radius = median trunk-voxel distance to the axis. Reject fat blobs
    // (walls, hedges) that reach this far only because they are not trees.
    std::vector<float> dists;
    dists.reserve(trunk.size());
    for (int idx : trunk) {
      const float dx = voxels[idx].pos.x() - cx;
      const float dy = voxels[idx].pos.y() - cy;
      dists.push_back(std::sqrt(dx * dx + dy * dy));
    }
    const float radius = std::max(medianOf(dists),
                                  0.5f * static_cast<float>(cfg_.voxel_size));
    if (radius > cfg_.max_radius) continue;

    // --- Angular coverage (primary info metric) ---
    // Bin observed trunk voxels by azimuth around the axis; coverage is the
    // fraction of sectors that hold at least one voxel. A trunk seen from one
    // side fills only the sectors on that arc; the occluded far side is absent
    // and reads as empty sectors -- exactly what the vantage circle then fills.
    std::vector<char> az_hit(cfg_.n_azimuth_bins, 0);
    float entropy_sum = 0.0f;
    for (int idx : trunk) {
      const float dx = voxels[idx].pos.x() - cx;
      const float dy = voxels[idx].pos.y() - cy;
      float a = std::atan2(dy, dx);
      if (a < 0.0f) a += two_pi;
      int b = static_cast<int>(a / two_pi * cfg_.n_azimuth_bins);
      if (b >= cfg_.n_azimuth_bins) b = cfg_.n_azimuth_bins - 1;
      az_hit[b] = 1;
      entropy_sum += normEntropy(voxels[idx].p_occ);
    }
    int filled = 0;
    for (char h : az_hit) filled += h ? 1 : 0;
    const float coverage = static_cast<float>(filled) / cfg_.n_azimuth_bins;

    // --- Occupancy entropy (secondary) --- thin / freshly-seen surface sits
    // near the Beta prior (p~0.5) and reads high; a well-hit surface reads ~0.
    const float mean_entropy = entropy_sum / static_cast<float>(trunk.size());

    // --- Vertical completeness (secondary) --- gaps between base and canopy
    // (e.g. mid-trunk occluded) show up as empty height bins.
    std::vector<char> z_hit(cfg_.n_height_bins, 0);
    const float z_span = std::max(height, 1e-3f);
    for (int idx : members) {
      int b = static_cast<int>((voxels[idx].pos.z() - base_z) / z_span *
                               cfg_.n_height_bins);
      if (b < 0) b = 0;
      if (b >= cfg_.n_height_bins) b = cfg_.n_height_bins - 1;
      z_hit[b] = 1;
    }
    int z_filled = 0;
    for (char h : z_hit) z_filled += h ? 1 : 0;
    const float vertical = static_cast<float>(z_filled) / cfg_.n_height_bins;

    // --- Combined deficit --- weighted, weights auto-normalised so the score
    // stays in [0, 1] regardless of how the caller sets them.
    const float wsum = cfg_.w_coverage + cfg_.w_entropy + cfg_.w_vertical;
    const float w = (wsum > 0.0f) ? wsum : 1.0f;
    const float deficit = (cfg_.w_coverage * (1.0f - coverage) +
                           cfg_.w_entropy * mean_entropy +
                           cfg_.w_vertical * (1.0f - vertical)) / w;

    TreeDetection d;
    d.center = Eigen::Vector3f(cx, cy, base_z);
    d.radius = radius;
    d.height = height;
    d.trunk_voxels = static_cast<int>(trunk.size());
    d.angular_coverage = coverage;
    d.mean_entropy = mean_entropy;
    d.vertical_completeness = vertical;
    d.info_deficit = deficit;
    d.under_informed = deficit > cfg_.deficit_thresh;
    out.push_back(d);
  }

  // Neediest first: the node emits the top under-informed trees and logs the
  // rest. Stable tie-break on center keeps the order deterministic.
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
