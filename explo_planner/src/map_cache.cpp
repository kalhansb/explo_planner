// Moved comments: doc/explo_planner_code_notes.md
#include "explo_planner/map_cache.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <scovox/uncertainty.hpp>
#include <scovox/voxel.hpp>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace explo_planner {

namespace {

// Rejects a non-positive or non-finite voxel size: Bonxai's posToCoord would
// cast non-finite values to int32 (UB, garbage coordinates). A config error, so
// it throws naming the value. (notes: mapcache-resolution-check)
double checkedResolution(double resolution, const char* where) {
  if (!(resolution > 0.0) || !std::isfinite(resolution)) {
    throw std::invalid_argument(
        std::string("MapCache: ") + where + " requires a finite positive voxel "
        "resolution, got " + std::to_string(resolution));
  }
  return resolution;
}

}  // namespace

MapCache::MapCache(double resolution)
    : resolution_(checkedResolution(resolution, "constructor")),
      grid_(std::make_unique<Grid>(resolution_)) {}

bool MapCache::updateFromScovoxMap(const scovox_msgs::msg::ScovoxMap& msg) {
  // Unbounded: ingest every voxel. ±inf bounds make the clip a no-op.
  constexpr float kInf = std::numeric_limits<float>::infinity();
  return updateFromScovoxMap(msg,
                             Eigen::Vector3f(-kInf, -kInf, -kInf),
                             Eigen::Vector3f( kInf,  kInf,  kInf));
}

bool MapCache::updateFromScovoxMap(const scovox_msgs::msg::ScovoxMap& msg,
                                   const Eigen::Vector3f& roi_min,
                                   const Eigen::Vector3f& roi_max) {
  // msg.resolution is untrusted: NaN or non-positive falls back to the last
  // good resolution; +inf passes the > 0.0f test, so the isfinite check drops
  // the message (returns false) instead of throwing.
  // (notes: mapcache-wire-resolution)
  const double res = msg.resolution > 0.0f ? static_cast<double>(msg.resolution)
                                           : resolution_;
  if (!(res > 0.0) || !std::isfinite(res)) return false;
  grid_ = std::make_unique<Grid>(res);
  resolution_ = res;

  auto acc = grid_->createAccessor();
  for (const auto& vx : msg.voxels) {
    const float x = vx.position.x, y = vx.position.y, z = vx.position.z;
    // Drop non-finite positions: posToCoord casts float->int32, which is UB for
    // NaN/inf. Must precede the clip — an inf coordinate would pass the AABB
    // test against ±inf bounds (the unbounded overload).
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      continue;
    // Clip to the ROI AABB (inclusive). Against ±inf bounds (unbounded overload)
    // every finite position passes, so this is free in the full-map case.
    if (x < roi_min.x() || x > roi_max.x() ||
        y < roi_min.y() || y > roi_max.y() ||
        z < roi_min.z() || z > roi_max.z())
      continue;
    // Drop voxels with non-finite Beta parameters: a NaN a_occ/a_free would
    // poison the per-voxel scorers and the aggregate metrics (mean_eig /
    // mean_entropy) for the rest of the run. (notes: mapcache-nonfinite-beta)
    if (!std::isfinite(vx.a_occ) || !std::isfinite(vx.a_free) ||
        vx.a_occ < 0.0f || vx.a_free < 0.0f)
      continue;
    UnifiedVoxel uv;
    uv.a_occ    = vx.a_occ;
    uv.a_free   = vx.a_free;
    uv.p_occ    = (uv.a_occ + uv.a_free > 0.0f)
                      ? uv.a_occ / (uv.a_occ + uv.a_free)
                      : 0.5f;
    uv.observed = true;
    auto coord = grid_->posToCoord(
        static_cast<double>(vx.position.x),
        static_cast<double>(vx.position.y),
        static_cast<double>(vx.position.z));
    acc.setValue(coord, uv);
  }
  return true;
}

void MapCache::updateFromLogOddsCloud(
    const sensor_msgs::msg::PointCloud2& msg, double resolution) {
  resolution_ = checkedResolution(resolution, "updateFromLogOddsCloud");
  grid_ = std::make_unique<Grid>(resolution_);

  sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> ip(msg, "occupancy_prob");

  auto acc = grid_->createAccessor();
  for (; ix != ix.end(); ++ix, ++iy, ++iz, ++ip) {
    // Organized / log-odds clouds mark invalid returns with NaN/inf xyz; skip
    // them before posToCoord (float->int32 cast is UB on non-finite input).
    if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz))
      continue;
    float p = *ip;
    // std::clamp passes NaN straight through (every comparison against NaN is
    // false), so guard explicitly before the clamp — otherwise log(NaN) makes
    // a_occ/a_free NaN and poisons the scorers.
    if (!std::isfinite(p)) continue;
    // Clamp to avoid log(0)
    p = std::clamp(p, 0.001f, 0.999f);

    // Synthesize Beta parameters from occupancy probability.
    // L = log(p/(1-p)), K = 2 + |L|*2 maps evidence strength.
    float L = std::log(p / (1.0f - p));
    float K = 2.0f + std::abs(L) * 2.0f;

    UnifiedVoxel uv;
    uv.a_occ    = p * K;
    uv.a_free   = (1.0f - p) * K;
    uv.p_occ    = p;
    uv.observed = true;

    auto coord = grid_->posToCoord(
        static_cast<double>(*ix),
        static_cast<double>(*iy),
        static_cast<double>(*iz));
    acc.setValue(coord, uv);
  }
}

UnifiedVoxel MapCache::getVoxel(const Eigen::Vector3f& pos) const {
  auto coord = posToCoord(pos);
  auto acc = grid_->createConstAccessor();
  const UnifiedVoxel* ptr = acc.value(coord);
  if (ptr) return *ptr;
  return UnifiedVoxel{};  // Default Beta(1,1) prior
}

const UnifiedVoxel* MapCache::getVoxelByCoord(const CoordT& c) const {
  auto acc = grid_->createConstAccessor();
  return acc.value(c);
}

size_t MapCache::voxelCount() const {
  return grid_->activeCellsCount();
}

bool MapCache::empty() const {
  return grid_->activeCellsCount() == 0;
}

MapCache::CoordT MapCache::posToCoord(const Eigen::Vector3f& pos) const {
  return grid_->posToCoord(
      static_cast<double>(pos.x()),
      static_cast<double>(pos.y()),
      static_cast<double>(pos.z()));
}

Eigen::Vector3f MapCache::coordToPos(const CoordT& c) const {
  auto p = grid_->coordToPos(c);
  return Eigen::Vector3f(
      static_cast<float>(p.x),
      static_cast<float>(p.y),
      static_cast<float>(p.z));
}

std::vector<Eigen::Vector3f> MapCache::findFrontierCentroids(
    float min_z, float max_z, float cluster_radius) const {
  // Frontier: free voxels (p_occ < 0.5) with at least one unknown neighbor.
  std::vector<Eigen::Vector3f> frontier_cells;

  auto acc = grid_->createConstAccessor();
  static const CoordT offsets[6] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

  grid_->forEachCell([&](const UnifiedVoxel& v, const CoordT& c) {
    if (v.p_occ >= 0.5f) return;  // Only free voxels
    auto pos = grid_->coordToPos(c);
    if (static_cast<float>(pos.z) < min_z ||
        static_cast<float>(pos.z) > max_z)
      return;

    for (const auto& off : offsets) {
      CoordT nb{c.x + off.x, c.y + off.y, c.z + off.z};
      if (!acc.value(nb)) {
        frontier_cells.emplace_back(
            static_cast<float>(pos.x),
            static_cast<float>(pos.y),
            static_cast<float>(pos.z));
        return;
      }
    }
  });

  if (frontier_cells.empty()) return {};

  // Cluster by coarse grid binning. A zero / negative / non-finite radius
  // would make inv_bin inf or NaN and fold every frontier cell into one bin
  // (or none); one-voxel bins are the tightest meaningful clustering.
  if (!(cluster_radius > 0.0f) || !std::isfinite(cluster_radius))
    cluster_radius = static_cast<float>(resolution_);
  float inv_bin = 1.0f / cluster_radius;
  struct BinKey {
    int bx, by, bz;
    bool operator==(const BinKey& o) const {
      return bx == o.bx && by == o.by && bz == o.bz;
    }
  };
  struct BinHash {
    size_t operator()(const BinKey& k) const {
      return std::hash<int64_t>()(
          (static_cast<int64_t>(k.bx) * 73856093) ^
          (static_cast<int64_t>(k.by) * 19349663) ^
          (static_cast<int64_t>(k.bz) * 83492791));
    }
  };
  struct BinData {
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    int count = 0;
  };

  std::unordered_map<BinKey, BinData, BinHash> bins;
  for (const auto& p : frontier_cells) {
    BinKey key{static_cast<int>(std::floor(p.x() * inv_bin)),
               static_cast<int>(std::floor(p.y() * inv_bin)),
               static_cast<int>(std::floor(p.z() * inv_bin))};
    auto& bd = bins[key];
    bd.sum += p;
    bd.count++;
  }

  std::vector<Eigen::Vector3f> centroids;
  centroids.reserve(bins.size());
  for (const auto& [key, bd] : bins) {
    centroids.push_back(bd.sum / static_cast<float>(bd.count));
  }
  return centroids;
}

float MapCache::groundZAt(float x, float y, float z_low, float z_high,
                          float occ_thresh, float stack_max_m) const {
  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
  if (!(z_high > z_low) || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z_low) || !std::isfinite(z_high))
    return kNaN;

  const float res = static_cast<float>(resolution_);
  auto acc = grid_->createConstAccessor();

  // Work in coord space so the scan can't skip or repeat cells through
  // float rounding. Coords are voxel corners: coordToPos(c).z == c.z * res.
  const auto c_lo = grid_->posToCoord(
      static_cast<double>(x), static_cast<double>(y),
      static_cast<double>(z_low));
  const auto c_hi = grid_->posToCoord(
      static_cast<double>(x), static_cast<double>(y),
      static_cast<double>(z_high));

  auto occupied = [&](int32_t cz) {
    const UnifiedVoxel* v = acc.value(CoordT{c_lo.x, c_lo.y, cz});
    return v && v->p_occ >= occ_thresh;
  };

  // Bottom-up: the lowest occupied voxel anchors the ground (canopy and
  // overhangs sit higher and are never reached).
  for (int32_t cz = c_lo.z; cz <= c_hi.z; ++cz) {
    if (!occupied(cz)) continue;
    // Walk up the contiguous occupied stack, capped at stack_max_m so a
    // wall/trunk column doesn't lift the ground to its top.
    const int32_t max_up = std::max(
        0, static_cast<int>(std::floor(stack_max_m / res)));
    int32_t top = cz;
    while (top < cz + max_up && occupied(top + 1)) ++top;
    // Top face of the top stack voxel = the ground surface elevation.
    return static_cast<float>(top + 1) * res;
  }
  return kNaN;
}

double MapCache::unknownColumnFraction(float min_x, float max_x,
                                       float min_y, float max_y) const {
  if (!std::isfinite(min_x) || !std::isfinite(max_x) ||
      !std::isfinite(min_y) || !std::isfinite(max_y) ||
      !(max_x > min_x) || !(max_y > min_y))
    return -1.0;

  // Work in coord space so the column count and the membership test share the
  // same floor() convention as voxel ingest (posToCoord). Inclusive bounds:
  // a voxel exactly on the max edge lands in the last column, mirroring the
  // inclusive AABB clip in updateFromScovoxMap.
  const auto c_min = grid_->posToCoord(
      static_cast<double>(min_x), static_cast<double>(min_y), 0.0);
  const auto c_max = grid_->posToCoord(
      static_cast<double>(max_x), static_cast<double>(max_y), 0.0);
  const int64_t nx = static_cast<int64_t>(c_max.x) - c_min.x + 1;
  const int64_t ny = static_cast<int64_t>(c_max.y) - c_min.y + 1;
  const int64_t total = nx * ny;
  if (total <= 0) return -1.0;

  // One walk over active cells, each packed to its (x, y) column: x in the high
  // 32 bits, y in the low (bijective). Shift on uint64_t: left-shifting a
  // negative signed coord is UB in C++17. (notes: mapcache-column-pack)
  std::unordered_set<uint64_t> observed;
  grid_->forEachCell([&](const UnifiedVoxel&, const CoordT& c) {
    if (c.x < c_min.x || c.x > c_max.x || c.y < c_min.y || c.y > c_max.y)
      return;
    observed.insert(
        (static_cast<uint64_t>(static_cast<uint32_t>(c.x)) << 32) |
        static_cast<uint32_t>(c.y));
  });

  return 1.0 - static_cast<double>(observed.size()) /
                   static_cast<double>(total);
}

MapCache::MapStats MapCache::computeStats() const {
  MapStats s;
  float sum_eig = 0.0f, sum_entropy = 0.0f, sum_variance = 0.0f;
  int frontier_count = 0, count = 0;

  auto acc = grid_->createConstAccessor();
  static const CoordT offsets[6] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

  grid_->forEachCell([&](const UnifiedVoxel& uv, const CoordT& c) {
    scovox::Voxel sv;
    sv.a_occ  = uv.a_occ;
    sv.a_free = uv.a_free;
    sum_eig      += scovox::expectedInformationGain(sv);
    sum_variance += scovox::variance(sv);
    sum_entropy  += scoring::entropy(uv);
    count++;
    // Frontier: free voxel (p_occ < 0.5) with at least one unknown (absent)
    // 6-neighbour. Mirrors findFrontierCentroids' membership test.
    if (uv.p_occ < 0.5f) {
      for (const auto& off : offsets) {
        CoordT nb{c.x + off.x, c.y + off.y, c.z + off.z};
        if (!acc.value(nb)) { frontier_count++; break; }
      }
    }
  });

  s.total_voxels    = count;
  s.frontier_voxels = frontier_count;
  if (count > 0) {
    s.mean_eig      = sum_eig / static_cast<float>(count);
    s.mean_entropy  = sum_entropy / static_cast<float>(count);
    s.mean_variance = sum_variance / static_cast<float>(count);
  }
  return s;
}

} // namespace explo_planner
