#include "explo_planner/map_cache.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <unordered_map>
#include <cmath>

namespace explo_planner {

MapCache::MapCache(double resolution)
    : resolution_(resolution),
      grid_(std::make_unique<Grid>(resolution)) {}

void MapCache::updateFromScovoxMap(const scovox_msgs::msg::ScovoxMap& msg) {
  double res = msg.resolution > 0.0f ? static_cast<double>(msg.resolution)
                                     : resolution_;
  grid_ = std::make_unique<Grid>(res);
  resolution_ = res;

  auto acc = grid_->createAccessor();
  for (const auto& vx : msg.voxels) {
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
}

void MapCache::updateFromLogOddsCloud(
    const sensor_msgs::msg::PointCloud2& msg, double resolution) {
  grid_ = std::make_unique<Grid>(resolution);
  resolution_ = resolution;

  sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> ip(msg, "occupancy_prob");

  auto acc = grid_->createAccessor();
  for (; ix != ix.end(); ++ix, ++iy, ++iz, ++ip) {
    float p = *ip;
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

  // Cluster by coarse grid binning.
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

} // namespace explo_planner
