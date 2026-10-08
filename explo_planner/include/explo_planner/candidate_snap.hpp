#pragma once
/// @file candidate_snap.hpp
/// @brief Moves a goal candidate off a bad planning-map cell onto the nearest
///        good one.
///
/// Frontier centroids are averages of 3D frontier voxels, and in a forest the
/// unseen space they border is usually behind or inside a bush — so the
/// centroid itself often lands on a blocked cell, right against one, or in a
/// pocket the robot cannot reach. Rejecting those candidates loses most
/// long-range targets; sending them makes the robot drive into the bush. The
/// snapper instead searches outward (nearest first) for a cell that is:
///   - inside the grid and not blocked,
///   - outside the CostGrid clearance zone (CostModel::clearance_m),
///   - known free, unless the candidate may stand on unknown ground (frontier
///     candidates — they border unexplored space by construction),
///   - reachable from the robot when the flood is usable,
///   - accepted by the caller (the ROI).
///
/// Cost: the search visits at most the precomputed disc of offsets within
/// radius_m (pi * (radius / resolution)^2 cells, ~315 at 2 m / 0.2 m), each an
/// O(1) lookup in the CostGrid layers. No allocation per call.

#include <Eigen/Core>
#include <functional>
#include <utility>
#include <vector>

#include "explo_planner/cost_grid.hpp"

namespace explo_planner {

class CandidateSnapper {
public:
  enum class Outcome {
    kKept,     ///< The candidate's own cell is already good.
    kSnapped,  ///< Moved to the centre of the nearest good cell (`out`).
    kNoSpot,   ///< No good cell within radius_m; candidate left unchanged.
  };

  /// `radius_m` <= 0 only checks the candidate's own cell (never moves it).
  explicit CandidateSnapper(float radius_m = 0.0f) : radius_m_(radius_m) {}

  /// `grid` must be built (and flooded, if `require_reachable`) from the
  /// planning map the candidates are judged against. `accept(x, y)` vets the
  /// cell centre of a snap target; it is not applied to the candidate's own
  /// position.
  Outcome snap(const CostGrid& grid, const Eigen::Vector3f& pos,
               bool allow_unknown, bool require_reachable,
               const std::function<bool(float, float)>& accept,
               Eigen::Vector2f& out);

  float radius() const { return radius_m_; }

private:
  float radius_m_ = 0.0f;
  /// Disc offsets sorted by distance, rebuilt when the grid resolution changes.
  float offsets_res_ = 0.0f;
  std::vector<std::pair<int, int>> offsets_;

  bool good(const CostGrid& grid, int gx, int gy, bool allow_unknown,
            bool require_reachable) const;
  void buildOffsets(float resolution);
};

}  // namespace explo_planner
