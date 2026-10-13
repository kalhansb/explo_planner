#pragma once
/// @file route_builder.hpp
/// @brief planner_mode 2d_route: the viewpoints of one route step and their
///        drive order, built from the planning_map (RP §5.2 items 3-9 as
///        amended by coverage-loops P2).
///
/// Pure, like grid_frontier: no clock, no node, no RNG, no logging. The only
/// side effect is on the caller's scratch CostGrid. Viewpoints are lattice
/// points and frontier centres that stand in a route cell within sensor range
/// of the tier cell and pass the walk's filters. They are tiered by the unknown
/// they reveal inside the tier cell, picked by a greedy cover and ordered as an
/// open route from the robot. "Map index" means gy * width + gx on the
/// planning_map; "cell" means a CellGrid id.

#include <cstdint>
#include <functional>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explo_planner/cell_world.hpp"
#include "explo_planner/cost_grid.hpp"
#include "explo_planner/failed_goal_blacklist.hpp"
#include "explo_planner/plan_map_query.hpp"

namespace explo_planner {

struct RouteParams {
  float  spacing_m       = 2.5f;   // route_viewpoint_spacing_m; also the route visited radius (RP §5.5)
  float  min_gain_m2     = 20.0f;  // route_min_gain_m2: tier entry, cover stop, residual drop
  int    max_viewpoints  = 12;     // route_max_viewpoints
  float  reorder_gain    = 0.10f;  // route_reorder_gain
  double min_goal_dist_m = 4.0;    // caller: max(goal_xy_tol_, cand_min_goal_dist_)
  double failed_radius_m = 2.0;    // caller: failed_goal_radius_m_
  float  hfov = 6.28318f; int h_rays = 96; float min_range = 0.3f; float max_range = 20.0f; // fov_cfg_2d_
  float identityM() const { return 0.5f * spacing_m; }       // 1.25 m, not a parameter
};

struct RouteInputs {
  const nav_msgs::msg::OccupancyGrid* map = nullptr;   // *latest_plan_map_, valid for the call only
  Roi2D roi{};                                         // {roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_}
  const CellGrid* grid = nullptr;                      // &cell_world_.grid()
  Eigen::Vector3f robot{0, 0, 0};                      // robot_pos; z copied onto waypoints
  std::vector<int> route_cells;                        // stand cells (P: whole list incl. finished + home;
                                                       // A: allocator tour; helper: list being walked)
  int tier_cell[2] = {-1, -1};                         // k / k+1 (A: F / F2); -1 = no such tier
  std::vector<int> later_cells[2];                     // tie-break cells for tier 1 / tier 2
  std::vector<Eigen::Vector3f> frontier_centres;       // findGridFrontiers(...) positions
  const CostGrid* robot_flood = nullptr;               // already built+flooded from robot
  bool skip_reachability = false;                      // node:3591-3603 result
  const FailedGoalBlacklist* visited = nullptr;        // pruned by caller (sim clock)
  const FailedGoalBlacklist* failed  = nullptr;        // pruned by caller
  std::function<bool(const Eigen::Vector3f&)> lost_to_claim;  // empty = coordination off
  std::vector<Eigen::Vector3f> prev_remaining;         // unpassed waypoints of the held route, in order
};

struct RouteMapGeom { int width = 0, height = 0; double origin_x = 0, origin_y = 0; float resolution = 0; };

/// Points rejected by each filter, in filter order. below_tier counts the
/// admissible points that are not members of the served tier (all of them
/// with kNone), so the counts and n_tier_members add up to n_points.
struct RouteRejects { int stand = 0, too_close = 0, map = 0, unreachable = 0,
                      visited = 0, failed = 0, claim = 0, below_tier = 0; };

struct RouteWaypoint {
  Eigen::Vector3f position{0, 0, 0};
  float yaw = 0.0f;              // toward the next waypoint; last = arrival bearing (§6 item 9)
  bool  is_frontier = false;
  int   candidate_index = -1;    // lattice points first (row-major), then frontier centres
  int   selection_rank = -1;     // greedy pick order (0 = first pick)
  int   marginal_cells = 0;      // marginal unknown map cells inside the tier cell at pick
  int   marginal_later_cells = 0;
  float leg_cost_m = 0.0f;       // from the previous waypoint (the robot for [0])
  std::vector<int32_t> residual_cells;   // that marginal tier-cell set, flat map indices
};

struct RoutePlan {
  enum class Outcome { kNone, kSingle, kRoute };
  Outcome outcome = Outcome::kNone;     // kNone -> caller runs the 2d_raycast walk (P2 item 5)
  int   tier = 0;                       // 1 or 2; 0 with kNone
  int   served_cell = -1;               // tier_cell[tier-1]
  std::vector<RouteWaypoint> waypoints; // in drive order
  float covered_m2 = 0, covered_later_m2 = 0, route_len_m = 0;
  bool  order_kept = false; int n_matched_prev = 0;
  int   n_points = 0, n_admissible = 0, n_tier_members = 0, n_floods = 0;
  RouteRejects rejects;
  RouteMapGeom geom;                    // of the map used; residual indices are only valid on it
};

/// RP §5.2 items 3-9 as amended by CL P2. pair_flood is caller-owned scratch
/// (a second CostGrid; the builder calls pair_flood.build(*in.map)).
/// kNone with zero counters on a null or unusable map, grid or robot flood, a
/// tier_cell[0] that is not a cell, or max_viewpoints < 1. Continuity applies
/// when at least one pick matches a prev_remaining waypoint.
RoutePlan buildRoute(const RouteInputs& in, const RouteParams& p, CostGrid& pair_flood);

// ---- exported for tests and for the node's refresh logic ----
std::vector<Eigen::Vector2f> routeLattice(const Roi2D& roi, float spacing_m);   // half-offset (§3.3)
float distToCellBox(const CellGrid& g, int id, float x, float y);               // 0 inside
int   minTierCells(float min_gain_m2, float resolution);   // ceil(min_gain/res^2 - 1e-6); 500 at 0.2 m

struct CoverSet { std::vector<int32_t> tier, later; };     // sorted, unique map indices
/// Lazy greedy on key (tier gain, later gain, -index); stops when the best
/// marginal tier gain < min_tier_cells or at max_picks. Returns picks in order;
/// optional outputs give each pick's marginal counts and marginal tier set.
/// Every index must be < universe.
std::vector<int> lazyGreedyCover(const std::vector<CoverSet>& sets, size_t universe,
                                 int min_tier_cells, int max_picks,
                                 std::vector<int>* marg_tier = nullptr,
                                 std::vector<int>* marg_later = nullptr,
                                 std::vector<std::vector<int32_t>>* marg_tier_sets = nullptr);

/// Open path, fixed start, free end: nearest insertion then 2-opt (first improvement).
/// d is square and symmetric, one row per node of from_start.
std::vector<int> orderOpenRoute(const std::vector<float>& from_start,
                                const std::vector<std::vector<float>>& d);
float openRouteLength(const std::vector<int>& order, const std::vector<float>& from_start,
                      const std::vector<std::vector<float>>& d);

/// RP §5.4 material change: head moved > identity, or |L_new - L_rem| > gain * L_rem.
/// A current length that is not finite always counts as a change.
bool routeMaterialChange(const Eigen::Vector3f& new_head, float new_len_m,
                         const Eigen::Vector3f& cur_head, float cur_remaining_len_m,
                         const RouteParams& p);

/// RP §5.4 residual: count of `cells` still unknown (<0); -1 if g's geometry != geom.
int routeStillUnknown(const nav_msgs::msg::OccupancyGrid& g, const RouteMapGeom& geom,
                      const std::vector<int32_t>& cells);
}  // namespace explo_planner
