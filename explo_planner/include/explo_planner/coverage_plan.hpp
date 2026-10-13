#pragma once
/// @file coverage_plan.hpp
/// @brief The stored coverage-loop plan (P arm): one ordered list of CellGrid
///        cells per robot, loaded from config/coverage_loops/plan_nN.yaml, and
///        the list walk that replaces the allocator's tour. Pure C++, no ROS.
///
/// The plan is made offline (analysis/coverage_loops/make_plan_yaml.py) and
/// checked again here, in full, because a plan that does not tile the grid
/// leaves cells nobody owns and the per-cell stop never fires. Every list ends
/// at the shared home cell. Lists are handed out by robot name: the list index
/// is not the fleet id.
///
/// The walk never re-orders (CL P3). A robot walks its own list, skipping
/// finished cells, then helps one fixed teammate (CL Q5): it walks that list
/// backwards from its last cell before home, then home.

#include <functional>
#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"

namespace explo_planner {

struct CoveragePlan {
  int version = 0, n_robots = 0, home_id = -1;
  std::vector<std::vector<int>> lists;   ///< CellGrid ids, home LAST in every list
  std::vector<std::string> robots;       ///< robot per list
  std::vector<int> helps;                ///< helps[i] = list that list i's robot helps
  std::string source_sha256;
};

/// Parse and validate a plan against this robot's grid and fleet. Checks
/// (spec_harness §7.4): version 1 and n_robots == team size; list robots ==
/// team, one list each; grid size, ROI, nx, ny, grid_hash == configHash() and
/// yaw_deg == 0; ids[i] == row*nx+col, in range; each list starts on a home
/// neighbour named by start_dir, steps 4-neighbour and ends at home; no
/// revisits; the lists tile every non-home cell; helps is a permutation with
/// no fixed point and helps_robot agrees. Returns false with *err naming the
/// first failure; never throws.
bool parseCoveragePlan(const std::string& yaml_text, const CellGrid& grid,
                       const std::vector<std::string>& team_names, CoveragePlan* out,
                       std::string* err);

/// Read `path` and parseCoveragePlan it. An unreadable file is a failure.
bool loadCoveragePlanFile(const std::string& path, const CellGrid& grid,
                          const std::vector<std::string>& team_names, CoveragePlan* out,
                          std::string* err);

/// Index of `robot`'s list, or -1 if it has none.
int planListOf(const CoveragePlan& p, const std::string& robot);

/// One step of the P-arm list walk (CL P1/P3/Q5).
struct PlanWalk {
  /// Unfinished walk cells in order: [0] = k, [1] = k+1 (the next unfinished).
  std::vector<int> remaining;
  /// Stand cells: the own list, or when helping the helped list (home incl.).
  std::vector<int> route_cells;
  bool helping = false;          ///< sticky once true
  int  help_list = -1;           ///< list being helped, -1 when not helping
  bool help_started_now = false; ///< true on the call that switched to helping
  int  help_target_cell = -1;    ///< helped list's last cell before home, or -1
};

/// Own list until every non-home own cell is finished, then (sticky) help:
/// walk helps[my]'s list backwards from its last non-home cell, then home.
/// Home is last in both walks. finished(c) is COVERED or COVERED_BY_OTHERS,
/// write-offs included. An out-of-range my_list gives an empty walk.
PlanWalk planWalk(const CoveragePlan& p, int my_list, bool helping_in,
                  const std::function<bool(int)>& finished);

}  // namespace explo_planner
