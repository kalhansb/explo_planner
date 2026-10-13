#include <gtest/gtest.h>
#include "explo_planner/coverage_plan.hpp"
#include "explo_planner/cell_world.hpp"
#include <algorithm>
#include <fstream>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace explo_planner;

namespace {

const std::vector<std::string> kTeam2{"atlas", "bestla"};
const std::vector<std::string> kTeam3{"atlas", "bestla", "husky"};
const std::vector<std::string> kTeam4{"atlas", "bestla", "husky", "skadi"};

const std::vector<std::string>& team(int n) {
  return n == 2 ? kTeam2 : n == 3 ? kTeam3 : kTeam4;
}

/// The node's grid for coverage loops: 5x5 cells of 20 m over [-50, 50]^2.
CellGrid grid20() { return makeCellGrid(-50.0f, 50.0f, -50.0f, 50.0f, 20.0f); }

/// The installed-format files, read from the source tree next to this test.
std::string planPath(int n) {
  const std::string here = __FILE__;
  return here.substr(0, here.find_last_of('/')) + "/../config/coverage_loops/plan_n" +
         std::to_string(n) + ".yaml";
}

std::string planText(int n) {
  std::ifstream f(planPath(n));
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

/// Loads plan_n<n>.yaml into *p. Call it under ASSERT_NO_FATAL_FAILURE, so a
/// plan that does not load stops the test before it indexes an empty plan.
void loadPlan(int n, CoveragePlan* p) {
  std::string err;
  ASSERT_TRUE(loadCoveragePlanFile(planPath(n), grid20(), team(n), p, &err)) << err;
}

/// `text` with the single occurrence of `from` replaced by `to`.
std::string edit(const std::string& text, const std::string& from, const std::string& to) {
  const size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << "fixture text not found: " << from;
  EXPECT_EQ(text.find(from, at + 1), std::string::npos) << "fixture text not unique: " << from;
  if (at == std::string::npos) return text;
  return text.substr(0, at) + to + text.substr(at + from.size());
}

/// Refused, for the reason `why` (a substring of *err), and *out left alone.
void expectRejected(const std::string& text, const std::vector<std::string>& names,
                    const std::string& why, const CellGrid& g = grid20()) {
  SCOPED_TRACE(why);
  CoveragePlan p;
  p.version = 99;
  std::string err;
  bool ok = true;
  EXPECT_NO_THROW(ok = parseCoveragePlan(text, g, names, &p, &err));
  EXPECT_FALSE(ok);
  EXPECT_NE(err.find(why), std::string::npos) << "err: " << err;
  EXPECT_EQ(p.version, 99);
}

using Finished = std::set<int>;

PlanWalk walk(const CoveragePlan& p, int my_list, bool helping, const Finished& done) {
  return planWalk(p, my_list, helping, [&done](int c) { return done.count(c) > 0; });
}

/// Every non-home cell of list i.
Finished jobsOf(const CoveragePlan& p, int i) {
  const auto& L = p.lists[i];
  return Finished(L.begin(), L.end() - 1);
}

}  // namespace

// --- The installed plans ----------------------------------------------------

TEST(CoveragePlan, LoadsTheThreeInstalledPlans) {
  for (int n : {2, 3, 4}) {
    SCOPED_TRACE("N=" + std::to_string(n));
    CoveragePlan p;
    std::string err = "stale";
    ASSERT_TRUE(loadCoveragePlanFile(planPath(n), grid20(), team(n), &p, &err)) << err;
    EXPECT_EQ(err, "");
    EXPECT_EQ(p.version, 1);
    EXPECT_EQ(p.n_robots, n);
    EXPECT_EQ(p.home_id, 12);
    EXPECT_EQ(p.source_sha256, "eccfb61344a19673");
    ASSERT_EQ(p.lists.size(), static_cast<size_t>(n));
    std::set<int> jobs;
    for (const auto& L : p.lists) {
      EXPECT_EQ(L.size(), static_cast<size_t>(24 / n + 1));   // equal split + home
      EXPECT_EQ(L.back(), 12);
      jobs.insert(L.begin(), L.end() - 1);
    }
    EXPECT_EQ(jobs.size(), 24u);
    EXPECT_EQ(jobs.count(12), 0u);
  }
}

// The handout by start bearing and the helper table (CL Q5, spec §7.1).
TEST(CoveragePlan, HandoutAndHelperTable) {
  CoveragePlan p2, p3, p4;
  ASSERT_NO_FATAL_FAILURE(loadPlan(2, &p2));
  ASSERT_NO_FATAL_FAILURE(loadPlan(3, &p3));
  ASSERT_NO_FATAL_FAILURE(loadPlan(4, &p4));
  EXPECT_EQ(p2.robots, (std::vector<std::string>{"bestla", "atlas"}));
  EXPECT_EQ(p2.helps, (std::vector<int>{1, 0}));
  EXPECT_EQ(p3.robots, (std::vector<std::string>{"husky", "atlas", "bestla"}));
  EXPECT_EQ(p3.helps, (std::vector<int>{2, 0, 1}));
  EXPECT_EQ(p4.robots, (std::vector<std::string>{"husky", "atlas", "bestla", "skadi"}));
  EXPECT_EQ(p4.helps, (std::vector<int>{1, 0, 3, 2}));
  EXPECT_EQ(p2.lists[0], (std::vector<int>{17, 22, 21, 20, 15, 10, 5, 0, 1, 6, 11, 16, 12}));
  EXPECT_EQ(p2.lists[1], (std::vector<int>{7, 2, 3, 4, 9, 14, 19, 24, 23, 18, 13, 8, 12}));
  EXPECT_EQ(p4.lists[3], (std::vector<int>{17, 22, 23, 24, 19, 18, 12}));
}

// The list index is not the fleet id: lists go by name.
TEST(CoveragePlan, ListsAreFoundByRobotName) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(4, &p));
  EXPECT_EQ(planListOf(p, "husky"), 0);
  EXPECT_EQ(planListOf(p, "atlas"), 1);
  EXPECT_EQ(planListOf(p, "bestla"), 2);
  EXPECT_EQ(planListOf(p, "skadi"), 3);
  EXPECT_EQ(planListOf(p, "curt"), -1);
  EXPECT_EQ(planListOf(p, ""), -1);
}

// Ids name the ground the node's CellGrid does: home is the cell holding every
// spawn pose, and the first N=2 cell [2,3] is the one north of it.
TEST(CoveragePlan, IdsAreTheNodesCellGridIds) {
  const CellGrid g = grid20();
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(2, &p));
  EXPECT_EQ(p.home_id, g.idAt(0.0f, 0.0f));
  EXPECT_EQ(p.home_id, g.idAt(0.0f, 6.0f));
  EXPECT_EQ(p.home_id, g.idAt(0.0f, -3.0f));
  EXPECT_EQ(p.lists[0][0], g.idAt(0.0f, 20.0f));
  EXPECT_EQ(g.col(p.lists[0][0]), 2);
  EXPECT_EQ(g.row(p.lists[0][0]), 3);
}

// Set equality, not order: team_robot_names is in fleet-id order.
TEST(CoveragePlan, TeamOrderDoesNotMatter) {
  CoveragePlan p;
  std::string err;
  EXPECT_TRUE(parseCoveragePlan(planText(3), grid20(), {"husky", "bestla", "atlas"}, &p, &err))
      << err;
  EXPECT_EQ(planListOf(p, "husky"), 0);
}

// The grid hash is accepted in decimal too, as a YAML dumper would write it.
TEST(CoveragePlan, DecimalGridHashIsAccepted) {
  const std::string t = edit(planText(2), "grid_hash: 0x4c6adaf7",
                             "grid_hash: " + std::to_string(grid20().configHash()));
  std::string err;
  EXPECT_TRUE(parseCoveragePlan(t, grid20(), kTeam2, nullptr, &err)) << err;
}

TEST(CoveragePlan, MissingFileIsRefused) {
  CoveragePlan p;
  std::string err;
  EXPECT_FALSE(loadCoveragePlanFile(planPath(5), grid20(), kTeam2, &p, &err));
  EXPECT_NE(err.find("cannot read"), std::string::npos) << err;
}

TEST(CoveragePlan, BadYamlIsRefusedNotThrown) {
  expectRejected("coverage_plan: {version: 1, lists: [", kTeam2, "YAML");
  expectRejected("", kTeam2, "no coverage_plan map");
  expectRejected("coverage_plan: 3", kTeam2, "no coverage_plan map");
  expectRejected(edit(planText(2), "home: [2, 2]", "home: [2, 2]]"), kTeam2, "YAML");
  expectRejected(edit(planText(2), "lists:  ", "lists: 7\n  bogus:  "), kTeam2, "lists must hold");
}

// A missing key is named, not reported as a yaml-cpp InvalidNode.
TEST(CoveragePlan, MissingKeysAreNamed) {
  expectRejected(edit(planText(2), "roi: [-50.0, 50.0, -50.0, 50.0], ", ""), kTeam2, "grid needs");
  expectRejected(edit(planText(2), "  home: [2, 2]", "  homes: [2, 2]"), kTeam2, "home must be");
  expectRejected(edit(planText(2), "  lists:  ", "  listz:  "), kTeam2, "lists must hold");
  expectRejected(edit(planText(2), "cells: [[2,3]", "cellz: [[2,3]"), kTeam2,
                 "lists[0] needs index, robot");
  expectRejected(edit(planText(2), "ids: [17,", "idz: [17,"), kTeam2,
                 "lists[0] needs index, robot");
}

// --- Check 1: version and team size -----------------------------------------

TEST(CoveragePlan, Check1Version) {
  expectRejected(edit(planText(2), "version: 1", "version: 2"), kTeam2, "version must be 1");
}

TEST(CoveragePlan, Check1TeamSize) {
  expectRejected(edit(planText(2), "n_robots: 2", "n_robots: 3"), kTeam2, "n_robots must equal");
  expectRejected(planText(2), kTeam3, "n_robots must equal");
  expectRejected(planText(4), kTeam3, "n_robots must equal");
}

TEST(CoveragePlan, SourceShaIsRequired) {
  expectRejected(edit(planText(2), "source_sha256: eccfb61344a19673", "source_sha: x"), kTeam2,
                 "source_sha256");
}

// --- Check 2: the list robots are the team, one list each ---------------------

TEST(CoveragePlan, Check2RobotOutsideTheTeam) {
  expectRejected(planText(2), {"atlas", "curt"}, "not in team_robot_names");
}

TEST(CoveragePlan, Check2TwoListsForOneRobot) {
  expectRejected(edit(planText(2), "index: 1, robot: atlas,", "index: 1, robot: bestla,"), kTeam2,
                 "has two lists");
}

TEST(CoveragePlan, IndexIsThePosition) {
  expectRejected(edit(planText(2), "index: 1,", "index: 0,"), kTeam2, ".index is 0");
}

// --- Check 3: grid ----------------------------------------------------------

TEST(CoveragePlan, Check3CellSizeAndCounts) {
  expectRejected(edit(planText(2), "cell_size_m: 20.0", "cell_size_m: 10.0"), kTeam2, "grid is");
  expectRejected(edit(planText(2), "nx: 5,", "nx: 4,"), kTeam2, "grid is");
  expectRejected(edit(planText(2), "ny: 5,", "ny: 6,"), kTeam2, "grid is");
  // A node running 10 m cells refuses a 20 m plan.
  expectRejected(planText(2), kTeam2, "grid is",
                 makeCellGrid(-50.0f, 50.0f, -50.0f, 50.0f, 10.0f));
  expectRejected(planText(2), kTeam2, "cell grid is empty", CellGrid{});
}

// configHash covers min_x/min_y but not the far edges, so each is its own check.
TEST(CoveragePlan, Check3Roi) {
  const std::string roi = "roi: [-50.0, 50.0, -50.0, 50.0]";
  expectRejected(edit(planText(2), roi, "roi: [-50.0, 50.0, -50.0, 40.0]"), kTeam2,
                 "grid.roi does not match");
  expectRejected(edit(planText(2), roi, "roi: [-40.0, 50.0, -50.0, 50.0]"), kTeam2,
                 "grid.roi does not match");
  expectRejected(edit(planText(2), roi, "roi: [-50.0, 50.0, -50.0]"), kTeam2, "grid needs");
}

TEST(CoveragePlan, Check3GridHash) {
  expectRejected(edit(planText(2), "grid_hash: 0x4c6adaf7", "grid_hash: 0x4c6adaf8"), kTeam2,
                 "grid_hash 0x4c6adaf8");
  expectRejected(edit(planText(2), "grid_hash: 0x4c6adaf7", "grid_hash: -1"), kTeam2, "grid needs");
  expectRejected(edit(planText(2), "grid_hash: 0x4c6adaf7", "grid_hash: 0x14c6adaf7"), kTeam2,
                 "grid needs");
}

TEST(CoveragePlan, Check3YawMustBeZero) {
  expectRejected(edit(planText(2), "yaw_deg: 0.0", "yaw_deg: 90.0"), kTeam2, "yaw_deg must be 0");
  expectRejected(edit(planText(2), "yaw_deg: 0.0", "yaw_deg: .nan"), kTeam2, "yaw_deg must be 0");
}

TEST(CoveragePlan, HomeMustBeInsideTheGrid) {
  expectRejected(edit(planText(2), "home: [2, 2]", "home: [5, 2]"), kTeam2, "home must be");
}

// --- Check 4: ids == row*nx + col, cells inside the grid --------------------

TEST(CoveragePlan, Check4IdsMatchCells) {
  expectRejected(edit(planText(2), "ids: [17, 22,", "ids: [17, 23,"), kTeam2,
                 "lists[0].ids[1] is 23, expected 22");
  expectRejected(edit(planText(2), "ids: [17, 22,", "ids: [17, 22.5,"), kTeam2, "integer id");
  expectRejected(edit(planText(2), "11, 16, 12]}", "11, 16]}"), kTeam2, "one id per cell");
}

// [5,0] has the in-range id 5 = 0*5+5, so only the cell range test sees it.
TEST(CoveragePlan, Check4CellsInsideTheGrid) {
  expectRejected(edit(planText(2), "[0,2], [0,1], [0,0]", "[0,2], [5,0], [0,0]"), kTeam2,
                 "lists[0].cells[6] [5,0] is outside the grid");
}

// --- Check 5: beside home, 4-neighbour steps, home last ------------------------

TEST(CoveragePlan, Check5EndsAtHome) {
  const std::string t = edit(edit(planText(2), "[1,2], [1,3], [2,2]]", "[1,2], [1,3]]"),
                             "11, 16, 12]}", "11, 16]}");
  expectRejected(t, kTeam2, "lists[0] must end at home [2,2]");
}

TEST(CoveragePlan, Check5FourNeighbourSteps) {
  const std::string t = edit(edit(planText(2), "[0,1], [0,0], [1,0]", "[0,1], [1,0], [0,0]"),
                             "5, 0, 1, 6", "5, 1, 0, 6");
  expectRejected(t, kTeam2, "steps [0,1] -> [1,0], not a 4-neighbour");
}

// The installed plans end two cells from home (the drive home is transit), so
// they load only because that step is exempt; this pins it on a 3x3 grid.
TEST(CoveragePlan, Check5StepIntoHomeIsExempt) {
  const CellGrid g3 = makeCellGrid(-30.0f, 30.0f, -30.0f, 30.0f, 20.0f);
  const std::string t =
      "coverage_plan:\n"
      "  version: 1\n  n_robots: 2\n  source_sha256: abc\n"
      "  grid: {cell_size_m: 20.0, roi: [-30.0, 30.0, -30.0, 30.0], nx: 3, ny: 3, yaw_deg: 0.0, "
      "grid_hash: " + std::to_string(g3.configHash()) + "}\n"
      "  home: [1, 1]\n"
      "  lists:\n"
      "    - {index: 0, robot: atlas, start_dir: N, helps: 1, helps_robot: bestla,\n"
      "       cells: [[1,2], [0,2], [0,1], [0,0], [1,1]], ids: [7, 6, 3, 0, 4]}\n"
      "    - {index: 1, robot: bestla, start_dir: S, helps: 0, helps_robot: atlas,\n"
      "       cells: [[1,0], [2,0], [2,1], [2,2], [1,1]], ids: [1, 2, 5, 8, 4]}\n";
  CoveragePlan p;
  std::string err;
  ASSERT_TRUE(parseCoveragePlan(t, g3, kTeam2, &p, &err)) << err;
  EXPECT_EQ(p.home_id, 4);
  EXPECT_EQ(p.lists[1], (std::vector<int>{1, 2, 5, 8, 4}));
}

// List 0 reversed is still a valid path that tiles, but starts two cells out.
TEST(CoveragePlan, Check5StartsBesideHome) {
  const std::string t = edit(
      edit(planText(2),
           "cells: [[2,3], [2,4], [1,4], [0,4], [0,3], [0,2], [0,1], [0,0], [1,0], "
           "[1,1], [1,2], [1,3], [2,2]]",
           "cells: [[1,3], [1,2], [1,1], [1,0], [0,0], [0,1], [0,2], [0,3], [0,4], "
           "[1,4], [2,4], [2,3], [2,2]]"),
      "ids: [17, 22, 21, 20, 15, 10, 5, 0, 1, 6, 11, 16, 12]",
      "ids: [16, 11, 6, 1, 0, 5, 10, 15, 20, 21, 22, 17, 12]");
  expectRejected(t, kTeam2, "lists[0] starts at [1,3], not beside home");
}

TEST(CoveragePlan, Check5StartDirNamesTheFirstCell) {
  expectRejected(edit(planText(2), "robot: bestla, start_dir: N", "robot: bestla, start_dir: E"),
                 kTeam2, "start_dir is E but its first cell is N of home");
}

// --- Check 6: no revisits, home only last -------------------------------------

TEST(CoveragePlan, Check6NoRevisits) {
  const std::string t = edit(edit(planText(2), "[1,2], [1,3], [2,2]]", "[1,2], [1,1], [2,2]]"),
                             "11, 16, 12]}", "11, 6, 12]}");
  expectRejected(t, kTeam2, "lists[0] visits [1,1] twice");
}

TEST(CoveragePlan, Check6HomeOnlyLast) {
  const std::string t = edit(edit(planText(2), "[1,2], [1,3], [2,2]]", "[1,2], [2,2], [2,2]]"),
                             "11, 16, 12]}", "11, 12, 12]}");
  expectRejected(t, kTeam2, "visits [2,2] twice (home only ends a list)");
}

// --- Check 7: the lists tile every non-home cell --------------------------------

TEST(CoveragePlan, Check7Disjoint) {
  const std::string t = edit(edit(planText(4), "[0,1], [1,1], [2,2]]", "[0,1], [0,2], [2,2]]"),
                             "ids: [7, 2, 1, 0, 5, 6, 12]", "ids: [7, 2, 1, 0, 5, 10, 12]");
  expectRejected(t, kTeam4, "cell [0,2] is on lists 0 and 2");
}

TEST(CoveragePlan, Check7EveryCellOwned) {
  const std::string t = edit(edit(planText(4), "[0,1], [1,1], [2,2]]", "[0,1], [2,2]]"),
                             "ids: [7, 2, 1, 0, 5, 6, 12]", "ids: [7, 2, 1, 0, 5, 12]");
  expectRejected(t, kTeam4, "cell [1,1] is on no list");
}

// --- Check 8: helps is a derangement, helps_robot agrees -------------------------

TEST(CoveragePlan, Check8NoSelfHelp) {
  expectRejected(edit(planText(2), "helps: 1, helps_robot: atlas", "helps: 0, helps_robot: bestla"),
                 kTeam2, "lists[0].helps is 0");
}

TEST(CoveragePlan, Check8HelpsInRange) {
  expectRejected(edit(planText(2), "helps: 1, helps_robot: atlas", "helps: 2, helps_robot: atlas"),
                 kTeam2, "lists[0].helps is 2");
  expectRejected(edit(planText(2), "helps: 1, helps_robot: atlas", "helps: -1, helps_robot: atlas"),
                 kTeam2, "lists[0].helps is -1");
}

// N=3 list 1 moved onto list 2: list 2 then has two helpers and list 0 none.
TEST(CoveragePlan, Check8Permutation) {
  expectRejected(edit(planText(3), "helps: 0, helps_robot: husky", "helps: 2, helps_robot: bestla"),
                 kTeam3, "list 2 already has a helper");
}

TEST(CoveragePlan, Check8HelpsRobotAgrees) {
  expectRejected(edit(planText(2), "helps: 1, helps_robot: atlas", "helps: 1, helps_robot: bestla"),
                 kTeam2, "lists[0].helps_robot is 'bestla'");
}

// --- planWalk ---------------------------------------------------------------

TEST(PlanWalk, FreshWalkIsTheOwnListWithHomeLast) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(2, &p));
  const PlanWalk w = walk(p, 0, false, {});
  EXPECT_EQ(w.remaining, p.lists[0]);
  EXPECT_EQ(w.remaining.back(), 12);
  EXPECT_EQ(w.route_cells, p.lists[0]);
  EXPECT_FALSE(w.helping);
  EXPECT_FALSE(w.help_started_now);
  EXPECT_EQ(w.help_list, -1);
  EXPECT_EQ(w.help_target_cell, -1);
}

// k is the first unfinished cell and k+1 the NEXT unfinished one, not the next
// stored one (BUILD_PLAN R12). No re-order (CL P3): the stored order holds.
TEST(PlanWalk, SkipsFinishedCellsInStoredOrder) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(2, &p));
  const PlanWalk w = walk(p, 0, false, {17, 21, 0, 6});
  EXPECT_EQ(w.remaining, (std::vector<int>{22, 20, 15, 10, 5, 1, 11, 16, 12}));
  // Route cells keep the finished ones: the cell before k is what looks into k.
  EXPECT_EQ(w.route_cells, p.lists[0]);
  const PlanWalk w2 = walk(p, 0, false, {22});
  ASSERT_GE(w2.remaining.size(), 2u);
  EXPECT_EQ(w2.remaining[0], 17);
  EXPECT_EQ(w2.remaining[1], 21);
}

// A written-off cell is a sticky COVERED, so the caller's finished() says yes
// and the walk skips it like any other finished cell.
TEST(PlanWalk, WrittenOffCellsAreSkipped) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(3, &p));
  const std::set<int> covered{7}, written_off{8, 4};
  const PlanWalk w = planWalk(p, 0, false, [&](int c) {
    return covered.count(c) > 0 || written_off.count(c) > 0;
  });
  EXPECT_EQ(w.remaining, (std::vector<int>{9, 3, 2, 1, 0, 12}));
}

// Help starts only once every non-home own cell is finished; home does not hold it.
TEST(PlanWalk, SwitchesToHelpOnlyWhenOwnJobsAreDone) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(4, &p));
  Finished done = jobsOf(p, 2);
  done.erase(16);   // list 2's last job cell
  PlanWalk w = walk(p, 2, false, done);
  EXPECT_FALSE(w.helping);
  EXPECT_FALSE(w.help_started_now);
  EXPECT_EQ(w.remaining, (std::vector<int>{16, 12}));

  done.insert(16);  // home (12) still unfinished
  w = walk(p, 2, false, done);
  EXPECT_TRUE(w.helping);
  EXPECT_TRUE(w.help_started_now);
  EXPECT_EQ(w.help_list, 3);
  EXPECT_EQ(w.help_target_cell, 18);
}

// The helped list walked from its back, home last; stand cells are the helped list.
TEST(PlanWalk, HelperWalkIsReversedEndingInHome) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(2, &p));
  const PlanWalk w = walk(p, 0, false, jobsOf(p, 0));
  ASSERT_TRUE(w.helping);
  EXPECT_EQ(w.help_list, 1);
  EXPECT_EQ(w.remaining, (std::vector<int>{8, 13, 18, 23, 24, 19, 14, 9, 4, 3, 2, 7, 12}));
  EXPECT_EQ(w.route_cells, p.lists[1]);
  EXPECT_EQ(w.help_target_cell, 8);
}

// Cells the owner already did are skipped; k+1 is the next unfinished from the back.
TEST(PlanWalk, HelperSkipsTheOwnersFinishedCells) {
  CoveragePlan p;   // atlas (list 1) helps husky (list 0)
  ASSERT_NO_FATAL_FAILURE(loadPlan(3, &p));
  Finished done = jobsOf(p, 1);
  done.insert({0, 1, 9});
  const PlanWalk w = walk(p, 1, true, done);
  EXPECT_EQ(w.remaining, (std::vector<int>{2, 3, 4, 8, 7, 12}));
  EXPECT_EQ(w.help_target_cell, 0);   // the target is the stored back, finished or not
}

// Once helping, an own cell going back to EXPLORING does not pull the robot home.
TEST(PlanWalk, HelpIsSticky) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(2, &p));
  const PlanWalk first = walk(p, 1, false, jobsOf(p, 1));
  ASSERT_TRUE(first.helping);
  EXPECT_TRUE(first.help_started_now);
  Finished done = jobsOf(p, 1);
  done.erase(24);
  const PlanWalk again = walk(p, 1, first.helping, done);
  EXPECT_TRUE(again.helping);
  EXPECT_FALSE(again.help_started_now);
  EXPECT_EQ(again.help_list, 0);
  EXPECT_EQ(again.remaining.front(), 16);
  EXPECT_EQ(again.route_cells, p.lists[0]);
  // Still helping, and still not "started now", with nothing finished at all.
  const PlanWalk none = walk(p, 1, true, {});
  EXPECT_TRUE(none.helping);
  EXPECT_FALSE(none.help_started_now);
}

// Spec §7.2: each owner list's helper heads for the owner's last cell before home.
TEST(PlanWalk, HelpTargetCellPerTeamSize) {
  const std::vector<std::vector<int>> target{{16, 8}, {0, 6, 18}, {6, 8, 16, 18}};
  for (int n : {2, 3, 4}) {
    CoveragePlan p;
    ASSERT_NO_FATAL_FAILURE(loadPlan(n, &p));
    for (int owner = 0; owner < n; ++owner) {
      SCOPED_TRACE("N=" + std::to_string(n) + " owner " + std::to_string(owner));
      const int helper = static_cast<int>(
          std::find(p.helps.begin(), p.helps.end(), owner) - p.helps.begin());
      ASSERT_LT(helper, n);
      const PlanWalk w = walk(p, helper, true, {});
      EXPECT_EQ(w.help_list, owner);
      EXPECT_EQ(w.help_target_cell, target[n - 2][owner]);
      EXPECT_EQ(w.remaining.front(), target[n - 2][owner]);
      EXPECT_EQ(w.remaining.back(), 12);
    }
  }
}

TEST(PlanWalk, EverythingFinishedLeavesNothing) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(3, &p));
  Finished all;
  for (int c = 0; c < 25; ++c) all.insert(c);
  const PlanWalk w = walk(p, 2, false, all);
  EXPECT_TRUE(w.helping);
  EXPECT_TRUE(w.remaining.empty());
  EXPECT_EQ(w.route_cells, p.lists[1]);
}

TEST(PlanWalk, HomeIsLastAndDroppedOnceFinished) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(4, &p));
  Finished done = jobsOf(p, 0);
  done.erase(6);
  EXPECT_EQ(walk(p, 0, false, done).remaining, (std::vector<int>{6, 12}));
  done.insert(12);
  EXPECT_EQ(walk(p, 0, false, done).remaining, (std::vector<int>{6}));
}

TEST(PlanWalk, UnknownListGivesAnEmptyWalk) {
  CoveragePlan p;
  ASSERT_NO_FATAL_FAILURE(loadPlan(2, &p));
  for (int bad : {-1, 2}) {
    const PlanWalk w = walk(p, bad, false, {});
    EXPECT_TRUE(w.remaining.empty());
    EXPECT_TRUE(w.route_cells.empty());
    EXPECT_FALSE(w.helping);
  }
}

// Random finished sets against the definition: remaining is the walk with the
// finished cells removed, and help starts exactly when the own jobs are done.
TEST(PlanWalk, RandomFinishedSetsMatchTheDefinition) {
  std::mt19937 rng(20261012);
  std::bernoulli_distribution coin(0.7);
  for (int n : {2, 3, 4}) {
    CoveragePlan p;
    ASSERT_NO_FATAL_FAILURE(loadPlan(n, &p));
    for (int trial = 0; trial < 300; ++trial) {
      Finished done;
      for (int c = 0; c < 25; ++c) {
        if (coin(rng)) done.insert(c);
      }
      for (int me = 0; me < n; ++me) {
        const auto& own = p.lists[me];
        const bool jobs_done = std::all_of(own.begin(), own.end() - 1,
                                           [&](int c) { return done.count(c) > 0; });
        const PlanWalk w = walk(p, me, false, done);
        ASSERT_EQ(w.helping, jobs_done);
        ASSERT_EQ(w.help_started_now, jobs_done);
        std::vector<int> expect_walk = own;
        if (jobs_done) {
          const auto& L = p.lists[p.helps[me]];
          expect_walk.assign(L.rbegin() + 1, L.rend());
          expect_walk.push_back(12);
        }
        std::vector<int> expect;
        for (int c : expect_walk) {
          if (!done.count(c)) expect.push_back(c);
        }
        ASSERT_EQ(w.remaining, expect);
      }
    }
  }
}
