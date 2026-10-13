#include "explo_planner/coverage_plan.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <sstream>
#include <utility>

namespace explo_planner {

namespace {

/// Metres to millimetres, as CellGrid::configHash quantises them.
int64_t mm(double v) { return std::llround(v * 1000.0); }

/// A scalar as T by yaml-cpp's own conversion, which reads 0x hex and a
/// leading-0 octal as PyYAML does in the harness check. False when absent or
/// not a T.
template <typename T>
bool read(const YAML::Node& n, T* v) {
  if (!n || !n.IsScalar()) return false;
  try {
    *v = n.as<T>();
  } catch (const YAML::Exception&) {
    return false;
  }
  return true;
}

bool readString(const YAML::Node& n, std::string* v) {
  if (!n || !n.IsScalar() || n.Scalar().empty()) return false;
  *v = n.Scalar();
  return true;
}

std::string cellStr(int col, int row) {
  return "[" + std::to_string(col) + "," + std::to_string(row) + "]";
}

/// The checks, numbered as in spec_harness §7.4. *err names the first failure.
/// A missing key reads as a zombie node, whose Is*() throws, so presence is
/// tested first.
bool parsePlan(const YAML::Node& root, const CellGrid& grid,
               const std::vector<std::string>& team_names, CoveragePlan* plan,
               std::string* err) {
  if (!root.IsMap() || !root["coverage_plan"] || !root["coverage_plan"].IsMap()) {
    *err = "no coverage_plan map at the top level";
    return false;
  }
  const YAML::Node cp = root["coverage_plan"];
  const int n_team = static_cast<int>(team_names.size());

  // 1. Version and team size.
  if (!read(cp["version"], &plan->version) || plan->version != 1) {
    *err = "version must be 1";
    return false;
  }
  if (!read(cp["n_robots"], &plan->n_robots) || plan->n_robots != n_team) {
    *err = "n_robots must equal the team size " + std::to_string(n_team);
    return false;
  }
  if (!readString(cp["source_sha256"], &plan->source_sha256)) {
    *err = "source_sha256 is missing";
    return false;
  }

  // 3. Grid. The ROI is compared with the grid's own extent, in millimetres as
  // configHash compares geometry.
  if (grid.size() <= 0) {
    *err = "the node's cell grid is empty";
    return false;
  }
  const YAML::Node g = cp["grid"];
  double cell = 0.0, yaw = 0.0, roi[4] = {0.0, 0.0, 0.0, 0.0};
  int nx = 0, ny = 0;
  uint32_t hash = 0;
  if (!g || !g.IsMap() || !read(g["cell_size_m"], &cell) || !read(g["nx"], &nx) ||
      !read(g["ny"], &ny) || !read(g["yaw_deg"], &yaw) ||
      !read(g["grid_hash"], &hash) || !g["roi"] || !g["roi"].IsSequence() ||
      g["roi"].size() != 4) {
    *err = "grid needs cell_size_m, roi [min_x, max_x, min_y, max_y], nx, ny, "
           "yaw_deg and grid_hash";
    return false;
  }
  for (int i = 0; i < 4; ++i) {
    if (!read(g["roi"][i], &roi[i])) {
      *err = "grid.roi[" + std::to_string(i) + "] is not a number";
      return false;
    }
  }
  if (mm(cell) != mm(grid.cell_size_m) || nx != grid.nx || ny != grid.ny) {
    std::ostringstream s;
    s << "grid is " << nx << "x" << ny << " cells of " << cell << " m; the node's is "
      << grid.nx << "x" << grid.ny << " of " << grid.cell_size_m << " m";
    *err = s.str();
    return false;
  }
  if (mm(roi[0]) != mm(grid.min_x) || mm(roi[1]) != mm(grid.maxX()) ||
      mm(roi[2]) != mm(grid.min_y) || mm(roi[3]) != mm(grid.maxY())) {
    *err = "grid.roi does not match the node's cell grid";
    return false;
  }
  if (hash != grid.configHash()) {
    std::ostringstream s;
    s << "grid_hash 0x" << std::hex << hash << " != the node's configHash() 0x"
      << grid.configHash();
    *err = s.str();
    return false;
  }
  // Rotation is future work (CL Q7): idAt is axis-aligned.
  if (yaw != 0.0) {
    *err = "grid.yaw_deg must be 0";
    return false;
  }

  const YAML::Node home = cp["home"];
  int hc = -1, hr = -1;
  if (!home || !home.IsSequence() || home.size() != 2 || !read(home[0], &hc) ||
      !read(home[1], &hr) || hc < 0 || hc >= nx || hr < 0 || hr >= ny) {
    *err = "home must be [col, row] inside the grid";
    return false;
  }
  plan->home_id = hr * nx + hc;

  const YAML::Node lists = cp["lists"];
  if (!lists || !lists.IsSequence() || static_cast<int>(lists.size()) != plan->n_robots) {
    *err = "lists must hold n_robots = " + std::to_string(plan->n_robots) + " entries";
    return false;
  }
  const int n = plan->n_robots;
  std::vector<std::string> helps_robot(n);
  std::vector<int> owner(grid.size(), -1);
  plan->lists.assign(n, {});
  plan->robots.assign(n, "");
  plan->helps.assign(n, -1);
  for (int i = 0; i < n; ++i) {
    const YAML::Node l = lists[i];
    const std::string at = "lists[" + std::to_string(i) + "]";
    int index = -1;
    std::string start_dir;
    if (!l.IsMap() || !read(l["index"], &index) || !readString(l["robot"], &plan->robots[i]) ||
        !readString(l["start_dir"], &start_dir) || !read(l["helps"], &plan->helps[i]) ||
        !readString(l["helps_robot"], &helps_robot[i]) || !l["cells"] ||
        !l["cells"].IsSequence() || !l["ids"] || !l["ids"].IsSequence()) {
      *err = at + " needs index, robot, start_dir, helps, helps_robot, cells and ids";
      return false;
    }
    // helps names lists by position, so the index must be the position.
    if (index != i) {
      *err = at + ".index is " + std::to_string(index);
      return false;
    }

    // 2. The list robots are the team, one list each.
    bool in_team = false;
    for (const std::string& t : team_names) in_team = in_team || t == plan->robots[i];
    if (!in_team) {
      *err = at + ".robot '" + plan->robots[i] + "' is not in team_robot_names";
      return false;
    }
    for (int j = 0; j < i; ++j) {
      if (plan->robots[j] == plan->robots[i]) {
        *err = "robot '" + plan->robots[i] + "' has two lists";
        return false;
      }
    }

    // 4. ids == row*nx + col, every cell inside the grid.
    const YAML::Node cells = l["cells"], ids = l["ids"];
    if (cells.size() < 2 || ids.size() != cells.size()) {
      *err = at + " needs at least one cell before home, and one id per cell";
      return false;
    }
    std::vector<int> cols, rows;
    for (size_t j = 0; j < cells.size(); ++j) {
      const std::string cj = at + ".cells[" + std::to_string(j) + "]";
      int c = -1, r = -1, id = -1;
      if (!cells[j].IsSequence() || cells[j].size() != 2 || !read(cells[j][0], &c) ||
          !read(cells[j][1], &r) || !read(ids[j], &id)) {
        *err = cj + " must be [col, row] with an integer id";
        return false;
      }
      if (c < 0 || c >= nx || r < 0 || r >= ny) {
        *err = cj + " " + cellStr(c, r) + " is outside the grid";
        return false;
      }
      if (id != r * nx + c) {
        *err = at + ".ids[" + std::to_string(j) + "] is " + std::to_string(id) + ", expected " +
               std::to_string(r * nx + c) + " for " + cellStr(c, r);
        return false;
      }
      cols.push_back(c);
      rows.push_back(r);
      plan->lists[i].push_back(id);
    }

    // 5. Starts beside home in start_dir, steps 4-neighbour, ends at home. The
    // step into home is transit and exempt.
    const std::vector<int>& L = plan->lists[i];
    const size_t last = L.size() - 1;
    if (L[last] != plan->home_id) {
      *err = at + " must end at home " + cellStr(hc, hr);
      return false;
    }
    const int dc = cols[0] - hc, dr = rows[0] - hr;
    const char* dir = dc == 0 && dr == 1 ? "N" : dc == 0 && dr == -1 ? "S"
                    : dc == 1 && dr == 0 ? "E" : dc == -1 && dr == 0 ? "W" : "";
    if (*dir == '\0') {
      *err = at + " starts at " + cellStr(cols[0], rows[0]) + ", not beside home";
      return false;
    }
    if (start_dir != dir) {
      *err = at + ".start_dir is " + start_dir + " but its first cell is " + dir + " of home";
      return false;
    }
    for (size_t j = 0; j + 1 < last; ++j) {
      if (std::abs(cols[j + 1] - cols[j]) + std::abs(rows[j + 1] - rows[j]) != 1) {
        *err = at + " steps " + cellStr(cols[j], rows[j]) + " -> " +
               cellStr(cols[j + 1], rows[j + 1]) + ", not a 4-neighbour";
        return false;
      }
    }

    // 6. No revisits; home only as the last entry. 7. Disjoint across lists.
    std::vector<int> pos(grid.size(), -1);
    for (size_t j = 0; j <= last; ++j) {
      const int id = L[j];
      if (pos[id] >= 0) {
        *err = at + " visits " + cellStr(cols[j], rows[j]) + " twice" +
               (id == plan->home_id ? " (home only ends a list)" : "");
        return false;
      }
      pos[id] = static_cast<int>(j);
      if (id == plan->home_id) continue;
      if (owner[id] >= 0) {
        *err = "cell " + cellStr(cols[j], rows[j]) + " is on lists " +
               std::to_string(owner[id]) + " and " + std::to_string(i);
        return false;
      }
      owner[id] = i;
    }
  }

  // 7. The lists tile every cell but home.
  for (int id = 0; id < grid.size(); ++id) {
    if (id != plan->home_id && owner[id] < 0) {
      *err = "cell " + cellStr(grid.col(id), grid.row(id)) + " is on no list";
      return false;
    }
  }

  // 8. helps is a permutation with no fixed point; helps_robot agrees.
  std::vector<bool> helped(n, false);
  for (int i = 0; i < n; ++i) {
    const int h = plan->helps[i];
    const std::string at = "lists[" + std::to_string(i) + "].helps";
    if (h < 0 || h >= n || h == i) {
      *err = at + " is " + std::to_string(h) + "; it must name another list";
      return false;
    }
    if (helped[h]) {
      *err = at + ": list " + std::to_string(h) + " already has a helper";
      return false;
    }
    helped[h] = true;
    if (helps_robot[i] != plan->robots[h]) {
      *err = at + "_robot is '" + helps_robot[i] + "' but list " + std::to_string(h) +
             " is " + plan->robots[h] + "'s";
      return false;
    }
  }
  return true;
}

}  // namespace

bool parseCoveragePlan(const std::string& yaml_text, const CellGrid& grid,
                       const std::vector<std::string>& team_names, CoveragePlan* out,
                       std::string* err) {
  std::string local_err;
  if (!err) err = &local_err;
  err->clear();
  CoveragePlan plan;
  try {
    if (!parsePlan(YAML::Load(yaml_text), grid, team_names, &plan, err)) return false;
  } catch (const std::exception& e) {
    *err = std::string("YAML: ") + e.what();
    return false;
  }
  if (out) *out = std::move(plan);
  return true;
}

bool loadCoveragePlanFile(const std::string& path, const CellGrid& grid,
                          const std::vector<std::string>& team_names, CoveragePlan* out,
                          std::string* err) {
  std::string local_err;
  if (!err) err = &local_err;
  std::ifstream f(path);
  if (!f) {
    *err = path + ": cannot read";
    return false;
  }
  std::stringstream s;
  s << f.rdbuf();
  if (!parseCoveragePlan(s.str(), grid, team_names, out, err)) {
    *err = path + ": " + *err;
    return false;
  }
  return true;
}

int planListOf(const CoveragePlan& p, const std::string& robot) {
  for (size_t i = 0; i < p.robots.size(); ++i) {
    if (p.robots[i] == robot) return static_cast<int>(i);
  }
  return -1;
}

PlanWalk planWalk(const CoveragePlan& p, int my_list, bool helping_in,
                  const std::function<bool(int)>& finished) {
  PlanWalk w;
  const int n = static_cast<int>(p.lists.size());
  if (my_list < 0 || my_list >= n) return w;
  const std::vector<int>& own = p.lists[my_list];

  // Home ends every list but does not hold the switch: it is everyone's.
  w.helping = helping_in;
  if (!w.helping) {
    w.helping = true;
    for (size_t i = 0; i + 1 < own.size(); ++i) {
      if (!finished(own[i])) {
        w.helping = false;
        break;
      }
    }
    w.help_started_now = w.helping;
  }

  std::vector<int> walk;
  if (!w.helping) {
    walk = own;
    w.route_cells = own;
  } else if (my_list < static_cast<int>(p.helps.size()) && p.helps[my_list] >= 0 &&
             p.helps[my_list] < n) {
    // From the back (CL Q5): the helped list reversed, home kept last.
    w.help_list = p.helps[my_list];
    const std::vector<int>& L = p.lists[w.help_list];
    if (!L.empty()) {
      walk.assign(L.rbegin() + 1, L.rend());
      walk.push_back(L.back());
    }
    if (L.size() >= 2) w.help_target_cell = L[L.size() - 2];
    w.route_cells = L;
  }
  for (const int c : walk) {
    if (!finished(c)) w.remaining.push_back(c);
  }
  return w;
}

}  // namespace explo_planner
