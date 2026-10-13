/// @file route_builder.cpp
/// @brief Definitions for the 2d_route viewpoint builder (see header).

#include "explo_planner/route_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>

#include "explo_planner/grid_frontier.hpp"

namespace explo_planner {

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();

// A lattice this large is a misconfigured ROI, not a route.
constexpr double kMaxLatticePoints = 1 << 20;

bool mapUsable(const nav_msgs::msg::OccupancyGrid& g) {
  return g.info.resolution > 0.0f && g.info.width > 0 && g.info.height > 0 &&
         g.data.size() == static_cast<size_t>(g.info.width) *
                              static_cast<size_t>(g.info.height);
}

float xyDist(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  return std::hypot(a.x() - b.x(), a.y() - b.y());
}

// Cheapest slot for v in an open path from the start: 0 is before the head,
// k is after path[k - 1]. Ties go to the earliest slot.
size_t cheapestSlot(const std::vector<int>& path, int v,
                    const std::vector<float>& from_start,
                    const std::vector<std::vector<float>>& d) {
  if (path.empty()) return 0;
  size_t best = 0;
  float best_c = from_start[v] + d[v][path[0]] - from_start[path[0]];
  for (size_t k = 1; k < path.size(); ++k) {
    const float c = d[path[k - 1]][v] + d[v][path[k]] - d[path[k - 1]][path[k]];
    if (c < best_c) { best_c = c; best = k; }
  }
  if (d[path.back()][v] < best_c) best = path.size();
  return best;
}

}  // namespace

std::vector<Eigen::Vector2f> routeLattice(const Roi2D& roi, float spacing_m) {
  std::vector<Eigen::Vector2f> pts;
  if (!(spacing_m > 0.0f) || !std::isfinite(spacing_m)) return pts;
  const double nx =
      std::floor((static_cast<double>(roi.max_x) - roi.min_x) / spacing_m);
  const double ny =
      std::floor((static_cast<double>(roi.max_y) - roi.min_y) / spacing_m);
  if (!(nx >= 1.0) || !(ny >= 1.0) || nx * ny > kMaxLatticePoints) return pts;
  // Half a spacing in from the corner: no point sits on a map-cell or grid
  // boundary, so float and double lookups agree. (notes: route-lattice-offset)
  pts.reserve(static_cast<size_t>(nx * ny));
  for (int j = 0; j < static_cast<int>(ny); ++j) {
    const float y = roi.min_y + (j + 0.5f) * spacing_m;
    for (int i = 0; i < static_cast<int>(nx); ++i)
      pts.emplace_back(roi.min_x + (i + 0.5f) * spacing_m, y);
  }
  return pts;
}

float distToCellBox(const CellGrid& g, int id, float x, float y) {
  if (!g.valid(id)) return kInf;
  const double x0 = g.min_x + static_cast<double>(g.col(id)) * g.cell_size_m;
  const double y0 = g.min_y + static_cast<double>(g.row(id)) * g.cell_size_m;
  const double dx = std::max({x0 - x, 0.0, x - (x0 + g.cell_size_m)});
  const double dy = std::max({y0 - y, 0.0, y - (y0 + g.cell_size_m)});
  return static_cast<float>(std::hypot(dx, dy));
}

int minTierCells(float min_gain_m2, float resolution) {
  if (!(resolution > 0.0f) || std::isnan(min_gain_m2))
    return std::numeric_limits<int>::max();
  // Integer compare against cell counts; the 1e-6 keeps float rounding just
  // above a whole count from costing an extra cell.
  const double cells =
      std::ceil(static_cast<double>(min_gain_m2) /
                    (static_cast<double>(resolution) * resolution) - 1e-6);
  return static_cast<int>(std::clamp(
      cells, 1.0, static_cast<double>(std::numeric_limits<int>::max())));
}

std::vector<int> lazyGreedyCover(
    const std::vector<CoverSet>& sets, size_t universe, int min_tier_cells,
    int max_picks, std::vector<int>* marg_tier, std::vector<int>* marg_later,
    std::vector<std::vector<int32_t>>* marg_tier_sets) {
  std::vector<int> picks;
  if (marg_tier) marg_tier->clear();
  if (marg_later) marg_later->clear();
  if (marg_tier_sets) marg_tier_sets->clear();
  if (max_picks < 1 || sets.empty()) return picks;

  std::vector<uint8_t> covered(universe, 0);
  // (tier gain, later gain, -index): the heap top is the lexicographic max.
  using Key = std::tuple<int, int, int>;
  std::priority_queue<Key> heap;
  for (size_t k = 0; k < sets.size(); ++k)
    heap.emplace(static_cast<int>(sets[k].tier.size()),
                 static_cast<int>(sets[k].later.size()), -static_cast<int>(k));
  auto uncovered = [&](const std::vector<int32_t>& v) {
    int n = 0;
    for (int32_t i : v) n += !covered[static_cast<size_t>(i)];
    return n;
  };

  while (!heap.empty() && static_cast<int>(picks.size()) < max_picks) {
    const int k = -std::get<2>(heap.top());
    heap.pop();
    const CoverSet& s = sets[static_cast<size_t>(k)];
    const Key fresh{uncovered(s.tier), uncovered(s.later), -k};
    // Stored keys only overstate (gains only fall, component-wise), so a
    // fresh key at least the next stored one is the true max.
    if (!heap.empty() && fresh < heap.top()) {
      heap.push(fresh);
      continue;
    }
    if (std::get<0>(fresh) < min_tier_cells) break;
    std::vector<int32_t> marg;
    marg.reserve(static_cast<size_t>(std::get<0>(fresh)));
    for (int32_t i : s.tier) {
      if (covered[static_cast<size_t>(i)]) continue;
      covered[static_cast<size_t>(i)] = 1;
      marg.push_back(i);
    }
    for (int32_t i : s.later) covered[static_cast<size_t>(i)] = 1;
    picks.push_back(k);
    if (marg_tier) marg_tier->push_back(std::get<0>(fresh));
    if (marg_later) marg_later->push_back(std::get<1>(fresh));
    if (marg_tier_sets) marg_tier_sets->push_back(std::move(marg));
  }
  return picks;
}

std::vector<int> orderOpenRoute(const std::vector<float>& from_start,
                                const std::vector<std::vector<float>>& d) {
  const int n = static_cast<int>(from_start.size());
  std::vector<int> path;
  path.reserve(static_cast<size_t>(n));
  std::vector<uint8_t> in_path(static_cast<size_t>(n), 0);
  // Nearest insertion: the node nearest the start or the path so far, ties to
  // the lower index, goes in at its cheapest slot.
  for (int step = 0; step < n; ++step) {
    int v = -1;
    float best = kInf;
    for (int u = 0; u < n; ++u) {
      if (in_path[u]) continue;
      float du = from_start[u];
      for (int a : path) du = std::min(du, d[a][u]);
      if (v < 0 || du < best) { best = du; v = u; }
    }
    path.insert(path.begin() + static_cast<std::ptrdiff_t>(
                                   cheapestSlot(path, v, from_start, d)), v);
    in_path[v] = 1;
  }
  // 2-opt with the start fixed and the end open: reversing path[i..j] swaps
  // the leg into path[i] for one into path[j], and the leg out of path[j]
  // for one out of path[i]. d is symmetric, so the inside is unchanged.
  auto into = [&](int i, int node) {
    return i == 0 ? from_start[node] : d[path[i - 1]][node];
  };
  for (int pass = 0; pass < 50; ++pass) {
    bool improved = false;
    for (int i = 0; i + 1 < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
        const bool tail = j < n - 1;
        const float delta =
            into(i, path[j]) + (tail ? d[path[i]][path[j + 1]] : 0.0f) -
            into(i, path[i]) - (tail ? d[path[j]][path[j + 1]] : 0.0f);
        if (delta < -1e-4f) {
          std::reverse(path.begin() + i, path.begin() + j + 1);
          improved = true;
        }
      }
    }
    if (!improved) break;
  }
  return path;
}

float openRouteLength(const std::vector<int>& order,
                      const std::vector<float>& from_start,
                      const std::vector<std::vector<float>>& d) {
  if (order.empty()) return 0.0f;
  float len = from_start[order[0]];
  for (size_t k = 1; k < order.size(); ++k) len += d[order[k - 1]][order[k]];
  return len;
}

bool routeMaterialChange(const Eigen::Vector3f& new_head, float new_len_m,
                         const Eigen::Vector3f& cur_head,
                         float cur_remaining_len_m, const RouteParams& p) {
  if (xyDist(new_head, cur_head) > p.identityM()) return true;
  if (!std::isfinite(cur_remaining_len_m)) return true;
  return std::abs(new_len_m - cur_remaining_len_m) >
         p.reorder_gain * cur_remaining_len_m;
}

int routeStillUnknown(const nav_msgs::msg::OccupancyGrid& g,
                      const RouteMapGeom& geom,
                      const std::vector<int32_t>& cells) {
  if (static_cast<int>(g.info.width) != geom.width ||
      static_cast<int>(g.info.height) != geom.height ||
      g.info.origin.position.x != geom.origin_x ||
      g.info.origin.position.y != geom.origin_y ||
      g.info.resolution != geom.resolution || !mapUsable(g))
    return -1;
  int n = 0;
  for (int32_t i : cells)
    if (i >= 0 && static_cast<size_t>(i) < g.data.size() &&
        g.data[static_cast<size_t>(i)] < 0)
      ++n;
  return n;
}

RoutePlan buildRoute(const RouteInputs& in, const RouteParams& p,
                     CostGrid& pair_flood) {
  RoutePlan plan;
  if (in.map) {
    plan.geom.width = static_cast<int>(in.map->info.width);
    plan.geom.height = static_cast<int>(in.map->info.height);
    plan.geom.origin_x = in.map->info.origin.position.x;
    plan.geom.origin_y = in.map->info.origin.position.y;
    plan.geom.resolution = in.map->info.resolution;
  }
  const CellGrid* grid = in.grid;
  if (!in.map || !mapUsable(*in.map) || !grid || grid->size() <= 0 ||
      !in.robot_flood || !grid->valid(in.tier_cell[0]) ||
      p.max_viewpoints < 1)
    return plan;

  const auto& map = *in.map;
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  const double res = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;
  const int ncell = grid->size();
  const int tiers[2] = {in.tier_cell[0],
                        grid->valid(in.tier_cell[1]) ? in.tier_cell[1] : -1};

  // 1. Points: the lattice, then the frontier centres, at the robot's z.
  const auto lattice = routeLattice(in.roi, p.spacing_m);
  const int n_lattice = static_cast<int>(lattice.size());
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(lattice.size() + in.frontier_centres.size());
  for (const auto& q : lattice) pts.emplace_back(q.x(), q.y(), in.robot.z());
  for (const auto& f : in.frontier_centres)
    pts.emplace_back(f.x(), f.y(), in.robot.z());
  plan.n_points = static_cast<int>(pts.size());

  // 2-3. Stand in a route cell within range of a tier cell, then the walk's
  // filters in admissible() order, except the visited radius (RP §5.5).
  std::vector<uint8_t> in_route(static_cast<size_t>(ncell), 0);
  for (int c : in.route_cells)
    if (grid->valid(c)) in_route[static_cast<size_t>(c)] = 1;
  auto inRange = [&](int t, const Eigen::Vector3f& q) {
    return tiers[t] >= 0 &&
           distToCellBox(*grid, tiers[t], q.x(), q.y()) <= p.max_range;
  };
  const float close2 =
      static_cast<float>(p.min_goal_dist_m * p.min_goal_dist_m);
  RouteRejects& rj = plan.rejects;
  std::vector<int> adm;  // admissible point indices, ascending
  for (int k = 0; k < plan.n_points; ++k) {
    const Eigen::Vector3f& q = pts[static_cast<size_t>(k)];
    const int id = grid->idAt(q.x(), q.y());
    if (id < 0 || !in_route[static_cast<size_t>(id)] ||
        !(inRange(0, q) || inRange(1, q))) { ++rj.stand; continue; }
    const float dx = q.x() - in.robot.x(), dy = q.y() - in.robot.y();
    if (dx * dx + dy * dy < close2) { ++rj.too_close; continue; }
    if (!isCellFree(map, q)) { ++rj.map; continue; }
    if (!in.skip_reachability && !in.robot_flood->reachable(q)) {
      ++rj.unreachable;
      continue;
    }
    if (in.visited && in.visited->isNear(q, p.spacing_m)) {
      ++rj.visited;
      continue;
    }
    if (in.failed && in.failed->isNear(q, p.failed_radius_m)) {
      ++rj.failed;
      continue;
    }
    if (in.lost_to_claim && in.lost_to_claim(q)) { ++rj.claim; continue; }
    adm.push_back(k);
  }
  plan.n_admissible = static_cast<int>(adm.size());

  // 4. Coverage sets, by the census arithmetic (censusFromPlanMap): idAt is
  // separable, so a column and a row lookup per map index give its cell.
  std::vector<int> col_of(static_cast<size_t>(w));
  std::vector<int> row_of(static_cast<size_t>(h));
  {
    float cx0 = 0.0f, cy0 = 0.0f;
    grid->centre(0, cx0, cy0);
    for (int gx = 0; gx < w; ++gx) {
      const int id = grid->idAt(static_cast<float>(ox + (gx + 0.5) * res), cy0);
      col_of[static_cast<size_t>(gx)] = id < 0 ? -1 : grid->col(id);
    }
    for (int gy = 0; gy < h; ++gy) {
      const int id = grid->idAt(cx0, static_cast<float>(oy + (gy + 0.5) * res));
      row_of[static_cast<size_t>(gy)] = id < 0 ? -1 : grid->row(id);
    }
  }
  // Only tier and later cells are kept; the ROI-edge strip maps to no cell.
  std::vector<uint8_t> interest(static_cast<size_t>(ncell), 0);
  for (int t = 0; t < 2; ++t) {
    if (tiers[t] < 0) continue;
    interest[static_cast<size_t>(tiers[t])] = 1;
    for (int c : in.later_cells[t])
      if (grid->valid(c)) interest[static_cast<size_t>(c)] = 1;
  }
  // Counted cells lie within gridVisibleUnknown's window, ceil(t_max) + 1
  // cells of the start; one more for margin. They are marked in a band of
  // whole rows and read back row by row, which yields them sorted.
  const double t_max = p.max_range / res;
  const int win =
      t_max > 0.0
          ? static_cast<int>(std::min<double>(std::ceil(t_max) + 2.0, h))
          : 0;
  std::vector<uint8_t> band(
      static_cast<size_t>(std::min(2 * win + 1, h)) * w, 0);
  std::vector<int32_t> raw;
  struct Cov {
    bool done = false;
    std::vector<int32_t> idx;   // sorted map indices in a cell of interest
    std::vector<uint16_t> cid;  // the cell of each
  };
  std::vector<Cov> cov(adm.size());
  auto coverage = [&](size_t a) {
    Cov& cv = cov[a];
    cv.done = true;
    const Eigen::Vector3f& q = pts[static_cast<size_t>(adm[a])];
    raw.clear();
    gridVisibleUnknown(map, in.roi, q, 0.0f, p.hfov, p.h_rays, p.min_range,
                       p.max_range, &raw);
    if (raw.empty()) return;
    const int px = static_cast<int>(std::floor((q.x() - ox) / res));
    const int py = static_cast<int>(std::floor((q.y() - oy) / res));
    const int x_lo = std::max(0, px - win), x_hi = std::min(w - 1, px + win);
    const int y_lo = std::max(0, py - win), y_hi = std::min(h - 1, py + win);
    const int64_t base = static_cast<int64_t>(y_lo) * w;
    for (int32_t i : raw) band[static_cast<size_t>(i - base)] = 1;
    cv.idx.reserve(raw.size());
    cv.cid.reserve(raw.size());
    for (int gy = y_lo; gy <= y_hi; ++gy) {
      uint8_t* row = &band[static_cast<size_t>(gy - y_lo) * w];
      const int r = row_of[static_cast<size_t>(gy)];
      for (int gx = x_lo; gx <= x_hi; ++gx) {
        if (!row[gx]) continue;
        row[gx] = 0;
        const int c = col_of[static_cast<size_t>(gx)];
        if (r < 0 || c < 0) continue;
        const int id = r * grid->nx + c;
        if (!interest[static_cast<size_t>(id)]) continue;
        cv.idx.push_back(gy * w + gx);
        cv.cid.push_back(static_cast<uint16_t>(id));
      }
    }
  };

  // 5. Tier 1 members see min_gain inside k; tier 2 only if tier 1 has none.
  const int min_tier = minTierCells(p.min_gain_m2, map.info.resolution);
  std::vector<CoverSet> members;
  std::vector<size_t> member_adm;  // admissible ordinal of each member
  int tier = -1;
  for (int t = 0; t < 2 && tier < 0; ++t) {
    const int T = tiers[t];
    if (T < 0) continue;
    std::vector<uint8_t> later(static_cast<size_t>(ncell), 0);
    for (int c : in.later_cells[t])
      if (grid->valid(c)) later[static_cast<size_t>(c)] = 1;
    for (size_t a = 0; a < adm.size(); ++a) {
      if (!inRange(t, pts[static_cast<size_t>(adm[a])])) continue;
      if (!cov[a].done) coverage(a);
      CoverSet s;
      for (size_t e = 0; e < cov[a].idx.size(); ++e) {
        const int c = cov[a].cid[e];
        if (c == T)
          s.tier.push_back(cov[a].idx[e]);
        else if (later[static_cast<size_t>(c)])
          s.later.push_back(cov[a].idx[e]);
      }
      if (static_cast<int>(s.tier.size()) >= min_tier) {
        members.push_back(std::move(s));
        member_adm.push_back(a);
      }
      // The cache is only reused by tier 2, which runs only if tier 1 is empty.
      if (!(t == 0 && members.empty() && tiers[1] >= 0))
        cov[a] = Cov{true, {}, {}};
    }
    if (!members.empty()) tier = t;
  }
  if (tier < 0) {
    rj.below_tier = plan.n_admissible;
    return plan;
  }
  plan.tier = tier + 1;
  plan.served_cell = tiers[tier];
  plan.n_tier_members = static_cast<int>(members.size());
  rj.below_tier = plan.n_admissible - plan.n_tier_members;

  // 6. Cover inside the tier cell, later route cells breaking ties.
  std::vector<int> m_tier, m_later;
  std::vector<std::vector<int32_t>> m_sets;
  const std::vector<int> picks =
      lazyGreedyCover(members, static_cast<size_t>(w) * h, min_tier,
                      p.max_viewpoints, &m_tier, &m_later, &m_sets);
  const int n = static_cast<int>(picks.size());
  std::vector<int> point_of(static_cast<size_t>(n));
  std::vector<Eigen::Vector3f> pos(static_cast<size_t>(n));
  for (int v = 0; v < n; ++v) {
    point_of[v] = adm[member_adm[static_cast<size_t>(picks[v])]];
    pos[v] = pts[static_cast<size_t>(point_of[v])];
  }

  // 7. Legs: the robot flood for the first, early-stopping pair floods for the
  // rest. A pair the pair flood cannot join costs the way through the robot.
  std::vector<float> c_r(static_cast<size_t>(n));
  for (int v = 0; v < n; ++v)
    c_r[v] = in.skip_reachability ? xyDist(pos[v], in.robot)
                                  : in.robot_flood->costTo(pos[v]);
  std::vector<std::vector<float>> d(
      static_cast<size_t>(n), std::vector<float>(static_cast<size_t>(n), 0.0f));
  if (n >= 2) {
    pair_flood.build(map);
    for (int i = 0; i + 1 < n; ++i) {
      const std::vector<Eigen::Vector3f> later_picks(pos.begin() + i + 1,
                                                     pos.end());
      pair_flood.floodUntil(pos[i], later_picks, 0.0f);
      ++plan.n_floods;
      for (int j = i + 1; j < n; ++j) {
        float c = pair_flood.costTo(pos[j]);
        if (!std::isfinite(c)) c = c_r[i] + c_r[j];
        d[i][j] = d[j][i] = c;
      }
    }
  }

  // 8. Order. Continuity: picks within identity of a held waypoint keep the
  // held order, the rest go in at their cheapest slots, and that seed (no
  // 2-opt) stands unless the fresh order is reorder_gain shorter.
  std::vector<int> order = orderOpenRoute(c_r, d);
  if (!in.prev_remaining.empty()) {
    const auto& prev = in.prev_remaining;
    std::vector<std::tuple<float, int, int>> pairs;  // (distance, old, pick)
    for (int o = 0; o < static_cast<int>(prev.size()); ++o)
      for (int v = 0; v < n; ++v) {
        const float dist = xyDist(prev[static_cast<size_t>(o)], pos[v]);
        if (dist <= p.identityM()) pairs.emplace_back(dist, o, v);
      }
    std::sort(pairs.begin(), pairs.end());
    std::vector<int> old_of(static_cast<size_t>(n), -1);
    std::vector<uint8_t> old_used(prev.size(), 0);
    for (const auto& [dist, o, v] : pairs) {
      if (old_of[v] >= 0 || old_used[static_cast<size_t>(o)]) continue;
      old_of[v] = o;
      old_used[static_cast<size_t>(o)] = 1;
      ++plan.n_matched_prev;
    }
    if (plan.n_matched_prev > 0) {
      // (old index, pick) and (candidate index, pick)
      std::vector<std::pair<int, int>> kept, rest;
      for (int v = 0; v < n; ++v) {
        if (old_of[v] >= 0) kept.emplace_back(old_of[v], v);
        else rest.emplace_back(point_of[v], v);
      }
      std::sort(kept.begin(), kept.end());
      std::sort(rest.begin(), rest.end());
      std::vector<int> seed;
      for (const auto& kv : kept) seed.push_back(kv.second);
      for (const auto& rv : rest)
        seed.insert(seed.begin() + static_cast<std::ptrdiff_t>(
                                       cheapestSlot(seed, rv.second, c_r, d)),
                    rv.second);
      if (!(openRouteLength(order, c_r, d) <=
            (1.0f - p.reorder_gain) * openRouteLength(seed, c_r, d))) {
        order = std::move(seed);
        plan.order_kept = true;
      }
    }
  }

  // 9. Waypoints in drive order.
  int sum_tier = 0, sum_later = 0;
  plan.waypoints.resize(static_cast<size_t>(n));
  for (int k = 0; k < n; ++k) {
    const int v = order[k];
    RouteWaypoint& wp = plan.waypoints[static_cast<size_t>(k)];
    wp.position = pos[v];
    wp.is_frontier = point_of[v] >= n_lattice;
    wp.candidate_index = point_of[v];
    wp.selection_rank = v;
    wp.marginal_cells = m_tier[v];
    wp.marginal_later_cells = m_later[v];
    wp.leg_cost_m = k == 0 ? c_r[v] : d[order[k - 1]][v];
    wp.residual_cells = std::move(m_sets[v]);
    plan.route_len_m += wp.leg_cost_m;
    sum_tier += m_tier[v];
    sum_later += m_later[v];
  }
  // Each yaw faces the next waypoint; the last keeps its arrival bearing so
  // nav does not turn on the spot at the end (§6 item 9).
  for (int k = 0; k < n; ++k) {
    const Eigen::Vector3f& from =
        k + 1 < n ? plan.waypoints[k].position
                  : (k > 0 ? plan.waypoints[k - 1].position : in.robot);
    const Eigen::Vector3f& to =
        k + 1 < n ? plan.waypoints[k + 1].position : plan.waypoints[k].position;
    plan.waypoints[k].yaw = std::atan2(to.y() - from.y(), to.x() - from.x());
  }
  plan.covered_m2 = static_cast<float>(sum_tier * res * res);
  plan.covered_later_m2 = static_cast<float>(sum_later * res * res);
  plan.outcome =
      n == 1 ? RoutePlan::Outcome::kSingle : RoutePlan::Outcome::kRoute;
  return plan;
}

}  // namespace explo_planner
