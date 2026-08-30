#include "explo_planner/cell_world.hpp"

#include "explo_planner/fleet_identity.hpp"
#include "explo_planner/map_cache.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace explo_planner {

namespace {

/// FNV-1a 32-bit, the same construction fleet_identity uses and for the same
/// reason: this value is compared between processes, so it may not be
/// std::hash. Kept local rather than shared because the two hash different
/// things and a shared helper would invite hashing them into one value, which
/// would stop an operator being told WHICH half of the config disagrees.
struct Fnv1a {
  uint32_t h = 2166136261u;
  void byte(uint8_t b) { h ^= b; h *= 16777619u; }
  void i64(int64_t v) {
    for (int i = 0; i < 8; ++i) byte(static_cast<uint8_t>((v >> (i * 8)) & 0xff));
  }
};

/// Quantise a metric quantity to millimetres for hashing. Two robots may reach
/// the same ROI through different float arithmetic (a param read as double and
/// narrowed, versus one computed from a launch expression); at millimetre
/// granularity those agree, and no real misconfiguration is finer.
int64_t mm(float v) {
  return static_cast<int64_t>(std::llround(static_cast<double>(v) * 1000.0));
}

constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

uint8_t normaliseForWire(CellStatus s) {
  switch (s) {
    case CellStatus::COVERED:
    case CellStatus::COVERED_BY_OTHERS:
      return 2;
    case CellStatus::EXPLORING:
    case CellStatus::EXPLORING_BY_OTHERS:
      return 1;
    case CellStatus::UNSEEN:
    default:
      return 0;
  }
}

CellStatus fromWire(uint8_t v) {
  switch (v) {
    case 1: return CellStatus::EXPLORING;
    case 2: return CellStatus::COVERED;
    default: return CellStatus::UNSEEN;
  }
}

bool isFirstHand(CellStatus s) {
  return s == CellStatus::UNSEEN || s == CellStatus::EXPLORING ||
         s == CellStatus::COVERED;
}

int exploredRank(CellStatus s) {
  switch (s) {
    case CellStatus::COVERED:
    case CellStatus::COVERED_BY_OTHERS:
      return 2;
    case CellStatus::EXPLORING:
    case CellStatus::EXPLORING_BY_OTHERS:
      return 1;
    case CellStatus::UNSEEN:
    default:
      return 0;
  }
}

const char* cellStatusName(CellStatus s) {
  switch (s) {
    case CellStatus::UNSEEN:              return "unseen";
    case CellStatus::EXPLORING:           return "exploring";
    case CellStatus::COVERED:             return "covered";
    case CellStatus::COVERED_BY_OTHERS:   return "covered_by_others";
    case CellStatus::EXPLORING_BY_OTHERS: return "exploring_by_others";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

int CellGrid::idAt(float x, float y) const {
  if (nx <= 0 || ny <= 0) return -1;
  if (!std::isfinite(x) || !std::isfinite(y)) return -1;
  // floor(), matching voxel ingest: a point exactly on an internal boundary
  // lands in the higher cell on every robot, rather than one each.
  const double c = std::floor((static_cast<double>(x) - min_x) / cell_size_m);
  const double r = std::floor((static_cast<double>(y) - min_y) / cell_size_m);
  if (c < 0.0 || c >= nx || r < 0.0 || r >= ny) return -1;
  return static_cast<int>(r) * nx + static_cast<int>(c);
}

void CellGrid::centre(int id, float& x, float& y) const {
  x = min_x + (static_cast<float>(col(id)) + 0.5f) * cell_size_m;
  y = min_y + (static_cast<float>(row(id)) + 0.5f) * cell_size_m;
}

uint32_t CellGrid::configHash() const {
  Fnv1a f;
  f.i64(mm(min_x));
  f.i64(mm(min_y));
  f.i64(mm(cell_size_m));
  f.i64(nx);
  f.i64(ny);
  return f.h;
}

CellGrid makeCellGrid(float min_x, float max_x, float min_y, float max_y,
                      float cell_size_m) {
  CellGrid g;  // size() == 0 until every check passes
  if (!std::isfinite(min_x) || !std::isfinite(max_x) ||
      !std::isfinite(min_y) || !std::isfinite(max_y) ||
      !std::isfinite(cell_size_m))
    return g;
  if (!(max_x > min_x) || !(max_y > min_y) || !(cell_size_m > 0.0f)) return g;

  // Round UP so the ROI is fully covered: a half cell of ground the planner
  // may drive into but the census cannot see would be permanently UNSEEN and
  // would hold the team short of complete forever.
  const double span_x = (static_cast<double>(max_x) - min_x) / cell_size_m;
  const double span_y = (static_cast<double>(max_y) - min_y) / cell_size_m;
  const double nx_d = std::ceil(span_x);
  const double ny_d = std::ceil(span_y);
  // Bound before multiplying: a cell size of 1 mm over a 1 km ROI overflows
  // int, and an overflowed count would pass a naive kMaxCells test.
  if (nx_d < 1.0 || ny_d < 1.0 || nx_d > kMaxCells || ny_d > kMaxCells)
    return g;
  if (nx_d * ny_d > kMaxCells) return g;

  g.min_x = min_x;
  g.min_y = min_y;
  g.cell_size_m = cell_size_m;
  g.nx = static_cast<int>(nx_d);
  g.ny = static_cast<int>(ny_d);
  return g;
}

// ---------------------------------------------------------------------------
// CellWorld
// ---------------------------------------------------------------------------

double CellWorld::CellObservation::unknownFraction() const {
  if (total_columns <= 0) return -1.0;  // "cannot measure", as MapCache does
  const double seen = static_cast<double>(observed_columns);
  const double tot  = static_cast<double>(total_columns);
  return 1.0 - std::min(seen, tot) / tot;
}

double CellWorld::CellObservation::frontierFraction() const {
  if (observed_voxels <= 0) return -1.0;  // same convention as above
  const double f = static_cast<double>(frontier_voxels);
  const double n = static_cast<double>(observed_voxels);
  return std::min(f, n) / n;
}

std::string CellWorld::configure(const CellGrid& grid, const Config& cfg,
                                 int self_id) {
  // Leave the world unconfigured until every check passes: a half-built cell
  // world publishes ids that do not name the ground the receiver thinks.
  grid_ = CellGrid{};
  cells_.clear();
  edges_.clear();
  dist_.clear();
  dist_dirty_ = true;
  self_id_ = -1;

  char buf[192];
  if (grid.size() <= 0)
    return "cell grid is empty (check roi bounds and cell_size_m)";
  if (grid.size() > kMaxCells) {
    std::snprintf(buf, sizeof(buf), "cell grid has %d cells, cap is %d",
                  grid.size(), kMaxCells);
    return buf;
  }
  // self_id is required, not optional. known_by is a bitmask addressed by id;
  // with no id every bit this robot would set is bit-nothing, and the
  // knowledge gate downstream would compute a correct-looking answer from an
  // always-empty mask. Refusing here is what keeps that from being silent —
  // callers gate on fleet identity being configured before enabling this.
  if (self_id < 0 || self_id >= kMaxTeamSize) {
    std::snprintf(buf, sizeof(buf),
                  "self robot id %d is outside [0, %d): cell world needs "
                  "fleet identity (team_robot_names)",
                  self_id, kMaxTeamSize);
    return buf;
  }
  if (!(cfg.covered_max_unknown >= 0.0) || !(cfg.covered_max_unknown <= 1.0) ||
      !(cfg.exploring_min_unknown >= 0.0) ||
      !(cfg.exploring_min_unknown <= 1.0))
    return "covered_max_unknown and exploring_min_unknown must lie in [0, 1]";
  // Strictly ordered, not sorted-for-you. An equal pair is a zero-width
  // hysteresis band, which is the flapping this exists to prevent; quietly
  // repairing it would leave the operator believing they had hysteresis.
  if (!(cfg.covered_max_unknown < cfg.exploring_min_unknown))
    return "covered_max_unknown must be strictly below exploring_min_unknown "
           "(otherwise there is no hysteresis band and cells flap)";
  if (!(cfg.covered_max_frontier_frac >= 0.0) ||
      !(cfg.covered_max_frontier_frac <= 1.0))
    return "covered_max_frontier_frac must lie in [0, 1]";
  if (cfg.min_observed_columns < 0) return "min_observed_columns must be >= 0";
  if (!(cfg.edge_max_blocked_fraction >= 0.0) ||
      !(cfg.edge_max_blocked_fraction <= 1.0))
    return "edge_max_blocked_fraction must lie in [0, 1]";

  grid_ = grid;
  cfg_ = cfg;
  self_id_ = self_id;
  cells_.assign(static_cast<size_t>(grid_.size()), Cell{});
  // Start fully connected: with no plan map yet, every edge is as good as the
  // straight-line fallback the planner already trusts.
  edges_.assign(static_cast<size_t>(grid_.size()) * grid_.size(), 1u);
  for (int i = 0; i < grid_.size(); ++i)
    edges_[static_cast<size_t>(i) * grid_.size() + i] = 0u;
  dist_dirty_ = true;
  return std::string();
}

CellStatus CellWorld::classify(CellStatus current,
                               const CellObservation& o) const {
  // total_columns == 0 is "not measured this tick", not "became unseen". A map
  // that has not been republished must never erase the census.
  if (o.total_columns <= 0) return current;

  // What this tick's evidence says on its own, ignoring history.
  CellStatus fresh = CellStatus::UNSEEN;
  if (o.observed_columns >= cfg_.min_observed_columns) {
    const double u = o.unknownFraction();
    // The frontier count vetoes PROMOTION only. It is the volumetric check the
    // 2.5D column measure cannot do (unknown pockets behind a trunk), but it
    // is also the noisier of the two — it counts absent 6-neighbours, so the
    // ROI boundary contributes to it. Using it to demote as well would put the
    // flap back in through a channel the hysteresis band does not cover.
    // A cell with observed columns but no observed voxels cannot exist, but
    // frontierFraction()'s "cannot measure" sentinel is negative and would
    // sail under any threshold, so the veto is written to require a real
    // measurement rather than to trust that it got one.
    const double ff = o.frontierFraction();
    fresh = (u >= 0.0 && u <= cfg_.covered_max_unknown &&
             ff >= 0.0 && ff <= cfg_.covered_max_frontier_frac)
                ? CellStatus::COVERED
                : CellStatus::EXPLORING;
  }

  if (!isFirstHand(current)) {
    // The current belief came from a peer that was there. My own map saying
    // "unknown" is not evidence against that — it is evidence I have not been.
    // So a relayed belief may only ever be RAISED by what I see, never lowered.
    // Equal rank still adopts, because upgrading provenance from relayed to
    // first-hand is real information for the merge guard table.
    return exploredRank(fresh) >= exploredRank(current) ? fresh : current;
  }

  // Hysteresis: leave COVERED only once the unknown fraction clears the far
  // side of the band. Inside the band the old belief holds.
  if (current == CellStatus::COVERED && fresh != CellStatus::COVERED) {
    const double u = o.unknownFraction();
    if (!(u > cfg_.exploring_min_unknown)) return CellStatus::COVERED;
  }
  // A weak-evidence tick cannot push a first-hand belief back to UNSEEN:
  // having been somewhere is not a thing the map can un-say.
  if (fresh == CellStatus::UNSEEN && current != CellStatus::UNSEEN)
    return current;
  return fresh;
}

int CellWorld::applyObservation(const std::vector<CellObservation>& obs) {
  if (!configured() || obs.size() != cells_.size()) return 0;
  int changed = 0;
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (obs[i].total_columns <= 0) continue;  // unmeasured: leave it alone
    cells_[i].obs = obs[i];
    const CellStatus s = classify(cells_[i].status, obs[i]);
    if (commitSelf(static_cast<int>(i), s)) ++changed;
    // Whatever the status now is, this robot holds it.
    markKnownBy(static_cast<int>(i), self_id_);
  }
  return changed;
}

bool CellWorld::commitSelf(int id, CellStatus s) {
  if (!grid_.valid(id)) return false;
  Cell& c = cells_[static_cast<size_t>(id)];
  if (c.status == s) return false;
  c.status = s;
  // Rule 1: update_id counts FIRST-HAND committed changes, and only this path
  // may touch it. Rule 2: the change invalidates what peers knew, so the mask
  // collapses to this robot alone.
  ++c.update_id;
  c.known_by = robotBit(self_id_);
  return true;
}

void CellWorld::markKnownBy(int cell_id, int robot_id) {
  if (!grid_.valid(cell_id)) return;
  cells_[static_cast<size_t>(cell_id)].known_by |= robotBit(robot_id);
}

double CellWorld::Census::coveredFraction() const {
  const int total = unseen + exploring + covered + exploring_by_others +
                    covered_by_others;
  if (total <= 0) return -1.0;
  return static_cast<double>(covered + covered_by_others) / total;
}

CellWorld::Census CellWorld::census() const {
  Census c;
  for (const Cell& cell : cells_) {
    switch (cell.status) {
      case CellStatus::UNSEEN:              ++c.unseen; break;
      case CellStatus::EXPLORING:           ++c.exploring; break;
      case CellStatus::COVERED:             ++c.covered; break;
      case CellStatus::EXPLORING_BY_OTHERS: ++c.exploring_by_others; break;
      case CellStatus::COVERED_BY_OTHERS:   ++c.covered_by_others; break;
    }
  }
  return c;
}

// --- Adjacency -------------------------------------------------------------

void CellWorld::rebuildEdges(const BlockedProbe& probe) {
  if (!configured()) return;
  const int n = grid_.size();
  edges_.assign(static_cast<size_t>(n) * n, 0u);
  for (int a = 0; a < n; ++a) {
    const int ca = grid_.col(a), ra = grid_.row(a);
    float ax, ay;
    grid_.centre(a, ax, ay);
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) continue;
        const int cb = ca + dc, rb = ra + dr;
        if (cb < 0 || cb >= grid_.nx || rb < 0 || rb >= grid_.ny) continue;
        const int b = rb * grid_.nx + cb;
        if (b < a) continue;  // symmetric; do each pair once
        bool enabled = true;
        if (probe) {
          float bx, by;
          grid_.centre(b, bx, by);
          const double blocked = probe(ax, ay, bx, by);
          // Negative is "no map here", which keeps the edge: an absent plan
          // map must not disconnect the graph, or the allocator would fall
          // back to straight lines everywhere the instant the map lagged.
          if (blocked >= 0.0 && blocked > cfg_.edge_max_blocked_fraction)
            enabled = false;
        }
        const uint8_t v = enabled ? 1u : 0u;
        edges_[static_cast<size_t>(a) * n + b] = v;
        edges_[static_cast<size_t>(b) * n + a] = v;
      }
    }
  }
  dist_dirty_ = true;
}

bool CellWorld::edgeEnabled(int a, int b) const {
  if (!grid_.valid(a) || !grid_.valid(b)) return false;
  return edges_[static_cast<size_t>(a) * grid_.size() + b] != 0u;
}

double CellWorld::centroidDistance(int a, int b) const {
  if (!grid_.valid(a) || !grid_.valid(b)) return -1.0;
  float ax, ay, bx, by;
  grid_.centre(a, ax, ay);
  grid_.centre(b, bx, by);
  return std::hypot(static_cast<double>(bx) - ax, static_cast<double>(by) - ay);
}

void CellWorld::recomputeDistances() const {
  const int n = grid_.size();
  dist_.assign(static_cast<size_t>(n) * n, kInf);
  for (int a = 0; a < n; ++a) {
    dist_[static_cast<size_t>(a) * n + a] = 0.0;
    for (int b = 0; b < n; ++b) {
      if (a == b || edges_[static_cast<size_t>(a) * n + b] == 0u) continue;
      dist_[static_cast<size_t>(a) * n + b] = centroidDistance(a, b);
    }
  }
  // Floyd-Warshall. O(n^3) but cached behind dist_dirty_, so it costs once per
  // edge change rather than once per query.
  for (int k = 0; k < n; ++k) {
    for (int i = 0; i < n; ++i) {
      const double dik = dist_[static_cast<size_t>(i) * n + k];
      if (dik == kInf) continue;
      for (int j = 0; j < n; ++j) {
        const double alt = dik + dist_[static_cast<size_t>(k) * n + j];
        double& d = dist_[static_cast<size_t>(i) * n + j];
        if (alt < d) d = alt;
      }
    }
  }
  dist_dirty_ = false;
}

double CellWorld::distance(int a, int b) const {
  if (!grid_.valid(a) || !grid_.valid(b)) return -1.0;
  if (dist_dirty_) recomputeDistances();
  const double d = dist_[static_cast<size_t>(a) * grid_.size() + b];
  return d == kInf ? -1.0 : d;
}

// ---------------------------------------------------------------------------
// Census from the fused map
// ---------------------------------------------------------------------------

std::vector<CellWorld::CellObservation> censusFromMap(const MapCache& map,
                                                      const CellGrid& grid) {
  std::vector<CellWorld::CellObservation> out;
  if (grid.size() <= 0) return out;
  out.assign(static_cast<size_t>(grid.size()), CellWorld::CellObservation{});

  const double res = map.resolution();
  if (!(res > 0.0) || !std::isfinite(res)) return out;

  // --- column-to-cell binning --------------------------------------------
  //
  // A voxel column has integer coord (cx, cy) and covers
  // [cx*res, (cx+1)*res) — Bonxai's coordToPos returns the LOW corner. Bin it
  // by that corner, the same floor() convention as ingest.
  //
  // ONE helper does the binning for both the denominator (total_columns) and
  // the numerator (observed_columns), rather than the counts being computed
  // analytically and the observations through CellGrid::idAt. Those two would
  // drift: at min_x = -15 and res = 0.2 the boundary column is
  // -75 * 0.2 == -15.000000000000002 in double, which floors to cell index -1,
  // but narrowed to float it is exactly -15.0f and floors to 0. One path would
  // then count a column the other did not, and every cell on the ROI's west
  // and south edges would report an unknown fraction slightly below the truth
  // — small enough to look like a threshold that needed tuning. Sharing the
  // arithmetic makes the two agree by construction.
  auto axis_index = [&](int64_t c, float lo, int ncells) -> int {
    const float p = static_cast<float>(static_cast<double>(c) * res);
    const double k =
        std::floor((static_cast<double>(p) - lo) / grid.cell_size_m);
    if (k < 0.0 || k >= ncells) return -1;
    return static_cast<int>(k);
  };
  // Columns decompose: which cell COLUMN a voxel column falls in depends only
  // on cx, which cell ROW only on cy. So two 1-D sweeps of a few hundred
  // entries each give every cell's column count, instead of a 2-D sweep that
  // would be millions of iterations on a large ROI.
  auto axis_counts = [&](float lo, int ncells, std::vector<int>& counts) {
    counts.assign(static_cast<size_t>(ncells), 0);
    const double hi = static_cast<double>(lo) + ncells * grid.cell_size_m;
    // One column of slack each side: the loop only has to be a superset, and
    // axis_index screens. Getting the bound exactly right in the presence of
    // the float narrowing above is not worth the two extra iterations.
    const int64_t c_lo = static_cast<int64_t>(std::floor(lo / res)) - 1;
    const int64_t c_hi = static_cast<int64_t>(std::floor(hi / res)) + 1;
    for (int64_t c = c_lo; c <= c_hi; ++c) {
      const int k = axis_index(c, lo, ncells);
      if (k >= 0) counts[static_cast<size_t>(k)]++;
    }
  };
  std::vector<int> cols_x, cols_y;
  axis_counts(grid.min_x, grid.nx, cols_x);
  axis_counts(grid.min_y, grid.ny, cols_y);
  for (int r = 0; r < grid.ny; ++r)
    for (int c = 0; c < grid.nx; ++c)
      out[static_cast<size_t>(r) * grid.nx + c].total_columns =
          cols_x[static_cast<size_t>(c)] * cols_y[static_cast<size_t>(r)];

  // --- one walk of the voxel grid ----------------------------------------
  //
  // Not a loop over MapCache::unknownColumnFraction: that walks the whole grid
  // per call, so a per-cell census would traverse a ~1.3 M-voxel map once per
  // cell, per planning tick. This pass costs the same as ONE such call.
  const auto& vg = map.grid();
  auto acc = vg.createConstAccessor();
  static const Bonxai::CoordT kOff[6] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                                         {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
  // Packed (x, y) column keys, deduplicated: x zero-extended into the high 32
  // bits, y into the low. Bijective for 32-bit coords, and the shift runs on
  // uint64_t because left-shifting a negative signed value (any column west of
  // the origin) is undefined behaviour. Same construction as
  // unknownColumnFraction, deliberately, so the two measures agree.
  std::unordered_set<uint64_t> observed;
  vg.forEachCell([&](const UnifiedVoxel& uv, const Bonxai::CoordT& c) {
    const int col = axis_index(c.x, grid.min_x, grid.nx);
    const int row = axis_index(c.y, grid.min_y, grid.ny);
    if (col < 0 || row < 0) return;  // outside the ROI
    auto& o = out[static_cast<size_t>(row) * grid.nx + col];
    ++o.observed_voxels;
    observed.insert((static_cast<uint64_t>(static_cast<uint32_t>(c.x)) << 32) |
                    static_cast<uint32_t>(c.y));
    // Frontier: a free voxel with at least one absent 6-neighbour. Mirrors
    // MapCache::computeStats and findFrontierCentroids exactly; a second
    // definition of "frontier" is a second thing to keep in step.
    if (uv.p_occ < 0.5f) {
      for (const auto& off : kOff) {
        if (!acc.value({c.x + off.x, c.y + off.y, c.z + off.z})) {
          ++o.frontier_voxels;
          break;
        }
      }
    }
  });

  for (uint64_t key : observed) {
    const int32_t cx = static_cast<int32_t>(static_cast<uint32_t>(key >> 32));
    const int32_t cy = static_cast<int32_t>(static_cast<uint32_t>(key));
    const int col = axis_index(cx, grid.min_x, grid.nx);
    const int row = axis_index(cy, grid.min_y, grid.ny);
    if (col >= 0 && row >= 0)
      ++out[static_cast<size_t>(row) * grid.nx + col].observed_columns;
  }

  return out;
}

}  // namespace explo_planner
