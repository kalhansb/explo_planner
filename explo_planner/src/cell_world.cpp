// Moved comments: doc/explo_planner_code_notes.md
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

/// FNV-1a 32-bit, as in fleet_identity: the value is compared between
/// processes, so not std::hash. Kept separate from fleet_identity's so a
/// mismatch still tells which half of the config disagrees.
/// (notes: cellworld-fnv1a-local)
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
  // self_id is required: known_by is an id-addressed bitmask, and without an id
  // the knowledge gate would compute from an always-empty mask. Callers gate on
  // fleet identity before enabling this. (notes: cellworld-self-id-required)
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
    // The frontier fraction vetoes promotion to COVERED only; using it to
    // demote would bring flapping back outside the hysteresis band. Its
    // negative cannot-measure sentinel must not pass, so the veto needs a real
    // measurement. (notes: cellworld-frontier-veto)
    const double ff = o.frontierFraction();
    fresh = (u >= 0.0 && u <= cfg_.covered_max_unknown &&
             ff >= 0.0 && ff <= cfg_.covered_max_frontier_frac)
                ? CellStatus::COVERED
                : CellStatus::EXPLORING;
  }

  if (!isFirstHand(current)) {
    // The current belief is relayed, so my own evidence may only raise it,
    // never lower it. Equal rank still adopts: upgrading provenance to
    // first-hand matters to the merge guard table.
    // (notes: cellworld-relayed-raise-only)
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

// --- Wire codec and merge --------------------------------------------------

std::vector<CellWorld::WireCell> CellWorld::toWire() const {
  std::vector<WireCell> out;
  out.reserve(cells_.size());
  for (size_t i = 0; i < cells_.size(); ++i) {
    WireCell w;
    w.id        = static_cast<uint16_t>(i);
    w.status    = normaliseForWire(cells_[i].status);  // rule 3
    w.known_by  = cells_[i].known_by;
    w.update_id = cells_[i].update_id;
    out.push_back(w);
  }
  return out;
}

uint32_t CellWorld::sharedHash() const {
  Fnv1a f;
  // The cell count is folded in first so grids of different size cannot collide
  // by accident. configHash() covers geometry; this guards callers that compare
  // digests without comparing grids. (notes: cellworld-shared-hash-count)
  f.i64(static_cast<int64_t>(cells_.size()));
  for (const auto& c : cells_) f.byte(normaliseForWire(c.status));
  return f.h;
}

uint32_t CellWorld::edgeHash() const {
  Fnv1a f;
  f.i64(static_cast<int64_t>(edges_.size()));
  // Byte-for-byte. edges_ is written as 0/1 everywhere in this file, but the
  // digest does not assume that: a future writer storing a weight there would
  // otherwise change the matrix without changing the hash, which is the exact
  // failure mode this exists to catch.
  for (uint8_t e : edges_) f.byte(e);
  return f.h;
}

int CellWorld::neighbourhood9(int id, int* out) const {
  if (!grid_.valid(id) || out == nullptr) return 0;
  const int c0 = grid_.col(id), r0 = grid_.row(id);
  int n = 0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      const int c = c0 + dc, r = r0 + dr;
      if (c < 0 || c >= grid_.nx || r < 0 || r >= grid_.ny) continue;
      out[n++] = r * grid_.nx + c;
    }
  }
  return n;
}

CellWorld::MergeStats CellWorld::mergeWire(int sender_id,
                                           const std::vector<WireCell>& cells,
                                           int local_priority_centre) {
  MergeStats st;
  char buf[160];
  if (!configured()) {
    st.refused = "cell world is not configured";
    return st;
  }
  // An out-of-range sender would make robotBit() zero, and every mask
  // operation below a no-op that still reported "applied". Refuse instead: the
  // caller has a fleet-identity problem and needs to be told, not accommodated.
  if (sender_id < 0 || sender_id >= kMaxTeamSize) {
    std::snprintf(buf, sizeof(buf),
                  "sender robot id %d is outside [0, %d)", sender_id,
                  kMaxTeamSize);
    st.refused = buf;
    return st;
  }
  if (sender_id == self_id_) {
    // Not merely useless: it would let this robot's own relayed census
    // overwrite its first-hand statuses through the peer path, which is the
    // one path that does not bump update_id.
    st.refused = "sender is this robot";
    return st;
  }

  // The local-priority neighbourhood, resolved once.
  int priority_ids[9];
  int n_priority = 0;
  if (local_priority_centre >= 0)
    n_priority = neighbourhood9(local_priority_centre, priority_ids);

  const uint32_t sender_bit = robotBit(sender_id);

  for (const WireCell& w : cells) {
    const int id = static_cast<int>(w.id);
    if (!grid_.valid(id)) { ++st.out_of_range; continue; }
    if (w.status > 2)     { ++st.bad_status;   continue; }

    Cell& c = cells_[static_cast<size_t>(id)];
    const CellStatus peer  = fromWire(w.status);
    const CellStatus local = c.status;

    // mTARE's local-priority rule: a peer update to a cell in the
    // local-priority neighbourhood that we are EXPLORING is refused entirely,
    // known_by included, so the peer is not credited with a status we are about
    // to change. (notes: cellworld-local-priority)
    if (n_priority > 0 && local == CellStatus::EXPLORING) {
      bool mine = false;
      for (int k = 0; k < n_priority; ++k)
        if (priority_ids[k] == id) { mine = true; break; }
      if (mine) { ++st.refused_local; continue; }
    }

    // Statuses agree: OR in the sender and its known_by (this makes the
    // knowledge gate work at N>=3). Deliberate asymmetry: adoption below resets
    // the mask to self and sender, since over-crediting suppresses needed
    // reconnections. (notes: cellworld-agree-mask-asymmetry)
    if (normaliseForWire(local) == w.status) {
      const uint32_t before = c.known_by;
      c.known_by |= w.known_by | sender_bit;
      if (c.known_by != before) ++st.known_by_only;
      else                      ++st.agreed_noop;
      continue;
    }

    // --- the guard table, on disagreement -----------------------------------
    CellStatus adopt = local;

    if (peer == CellStatus::COVERED) {
      // A peer's first-hand COVERED beats anything of ours that is not itself
      // a first-hand COVERED — and a first-hand COVERED cannot disagree with
      // it, because it would have matched on the wire above.
      adopt = CellStatus::COVERED_BY_OTHERS;
    } else if (peer == CellStatus::EXPLORING) {
      if (local == CellStatus::UNSEEN) {
        adopt = CellStatus::EXPLORING_BY_OTHERS;
      } else if (local == CellStatus::COVERED_BY_OTHERS) {
        // Peer EXPLORING replaces a local COVERED_BY_OTHERS only from a robot
        // in known_by (a retraction by a source of that COVERED); a third
        // robot's staler EXPLORING is refused. Identical to the plain rule at
        // N=2. (notes: cellworld-exploring-retraction)
        if (maskHas(c.known_by, sender_id)) adopt = CellStatus::EXPLORING_BY_OTHERS;
      }
      // local first-hand COVERED: never downgraded by a peer. local
      // EXPLORING/EXPLORING_BY_OTHERS: matched on the wire above.
    }
    // peer UNSEEN carries no information and changes nothing: a robot that has
    // not seen a cell is not evidence that nobody has.

    if (adopt == local) { ++st.refused_guard; continue; }

    c.status = adopt;
    // NOT commitSelf(): this is relayed belief, so update_id must not move
    // (rule 1). The mask collapses to the two robots that can actually vouch
    // for this status — see the asymmetry note above.
    c.known_by = robotBit(self_id_) | sender_bit;
    ++st.applied;
  }
  return st;
}

std::vector<int> CellWorld::interceptCandidates(int robot_id) const {
  std::vector<int> out;
  if (robotBit(robot_id) == 0u) return out;   // no bit, so no claim to check
  // Asking for this robot returns nothing: self is in known_by of every cell
  // observed or adopted, so the query would otherwise return a large spurious
  // search list. (notes: cellworld-intercept-self)
  if (robot_id == self_id_) return out;
  for (size_t i = 0; i < cells_.size(); ++i) {
    const Cell& c = cells_[i];
    if (!maskHas(c.known_by, robot_id)) continue;
    if (exploredRank(c.status) >= 2) continue;  // finished: nobody works here
    out.push_back(static_cast<int>(i));
  }
  return out;
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
  // A column covers [cx*res, (cx+1)*res) (coordToPos returns the low corner);
  // bin it with floor() as ingest does. total_columns and observed_columns
  // share this helper so float rounding cannot make them disagree.
  // (notes: cellworld-column-binning)
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
  // Deduplicated column keys: x zero-extended into the high 32 bits, y into the
  // low; shift as uint64_t, since shifting a negative signed value is UB. Same
  // construction as unknownColumnFraction, so the measures agree.
  // (notes: cellworld-packed-column-keys)
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
