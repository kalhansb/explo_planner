#pragma once
/// @file cell_world.hpp
/// @brief The coarse global map: a 2D grid of ~10 m cells over the ROI, each
///        carrying an exploration status, who is known to have seen it, and a
///        local revision counter. Port of M-TARE's grid world, flattened.
///        See docs/mtare_evolution_plan.md §3.1-§3.2.
///
/// This exists because the planner's frontier/EIG machinery is entirely local:
/// it can pick the best next viewpoint but has no representation in which two
/// robots could agree to divide a forest. The cell world is that
/// representation — small enough to broadcast whole at 1 Hz, coarse enough
/// that two robots' independent observations of the same ground agree.
///
/// M-TARE's grid world is genuinely 3D. This one is not, deliberately: the
/// vehicle is ground-restricted (planner_method.md §14), so a column of cells
/// would only ever have one occupied layer and would triple the wire cost to
/// say so.
///
/// THREE RULES CARRY THE DESIGN, and each is load-bearing rather than tidy:
///
///   1. `update_id` is LOCAL. It counts committed status changes this robot
///      observed FIRST-HAND; the merge path never touches it. It is therefore
///      not an ordering of information quality and must never be compared
///      across robots — a robot out of comms for ten minutes legitimately
///      re-commits EXPLORING at a high count against a peer's first-hand
///      COVERED, and "higher wins" would regress the team's census. Conflicts
///      resolve by the guard table plus a more-explored ordering instead.
///
///   2. `known_by` RESETS on every committed status change, to {self}. A
///      change invalidates what peers knew about the cell. Without the reset
///      the mask only grows, and the §3.6 knowledge gate — "does my missing
///      peer already know everything I could tell it?" — answers yes forever
///      for any cell a peer ever heard of. A gate that cannot fire is the
///      failure mode this repo keeps rediscovering, so it is prevented here,
///      in the data structure, rather than checked for downstream.
///
///   3. The WIRE vocabulary is three values; the LOCAL one is five. The
///      `*_BY_OTHERS` statuses are bookkeeping about where a belief came
///      from, and are meaningless at the far end of a relay (whose "others"?).
///      They are normalised away on send. Without that, relayed state at N=3
///      is uninterpretable and the allocator's solve-same premise breaks.
///
/// Hysteresis on the status thresholds is the fourth, smaller rule: a single
/// threshold makes a cell hovering near it flap between EXPLORING and COVERED
/// every tick, and since every flap is a committed change, flapping would
/// reset `known_by` continuously and turn rule 2 into a knowledge gate that
/// never suppresses anything.
/// Moved comments: doc/cell_world_notes.md

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace explo_planner {

class MapCache;

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

/// Cell exploration status. The first three values are the ENTIRE wire
/// vocabulary and their numeric values match CellState.msg's constants; the
/// last two are local-only provenance and never leave this process.
///
/// NOGO is not ported: impassable terrain is left to the navigator and the plan
/// map. (notes: cell-no-nogo)
enum class CellStatus : uint8_t {
  UNSEEN              = 0,
  EXPLORING           = 1,
  COVERED             = 2,
  COVERED_BY_OTHERS   = 3,
  EXPLORING_BY_OTHERS = 4,
};

/// The wire value for a local status: the `*_BY_OTHERS` provenance is dropped.
uint8_t normaliseForWire(CellStatus s);

/// Wire value -> status. Anything outside {0,1,2} is UNSEEN: a peer that sends
/// a local-only or garbage value is not trusted to have meant something
/// clever by it, and UNSEEN is the value that cannot cause a bad merge (the
/// guard table lets peer-UNSEEN change nothing).
CellStatus fromWire(uint8_t v);

/// True for the statuses this robot observed itself. First-hand belief
/// outranks relayed belief in the merge guard table regardless of counters.
bool isFirstHand(CellStatus s);

/// How explored a status claims the cell is: COVERED 2 > EXPLORING 1 >
/// UNSEEN 0, with each `*_BY_OTHERS` ranking as its base status. This is the
/// total order every remaining merge conflict resolves toward, so that a lost
/// or reordered message can DELAY the census but never regress it.
int exploredRank(CellStatus s);

const char* cellStatusName(CellStatus s);

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Row-major cell grid over the ROI. Ids name the same ground on every robot
/// only if all share the ROI and cell size, so TeamWorld's config check must
/// fold in configHash(). (notes: cell-grid-shared-geometry)
struct CellGrid {
  float min_x = 0.0f, min_y = 0.0f;
  float cell_size_m = 10.0f;
  int   nx = 0, ny = 0;

  int   size()  const { return nx * ny; }
  bool  valid(int id) const { return id >= 0 && id < size(); }
  int   col(int id) const { return id % nx; }
  int   row(int id) const { return id / nx; }

  /// Cell containing (x, y), or -1 outside the ROI. Uses floor(), matching
  /// the voxel-ingest convention, so a point exactly on an internal boundary
  /// lands in the higher cell on both robots rather than one each.
  int   idAt(float x, float y) const;

  /// Centre of a cell in map coordinates. Undefined for an invalid id, which
  /// callers are expected to have screened with valid().
  void  centre(int id, float& x, float& y) const;

  float maxX() const { return min_x + nx * cell_size_m; }
  float maxY() const { return min_y + ny * cell_size_m; }

  /// FNV-1a over the geometry, for the cross-robot config check. Quantised to
  /// millimetres before hashing: two robots that computed the same ROI through
  /// different float arithmetic must not be told their maps disagree, and a
  /// real misconfiguration is never sub-millimetre.
  uint32_t configHash() const;
};

/// Grid over the ROI at cell_size_m, cell count rounded up. Returns size() == 0
/// on non-finite or degenerate bounds, a non-positive cell size, or more than
/// kMaxCells cells. (notes: cell-make-grid)
CellGrid makeCellGrid(float min_x, float max_x, float min_y, float max_y,
                      float cell_size_m);

/// Hard cap on cells. uint16 ids allow far more, but the allocator is
/// documented as bounded at "<=400 cells, <=8 robots" and the all-pairs
/// distance table is O(n^3); 4096 cells would be a 68-billion-operation
/// rebuild that would look like a hung planner, not a slow one.
inline constexpr int kMaxCells = 1024;

// ---------------------------------------------------------------------------
// The world
// ---------------------------------------------------------------------------

class CellWorld {
public:
  struct Config {
    /// Hysteresis pair: COVERED at or below covered_max_unknown, EXPLORING
    /// again only above exploring_min_unknown; configure() refuses covered >=
    /// exploring. World-calibrated: the defaults assume a saturating map.
    /// (notes: cell-hysteresis-pair)
    double covered_max_unknown   = 0.15;
    double exploring_min_unknown = 0.35;

    /// A cell whose frontier fraction exceeds this cannot become COVERED: the
    /// column measure cannot see unknown volume behind a trunk. A fraction of
    /// observed voxels, world-calibrated; 1.0 disables it.
    /// (notes: cell-frontier-frac)
    double covered_max_frontier_frac = 0.90;

    /// Columns that must be observed before a cell can leave UNSEEN. One
    /// stray voxel from a distant lidar return should not promote a cell the
    /// robot has never actually been near.
    int min_observed_columns = 4;

    /// Fraction of the line between two cell centres that may be occupied or
    /// unknown in the plan map before the edge is disabled; with no plan-map
    /// probe every edge stays enabled. (notes: cell-edge-blocked-fraction)
    double edge_max_blocked_fraction = 0.5;
  };

  /// One cell's observation, as measured from the map. Kept separate from the
  /// status machine so the machine — the part with the interesting rules — is
  /// testable without a Bonxai grid.
  struct CellObservation {
    int total_columns    = 0;   ///< columns the cell contains at map resolution
    int observed_columns = 0;   ///< of those, ones with any voxel in them
    int frontier_voxels  = 0;
    int observed_voxels  = 0;

    /// Fraction of columns never observed, or -1.0 for "cannot measure"
    /// (an empty or degenerate cell) — the same convention
    /// MapCache::unknownColumnFraction uses.
    double unknownFraction() const;

    /// Frontier voxels over observed voxels, or -1.0 (cannot measure) when the
    /// cell holds none. The sentinel is negative and would pass a threshold
    /// test; classify() screens it out first.
    /// (notes: cell-frontier-fraction-sentinel)
    double frontierFraction() const;
  };

  struct Cell {
    CellStatus status    = CellStatus::UNSEEN;
    uint32_t   known_by  = 0;
    uint32_t   update_id = 0;
    /// Last observation applied, for diagnostics and the census event.
    CellObservation obs;
  };

  CellWorld() = default;

  /// (Re)configures geometry and thresholds: an empty string on success, else
  /// the reason, leaving the world unconfigured (size() == 0). Reconfiguring
  /// clears a populated world: cell ids are geometry. (notes: cell-configure)
  std::string configure(const CellGrid& grid, const Config& cfg, int self_id);

  bool configured() const { return grid_.size() > 0; }
  const CellGrid& grid() const { return grid_; }
  const Config& config() const { return cfg_; }
  int  size() const { return grid_.size(); }
  int  selfId() const { return self_id_; }

  const Cell& cell(int id) const { return cells_[id]; }
  CellStatus status(int id) const { return cells_[id].status; }

  /// Applies a self-observation census of exactly size() entries; returns the
  /// count of changed cells (each bumps update_id, resets known_by to {self}).
  /// total_columns == 0 = not measured: the cell is left alone.
  /// (notes: cell-apply-observation)
  int applyObservation(const std::vector<CellObservation>& obs);

  /// The status the machine would assign, given the current status. Exposed
  /// because it is the whole of the hysteresis rule and testing it directly
  /// beats inferring it from applyObservation's return count.
  CellStatus classify(CellStatus current, const CellObservation& o) const;

  /// Commit a status arrived at first-hand: on a CHANGE, bump update_id and
  /// reset known_by to {self} (rule 2). Returns true if it changed anything.
  /// This is the ONLY path that touches update_id.
  bool commitSelf(int id, CellStatus s);

  /// Record that robot `id` is known to have seen this cell in its CURRENT
  /// status. Merging uses this; it never bumps update_id.
  void markKnownBy(int cell_id, int robot_id);

  // --- Wire codec and merge ----------------------------------------------
  // The codec lives beside the status machine because the merge guard table is
  // written against the normalised wire vocabulary (rule 3); change them
  // together. (notes: cell-codec-beside-machine)

  /// One cell as it travels: exactly the fields of CellState.msg, decoupled
  /// from the generated type so this unit and its tests do not depend on ROS.
  /// `status` is the RAW wire byte, not a CellStatus, so a peer sending a
  /// value outside {0,1,2} can be counted rather than silently coerced.
  struct WireCell {
    uint16_t id        = 0;
    uint8_t  status    = 0;
    uint32_t known_by  = 0;
    uint32_t update_id = 0;
  };

  /// This robot's whole census in the wire vocabulary, one entry per cell, in
  /// id order. Full state every time — see TeamWorld.msg on why there are no
  /// deltas.
  std::vector<WireCell> toWire() const;

  /// FNV-1a over every cell's normalised wire status in id order, nothing else,
  /// so robots with the same shared belief match. update_id and known_by are
  /// excluded on purpose (rules 1 and 2). (notes: cell-shared-hash)
  uint32_t sharedHash() const;

  /// FNV-1a over edges_, size folded in first. A detector only: edges_ is never
  /// exchanged, so it can differ while sharedHash() agrees. Nothing may gate on
  /// it, and it must never go on the wire. (notes: cell-edge-hash)
  uint32_t edgeHash() const;

  /// What one mergeWire() call did. Each refusal has its own counter:
  /// agreement, local-priority refusal and a config fault all otherwise look
  /// like a merge that changed nothing. (notes: cell-merge-stats)
  struct MergeStats {
    /// Non-empty when the merge was refused WHOLESALE and no cell was
    /// examined. A zero-count result with an empty reason (a peer that agrees)
    /// and one with a reason (a bug upstream) are different events, and a
    /// caller that cannot tell them apart will report the second as the first.
    std::string refused;

    int applied         = 0;  ///< cells whose local status changed
    int known_by_only   = 0;  ///< statuses already agreed; only the mask grew
    int agreed_noop     = 0;  ///< agreed, and we already knew everyone it named
    int refused_guard   = 0;  ///< the guard table declined the peer's claim
    int refused_local   = 0;  ///< local-priority: my neighbourhood, my call
    int out_of_range    = 0;  ///< id not in this grid — config fault, not noise
    int bad_status      = 0;  ///< byte outside {0,1,2}: not a wire status

    /// Every entry offered lands in exactly one bucket, so a caller can assert
    /// this against the message's cell count and catch a merge that silently
    /// skipped a case the table does not handle.
    int examined() const {
      return applied + known_by_only + agreed_noop + refused_guard +
             refused_local + out_of_range + bad_status;
    }
  };

  /// Guard-table merge of a peer's census; an out-of-range sender_id is refused
  /// wholesale. local_priority_centre (-1 = off): first-hand EXPLORING in its
  /// 3x3 refuses the peer. Never touches update_id or lowers a first-hand
  /// COVERED. (notes: cell-merge-wire)
  MergeStats mergeWire(int sender_id, const std::vector<WireCell>& cells,
                       int local_priority_centre = -1);

  /// `id` and its up-to-8 grid neighbours, written to `out` (room for 9
  /// required), returning the count. Used by the local-priority rule now and
  /// by P3's focus filtering later — one definition of "neighbourhood", so the
  /// two cannot drift apart.
  int neighbourhood9(int id, int* out) const;

  /// Cells this robot believes robot_id has seen in their current status and
  /// nobody considers finished; empty means lost. Relies on the known_by reset
  /// (rule 2). Empty for this robot's own id.
  /// (notes: cell-intercept-candidates)
  std::vector<int> interceptCandidates(int robot_id) const;

  /// Counts by status, for the census event and the RViz layer.
  struct Census {
    int unseen = 0, exploring = 0, covered = 0;
    int exploring_by_others = 0, covered_by_others = 0;
    /// Fraction of cells this robot considers finished by anyone. Comparable
    /// to coverageUnknownFraction only up to cell quantisation — a cell counts
    /// wholly covered or not at all — which is the bound P1's gate checks.
    double coveredFraction() const;
  };
  Census census() const;

  // --- Adjacency graph ---------------------------------------------------
  // 8-connected centroids with Euclidean weights: a cheap approximation of
  // drivable distance, used only as a ranking input to the allocator.
  // (notes: cell-adjacency-graph)

  /// Probe returning the fraction of the straight line (x0,y0)-(x1,y1) that is
  /// occupied or unknown in the plan map, or a negative value for "no map".
  using BlockedProbe = std::function<double(float, float, float, float)>;

  /// Rebuild the edge set. With no probe (or a probe returning negative) every
  /// edge is enabled. Marks the distance table dirty; distances are recomputed
  /// lazily on the next distance() call, so a rebuild in a tick that never
  /// asks for a distance costs nothing.
  void rebuildEdges(const BlockedProbe& probe = nullptr);

  bool edgeEnabled(int a, int b) const;

  /// Shortest-path distance over enabled edges, or negative if unreachable. The
  /// all-pairs table (Floyd-Warshall, O(n^3)) is cached and recomputed on the
  /// first call after an edge change. (notes: cell-distance-table)
  double distance(int a, int b) const;

  /// Straight-line centroid distance, always available and never blocked.
  /// The allocator uses it as the fallback when the graph says unreachable,
  /// so an over-aggressive edge probe degrades the ranking instead of
  /// removing cells from consideration entirely.
  double centroidDistance(int a, int b) const;

private:
  CellGrid          grid_;
  Config            cfg_;
  int               self_id_ = -1;
  std::vector<Cell> cells_;

  /// Row-major size()xsize() adjacency, enabled flags only.
  std::vector<uint8_t> edges_;
  mutable std::vector<double> dist_;
  mutable bool dist_dirty_ = true;

  void recomputeDistances() const;
};

// ---------------------------------------------------------------------------
// Census from the fused map
// ---------------------------------------------------------------------------

/// Measures every cell in one walk of the voxel grid. Column semantics match
/// unknownColumnFraction (any voxel observes its column). Empty if
/// unconfigured; unreached cells get observed_columns == 0.
/// (notes: cell-census-from-map)
std::vector<CellWorld::CellObservation> censusFromMap(const MapCache& map,
                                                      const CellGrid& grid);

} // namespace explo_planner
