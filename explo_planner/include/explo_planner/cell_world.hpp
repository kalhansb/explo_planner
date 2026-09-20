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
/// NOGO is deliberately not ported: mTARE uses it for terrain the vehicle
/// cannot enter, which here is the navigator's and the plan map's job, and a
/// second untested notion of impassability would just be somewhere else for
/// the two to disagree.
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

/// The cell grid: row-major over the ROI, derived from params every robot
/// shares. A cell id names the same ground on every robot ONLY if all of them
/// derived it from the same ROI and cell size, which is why configHash()
/// exists and why TeamWorld's config check must fold it in — a fleet whose
/// ROI params differ has every id pointing somewhere else than the sender
/// meant, and nothing in the cell data itself would look wrong.
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

/// Build a grid covering [min_x, max_x] x [min_y, max_y] at cell_size_m,
/// rounding the cell count UP so the ROI is fully covered. Returns a grid with
/// size() == 0 on non-finite or degenerate bounds, a non-positive cell size,
/// or a cell count above kMaxCells — every one of which is a configuration
/// error, and none of which may produce a half-built grid that merges.
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
    /// Hysteresis PAIR on the unknown-column fraction. A cell becomes COVERED
    /// at or below `covered_max_unknown` and reverts to EXPLORING only above
    /// `exploring_min_unknown`. They must satisfy
    /// covered_max_unknown < exploring_min_unknown, or there is no hysteresis
    /// band and the flap this is here to prevent comes straight back;
    /// configure() refuses the pair rather than silently ordering them.
    ///
    /// THESE ARE WORLD-CALIBRATED KNOBS, NOT CONSTANTS. They belong to the
    /// same family as `done_unknown_fraction` in shared_params.yaml, and for
    /// exactly the same reason: a cell's unknown fraction counts columns
    /// holding no voxel at all, and in any world where some columns can never
    /// be observed — outside the ingest z band, under canopy, permanently
    /// occluded — the measure has a floor well above zero that no amount of
    /// exploring removes. `done_unknown_fraction` COMPILES IN at 0.05 (the
    /// dp() fallback in the node) and is 0.64 everywhere it is actually
    /// configured — shared_params.yaml:348 and the sim harness both — for
    /// precisely this reason; the pair below is overridden by that harness for
    /// the same reason.
    ///
    /// Say "compiled default", not "shipped": the shipped yaml has read 0.64
    /// since the C4 calibration, and 0.05 is 13x tighter than the ~0.486
    /// achievable floor, i.e. a criterion no run in this world can ever meet.
    ///
    /// The defaults are the SATURATING-MAP values: they assume, as the 0.05
    /// compiled fallback does, a map in which a swept cell really
    /// does approach zero unknown. Ship-elsewhere and they are wrong, in a
    /// direction that is loud rather than silent — every cell pins at
    /// EXPLORING, which is what a floor above the threshold looks like.
    ///
    /// Recognising that failure mode is the whole reason `cell_census` carries
    /// cell_unknown_{min,p10,median}. An all-EXPLORING census is what you get
    /// both from a broken census and from a threshold nothing in this world can
    /// reach, and the two are indistinguishable without the distribution. Read
    /// it off a run before touching these:
    ///
    ///   cell_unknown_min      the best cell anywhere on the map. If this
    ///                         plateaus above covered_max_unknown, NOTHING can
    ///                         ever be promoted and the threshold is the bug.
    ///   cell_unknown_p10      promote here for a selective census.
    ///   cell_unknown_median   promote here for one that broadly agrees with a
    ///                         mission-level completion criterion.
    ///
    /// Measured for the flat-forest sim world, and calibrated there in
    /// sim/run_explo_sim_rviz.sh rather than here — see
    /// docs/mtare_evolution_plan.md §3.1.1 for the numbers and the procedure.
    double covered_max_unknown   = 0.15;
    double exploring_min_unknown = 0.35;

    /// A cell with frontier voxels still in it is not finished, however few
    /// unknown columns remain — the column measure is 2.5D and cannot see a
    /// pocket of unknown volume behind a trunk.
    ///
    /// A FRACTION of the cell's observed voxels, not a count of them. The
    /// count this replaced (4) could not have been right at any setting: it is
    /// measured in voxels, so its meaning changes with the cell size and the
    /// map resolution, and at 10 m cells and 0.1 m voxels a well-swept cell
    /// holds tens of thousands of voxels. Any human-looking number vetoes
    /// every cell forever, which is what it did.
    ///
    /// World-calibrated like the pair above, and permissive by default. How
    /// much signal this carries is a property of how the map stores free
    /// space: a "frontier" voxel is one with an absent 6-neighbour, so in a
    /// densely ray-traced map only the true boundary qualifies and the check
    /// is sharp, while in a sparse surface-shell map almost everything does
    /// and it is nearly saturated. In the flat-forest sim it is the latter —
    /// measured medians sat at 0.79–0.89 all run with the best-mapped cell at
    /// 0.83, so a threshold anywhere in that band splits the population near
    /// its own median and mostly reports noise. The harness therefore raises
    /// it to 0.95 there, where it guards outliers instead, and the column
    /// measure does the discriminating. Setting it to 1.0 disables it
    /// outright; the "cannot measure" case is rejected either way.
    double covered_max_frontier_frac = 0.90;

    /// Columns that must be observed before a cell can leave UNSEEN. One
    /// stray voxel from a distant lidar return should not promote a cell the
    /// robot has never actually been near.
    int min_observed_columns = 4;

    /// Fraction of the straight line between two cell centres that may be
    /// occupied/unknown in the plan map before the edge is disabled. Only
    /// consulted when the caller supplies a plan-map probe; with no probe all
    /// edges stay enabled, matching the planner's existing straight-line
    /// fallback philosophy.
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

    /// Frontier voxels as a fraction of the cell's observed voxels, or -1.0
    /// for "cannot measure" when the cell holds no voxels — the same
    /// convention as unknownFraction(), so callers have one rule to remember.
    /// Note the sentinel is NEGATIVE and a threshold comparison would pass it:
    /// classify() screens on observed columns before it gets here.
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

  /// (Re)configure geometry and thresholds. Returns an empty string on
  /// success, or an operator-legible reason for refusing. On refusal the world
  /// is left UNCONFIGURED (size() == 0) rather than partly built, because a
  /// half-configured cell world would publish ids that mean nothing.
  ///
  /// Reconfiguring a populated world CLEARS it: cell ids are geometry, so
  /// keeping the old statuses would silently re-point every one of them.
  std::string configure(const CellGrid& grid, const Config& cfg, int self_id);

  bool configured() const { return grid_.size() > 0; }
  const CellGrid& grid() const { return grid_; }
  const Config& config() const { return cfg_; }
  int  size() const { return grid_.size(); }
  int  selfId() const { return self_id_; }

  const Cell& cell(int id) const { return cells_[id]; }
  CellStatus status(int id) const { return cells_[id].status; }

  /// Apply a fresh self-observation census. `obs` must have exactly size()
  /// entries. Returns the number of cells whose status CHANGED — each of which
  /// bumped its update_id and reset its known_by to {self}.
  ///
  /// An entry with total_columns == 0 is "not measured this tick" and leaves
  /// the cell alone; it is not evidence that the cell became unseen.
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
  //
  // The codec is here, next to the status machine, rather than in the node.
  // Normalisation is rule 3 and the merge guard table is written against the
  // NORMALISED vocabulary; if the two lived apart, a change to one could be
  // made without the other, and the failure — a `*_BY_OTHERS` reaching a
  // receiver — is silent at the point it happens and only shows up as an
  // allocator that disagrees with itself several phases later.

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

  /// FNV-1a over the SHARED part of the census: every cell's wire status, in
  /// id order, and nothing else. Two robots holding the same shared belief
  /// produce the same value; one cell apart produces a different one.
  ///
  /// This exists because the aggregate counts cannot answer the question the
  /// P2 gate asks. "Both cell worlds converged" is a claim about which cells,
  /// and two robots can hold identical unseen/exploring/covered totals over
  /// completely different ground — most easily right after a dropout, when
  /// each has covered about as much as the other somewhere else. A count-based
  /// readout calls that convergence. This does not.
  ///
  /// Deliberately NOT a hash of the whole cell record:
  ///   * update_id is LOCAL and never comparable across robots (rule 1), so
  ///     folding it in would guarantee two agreeing robots disagree here.
  ///   * known_by is reset to {self} on every committed change (rule 2), so it
  ///     legitimately differs between two robots that reached the same status
  ///     by different routes.
  ///   * the status is NORMALISED, for the same reason it is on the wire: a
  ///     cell this robot covered itself and one it learned a peer covered are
  ///     the same shared fact, and the `*_BY_OTHERS` distinction is local
  ///     bookkeeping about provenance.
  /// What is left is exactly the state the exchange is supposed to make agree.
  uint32_t sharedHash() const;

  /// FNV-1a over the traversability matrix `edges_`, plus its side length.
  ///
  /// R3 / §3.6. This is the OTHER half of what two robots have to agree on for
  /// "solve-same-take-own" to be arithmetic rather than hope, and it is the
  /// half nothing could see. `sharedHash()` deliberately covers only the
  /// statuses — the state the exchange is supposed to make agree. The cost
  /// matrix is NOT exchanged: `mergeWire` reconciles status and `known_by` and
  /// never touches `edges_`, so two robots can hold identical statuses,
  /// identical masks and an identical `sharedHash()` while every `costMm()`
  /// between them differs, and every tour with it. Unlike the pose-staleness
  /// channel this does not shrink when the link comes back up.
  ///
  /// So this is a DETECTOR, not a fix, and it must stay one: nothing consumes
  /// it, nothing gates on it, and it must never be put on the wire on the
  /// strength of an argument. Measure how often it differs first — a
  /// disagreement rate near zero would mean the whole concern is theoretical,
  /// and that is a result worth having before spending a binary generation.
  ///
  /// Size is folded in first for the same reason as in sharedHash(): two grids
  /// of different size are not comparable at all, so their digests must not be
  /// able to collide by accident.
  uint32_t edgeHash() const;

  /// What one mergeWire() call did. Every refusal has its own counter because
  /// "the merge changed nothing" has several causes with opposite meanings: a
  /// peer that agrees with us (healthy), a peer whose every update the local
  /// -priority rule refused (healthy, and expected while exploring), and a
  /// peer whose ids do not land in our grid at all (a config fault that would
  /// otherwise look exactly like agreement).
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

  /// Merge a peer's normalised census (mTARE's guard table; §3.2).
  ///
  /// `sender_id` is the peer's fleet id and must be inside the policy cap: an
  /// out-of-range id would make every mask operation a silent no-op, so it is
  /// refused wholesale instead.
  ///
  /// `local_priority_centre` is the cell this robot currently occupies, or -1
  /// to disable the rule. That cell and its 8 neighbours are this robot's own
  /// neighbourhood: while it holds a FIRST-HAND EXPLORING there, a peer's
  /// claim about the same ground is refused outright. The robot is standing in
  /// it and the peer is not.
  ///
  /// Never touches update_id (rule 1) and never lowers a first-hand COVERED.
  MergeStats mergeWire(int sender_id, const std::vector<WireCell>& cells,
                       int local_priority_centre = -1);

  /// `id` and its up-to-8 grid neighbours, written to `out` (room for 9
  /// required), returning the count. Used by the local-priority rule now and
  /// by P3's focus filtering later — one definition of "neighbourhood", so the
  /// two cannot drift apart.
  int neighbourhood9(int id, int* out) const;

  /// Where a search for a peer that has gone missing could plausibly find it:
  /// cells this robot believes `robot_id` has seen IN THEIR CURRENT STATUS and
  /// which nobody considers finished. Port of mTARE's CheckLostRobot — an
  /// empty result means there is nowhere left to look, which is what "lost"
  /// means operationally.
  ///
  /// This is only meaningful BECAUSE of rule 2. known_by resets on every
  /// committed status change, so a cell the peer knew about and which has
  /// since changed drops out of the set on its own. Without the reset the mask
  /// would only grow and this would return everywhere the peer had ever been,
  /// forever — a search list that never shrinks, and a lost test that can
  /// never fire.
  ///
  /// Empty for this robot's own id: self is in almost every mask, so the
  /// answer would otherwise be most of the map for the one robot that cannot
  /// be missing.
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
  //
  // Replaces mTARE's keypose graph plus roadmap. 8-connected centroids with
  // Euclidean weights: an approximation of drivable distance, and a
  // deliberately cheap one, since it is a ranking input to the allocator
  // rather than a path anybody drives.

  /// Probe returning the fraction of the straight line (x0,y0)-(x1,y1) that is
  /// occupied or unknown in the plan map, or a negative value for "no map".
  using BlockedProbe = std::function<double(float, float, float, float)>;

  /// Rebuild the edge set. With no probe (or a probe returning negative) every
  /// edge is enabled. Marks the distance table dirty; distances are recomputed
  /// lazily on the next distance() call, so a rebuild in a tick that never
  /// asks for a distance costs nothing.
  void rebuildEdges(const BlockedProbe& probe = nullptr);

  bool edgeEnabled(int a, int b) const;

  /// Shortest-path distance over enabled edges, or a negative value if `b` is
  /// unreachable from `a`. Computes the all-pairs table on first call after an
  /// edge change (Floyd-Warshall, O(n^3): ~1 ms at 100 cells, tens of ms at
  /// 400 — which is why it is cached and dirty-flagged rather than recomputed
  /// per query. If it ever shows up in plan_time_ms, the fix is
  /// Dijkstra-per-source, not a smaller cap).
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

/// Measure every cell in ONE walk of the voxel grid.
///
/// Not a loop over MapCache::unknownColumnFraction, which walks the whole grid
/// per call: at 400 cells that is 400 full traversals of a ~1.3 M-voxel map,
/// per planning tick. This is a single pass that bins each voxel into its
/// cell, plus a pass over the deduplicated column set — the same cost as ONE
/// unknownColumnFraction call for the whole census.
///
/// Column semantics deliberately match unknownColumnFraction's, so the two
/// measures can be compared as P1's gate requires: a column counts as observed
/// when any voxel (free OR occupied) projects into it, and volumetric unknowns
/// behind trunks are ignored.
///
/// Returns an empty vector if the grid is unconfigured; entries for cells the
/// map does not reach have total_columns > 0 and observed_columns == 0, which
/// the status machine reads as genuinely unseen rather than unmeasured.
std::vector<CellWorld::CellObservation> censusFromMap(const MapCache& map,
                                                      const CellGrid& grid);

} // namespace explo_planner
