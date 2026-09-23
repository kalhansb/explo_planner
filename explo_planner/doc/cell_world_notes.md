# cell_world.hpp — design notes and history

The long comments of `include/explo_planner/cell_world.hpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Status](#status) — 1
- [CellGrid — declarations](#cellgrid--declarations) — 1
- [makeCellGrid](#makecellgrid) — 1
- [Config — declarations](#config--declarations) — 3
- [CellObservation — declarations](#cellobservation--declarations) — 1
- [CellWorld — declarations](#cellworld--declarations) — 10
- [censusFromMap](#censusfrommap) — 1

## Status

### cell-no-nogo

**Why NOGO is not ported** — attached to `enum class CellStatus : uint8_t {` (line 66)

```text
NOGO is deliberately not ported: mTARE uses it for terrain the vehicle
cannot enter, which here is the navigator's and the plan map's job, and a
second untested notion of impassability would just be somewhere else for
the two to disagree.
```

## CellGrid — declarations

### cell-grid-shared-geometry

**Cell ids depend on shared geometry** — attached to `struct CellGrid {` (line 103)

```text
The cell grid: row-major over the ROI, derived from params every robot
shares. A cell id names the same ground on every robot ONLY if all of them
derived it from the same ROI and cell size, which is why configHash()
exists and why TeamWorld's config check must fold it in — a fleet whose
ROI params differ has every id pointing somewhere else than the sender
meant, and nothing in the cell data itself would look wrong.
```

## makeCellGrid

### cell-make-grid

**Building the cell grid** — attached to `CellGrid makeCellGrid(float min_x, float max_x, float min_y, float max_y,` (line 138)

```text
Build a grid covering [min_x, max_x] x [min_y, max_y] at cell_size_m,
rounding the cell count UP so the ROI is fully covered. Returns a grid with
size() == 0 on non-finite or degenerate bounds, a non-positive cell size,
or a cell count above kMaxCells — every one of which is a configuration
error, and none of which may produce a half-built grid that merges.
```

## Config — declarations

### cell-hysteresis-pair

**Cell status hysteresis thresholds** — attached to `double covered_max_unknown   = 0.15;` (line 159)

```text
Hysteresis PAIR on the unknown-column fraction. A cell becomes COVERED
at or below `covered_max_unknown` and reverts to EXPLORING only above
`exploring_min_unknown`. They must satisfy
covered_max_unknown < exploring_min_unknown, or there is no hysteresis
band and the flap this is here to prevent comes straight back;
configure() refuses the pair rather than silently ordering them.

THESE ARE WORLD-CALIBRATED KNOBS, NOT CONSTANTS. They belong to the
same family as `done_unknown_fraction` in shared_params.yaml, and for
exactly the same reason: a cell's unknown fraction counts columns
holding no voxel at all, and in any world where some columns can never
be observed — outside the ingest z band, under canopy, permanently
occluded — the measure has a floor well above zero that no amount of
exploring removes. `done_unknown_fraction` COMPILES IN at 0.05 (the
dp() fallback in the node) and is 0.64 everywhere it is actually
configured — shared_params.yaml:348 and the sim harness both — for
precisely this reason; the pair below is overridden by that harness for
the same reason.

Say "compiled default", not "shipped": the shipped yaml has read 0.64
since the C4 calibration, and 0.05 is 13x tighter than the ~0.486
achievable floor, i.e. a criterion no run in this world can ever meet.

The defaults are the SATURATING-MAP values: they assume, as the 0.05
compiled fallback does, a map in which a swept cell really
does approach zero unknown. Ship-elsewhere and they are wrong, in a
direction that is loud rather than silent — every cell pins at
EXPLORING, which is what a floor above the threshold looks like.

Recognising that failure mode is the whole reason `cell_census` carries
cell_unknown_{min,p10,median}. An all-EXPLORING census is what you get
both from a broken census and from a threshold nothing in this world can
reach, and the two are indistinguishable without the distribution. Read
it off a run before touching these:

  cell_unknown_min      the best cell anywhere on the map. If this
                        plateaus above covered_max_unknown, NOTHING can
                        ever be promoted and the threshold is the bug.
  cell_unknown_p10      promote here for a selective census.
  cell_unknown_median   promote here for one that broadly agrees with a
                        mission-level completion criterion.

Measured for the flat-forest sim world, and calibrated there in
sim/run_explo_sim_rviz.sh rather than here — see
docs/mtare_evolution_plan.md §3.1.1 for the numbers and the procedure.
```

### cell-frontier-frac

**Frontier fraction threshold** — attached to `double covered_max_frontier_frac = 0.90;` (line 207)

```text
A cell with frontier voxels still in it is not finished, however few
unknown columns remain — the column measure is 2.5D and cannot see a
pocket of unknown volume behind a trunk.

A FRACTION of the cell's observed voxels, not a count of them. The
count this replaced (4) could not have been right at any setting: it is
measured in voxels, so its meaning changes with the cell size and the
map resolution, and at 10 m cells and 0.1 m voxels a well-swept cell
holds tens of thousands of voxels. Any human-looking number vetoes
every cell forever, which is what it did.

World-calibrated like the pair above, and permissive by default. How
much signal this carries is a property of how the map stores free
space: a "frontier" voxel is one with an absent 6-neighbour, so in a
densely ray-traced map only the true boundary qualifies and the check
is sharp, while in a sparse surface-shell map almost everything does
and it is nearly saturated. In the flat-forest sim it is the latter —
measured medians sat at 0.79–0.89 all run with the best-mapped cell at
0.83, so a threshold anywhere in that band splits the population near
its own median and mostly reports noise. The harness therefore raises
it to 0.95 there, where it guards outliers instead, and the column
measure does the discriminating. Setting it to 1.0 disables it
outright; the "cannot measure" case is rejected either way.
```

### cell-edge-blocked-fraction

**Edge blocking threshold** — attached to `double edge_max_blocked_fraction = 0.5;` (line 237)

```text
Fraction of the straight line between two cell centres that may be
occupied/unknown in the plan map before the edge is disabled. Only
consulted when the caller supplies a plan-map probe; with no probe all
edges stay enabled, matching the planner's existing straight-line
fallback philosophy.
```

## CellObservation — declarations

### cell-frontier-fraction-sentinel

**Frontier fraction sentinel** — attached to `double frontierFraction() const;` (line 259)

```text
Frontier voxels as a fraction of the cell's observed voxels, or -1.0
for "cannot measure" when the cell holds no voxels — the same
convention as unknownFraction(), so callers have one rule to remember.
Note the sentinel is NEGATIVE and a threshold comparison would pass it:
classify() screens on observed columns before it gets here.
```

## CellWorld — declarations

### cell-configure

**Configure refuses rather than half-builds** — attached to `std::string configure(const CellGrid& grid, const Config& cfg, int self_id);` (line 277)

```text
(Re)configure geometry and thresholds. Returns an empty string on
success, or an operator-legible reason for refusing. On refusal the world
is left UNCONFIGURED (size() == 0) rather than partly built, because a
half-configured cell world would publish ids that mean nothing.

Reconfiguring a populated world CLEARS it: cell ids are geometry, so
keeping the old statuses would silently re-point every one of them.
```

### cell-apply-observation

**Applying a census** — attached to `int applyObservation(const std::vector<CellObservation>& obs);` (line 295)

```text
Apply a fresh self-observation census. `obs` must have exactly size()
entries. Returns the number of cells whose status CHANGED — each of which
bumped its update_id and reset its known_by to {self}.

An entry with total_columns == 0 is "not measured this tick" and leaves
the cell alone; it is not evidence that the cell became unseen.
```

### cell-codec-beside-machine

**Why the codec sits here** — attached to `struct WireCell {` (line 318)

```text
The codec is here, next to the status machine, rather than in the node.
Normalisation is rule 3 and the merge guard table is written against the
NORMALISED vocabulary; if the two lived apart, a change to one could be
made without the other, and the failure — a `*_BY_OTHERS` reaching a
receiver — is silent at the point it happens and only shows up as an
allocator that disagrees with itself several phases later.
```

### cell-shared-hash

**What sharedHash covers** — attached to `uint32_t sharedHash() const;` (line 342)

```text
FNV-1a over the SHARED part of the census: every cell's wire status, in
id order, and nothing else. Two robots holding the same shared belief
produce the same value; one cell apart produces a different one.

This exists because the aggregate counts cannot answer the question the
P2 gate asks. "Both cell worlds converged" is a claim about which cells,
and two robots can hold identical unseen/exploring/covered totals over
completely different ground — most easily right after a dropout, when
each has covered about as much as the other somewhere else. A count-based
readout calls that convergence. This does not.

Deliberately NOT a hash of the whole cell record:
  * update_id is LOCAL and never comparable across robots (rule 1), so
    folding it in would guarantee two agreeing robots disagree here.
  * known_by is reset to {self} on every committed change (rule 2), so it
    legitimately differs between two robots that reached the same status
    by different routes.
  * the status is NORMALISED, for the same reason it is on the wire: a
    cell this robot covered itself and one it learned a peer covered are
    the same shared fact, and the `*_BY_OTHERS` distinction is local
    bookkeeping about provenance.
What is left is exactly the state the exchange is supposed to make agree.
```

### cell-edge-hash

**edgeHash is a detector only** — attached to `uint32_t edgeHash() const;` (line 366)

```text
FNV-1a over the traversability matrix `edges_`, plus its side length.

R3 / §3.6. This is the OTHER half of what two robots have to agree on for
"solve-same-take-own" to be arithmetic rather than hope, and it is the
half nothing could see. `sharedHash()` deliberately covers only the
statuses — the state the exchange is supposed to make agree. The cost
matrix is NOT exchanged: `mergeWire` reconciles status and `known_by` and
never touches `edges_`, so two robots can hold identical statuses,
identical masks and an identical `sharedHash()` while every `costMm()`
between them differs, and every tour with it. Unlike the pose-staleness
channel this does not shrink when the link comes back up.

So this is a DETECTOR, not a fix, and it must stay one: nothing consumes
it, nothing gates on it, and it must never be put on the wire on the
strength of an argument. Measure how often it differs first — a
disagreement rate near zero would mean the whole concern is theoretical,
and that is a result worth having before spending a binary generation.

Size is folded in first for the same reason as in sharedHash(): two grids
of different size are not comparable at all, so their digests must not be
able to collide by accident.
```

### cell-merge-stats

**Why each refusal has a counter** — attached to `struct MergeStats {` (line 389)

```text
What one mergeWire() call did. Every refusal has its own counter because
"the merge changed nothing" has several causes with opposite meanings: a
peer that agrees with us (healthy), a peer whose every update the local
-priority rule refused (healthy, and expected while exploring), and a
peer whose ids do not land in our grid at all (a config fault that would
otherwise look exactly like agreement).
```

### cell-merge-wire

**Merging a peer census** — attached to `MergeStats mergeWire(int sender_id, const std::vector<WireCell>& cells,` (line 419)

```text
Merge a peer's normalised census (mTARE's guard table; §3.2).

`sender_id` is the peer's fleet id and must be inside the policy cap: an
out-of-range id would make every mask operation a silent no-op, so it is
refused wholesale instead.

`local_priority_centre` is the cell this robot currently occupies, or -1
to disable the rule. That cell and its 8 neighbours are this robot's own
neighbourhood: while it holds a FIRST-HAND EXPLORING there, a peer's
claim about the same ground is refused outright. The robot is standing in
it and the peer is not.

Never touches update_id (rule 1) and never lowers a first-hand COVERED.
```

### cell-intercept-candidates

**Where a lost peer could be** — attached to `std::vector<int> interceptCandidates(int robot_id) const;` (line 441)

```text
Where a search for a peer that has gone missing could plausibly find it:
cells this robot believes `robot_id` has seen IN THEIR CURRENT STATUS and
which nobody considers finished. Port of mTARE's CheckLostRobot — an
empty result means there is nowhere left to look, which is what "lost"
means operationally.

This is only meaningful BECAUSE of rule 2. known_by resets on every
committed status change, so a cell the peer knew about and which has
since changed drops out of the set on its own. Without the reset the mask
would only grow and this would return everywhere the peer had ever been,
forever — a search list that never shrinks, and a lost test that can
never fire.

Empty for this robot's own id: self is in almost every mask, so the
answer would otherwise be most of the map for the one robot that cannot
be missing.
```

### cell-adjacency-graph

**The cell adjacency graph** — attached to `using BlockedProbe = std::function<double(float, float, float, float)>;` (line 471)

```text
Replaces mTARE's keypose graph plus roadmap. 8-connected centroids with
Euclidean weights: an approximation of drivable distance, and a
deliberately cheap one, since it is a ranking input to the allocator
rather than a path anybody drives.
```

### cell-distance-table

**Cached all-pairs distances** — attached to `double distance(int a, int b) const;` (line 489)

```text
Shortest-path distance over enabled edges, or a negative value if `b` is
unreachable from `a`. Computes the all-pairs table on first call after an
edge change (Floyd-Warshall, O(n^3): ~1 ms at 100 cells, tens of ms at
400 — which is why it is cached and dirty-flagged rather than recomputed
per query. If it ever shows up in plan_time_ms, the fix is
Dijkstra-per-source, not a smaller cap).
```

## censusFromMap

### cell-census-from-map

**One-pass census from the map** — attached to `std::vector<CellWorld::CellObservation> censusFromMap(const MapCache& map,` (line 521)

```text
Measure every cell in ONE walk of the voxel grid.

Not a loop over MapCache::unknownColumnFraction, which walks the whole grid
per call: at 400 cells that is 400 full traversals of a ~1.3 M-voxel map,
per planning tick. This is a single pass that bins each voxel into its
cell, plus a pass over the deduplicated column set — the same cost as ONE
unknownColumnFraction call for the whole census.

Column semantics deliberately match unknownColumnFraction's, so the two
measures can be compared as P1's gate requires: a column counts as observed
when any voxel (free OR occupied) projects into it, and volumetric unknowns
behind trunks are ignored.

Returns an empty vector if the grid is unconfigured; entries for cells the
map does not reach have total_columns > 0 and observed_columns == 0, which
the status machine reads as genuinely unseen rather than unmeasured.
```
