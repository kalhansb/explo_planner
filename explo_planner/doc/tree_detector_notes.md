# tree_detector.cpp — design notes and history

The long comments of `src/tree_detector.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [safeRes](#saferes) — 1
- [normEntropy](#normentropy) — 1
- [fitCrossSection](#fitcrosssection) — 2
- [connectionRadius](#connectionradius) — 1
- [connectionRadiusSqCells](#connectionradiussqcells) — 1
- [clusterVoxels](#clustervoxels) — 2
- [fitAndScore](#fitandscore) — 5
- [detectSemantic](#detectsemantic) — 1
- [detectGeometric](#detectgeometric) — 4
- [TreeDetector::detect](#treedetectordetect) — 1

## safeRes

### tree-safe-res

**The non-positive resolution fallback** — attached to `double safeRes(double res) { return (res > 0.0) ? res : 0.15; }` (line 41)

```text
Voxel size to divide by, carrying the same fallback fitAndScore() already
applies to the same field. A non-positive resolution is a configuration
error rather than a geometry, and every division by it in this file is
unrecoverable once it slips through: 1.0/0.0 is +inf, inf*0.0 is NaN, and a
float->int32 cast of either is undefined behaviour, so one bad parameter
turns the clustering hash key of every voxel into whatever the compiler felt
like emitting, and turns connectionRadius()'s window into a loop bound of
indeterminate size. The guard was present in fitAndScore() and absent
everywhere else, which is the asymmetry this closes; tree_detector_node
clamps n_azimuth_bins / n_height_bins at load but does NOT clamp
voxel_size, and it re-reads voxel_size from the incoming map's resolution on
every scan, so the value arriving here is not one this file may assume.
(2026-09-18)
```

## normEntropy

### tree-norm-entropy-nan

**Normalised entropy and NaN screening** — attached to `float normEntropy(float p) {` (line 67)

```text
Binary occupancy entropy of p, normalised to [0, 1] (H / ln2). p is clamped
off the {0,1} rails so the log is finite; a fully-decided voxel reads ~0.

A non-finite p reads maximally uncertain (1.0) rather than propagating:
std::min/std::max do NOT sanitise NaN, so without this a single poisoned
voxel would carry NaN through mean_entropy into info_deficit, where (a)
`deficit > deficit_thresh` compares false and the tree silently reads
well-observed, and (b) the std::sort comparator on info_deficit stops being
a strict weak ordering -- undefined behaviour. 1.0 is the conservative
reading: an undecidable voxel is one we know nothing about.
```

## fitCrossSection

### tree-cross-section-fit

**Why the trunk axis is circle-fitted** — attached to `AxisFit fitCrossSection(const TreeDetectorConfig& cfg,` (line 92)

```text
Algebraic (Taubin) circle fit over pooled trunk residuals.

Why this exists: the obvious axis estimator -- the per-component median of
the trunk voxels -- puts the "axis" wherever the observed voxels are. A
trunk seen from ONE side therefore lands its centre ON the observed arc
instead of behind it, and the azimuths of those voxels then fan out around
that point through every sector. angular_coverage reads ~1.0 for a
half-observed trunk: the primary information metric goes blind exactly when
the tree most needs circling, which is what killed the self-closing loop on
the map-test-2 bag (13/17 real trunks read coverage 1.00, and the reachable
deficit then capped below deficit_thresh so none could ever be nominated).

Fitting a circle instead recovers a centre that does NOT follow the data:
for a one-sided arc the fitted centre sits behind the arc, so the arc spans
its true angular extent and coverage reads ~the fraction actually seen.

The caller pools residuals from every z-layer, each de-referenced by its OWN
median, so axis tilt or bend cancels and all layers superimpose onto one
cross-section. Pooling is what makes the fit conditionable at map
resolution: a single 0.15 m layer of an r=0.25 m trunk holds only ~10
surface voxels, far too few, while the pooled set holds hundreds.

Returns ok=false for any residual set the fit cannot describe -- collinear
(a flat wall patch, det ~ 0), or a short shallow arc that extrapolates to an
implausibly large circle. Those are "this is not a trunk cross-section",
so the caller falls back to the median rather than dropping the cluster.
```

### tree-taubin-not-kasa

**Taubin rather than Kasa** — attached to `double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;` (line 128)

```text
Taubin's gradient-weighted algebraic fit, not the simpler Kasa fit: Kasa is
heavily biased on PARTIAL arcs -- it pulls the centre toward the arc and
shrinks the radius, which is precisely the regime that matters here (a
one-sided trunk is nothing but a partial arc). Measured on the 120-degree
test arc, Kasa put the centre 0.28 m off and the radius at 0.30 m for a
true 0.40 m; Taubin is near-unbiased over the same data.
```

## connectionRadius

### tree-connection-cube

**The flood-fill candidate cube** — attached to `int connectionRadius(const TreeDetectorConfig& cfg) {` (line 206)

```text
Half-width in cells of the CANDIDATE CUBE the flood-fill scans around each
cell. This is a prefilter and nothing more — connectionRadiusSqCells() below
is what decides whether a candidate actually joins. A window > 1 voxel
bridges the 1-2 cell gaps a thin/sparse trunk surface leaves under strict
26-connectivity, without merging trees metres apart.

The cube always encloses the Euclidean ball it stands in for, so nothing is
lost by prefiltering: lround(x) >= floor(x) for x >= 0, and a ball of radius
x holds no integer offset with any component above floor(x).
```

## connectionRadiusSqCells

### tree-connection-radius-euclidean

**Euclidean connection radius** — attached to `float connectionRadiusSqCells(const TreeDetectorConfig& cfg) {` (line 220)

```text
SQUARED connection radius in cells, for the Euclidean acceptance test.

Connectivity used to be decided by the cube alone, i.e. as a Chebyshev
(axis-max) distance, so the effective merge distance was the cube's CORNER —
up to sqrt(3) * cluster_tol_m. At the shipped 0.15 m voxel and 0.30 m
tolerance that is 0.52 m, and two stems 0.520 m apart merged into a single
tree, which changes the reported tree count: the quantity the whole detector
exists to produce, silently, in the direction of under-counting a stand.
Testing a Euclidean radius makes the merge distance the same on every
bearing instead of reaching 73% further along the diagonals. (2026-09-18)

The radius is the cube's INSCRIBED ball, not the raw cluster_tol_m ball, and
the difference is deliberate. Those two agree at the shipped configuration —
0.30 / 0.15 is exactly 2 cells, so the merge distance is 0.30 m on every
bearing and the 0.52 m corner is gone — but they part company whenever
cluster_tol_m / voxel_size rounds UP in connectionRadius(). Taking the raw
ball there would tighten connectivity below what the cube has always
delivered along its own axes, which is a second, unasked-for behaviour
change and a destructive one: at 0.30 m tolerance on a 0.20 m map it drops
the axial reach from 2 cells to 1, and a trunk whose voxel column skips a
layer — which a float32 lattice landing on a cell boundary does — splits in
half and is reported at half its height. Shrinking the cube to its inscribed
ball removes the anisotropy without removing any reach. Where the rounding
binds, the merge distance is connectionRadius() * voxel_size (at most
cluster_tol_m + voxel_size/2); that rounding predates this change and is
unaltered by it.

Floored at 3 — the (1,1,1) corner — because the inscribed ball of the R = 1
cube is the whole cube, and anything less would silently drop to
6-connectivity (faces only) and shred every thin trunk surface into separate
clusters, which is a worse failure than the one being fixed.
```

## clusterVoxels

### tree-cluster-voxels

**Flood-fill cluster labelling** — attached to `std::vector<std::vector<int>> clusterVoxels(const std::vector<SemVoxel>& voxels,` (line 257)

```text
Flood-fill connected-component labelling over `sel` (indices into `voxels`),
joining cells within the Euclidean radius `r2_cells` (squared, in cells) and
using the (2R+1)^3 window only to enumerate candidates. `sel` must hold at
most one voxel per grid cell (both front-ends dedup while gating). Returns
clusters as vectors of voxel indices.
```

### tree-euclidean-join-test

**The Euclidean join test** — attached to `const float d2 = static_cast<float>(dx * dx + dy * dy + dz * dz);` (line 288)

```text
The cube enumerates candidates; THIS decides. Accepting the whole
cube made connectivity a Chebyshev test, so the real merge radius
was the corner, sqrt(3) * cluster_tol_m — 0.52 m at the shipped
0.30 m, enough to fuse two 0.520 m-apart stems into one tree and
change the count the detector reports. Kept before the hash
lookup, so the corners of the cube cost three integer multiplies
rather than a find(): at the default radius this does about a
quarter of the lookups the old loop did. (2026-09-18)
```

## fitAndScore

### tree-radius-from-median

**Radius from median distance** — attached to `std::vector<float> dists;` (line 374)

```text
Take the CENTRE from the fit but the RADIUS from the median distance to
it -- not fit.radius. The fit is least-squares, so it chases every voxel
in the cluster, and a geometric-mode "trunk" is not a clean shell: it
carries undergrowth, branch stubs and neighbouring stems that survived
clustering. On the map-test-2 bag fit.radius blew the two ground-truth
trunks out to 1.00 m and 0.72 m against true radii of 0.25 m and 0.27 m,
while the median distance to the same fitted centre stays on the trunk
surface. This matters downstream: the planner sets vantage standoff to
radius + vantage_standoff_m, so an inflated radius mis-sizes the ring.
```

### tree-median-fallback

**Median fallback on a degenerate fit** — attached to `std::vector<float> xs, ys;` (line 395)

```text
Degenerate fit (flat patch, or too short an arc to constrain a circle):
fall back to the old estimator -- per-component median of trunk XY, and
radius as the median distance to it. This is BIASED for a one-sided
trunk, and angular_coverage below inherits that bias, so the detection
is flagged axis_fitted=false for the caller to discount.
```

### tree-azimuth-bins-floor

**Flooring the azimuth bin count** — attached to `const int n_az = std::max(cfg.n_azimuth_bins, 1);` (line 432)

```text
The bin count is normalised locally, exactly as bearingBit() and
bearingCoverage() further down already normalise the same parameter. A zero
is not a configuration this block can express: az_hit would be empty, `b`
would clamp to -1 and write off the front of it, and `filled / n` below
would be 0/0 = NaN. tree_detector_node clamps both bin counts to >= 1 at
parameter load, so none of that is reachable from the campaign harness
today — but TreeDetectorConfig is a plain struct of public fields and
TreeDetector is public API, so unreachable here only means one caller away
from reachable, and what waits there is undefined behaviour rather than a
wrong number. A NaN coverage would then INVERT the verdict at the bottom of
this function (`deficit > deficit_thresh` compares false against NaN, so an
unmeasurable tree reads well-observed and can never be nominated) and
destroy the strict weak ordering std::sort relies on in
TreeDetector::detect — the same failure normEntropy() is guarded against
at the top of this file, arriving by a different route. (2026-09-18)
```

### tree-empty-trunk-entropy

**Entropy of an empty trunk** — attached to `const float mean_entropy =` (line 486)

```text
The denominator is floored for the same reason and with the same
consequences as n_az above: an empty trunk gives 0/0 = NaN, which reaches
under_informed and the sort comparator. It is NOT ruled out by the
min_trunk_voxels gate at the top of this function — with min_trunk_voxels
set to 0 that gate passes an empty set, and detectSemantic's fallback to
the whole cluster is written as `trunk.size() < min_trunk_voxels` and so
does not fire either, leaving trunk genuinely empty. 0 is the right value
for the entropy of nothing: it contributes no deficit of its own, and the
empty cluster is rejected on radius or coverage instead. (2026-09-18)
```

### tree-height-bins-floor

**Flooring the height bin count** — attached to `const int n_z = std::max(cfg.n_height_bins, 1);` (line 501)

```text
Normalised locally for the reasons given at n_az above — empty vector, an
index of -1 written into it, and a 0/0 NaN that inverts under_informed and
poisons the sort comparator.
```

## detectSemantic

### tree-semantic-components

**Semantic connected components** — attached to `const auto clusters =` (line 562)

```text
2. Connected-component labelling over the vegetation voxels (26-neighbour
   window). One component ~ one tree; well-separated trunks land in
   distinct components. (Touching canopies can merge two trees into one
   component — an accepted v1 limitation; the trunk-band fit below still
   recovers a single dominant axis. Geometric mode's stem-slice clustering
   does not share this limitation.)
```

## detectGeometric

### tree-geometric-front-end

**The geometric front-end** — attached to `std::vector<TreeDetection> detectGeometric(const TreeDetectorConfig& cfg,` (line 596)

```text
Geometric front-end (LiDAR-only maps, no semantic records): "tree" must
come from shape alone. Terrain is estimated and removed (else the ground
plane connects the whole forest into one component), stems are clustered in
a height slice above local ground, shape-gated by PCA (linear + vertical),
and the rest of each tree is attached to its nearest stem axis.
```

### tree-terrain-bilinear

**Bilinear terrain height** — attached to `const auto terrainAt = [&](float x, float y) {` (line 657)

```text
Continuous terrain height at (x, y): bilinear interpolation between the
filtered cell-centre samples. A piecewise-constant lookup under-estimates
ground by up to grade*cell within a cell (min-z sits at the downslope
edge), which leaked ground through the margin gate on grades steeper than
~ground_margin_m/terrain_cell_m (~22 deg at defaults) and could fuse a
trunk with a ground ribbon into a tilt-rejected cluster. Interpolation
reconstructs a planar slope exactly up to the constant grade*cell/2 min-z
offset, doubling the tolerated grade (~2*margin/cell, ~38 deg at
defaults). Missing neighbour cells fall back to the query's own cell.
```

### tree-stem-shape-gates

**Stem shape gates** — attached to `const auto clusters =` (line 702)

```text
4. Cluster the stem slice, then shape-gate each candidate: a trunk is an
   elongated, near-vertical column. PCA eigenvalues (ascending l0<=l1<=l2)
   give linearity (l2-l1)/l2 ~ 1 for a column and ~0 for an isotropic
   bush, which is what this gate rejects. Walls are NOT caught by
   linearity: any wall longer than its in-slice height reads linear along
   its length. The verticality gate is what rejects walls, fallen logs and
   ground ribbons — their principal axis is horizontal.
```

### tree-attach-to-stem

**Attaching the rest of each tree** — attached to `const auto comps =` (line 759)

```text
5. Attach the rest of each tree (lower trunk below the slice, canopy above
   it) to its stem. Ownership needs more than XY proximity: a bare
   nearest-axis sweep over all above-ground voxels lets unconnected
   structure — a shape-rejected wall face, a neighbour's overhanging
   crown, floating clutter — donate voxels into a stem's member set,
   inflating height / vertical completeness and even reviving
   sub-min-height stumps. Instead:
     * connected components over the whole above-ground set;
     * a stem's own slice voxels always stay its own (trunk ⊆ members,
       which fitAndScore's base/height math relies on);
     * every other voxel joins the nearest stem axis within
       attach_radius_m FROM ITS OWN COMPONENT — nearest-axis only
       arbitrates genuinely fused neighbours (touching canopies), and a
       component holding no accepted stem is discarded whole.
   A fragment separated from its stem by a > cluster_tol_m map gap is
   dropped rather than attached; vertical completeness still reads gaps
   that lie inside the connected extent.
```

## TreeDetector::detect

### tree-sort-strict-weak-order

**Sorting by info deficit** — attached to `std::sort(out.begin(), out.end(),` (line 916)

```text
Neediest first: the node emits the top under-informed trees and logs the
rest. Stable tie-break on center keeps the order deterministic.

This comparator is only a strict weak ordering because info_deficit is
guaranteed finite, and that guarantee is not local to it — it is held up by
normEntropy()'s non-finite screen and by the n_az / n_z / trunk.empty()
denominators in fitAndScore(). A single NaN deficit makes every comparison
against it false, the ordering stops being strict-weak, and std::sort is
then undefined behaviour rather than merely misordered. Anything that
introduces a new term into infoDeficit() has to carry its own screen for
this to keep holding. (2026-09-18)
```
