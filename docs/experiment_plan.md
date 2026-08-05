# Experiment plan: does EIG beat entropy, and is vegetation the reason?

*Status: this file was previously empty. Written 2026-08-05.*

## 0. The claim under test

> A log-odds cell at 0.5 could be unobserved or heavily contradicted; the Beta
> map separates them, and only the first is worth driving to. That is why EIG
> beats entropy in vegetation, where transmissive foliage manufactures large
> volumes of permanently ambiguous occupancy that an entropy planner keeps
> re-measuring.

The claim is attractive because it is mechanistic — it does not merely assert
that one scorer wins, it says *why*, and *where*. That makes it falsifiable in
three separate places, and it can fail at any one of them while still "winning"
overall for an unrelated reason. The plan below tests the three parts in
increasing order of cost, so a failure is found before the expensive runs.

The divergence the claim rests on is exact and worth stating numerically. For a
voxel at occupancy 0.5, Shannon entropy is `ln 2 = 0.693` regardless of how much
evidence produced it. Expected information gain is `0.193` for the unobserved
prior `Beta(1,1)` and falls toward `0` as evidence accumulates — about `0.001`
at `Beta(500,500)`. Entropy cannot tell those two voxels apart by construction;
EIG separates them by two orders of magnitude. The question is not whether the
functions differ — they provably do — but whether **maps of real vegetation
actually contain the population of voxels where they differ**, and whether that
population is large enough to change where a robot drives.

## 1. Hypotheses

**H1 — the population exists (representation).** Maps of vegetation contain a
substantial volume of voxels with high entropy and low EIG: heavily observed and
still ambiguous. Predicted to be concentrated in the canopy band and absent, or
much rarer, in structured environments.

**H2 — it changes behaviour.** An entropy-driven planner directs a
disproportionate share of its viewpoints at already-saturated volume, and
revisits it; an EIG-driven planner does not. This is the actual causal link and
the one most likely to be waved through without evidence.

**H3 — it costs performance.** EIG reaches a given coverage in less distance
travelled and fewer steps than entropy, and the gap is larger in vegetation than
in a structured control environment.

H1 is a property of the map alone and needs no planner run. H2 and H3 need
paired runs. **The environment contrast in H1 and H3 is not optional**: without
a structured control, a result showing EIG > entropy demonstrates only that EIG
is a better scorer, not that foliage is the reason — which is the part of the
claim that is actually novel.

## 2. What already exists, and what is missing

**The baseline scorers are already here.** `scoring.cpp` on the current branch
implements `eig`, `entropy`, `frontier` and `random` behind a `scoring::create()`
factory, all covered by `test_scoring.cpp`. Only the node's selection knob was
removed: `explo_planner_node.cpp:943` hardcodes `score_fn_ = scoring::eig`.
Restoring the comparison is a parameter declaration and one factory call — not a
port.

**Do not merge the `experiments` branch.** It carries the original comparison
harness, but it predates the current package entirely: flat repository layout,
no exploitation, no terrain-relative mode, no coordination, no proximity guard
(`git diff` against the current branch is +4.3 k / −13.5 k lines). Take the
`planner_type` parameter from it; leave the rest.

**The simulated forest can in principle reproduce the mechanism.** The
`flatforest` world is 91 oak trees whose *collision* geometry is the full
`oak_tree.dae` mesh — canopy included, not a simplified cylinder. Lidar rays
therefore pass through gaps in the leaf mesh or strike a leaf triangle depending
on viewpoint, which is exactly the mixed-evidence process the claim describes.
Range noise is Gaussian at σ = 0.01 m. (Note for any future world: `bush_05` and
similar models collide as plain boxes, so they produce crisp occupancy and are
useless for this test.)

**The existing snapshots cannot test H1, and this is the blocking gap.** The
`*_snap_t####.npz` files store `eig`, `var`, `conf` and `cls` only — no
occupancy probability and no `a_occ`/`a_free` — so entropy is not recoverable
from them. Worse, the source cloud is filtered to *occupied voxels only*: across
223 k voxels in the last `runB1` snapshot every single one has EIG < 0.05
against a theoretical maximum of 0.193, median 0.0022. The saved population is
entirely saturated. The ambiguous voxels the claim is about are precisely what
the current pipeline discards before writing.

## 3. Work, in order

### Step 1 — instrument (blocking, cheap)

The published cloud already carries `a_occ` and `a_free` as fields; the snapshot
writer drops them. Extend `metrics_logger.py` to save `a_occ`, `a_free` and
`occupancy_prob`, and take the snapshot from an **unfiltered** voxel set rather
than the occupied-only cloud. Nothing downstream can be tested until this lands,
and it costs nothing to run.

### Step 2 — test H1 offline (decisive, no runs needed)

From instrumented maps, plot the joint distribution of `(H(p), EIG)` per voxel
and report the mass in the high-entropy/low-EIG quadrant, broken down by height
band (ground / trunk / canopy) and by environment.

Datasets: (a) the real forest bags — `2026_07_06__bunker` and
`2026_07_06__curt`, which are the only genuine foliage in hand; (b) one
instrumented `flatforest` sim run; (c) one structured control run.

This is the decision point. If the quadrant is empty in the real bags, the
claim's mechanism is wrong and no amount of planner running will rescue it — the
honest move then is to drop the causal story and claim only what the runs
support. If it is populated in the bags but empty in sim, the sim canopy does not
reproduce real foliage and **all evidence for this claim must come from field
data**, with sim reserved for the controlled planner A/B in Step 4.

### Step 3 — restore the scorer knob

Declare a `planner_type` parameter (`eig` | `entropy` | `frontier` | `random`,
default `eig`) and replace the hardcoded assignment with
`scoring::create(planner_type)`. Set `RobotIntent.planner_type` from the same
value so mixed-scorer runs are attributable in the logs. Guard: `ssmi` routes to
a different evaluator path, so it must not be selectable here by accident.

### Step 4 — paired runs for H2 and H3

Single robot, coordination **off**. Multi-robot deconfliction changes which
viewpoints are available and would confound the scorer comparison; the
proximity guard adds hold time that contaminates distance-per-coverage. Run the
team comparison later, separately, if it is wanted at all.

| Factor | Levels |
| --- | --- |
| Scorer | `eig`, `entropy` (add `frontier`, `random` as floor references) |
| Environment | `flatforest` (vegetation), structured control (`maze` / boxed `flat_plane`) |
| Repeats | ≥ 5 per cell, fixed distinct start poses, identical across scorers |

Everything else held fixed: same world, same start poses, same step budget, same
ROI, same sensor model. Pair by start pose so the comparison is within-pair
rather than across pooled means — the variance between start poses in a forest
is large enough to swamp the effect otherwise.

**H3 metrics** come from the existing CSV with no changes: `distance_traveled`
and `step` at fixed coverage thresholds (50/80/95 % of observed columns),
`total_observed_voxels` and `frontier_voxels` over time, and steps to
termination. Report coverage-per-metre curves, not just endpoints — the claim
predicts the curves separate progressively, not that they end differently.

**H2 needs one new logged quantity.** For each selected goal, record the split of
its scored ray volume into unobserved versus already-observed voxels.
`FovEvaluator::EvalResult` already computes exactly this (`unknown_count`,
`observed_count`); it is discarded after the score is taken. Log both for the
selected candidate. The prediction is direct: the entropy planner's selected
viewpoints should carry a systematically higher already-observed fraction. Add a
revisit measure — cumulative path length within 2 m of previously occupied
positions — as the second, independent line of evidence for "keeps
re-measuring".

## 4. The confound to state in the paper

The entropy baseline runs on **the same Beta map**. The experiment therefore
isolates the *scoring function*, not the *map representation*: it shows that
entropy cannot exploit the evidence count the Beta posterior carries, which is
the mechanism, but it is not a comparison against a log-odds mapper.

The claim as currently worded ("why a Beta map rather than log-odds") overstates
what this design can support. Either narrow the wording to what is tested — the
Beta posterior makes the unobserved/contradicted distinction *available*, and a
scorer that ignores it pays for that — or add a genuine log-odds mapping
ablation, which is a mapper change and considerably more work. **The narrowed
claim is recommended**: it is what the evidence will support, and it is
sufficient for the argument the planner rests on.

A second, smaller caveat: `scoring::entropy` reads `p_occ`, and unobserved voxels
default to `Beta(1,1)` → p = 0.5 → maximum entropy. The baseline therefore
behaves exactly like the log-odds straw man it is meant to represent, which is
the intended semantics — but it should be stated rather than left for a reviewer
to discover.

## 5. Anticipated outcomes

If the claim holds: the high-entropy/low-EIG quadrant is populated in the field
bags and concentrated in the canopy; the entropy planner's viewpoints show a
higher already-observed fraction and more revisiting; and the coverage-per-metre
gap is significant in the forest and small or absent in the structured control.

The most likely partial outcome, given what the current snapshots show, is that
**sim under-produces the ambiguity** and only the field bags carry it. That is a
publishable result in itself, but it changes the paper's structure: the
mechanism evidence becomes field-derived and the sim runs demonstrate only the
planner comparison. Deciding that at Step 2 — before any run is executed — is
the point of this ordering.
