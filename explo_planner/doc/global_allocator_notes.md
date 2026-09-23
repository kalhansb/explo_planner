# global_allocator.hpp, global_allocator.cpp — design notes and history

The long comments of global_allocator.hpp, global_allocator.cpp, moved out of the code on 2026-09-23 so the sources carry short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

One section per source file. Each entry names the function (or section) the comment sat in, the line of code it was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [include/explo_planner/global_allocator.hpp](#includeexplo_plannerglobal_allocatorhpp) — 11
- [src/global_allocator.cpp](#srcglobal_allocatorcpp) — 8

## include/explo_planner/global_allocator.hpp

### alloc-off-frontier

**Why off_frontier is separate from finished** — in `AllocRobot — declarations`, attached to `bool off_frontier = false;` (line 51)

```text
The robot has left the frontier: homing or done, i.e. TeamWorld/mode >=
MODE_HOMING (generation 33, R4). A STRICT SUPERSET of `finished` — every
finished robot is also off the frontier — and it is a separate field
rather than a widening of that one because the two answer different
questions and the codebase asks both.

THIS ONE decides who is IN THE ALLOCATION PROBLEM, and only that: a robot
driving home will claim no more ground, so reserving cells for it under
the makespan balance starves the robots that are still working. That is
the same argument `finished` was dropped on, extended to the window
between leaving for home and announcing the arrival — a window `finished`
cannot see, and the one where the reservation costs the most.

`finished` KEEPS every other reader it had, deliberately. The reconnect
gate must still PRICE a homing peer rather than skip it (its map is worth
exactly as much parked as moving), and all_in_comms must still count it,
because a homing robot out of contact is a real break in the team.
```

### alloc-empty-tour-is-normal

**An empty tour is a normal outcome** — in `Allocation — declarations`, attached to `struct Allocation {` (line 71)

```text
The result: one ordered tour per robot that got any cells, plus enough
bookkeeping to explain an empty one. An empty tour is a NORMAL outcome
(nothing left to explore, or every remaining cell masked away from this
robot) and callers must fall back to unrestricted planning rather than
treating it as a fault.
```

### alloc-hash-problem-digest

**What alloc_hash digests and why** — in `Allocation — declarations`, attached to `unsigned int alloc_hash = 0;` (line 95)

```text
R3 / §3.6. FNV-1a over the PROBLEM this solve was given, not over its
answer: the vehicle set sorted by id as
`(id, cell, in_comms, finished, off_frontier)`,
then the candidate cell ids in ascending order, then the world's
`edgeHash()`, then `cfg.comms_mask` and `cfg.max_candidates`. Two robots
that solve the same problem must produce the same value; two robots that
produce different tours with the SAME value have found a determinism bug
in the solver, and two robots with different values were never solving
the same problem and their disagreement is not the allocator's fault.

`cfg.polish_passes` is deliberately NOT in it, and the two that are were
added 2026-09-18. The line is drawn at whether a field changes the
PROBLEM or the approach to it: comms_mask changes the feasible assignment
set, max_candidates decides refused-versus-solved, and 2-opt passes change
only the tours. That last exclusion is what keeps the second reading above
meaningful -- two robots differing only in polish_passes really are
getting different answers to one problem, and folding it in would restate
that as "different problems" and lose it. See the digest block in
global_allocator.cpp for the full argument, and
AllocHash.SolverConfigSplitsOnWhatChangesTheProblem for the assertions.

That distinction is the whole point. `shared_hash` was being used as the
join key for "both robots saw the same world", and it cannot bear that --
for a STRUCTURAL reason, which is the one to rely on: `shared_hash`
covers cell statuses, while the problem also contains a vehicle set (not
part of the world at all) and a cost matrix (never exchanged). Equal
`shared_hash` therefore does not mean the same problem, so restricting an
agreement analysis to it does not restrict it to the same problem, and
every such analysis to date is unsound for that reason.

An earlier version of this comment quantified the leak as "4.5% at N=2
and 23% at N=4". **Do not cite those numbers.** They could not be
reproduced from the banked campaigns -- no denominator, filter or time
window was recorded with them, five reconstructions disagreed and three
of the five inverted the N ordering. A leak that looks like a clean
team-size gradient is also the exact shape of an artifact this project
has been caught by before. The structural argument above needs no
measurement and is not weakened by withdrawing one.

Computed INSIDE solve(), from the same values the solve used, rather than
re-derived by the caller. A digest of the inputs that is assembled a
second time somewhere else is a digest of a second thing that is believed
to be equal, and drift between the two would be invisible in exactly the
way this is supposed to make visible.

Pure instrumentation. Nothing branches on it.
```

### alloc-edge-hash-attribution

**Attributing an alloc_hash disagreement** — in `Allocation — declarations`, attached to `unsigned int edge_hash = 0;` (line 143)

```text
The `edgeHash()` component of `alloc_hash`, logged separately so a
disagreement can be attributed. If two robots differ in `alloc_hash` but
agree here, they disagreed about the fleet, the candidate set, or one of
the two config terms; if they differ here too, their local maps produced
different traversability and no amount of status exchange would have
fixed it.

That third arm is checkable without a second digest WHENEVER P3 IS ON:
both values are written to the run params (`global_alloc_comms_mask`,
`global_alloc_max_candidates`), so compare those across the two robots'
logs and eliminate them before reaching for the fleet or the candidates.
Every arm of every campaign runs global_alloc_enable=true, so in practice
they are always there.

WITH P3 OFF THEY ARE BOTH ABSENT, and this said "written to the run
params" flat until 2026-09-18. The two addParam calls sit inside `if
(global_alloc_enable_)` (explo_planner_node.cpp), while the solver is
still reached by the §3.6 gate and by the rendezvous scheduler, which
re-read max_candidates on their own paths and log nothing;
`global_alloc_comms_mask` is not even declared there, on purpose, because
the gate overwrites it per solve. So an allocator-off configuration
produces edge_hash values with no record of the config that made them.
Nothing to fix while no such campaign exists — but do not read the
absence of the params as "the defaults were used".
```

### alloc-comms-mask

**The no-comms mask** — in `Config — declarations`, attached to `bool comms_mask = false;` (line 177)

```text
Apply the no-comms mask: a robot that is out of comms may only be
assigned cells whose known_by already includes it. The port of mTARE's
INF-masking in GetDistanceMatricesNoComms, and the thing that makes an
`assume_comms` counterfactual mean something — the cost gap between a
masked and an unmasked solve IS the reconnection value P4 gates on.

Default false, so the plain solve is the unmasked one.
```

### alloc-polish-passes

**Capped deterministic 2-opt polish** — in `Config — declarations`, attached to `int polish_passes = 2;` (line 186)

```text
2-opt passes over each tour after assignment. Capped because this runs
every planning cycle and an uncapped local search has no bound anyone
has measured; 0 disables it. The polish is deterministic (first
improving move in (i, j) order, not best), so it cannot become a source
of cross-robot disagreement.
```

### alloc-max-candidates-refusal

**Refusing oversized candidate sets** — in `Config — declarations`, attached to `int max_candidates = 256;` (line 193)

```text
Refuse to solve above this many candidate cells. The assignment loop is
O(n^2 * robots * tour) and n is the EXPLORING set, not the whole grid —
typically tens. A grid-sized candidate set means something upstream is
wrong (every cell stuck EXPLORING because a threshold is unreachable in
this world — see cell_world.hpp's Config notes), and grinding through it
for seconds inside a planning tick would present as a hung planner. It
is a refusal, with a reason, rather than a silent truncation to the
first N: a truncated allocation is one both robots would compute
differently the moment their candidate sets differ by one cell.
```

### alloc-solve-candidates

**Solve inputs and the candidate set** — in `GlobalAllocator — declarations`, attached to `static Allocation solve(const CellWorld& world,` (line 205)

```text
Solve. `world` supplies the cell statuses, the known_by masks and the
distances; `robots` is the vehicle set in any order (it is sorted
internally by id, so the caller's order cannot affect the result).

Candidate cells are those with status EXPLORING or EXPLORING_BY_OTHERS.
Both, per §3.4: restricting to first-hand EXPLORING would have the two
robots solving provably different problems immediately after every
exchange, which destroys the solve-same premise this whole design rests
on. A cell one robot covered itself and one it learned about are the same
work to divide.
```

### alloc-costmm-fallback

**Cell cost and its straight-line fallback** — in `GlobalAllocator — declarations`, attached to `static long long costMm(const CellWorld& world, int a, int b);` (line 219)

```text
Cost between two cells in the quantised unit, exposed because it is the
determinism-critical primitive and testing it through solve() alone would
leave the quantisation boundary untested.

Falls back to straight-line centroid distance when the cell graph reports
`b` unreachable from `a`, so an over-aggressive edge probe DEGRADES the
ranking rather than removing cells from consideration entirely. An
unreachable cell that vanished from the problem would be a cell no robot
is ever sent to clear, and it would stay EXPLORING forever.
```

### alloc-route-cost-open

**Open route cost shared with the scheduler** — in `GlobalAllocator — declarations`, attached to `static long long routeCostMm(const CellWorld& world, int start,` (line 230)

```text
Open-route cost of `tour` driven from cell `start`, in the quantised unit.
Open, not closed: the robots are not coming back, so there is no closing
leg — adding one would inflate every cost by a term depending only on the
tour's last cell, biasing the makespan balance toward tours that happen to
end near their origin.

Exposed because §3.5's rendezvous objective is a DIFFERENCE OF MAKESPANS,
and a difference is only meaningful if both sides come from the same cost
function. A second, private copy in the scheduler would be free to drift
from this one — and the symptom of that drift is a penalty that ranks
cells the allocator would not, which is invisible in every log.
```

### alloc-stale-focus-demotion

**Demoting a stale focus cell** — in `shouldDemoteStaleFocus`, attached to `bool shouldDemoteStaleFocus(int skips, int k, CellStatus status);` (line 245)

```text
The staleness rule's decision (§3.4): should a focus cell that keeps
producing no admissible candidate be written off as COVERED?

A free function, and not simply an `if` in doPlan, because of what the
answer costs when it is wrong. A demotion is a FIRST-HAND COVERED, and the
merge guard exists precisely to stop a peer lowering one of those again —
so a spurious demotion writes that ground off for the whole team, for the
rest of the run, with no path back. That is a decision worth being able to
test at its boundary rather than one to read out of a 900-line function.

`skips` counts consecutive planning ticks on which this cell was the focus
and the goal came from somewhere else. `k` is the threshold. `status` must
be the status read AFTER a fresh census: the whole point of the re-measure
is that a cell which has quietly been cleared must not be demoted, and
passing the pre-census status would silently remove that protection while
leaving every caller looking correct.
```

## src/global_allocator.cpp

### alloc-mm-quantum

**Millimetre cost quantum** — in `File scope`, attached to `constexpr double kMmPerM = 1000.0;` (line 13)

```text
Millimetres. The quantum has to be coarse enough that two robots'
independently accumulated route costs land in the same bucket despite
float rounding, and fine enough that genuinely different tours do not tie:
cells are 10 m, so a millimetre is four orders of magnitude below anything
the allocator is trying to distinguish.
```

### alloc-fnv1a-local-copy

**The local FNV-1a copy** — in `Fnv1a — declarations`, attached to `struct Fnv1a {` (line 24)

```text
FNV-1a 32-bit. A second local copy of the construction in cell_world.cpp,
kept local for the reason stated there: the two hash different things, and
a shared helper invites folding them into one value, which would stop a
reader being told WHICH half disagrees.

The two copies do NOT have to stay byte-compatible with each other. Nothing
compares an alloc_hash to a sharedHash; the only comparison is between two
PROCESSES running the same binary, where both copies are identical by
construction.

NOTHING PINS A LITERAL, and this said "test_global_allocator pins alloc_hash
to a literal" until 2026-09-18. There is not one hash constant in that file
or any other. What the AllocHash group actually pins is RELATIONAL — equal
problems digest equal, an unformed problem digests 0, and each of six
channels shared_hash is blind to moves the digest — and every one of those
statements holds under ANY injective-enough hash. Swap FNV-1a for a
different multiplier here and the whole suite still passes.

That gap is smaller than it sounds and is not worth a literal. The value is
compared only within one binary, so a silent change costs nothing live; it
costs only the offline join of alloc_hash columns ACROSS binary generations,
which no analysis does (they join two robots of one campaign). And a literal
would have to be re-baselined on every legitimate change to the digest input
— plus it would ride on route costs that reach the digest through a double
and an llround, so it would be pinning the optimiser's floating-point as
well as the hash. Relational tests were the right call. Just do not read
this paragraph as saying the construction itself is guarded.
```

### alloc-candidate-scan-first

**Candidate scan before vehicle filter** — in `GlobalAllocator::solve`, attached to `std::vector<int> cand;` (line 131)

```text
Scanned BEFORE the vehicle filter, not because the solve needs it here but
because the R3 digest below has to cover the whole problem and has to be
written on every return path that formed one. The refusal ORDER is
unchanged: the `cand.empty()` and max_candidates returns stay where they
always were, after the vehicle refusal, so which reason a caller sees is
exactly what it was.
```

### alloc-problem-digest

**The allocation problem digest** — in `GlobalAllocator::solve`, attached to `{` (line 145)

```text
Assembled here, once, from the same values the solve is about to use, and
written before any refusal — a refused solve still had a problem, and
"these two robots refused for different reasons" is only interpretable if
you can first establish they were refusing the same thing.

Ordering is imposed explicitly at every level, because the input vector's
order is the caller's and must not reach a value two processes compare:
vehicles by id, candidates ascending (the scan above already produces
them that way, and it is asserted rather than assumed by construction
since `world.size()` walks ids in order).

The vehicle list hashed here is `robots_in` ENTIRE — including robots the
filter below is about to drop as finished or unlocatable. That is
deliberate: "my peer is finished" versus "my peer is still working" is a
disagreement about the problem, and it is one of the two channels §3.6
says nothing could see. Folding in only the survivors would hide exactly
the case that matters.

TWO OF Config's THREE FIELDS ARE PART OF THE PROBLEM (2026-09-18). All
three were excluded before, deliberately and with a test asserting it
(AllocHash.SolverConfigIsNotPartOfTheProblem), on the grounds that config
is a property of the SOLVER and a mismatch is a deployment fault the run
params already record. That reasoning is exactly right for one field and
wrong for the other two, so the split is now drawn where the distinction
actually falls rather than around the whole struct:

  comms_mask     IN. It restricts which cells a disconnected robot may be
                 assigned AT ALL, so it changes the feasible set — that is
                 the problem, not an approach to it. And this is not a
                 hypothetical misconfiguration: the flag exists so that the
                 gap between a masked and an unmasked solve can be measured
                 (see Config::comms_mask — "the cost gap ... IS the
                 reconnection value P4 gates on"). Those two solves differ
                 in NOTHING ELSE, so with comms_mask excluded the one
                 comparison the flag was added to support is precisely the
                 one where the digest declares both sides identical.

  max_candidates IN. It decides refused versus solved. A refused solve
                 carries its digest on purpose, so that "these two robots
                 refused for different reasons" is interpretable — but with
                 the threshold excluded, a robot that refused and a robot
                 that solved the same candidate set under a laxer cap agree
                 on the key, and the digest reports them as having faced
                 the same thing when the cap is the entire difference.

  polish_passes  OUT, and the old rationale survives intact here. 2-opt
                 passes change the tours and nothing else: same vehicles,
                 same candidates, same feasible set, same refusal. Two
                 robots differing only in polish_passes ARE solving the
                 same problem and getting different answers to it, which is
                 the one thing the digest is supposed to be able to say.
                 Folding it in would convert that finding into a silent
                 "different problem" and lose it.

Latent today either way: one launch supplies every robot in a cell, so all
three are equal across the fleet by construction. That is a property of the
CALLER, and it is not what the digest claims to depend on.

Folded in LAST, after the world's edge hash, so the contribution order of
everything already being logged is untouched. Values are not comparable
across this change, which is the standing rule anyway: never join across
generations.
```

### alloc-digest-off-frontier

**off_frontier is in the digest** — in `GlobalAllocator::solve`, attached to `f.byte(r->off_frontier ? 1u : 0u);` (line 222)

```text
IN, and not optional: it decides which vehicles the solve is given, so
two robots disagreeing about a peer's mode are solving different
problems and the digest has to say so. Leaving it out would let them
agree on the key while allocating over different vehicle sets, which
is the exact failure the split above exists to make visible.
```

### alloc-vehicle-set

**Building the vehicle set** — in `GlobalAllocator::solve`, attached to `struct Veh { int idx; int id; int cell; bool in_comms; };` (line 242)

```text
Sorted by id so the caller's vector order cannot reach the result, and
carrying the caller's index so the output stays index-aligned with the
input the caller passed. A robot that has left the frontier is removed
outright and its cells return to the pool (§3.4) -- leaving it in with an
empty tour would let the makespan balance keep reserving work for a robot
that has stopped. `off_frontier` is the superset test and `finished`
implies it, so the pair below is one condition written as two for the
reader; see AllocRobot for why they are separate fields.
```

### alloc-drop-unlocatable

**Unlocatable robots are dropped** — in `GlobalAllocator::solve`, attached to `if (!world.grid().valid(r.cell)) continue;` (line 255)

```text
An unlocatable robot is dropped rather than defaulted to some cell: every
cost involving it would be fiction, and a fiction that changes the
makespan changes the OTHER robot's tour too. Dropped, it simply gets no
focus cell and falls back to unrestricted planning, which is the
documented degradation.
```

### alloc-greedy-insertion

**Greedy makespan-balanced insertion** — in `GlobalAllocator::solve`, attached to `std::vector<uint8_t> taken(nc, 0);` (line 296)

```text
Each round: over every (unassigned cell, vehicle, insertion position),
find the one whose insertion leaves the SMALLEST resulting makespan, and
commit it. Ties break on (cell id, robot id, position) -- a total order
over values both robots agree on -- and the scan visits them in exactly
that order under a strict <, so the first minimum found is the tie-break
winner without a separate comparison.
```
