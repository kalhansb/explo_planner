# metrics_logger.hpp — design notes and history

The long comments of `include/explo_planner/metrics_logger.hpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [StepMetrics — declarations](#stepmetrics--declarations) — 7
- [MetricsLogger — declarations](#metricslogger--declarations) — 1

## StepMetrics — declarations

### metrics-info-gain-std

**Why info_gain_std is recorded** — attached to `float  info_gain_std         = 0.0f;` (line 37)

```text
Population std of info_gain ACROSS the candidate set on this step — how
much the viewpoints actually disagree about information, which the mean
cannot show. U = info/(eps+cost) degenerates to argmin-cost as this goes
to zero: with every candidate scoring alike, the numerator stops
discriminating and only distance decides. It is therefore the quantity
that says whether a change to the FOV model helped or flattened the
field. NOTE: this is NOT adjacent to mean_info_gain in the CSV — see the
append-only note in metrics_logger.cpp.
```

### metrics-selected-utility-alias

**selected_utility is an alias** — attached to `float  selected_utility      = 0.0f;` (line 49)

```text
EXACT ALIAS of selected_score — both are assigned current_goal_.score, and
all 3432 rows of the g6pilot campaign were identical. The planner has one
scalar objective; there is no second "utility" quantity for this to hold.
It is NOT removed because the CSV is append-only and read positionally by
archive scripts (see metrics_logger.cpp), so deleting a mid-file column
would silently shift every column after it. Treat as redundant: never
decompose score against utility, and never fit one on the other — that is
a regression of a column on itself.

The identity is checked at CAMPAIGN-SCORING time, not build time: the
per-generation gate script (gate_g8.py check 15) re-derives it over every
CSV row before any result is read. Those scripts are deliberately not in
this tree — they pin one generation's identity and are written fresh per
campaign — so nothing here enforces the alias, and a divergence would
surface as a gate failure on the data rather than as a build failure.
```

### metrics-reconnect-timer-rows

**Timer rows and reconnect columns** — attached to `std::string state                    = "UNKNOWN";` (line 78)

```text
Reconnection diagnostics. Rows are emitted on a periodic timer in EVERY
state, not just at LOG_STEP, so a minutes-long reconnect manoeuvre samples
the coverage curve instead of leaving a gap in it.

ALL DURATIONS HERE ARE SIM SECONDS under use_sim_time (the node reads
this->now()/get_clock()). At RTF != 1 they are not wall seconds, and the
sampling period is likewise a sim-time period.

`state` is the planner state at emit time, from stateName(). It is also the
row discriminator: a row with state == "LOG_STEP" is an end-of-step row
(plan-attribution columns populated); every other value is a timer row
(plan-attribution columns are zero by construction — there was no plan on
that tick to attribute them to). The timer deliberately skips LOG_STEP ticks
so the two kinds never collide.

The reconnect_* pair is -1 whenever no manoeuvre is in flight, which keeps 0
meaning "arrived / just started" rather than "not applicable".
reconnect_elapsed_sec is raw elapsed sim time since the manoeuvre was armed
and is NOT bounded by pursuit_budget_max_sec: proximity-hold time is refunded to
the pursuit budget clock but not to this one, and in HYBRID the meeting-point
leg runs on the same clock after the chase is spent.

reconnect_range_to_goal_m is REMAINING straight-line XY range to the
manoeuvre goal — a decreasing sample, not a travelled distance. Named for
what it is: as `reconnect_distance_m` it read like the manoeuvre's path
cost, which inverts the comparison it would be used for (a robot that
drives far and arrives ends near 0; one that gives up early stays large).
Travelled distance is `distance_traveled` differenced across the rows where
reconnect_elapsed_sec >= 0.
```

### metrics-unknown-fraction

**The coverage-termination column** — attached to `float  unknown_fraction         = -1.0f;` (line 111)

```text
Coverage-termination measure, as evaluated this tick, plus which source
produced it ("scovox" 2.5D column coverage of the ROI footprint, or
"planning_map" 2D cell coverage). This IS the primary endpoint quantity and
the one the DONE criterion is compared against, and until now it existed
nowhere in the CSV — it reached stdout only inside the branch where it had
already dropped below threshold, so a run could not be scored on the
criterion it terminates on. total_observed_voxels is not a substitute: it
is a 3D active-cell count whose relationship to column coverage varies with
occlusion and the z-band.

-1 when it cannot be measured (source "planning_map" with no planning_map
received, or a degenerate ROI), matching coverageUnknownFraction().
```

### metrics-separation-columns

**Team-separation diagnostic columns** — attached to `float  sep_peer_dist_m          = -1.0f;` (line 126)

```text
Team-separation diagnostics (separation.hpp). Exploration rows only; an
exploitation row carries the "not applicable" defaults below, because the
vantage chooser does not consult the term. So does a row emitted by the
periodic metrics timer rather than by a planning step — that path builds a
fresh StepMetrics and never sees these fields at all. Averaging any of
these columns over a whole CSV therefore mixes in rows the term never
touched and dilutes the treatment toward "no effect": filter to
state == "LOG_STEP" and phase == "explore" first.

The first two are measured in EVERY arm, including arms with the term
switched off, and that is the point of them. The mechanism this term
replaces, MinPos, has exactly one observable (rejected_by_minpos) and it
reads 0 for a whole campaign — which cannot distinguish a veto that never
mattered from a veto that never ran. These columns are built so the same
ambiguity cannot happen twice: an off-arm row still records how close to
its teammate the planner sent this robot, so the treated arm has a
measured counterfactual rather than an assumed one.

  sep_peer_dist_m     metres from the SELECTED goal to the nearest
                      eligible teammate position; -1 for "no eligible
                      teammate" (none known, none fresh, or all finished).
                      Not the robot-to-robot separation — that is derivable
                      from the pose stream; this is what the planner chose.
  sep_discount        the multiplier actually applied to the selected
                      candidate's utility. 1.0 means unchanged, which is
                      what an off arm always writes. NOTE that this is
                      also the factor by which selected_score and
                      selected_utility above have ALREADY been reduced on
                      this row: the term scales the candidate's score in
                      place, so in a treated arm those two columns carry
                      the discounted number and in a control arm they
                      carry the raw one. To compare them across arms,
                      divide by this column first.
  sep_reordered       did the term change where the robot was sent?
                      1 yes, 0 no, -1 the question was not asked. It is a
                      comparison of PICKS, not of ordering: the planner is
                      re-run over the same candidates with the discount
                      removed and the two selections compared. -1 covers
                      the term being off, no candidate being selected, and
                      a pick that came from the blacklist-amnesty fallback,
                      which orders by failure age and which the term cannot
                      reach. -1 must not be pooled with 0: one means the
                      term did not move the goal, the other means nobody
                      asked.
  sep_eligible_peers  how many teammates were eligible to repel this tick.
                      Zero here makes every other column vacuous, so it is
                      the first thing to read. Structurally zero for a whole
                      run when TEAM_WORLD=0, since the term's anchors come
                      from the team model — such a cell yields no
                      counterfactual, only the confirmation that it cannot.
```

### metrics-plan-rejection-columns

**Plan rejection columns on every row** — attached to `int    plan_cand_total          = -1;` (line 182)

```text
These are populated on a timer row as well as a LOG_STEP row, and they have
to be, because of what they are for.

THEY ARE NOT THE ONLY ONES, which is what this said until 2026-09-18. The
authority is ExploPlannerNode::fillCommonMetrics — read it, do not count
from here — and everything it touches is on every row: step, sim_time_sec,
distance_traveled, coord_active_peers, prox_hold_count, prox_hold_total_sec,
phase and state unconditionally, plus unknown_fraction and coverage_source,
plus reconnect_elapsed_sec / reconnect_range_to_goal_m / team_complete /
pursue_peer / pursue_quarry_live under their own state guards. What is
special about the plan_rej_* family is not that it is alone in surviving a
timer row, it is that it was MOVED there deliberately (see below) after a
stall proved unreadable; the rest were always common.

A robot whose every candidate is rejected never completes a step, so
it emits no LOG_STEP row at all — the run keeps producing timer rows with
state == "PLAN" and every plan-attribution column zero by construction. In
at1 seed 2 that is 60 rows across a 1094 s stall saying only "still in
PLAN", while the planner's own log had the answer (map=82 unreach=94) the
whole time. Reconstructing it meant reading 2.6 MB of unstructured text per
robot-run. So these carry forward from the most recent doPlan attempt and
are readable on every row.

They do NOT replace rejected_by_minpos / rejected_by_unreachable above.
Those keep their existing LOG_STEP-only semantics exactly, so no archived
CSV changes meaning and no analysis written against them changes answer.
The duplication is deliberate: a profile split across two column families
with two different row semantics is a trap, so this set is complete on its
own and self-consistent (plan_rej_* sums to plan_cand_total on any row
where plan_stall_ticks > 0, since that is what "all rejected" means).

-1 means NO PLANNING ATTEMPT HAS HAPPENED YET, which is not the same as a
measured zero. That distinction is the whole lesson of rejected_by_minpos
reading 0 for a campaign: nothing in the data could separate "the veto
never mattered" from "the veto never ran". A 0 in these columns is always
a measurement.

Staleness caveat: doPlan has earlier returns (no map yet, no candidates
generated). Those leave the previous attempt's values standing, because a
tick that never reached the admissibility filter has no profile of its own
to report. plan_stall_ticks is the discriminator for freshness — it is
written on every tick that DID reach the filter.

  plan_cand_total    candidates the admissibility filter was given
  plan_rej_close     rejected for being too close to the robot
  plan_rej_map       rejected by the single-cell free check on the
                     planning map. Seed 2's second-largest cause and the
                     one with no column at all before this.
  plan_rej_unreach   rejected by the cost-grid flood
  plan_rej_blacklist rejected by visited_goals_ or failed_goals_ — the
                     UNION, unchanged since it was added, so it stays
                     comparable across schema versions
  plan_rej_visited   the visited_goals_ half of that union, broken out in
                     v8. The two halves get different treatment at the
                     starvation amnesty (failed first, visited only as a
                     last resort) and they mean different things — a
                     visited rejection is this robot's own fresh trail, a
                     failed one is a goal nav could not reach. The union
                     alone could not say which one starved a tick.
                     plan_rej_blacklist - plan_rej_visited recovers the
                     failed-only count. -1 on a pre-v8 file, which is not
                     zero.
  plan_rej_minpos    rejected by a peer's MinPos claim
  plan_stall_ticks   consecutive ticks on which EVERY candidate was
                     rejected, 0 on a tick that selected a goal. This is
                     the starvation length, and it is the column that
                     turns "the robot sat in PLAN" into "the robot could
                     not plan for N ticks and here is why".

THIS LIST IS STRUCT ORDER, AND STRUCT ORDER IS NOT CSV ORDER. plan_rej_visited
is declared here where it belongs by meaning, next to the union it splits,
but the CSV header appends it LAST of all columns — writeHeader in
metrics_logger.cpp is the only authority on position. Every other field in
this block happens to agree; this one does not, and a positional reader
built by counting down this list would read plan_rej_minpos as visited and
shift everything after it. Read the header line. It is written on every
file for exactly this reason.
```

### metrics-pursuit-columns

**Pursuit and team-completeness columns** — attached to `std::string pursue_peer        = "";` (line 270)

```text
§3.4 is a DISAGREEMENT BETWEEN TWO PREDICATES, and neither of them was
observable. The pursuit release fires on
    teamComplete(active, expected) || quarry_heard
while the outcome classifier writes "reconnected" only on
    teamComplete(live, expected)
so a chase released because its own quarry came back is labelled
`abandoned` whenever some OTHER peer is still missing. At N=4 pursuit that
produced 155 abandoned / 18 gave_up / 0 reconnected across 173 chases: the
arm's headline success metric reads 0% by construction, and nothing in any
artifact could show that the chases had in fact succeeded.

These three columns make the two predicates separately readable on every
row, so the next campaign can re-derive the outcome from data instead of
trusting the label:
  pursue_peer         the quarry's robot id, "" when not pursuing
  pursue_quarry_live  1 / 0 quarry heard this tick; -1 = not pursuing
  team_complete       1 / 0 teamComplete(live, expected); -1 = the
                      question was not asked (no coordination table, or
                      rendezvous_expected_peers is the 0 "inert" default).
                      NOT a measured 0: teamComplete() itself answers
                      false for an unconfigured team, so writing its
                      verdict unconditionally would turn "nobody asked"
                      into "the team was incomplete" and average as one.
"Pursuing" here means the CHASE is live, which is not the same as the
planner state being PURSUE: a proximity hold taken mid-chase parks the
robot in PROXIMITY_HOLD without ending the chase (the node refunds the held
time to the pursuit budget for exactly that reason). Both of the first two
columns are written on those rows too. They were not, once, and the rows
lost were the closing seconds of chases — concentrated where the §3.4 case
lives, and asymmetric by robot id because right-of-way is lexicographic.
A row with pursue_quarry_live=1 and team_complete=0 at the end of a chase
IS the §3.4 case, and is now countable.

NOTE ON `target_id`: the deferred item that prompted this asked to "write
target_id". It is deliberately NOT reused. target_id means "active exploit
target" and reads -1 for every archived row because exploitation is off;
putting a peer identity there would make one column mean two incompatible
things depending on a different column's value, and every script that
already reads target_id would silently start answering a question it was
not asked. Same rule as LegacyRejectionColumnsAreUnchanged: old columns
keep their old meaning, new meanings get new names.

-1 is "not pursuing", not a measured false — the same sentinel discipline
as the plan_* block, and for the same reason.
```

## MetricsLogger — declarations

### metrics-sticky-write-failure

**Detecting lost CSV writes** — attached to `bool ok() const { return error_.empty(); }` (line 327)

```text
Has every write so far actually reached the file?

The constructor refuses a CSV it cannot OPEN, which covers the whole
misconfiguration family (missing directory, read-only mount, bad
permissions). It cannot cover the failures that happen after the open
succeeds, and those are the ones a campaign actually hits: a disk that
fills mid-run leaves a truncated planner_atlas.csv, and an output path
pointed at a pseudo-file that accepts an open and rejects every write
(MetricsLogger("/dev/full") is the reproducer) logs ~3000 steps that all
vanish. std::ofstream reports both by latching a bit and then silently
discarding every subsequent `<<`, so without this the run completes, the
node logs "Step N logged" for every step, and nothing anywhere says the
data is gone.

This class has no logger handle and deliberately does not acquire one --
it is the pure CSV writer and keeping it ROS-free is what lets the schema
tests run without a graph -- and it does NOT throw from logStep(), because
the per-step path runs inside a planner tick where an exception would take
the run down harder than the lost metrics do. So the failure is recorded
here instead, STICKILY (first failure only; a per-step report would emit
one line per tick for the rest of the run), and THE CALLER IS EXPECTED TO
SURFACE IT: poll ok() alongside the step log and push error() out through
the node's own logger, or a truncated CSV is still silent.
```
