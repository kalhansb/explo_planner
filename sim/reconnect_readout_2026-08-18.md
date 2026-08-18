# Reconnection experiment — readout at the 2-seed checkpoint

Campaign stopped by request after 8 of 40 cells. Everything below comes from
p12 (13 cells, budget 600), p13smoke (2 cells), and p13 (8 cells, budget 2400).

## 1. The chase fix works

A distance veto in `startPursuit` rejected every chase. The gate compares
`trail_m * nav_safety_factor / nav_speed_est` against `pursuit_budget_max_sec`,
which at 600 s admits chases only inside **30 m**, while the robots routinely
separate to 60–85 m. Raising the budget to 2400 moves that radius to **120 m**.

| | dispatches admitted as `chase` |
|---|---|
| p12, budget 600 | **0 / 22** |
| p13smoke, budget 2400 | 6 / 6 |
| p13, budget 2400 | **9 / 9** |

The chase also releases: 66–109 s against a 2400 s cap, one episode per robot,
no re-dispatch. All 8 p13 cells reached `all_done`; all 8 gate verdicts CLEAN.

Before the fix, `hybrid` silently degenerated into `rendezvous` and `pursuit`
into `off`, so two of four arms were not being tested at all.

## 2. The result that decides everything else: the noise floor

p12 and p13 share seeds and differ only in `pursuit_budget_max_sec`. The `off`
and `rendezvous` arms never enter PURSUE, so that parameter cannot reach them —
their same-seed cells across the two campaigns are **repeat runs of identical
conditions on a byte-identical binary** (sha `de822611e451dd33`).

| arm | seed | p12 | p13 | difference |
|---|---|---|---|---|
| off | 1 | 1576 | 1092 | 484 s (44%) |
| off | 2 | 1442 | 1431 | 11 s (1%) |
| rendezvous | 1 | 1277 | 2493 | **1216 s (95%)** |
| rendezvous | 2 | 2300 | 1405 | 895 s (64%) |

Nothing was changed between these pairs. Estimated SD ≈ **900 s** (4 pairs —
order-of-magnitude only). The seed pins the radio fading trace and nothing else;
the world and both spawn poses are fixed in every run. The only unseeded noise
source in the simulator is the lidar's 1 cm gaussian, far too small to account
for a 1216 s divergence on its own — section 6a gives the mechanism that does.

Note the heterogeneity: one pair replicated to within 11 s, another diverged by
1216 s. That is the signature of a system with decision bifurcations — most runs
track each other, some flip a choice early and cascade.

## 3. Consequence: the observed arm differences are not distinguishable from noise

p13 completion times, n=2 per arm:

| arm | seed 1 | seed 2 | within-arm spread |
|---|---|---|---|
| pursuit | 1189 | 1105 | 84 s |
| off | 1092 | 1431 | 339 s |
| hybrid | 1047 | 1846 | 799 s |
| rendezvous | 2493 | 1405 | 1088 s |

The **within-arm** spreads (84–1088 s) are the same size as the **between-arm**
differences. No ordering here is supportable.

Power, at SD ≈ 900 s:

| difference to detect | runs needed per arm |
|---|---|
| 300 s | ~157 |
| 500 s | ~56 |
| 800 s | ~22 |
| 1200 s | ~10 |

**The planned 40-cell campaign (n=10 per arm) could only have detected a
~1200 s effect** — larger than any difference seen. Completing it would have
cost ~16 h and returned "no significant difference" almost regardless of the
truth. Stopping at the checkpoint was the right call.

## 4. The benefit metric as originally specified does not work

"Shared map voxels at reconnect" cannot separate the arms, because the `off`
arm — which never deliberately reconnects — has the **highest** median gain
(1,437,430 in p12). Robots drift back into range by accident in every arm and
the maps merge whenever they do. The metric measures *the merge*, which is
common to all arms, not *the reconnection behaviour*, which is the treatment.

Two further problems:
- **Contamination.** A robot keeps mapping alone during the settle window at
  200–3900 voxels/s, so 120 s of solo driving accounts for 24k–470k voxels —
  the same order as the "gain".
- **My baseline correction is unreliable.** Subtracting a 60 s slope projected
  over 120 s overshoots during fast-mapping bursts and produced a −498,221
  outlier. `vox_net` should not be quoted until rebuilt.

## 5. What replaces it: time out of contact

Disconnected fraction is scale-free, so runs of different length compare
directly — which matters because run length is itself an outcome.

| arm | p12 median disc% | p13 median disc% |
|---|---|---|
| off | 72.3% | 69.0% |
| pursuit | 40.0% | 55.0% |
| hybrid | 41.1% | 54.1% |
| rendezvous | 31.3% | 43.0% |

`off` is worst in **both** campaigns and `rendezvous` best in both. That is the
only ordering that replicates across two independent campaigns. But the repeat
runs show disc% is itself noisy (median |difference| 24.8 pp, up to 44.1 pp), so
treat this as a direction, not a measurement.

Caveat on the per-cell table: it also splits outages by whether a dispatch
occurred, and dispatched outages look longer. **That comparison is confounded** —
a dispatch only fires after 240 s of silence, so any outage with a dispatch is
≥240 s by construction and short outages can never have one.

## 6. A mechanism worth testing directly

`p13_rendezvous_seed1` had the **best** connectivity of all 8 cells (20.7%
disconnected) and the **worst** completion time (2493 s). Robots that keep going
to meet each other stay in touch but stop exploring.

If that holds, the honest headline is not "reconnecting helps" but
**"reconnecting trades exploration speed for connectivity, and the strategies
differ in how badly"** — which is the cost/benefit ledger this experiment exists
to produce. It is a within-run mechanism, so it is testable at far smaller n
than the between-arm completion-time comparison.

## 6a. Where the variance actually comes from: the stopping rule

Runs terminate at `unknown_fraction <= 0.55` — with **55% of the ROI still
unknown**, because trunk shadowing makes most of this world permanently
unobservable. `t_0.55` and `run_end_t_sim` are 30–380 s apart in 22 of 23 cells,
so crossing the criterion *is* what ends a run.

Per-run worst single band of unknown-fraction progress:

| | median worst stall | share of run | where |
|---|---|---|---|
| short, t < 2000 s (n=18) | 341 s | 24% | mostly mid-run, 0.70→0.65 |
| long, t >= 2000 s (n=5) | **1404 s** | **56%** | **all in the last three bands** |

A long run is not uniformly slower — it is an ordinary run plus one long stall
immediately before the criterion. `p13_rendezvous_seed1` led its twin at every
checkpoint down to 0.70, then spent 1404 s on a single band.

Marginal cost across all 23 runs:

| stage | metres per 0.01 | seconds per 0.01 |
|---|---|---|
| early, 0.85→0.75 | 5 | 14 |
| middle, 0.75→0.65 | 10 | 35 |
| last band, 0.56→0.55 | 20 | 61 |

No run ever reached below **0.5053**. `p12_pursuit_seed1` drove 1545 m — over 3x
the typical 460 m — and reached only 0.5167. The criterion therefore sits ~0.045
above the best value ever observed, on a curve costing 4x what the early bands
cost.

**Consequence:** completion time is partly measuring how long a run takes to
scrape past a threshold set near the coverage floor, which turns on whether a
reachable unknown pocket happens to be close by. That is a far better
explanation of the ~900 s spread than anything else examined.

Stated honestly: the 4x cost rise and the 0.5053 best-ever are measured; a hard
asymptote at ~0.50 is *inferred*, not proven. It could be a steep slope. The
distinction decides how far the criterion should move.

The scenario file already prescribes the test:

> flatforest's 0.4922 floor and 0.55 criterion were calibrated against 74
> stems/ha and do not transfer: three times the trunks means more
> permanently-shadowed voxels and a higher floor. Before running anything that
> TERMINATES on coverage in this world, re-measure the unknown-fraction floor.
> "No criterion window exists any more" is a real possible outcome here.

We are at 250 stems/ha on a threshold calibrated for 74.

## 7. Scope limits

All 40 cells were to run in **one** configuration:
- world `flatforest_dense` (250 stems/ha), fixed
- spawn poses fixed — atlas (0,0) yaw 0, bestla (0,3) yaw π, 3 m apart
- `tx_power_dbm` 30.0, fixed

The seed varies **only the radio fading realisation**. So this tests one forest,
one starting geometry, one radio power, across draws of radio luck.

## 8. Recommendations for the next campaign

Completion time stays the primary endpoint — it is the operationally meaningful
one. The work below is what it takes to make it measurable.

1. **Re-measure the unknown-fraction floor for `flatforest_dense` first**, per
   the scenario file's own warning, and move `DONE_UNKNOWN` off the floor. This
   is the highest-value item: section 6a shows the criterion, not the strategy,
   is generating most of the spread. A handful of long single-robot runs settles
   it, and it fixes the endpoint for every campaign afterwards.
2. **Then re-estimate the noise floor** on the corrected criterion — the SD ≈
   900 s figure rests on 4 pairs and should shrink once the stopping rule is off
   the steep part of the curve. Repeat runs are cheap and size everything else.
3. Carry disconnected fraction and the section 6 within-run mechanism as
   secondary endpoints — both are testable at far smaller n and neither depends
   on the criterion's placement.
4. Rebuild the `vox_net` baseline before quoting it.
5. Additional environments (sparse forest, varied spawn separation) — deferred
   by the user to a later campaign.

## Data

- `/tmp/hmr_campaign/p13_{off,rendezvous,pursuit,hybrid}_seed{1,2}` — 8 cells
- `/tmp/hmr_campaign/p12_*` — 13 cells at budget 600
- `/tmp/hmr_campaign/p13smoke_*` — 2 validation cells
- Analysis: `reconnect_value.py` (cost/benefit/timing), `replication.py`
  (noise floor and power)

Resuming the remaining 32 cells needs no special handling: relaunch
`p13_full.sh` and the resume skips every cell that already carries a
`run_end_reason`.
