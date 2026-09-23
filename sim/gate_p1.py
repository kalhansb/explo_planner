#!/usr/bin/env python3
# Moved comments: docs/sim_notes/gate_p1_notes.md
"""P1 smoke gate: does the coarse cell census agree with the planner's own
coverage measure, and does it converge?

docs/mtare_evolution_plan.md words P1's gate as

    smoke run shows census converging to COVERED as coverage saturates,
    agreeing with `coverageUnknownFraction` to within a cell-quantisation
    bound.

"Agreeing to within a cell-quantisation bound" is not a tolerance somebody
picks. There is an EXACT inequality between the two numbers, derived below, and
this file checks that inequality rather than a fudge factor. Both numbers ride
on the same `cell_census` row precisely so this check needs no join: the sim is
nondeterministic run-to-run (the same seed has produced 1803 s and 856 s), so
matching a `cell_census` to a `coverage_milestone` on a timestamp would compare
two different ticks and report the difference as a disagreement.

THE BOUND
---------
Write

    thr   = cell_exploring_min_unknown       (the hysteresis RELEASE threshold)
    n_cov = the row's `covered` count        (FIRST-HAND covered only)
    N     = the row's `cells_total`
    f     = n_cov / N
    R     = grid_area / roi_area             (>= 1, see below)

A cell counted COVERED on this row is COVERED as of THIS tick's measurement,
and CellWorld::classify only lets a COVERED cell survive a non-COVERED
observation while its own unknown fraction stays <= thr. Note this is the
RELEASE threshold, not `cell_covered_max_unknown`: a cell is promoted at 0.15
but held down to 0.35, so 0.35 is the number a currently-COVERED cell is
guaranteed to satisfy. Using the promotion threshold here would be a bound the
code never promised and would fail spuriously the moment hysteresis did its job.

Every other cell satisfies the trivial `unknown <= 1`. Columns partition across
cells, so summing over cells and dividing by the ROI's column count:

    roi_unknown_fraction  <=  R * ( thr * f  +  1 * (1 - f) )

R appears because the cell grid is rounded UP to whole cells and can therefore
overhang the ROI, so the sum over cells covers at least the ROI's columns and
possibly more. R >= 1 always, and R == 1 exactly when the ROI divides evenly by
the cell size (the sim's +/-50 m ROI at 10 m cells does). It only ever LOOSENS
the bound, so a gate that ignored it would be stricter than the code promises.

`covered_by_others` is deliberately NOT in f. That status is relayed belief
about ground this robot may never have seen, so its cells say nothing about
this robot's own map, which is what roi_unknown_fraction measures. In P1 there
is no exchange and the count is always 0 — asserted below, so that when P2
turns exchange on, this file does not quietly start counting hearsay as
evidence.

The inequality is ONE-SIDED, and that is the useful side: it bites when the
census claims MORE coverage than the map supports. That is the failure mode
worth gating — a census that over-reports is what would later make the
allocator write off ground nobody has been to.

WHAT ELSE IS CHECKED
--------------------
An agreement check alone passes trivially on a run that never explored
anything (f = 0, bound = R >= 1, nothing to violate). So this file also
requires the run to have actually saturated, and refuses rather than passes
when it did not — a gate that cannot fail on the run it was handed has stopped
checking, and stating that loudly is the whole point of the NOT-APPLICABLE
verdict here.

The other checks are cheap identities that catch wiring errors an eyeballed
plot never would:

  * the five statuses sum to cells_total, and covered_fraction is
    (covered + covered_by_others) / cells_total;
  * the grid geometry is constant within a robot and IDENTICAL across robots.
    Two robots whose grid_hash differs do not mean the same ground by the same
    cell id; every cross-robot mechanism from P2 on is built on that identity,
    and this is the first run in which it is checkable;
  * sum of `changed` over the rows == the final `commits_total`. Exact in P1:
    applyObservation counts exactly the cells commitSelf accepted, and
    commitSelf is the only path that bumps update_id. It doubles as a
    lost-event check, since a dropped census row breaks the identity.

Usage:
    ./gate_p1.py RUN_DIR [RUN_DIR ...]
Exit status 0 = PASS, 1 = FAIL (including NOT-APPLICABLE).
"""

import json
import os
import sys

import event_log


class GateError(Exception):
    pass


# Slack as a fraction of ROI columns: censusFromMap and
# MapCache::unknownColumnFraction differ by at most one column-row per ROI edge,
# 4e-3 at the 100 m ROI and 0.1 m voxels. Fixed, so interior divergence is not
# absorbed. (notes: gate-p1-edge-slack)
EDGE_SLACK = 4.0e-3

# Below this many census rows a robot has not been sampled enough for the
# convergence statistic to mean anything, and a gate reading two rows would
# report a perfect rank correlation on noise.
MIN_ROWS = 10

# Required drop in roi_unknown_fraction between the first and last census for
# the run to count as saturated. Not a coverage target: a threshold on the final
# value would only encode the ROI's size. (notes: gate-p1-min-progress)
MIN_PROGRESS = 0.05

# Rank correlation floor for "the census tracks the coverage measure".
MAX_RHO = -0.7


def spearman(xs, ys):
    """Spearman rank correlation, ties averaged.

    Hand-rolled because this repo has no scipy dependency, and hand-rolled
    statistics in this repo have a history (a hand-rolled sampler once gave a
    biased permutation p). So: ties are averaged rather than broken, which is
    the part that matters here — covered_fraction is quantised to 1/N and is
    ALL ties in the early part of a run, and rank-breaking ties arbitrarily
    would manufacture a correlation out of the order the rows happen to sit in.
    """
    n = len(xs)
    if n < 3:
        return None

    def ranks(v):
        order = sorted(range(n), key=lambda i: v[i])
        r = [0.0] * n
        i = 0
        while i < n:
            j = i
            while j + 1 < n and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1.0
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r

    rx, ry = ranks(xs), ranks(ys)
    mx, my = sum(rx) / n, sum(ry) / n
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    dx = sum((a - mx) ** 2 for a in rx)
    dy = sum((b - my) ** 2 for b in ry)
    if dx <= 0.0 or dy <= 0.0:
        return None  # one side is constant: no correlation is defined
    return num / (dx * dy) ** 0.5


def check_robot(name, path, fails, notes):
    evs = event_log.read_events(path)
    start = next((e for e in evs if e["event"] == "run_start"), None)
    if start is None:
        raise GateError(f"{path}: no run_start")
    params = start.get("params", {})

    enabled = params.get("cell_world_enable")
    if not enabled:
        # Not a skip. This gate was pointed at this run to check the cell
        # world, and a run without it cannot answer.
        fails.append(f"{name}: cell_world_enable is {enabled!r} — this run "
                     f"carries no cell world, so nothing here was tested")
        return None

    rows = [e for e in evs if e["event"] == "cell_census"]
    if len(rows) < MIN_ROWS:
        fails.append(f"{name}: {len(rows)} cell_census rows, need >= {MIN_ROWS}")
        return None

    thr = params.get("cell_exploring_min_unknown")
    if thr is None:
        fails.append(f"{name}: run_start has no cell_exploring_min_unknown, so "
                     f"the agreement bound cannot be computed from the run's "
                     f"own configuration and this gate has nothing to test "
                     f"against")
        return None

    # R = grid area / ROI area, from the run's own geometry.
    cs = rows[0]["cell_size_m"]
    nx, ny = rows[0]["nx"], rows[0]["ny"]
    roi_w = params["roi_max_x"] - params["roi_min_x"]
    roi_h = params["roi_max_y"] - params["roi_min_y"]
    if roi_w <= 0 or roi_h <= 0:
        raise GateError(f"{name}: degenerate ROI {roi_w} x {roi_h}")
    R = (nx * cs * ny * cs) / (roi_w * roi_h)
    if R < 1.0 - 1e-9:
        fails.append(f"{name}: cell grid ({nx * cs:g} x {ny * cs:g} m) is "
                     f"SMALLER than the ROI ({roi_w:g} x {roi_h:g} m) — the "
                     f"census is measuring less ground than the coverage "
                     f"measure it is being compared against")
        return None

    # --- per-row identities and the agreement bound ---------------------
    worst = None
    changed_sum = 0
    for e in rows:
        if (e["nx"], e["ny"], e["cell_size_m"], e["grid_hash"]) != \
                (nx, ny, cs, rows[0]["grid_hash"]):
            fails.append(f"{name}: grid geometry changed mid-run at "
                         f"t_sim={e['t_sim_sec']:.1f}")
            return None

        counts = (e["unseen"] + e["exploring"] + e["covered"] +
                  e["exploring_by_others"] + e["covered_by_others"])
        if counts != e["cells_total"]:
            fails.append(f"{name}: statuses sum to {counts}, cells_total is "
                         f"{e['cells_total']} at t_sim={e['t_sim_sec']:.1f} — "
                         f"the census is not a partition of the grid")
            return None
        if e["cells_total"] != nx * ny:
            fails.append(f"{name}: cells_total {e['cells_total']} != nx*ny "
                         f"{nx * ny}")
            return None

        want = (e["covered"] + e["covered_by_others"]) / e["cells_total"]
        if abs(e["covered_fraction"] - want) > 1e-6:
            fails.append(f"{name}: covered_fraction {e['covered_fraction']:.6f} "
                         f"!= (covered+covered_by_others)/cells_total "
                         f"{want:.6f} at t_sim={e['t_sim_sec']:.1f}")
            return None

        # P1 has no exchange. If this ever trips it means either a merge path
        # landed early or the status vocabulary drifted — and either way the
        # bound below would start crediting hearsay as measurement.
        if e["covered_by_others"] or e["exploring_by_others"]:
            fails.append(f"{name}: second-hand statuses present in a P1 run "
                         f"({e['exploring_by_others']} exploring_by_others, "
                         f"{e['covered_by_others']} covered_by_others) at "
                         f"t_sim={e['t_sim_sec']:.1f} — there is no exchange "
                         f"in this phase for them to have come from")
            return None

        changed_sum += e["changed"]

        # The order statistics have to BE order statistics. A distribution
        # wired to the wrong cell set still writes all five fields, and the
        # reachability diagnosis below would then be built on them.
        if e["cells_measured"] > e["cells_total"]:
            fails.append(f"{name}: cells_measured {e['cells_measured']} > "
                         f"cells_total {e['cells_total']} at "
                         f"t_sim={e['t_sim_sec']:.1f}")
            return None
        if e["cells_measured"] > 0:
            q = (e["cell_unknown_min"], e["cell_unknown_p10"],
                 e["cell_unknown_median"])
            if not all(0.0 <= x <= 1.0 for x in q) or \
                    not (q[0] <= q[1] <= q[2] + 1e-12):
                fails.append(f"{name}: unknown-fraction quantiles are not "
                             f"ordered in [0,1] (min {q[0]:.4f}, p10 "
                             f"{q[1]:.4f}, median {q[2]:.4f}) at "
                             f"t_sim={e['t_sim_sec']:.1f}")
                return None
            ff = (e["cell_frontier_frac_min"], e["cell_frontier_frac_median"],
                  e["cell_frontier_frac_at_best_unknown"])
            if not all(0.0 <= x <= 1.0 for x in ff) or ff[0] > ff[1] + 1e-12:
                fails.append(f"{name}: frontier-fraction quantiles are not "
                             f"ordered in [0,1] (min {ff[0]:.4f}, median "
                             f"{ff[1]:.4f}) at t_sim={e['t_sim_sec']:.1f}")
                return None
            # The joint reading is one of the cells in the marginal, so it
            # cannot sit below the marginal's minimum; this catches the field
            # being wired to a cell outside the set.
            # (notes: gate-p1-joint-reading-in-marginal)
            if ff[2] < ff[0] - 1e-12:
                fails.append(f"{name}: cell_frontier_frac_at_best_unknown "
                             f"{ff[2]:.4f} is below the frontier-fraction "
                             f"minimum {ff[0]:.4f} at "
                             f"t_sim={e['t_sim_sec']:.1f} — it is not one of "
                             f"the cells it claims to be drawn from")
                return None
            if e["cells_frontier_ok"] > e["cells_measured"]:
                fails.append(f"{name}: cells_frontier_ok "
                             f"{e['cells_frontier_ok']} > cells_measured "
                             f"{e['cells_measured']} at "
                             f"t_sim={e['t_sim_sec']:.1f}")
                return None

        u = e["roi_unknown_fraction"]
        if u < 0.0:
            continue  # -1 = the planner could not measure; nothing to compare
        f = e["covered"] / e["cells_total"]
        bound = R * (thr * f + (1.0 - f)) + EDGE_SLACK
        slack = bound - u
        if worst is None or slack < worst[0]:
            worst = (slack, e, f, bound)

    if worst is None:
        fails.append(f"{name}: no census row carried a measurable "
                     f"roi_unknown_fraction — the two measures were never "
                     f"compared")
        return None

    slack, e, f, bound = worst
    if slack < 0.0:
        fails.append(
            f"{name}: AGREEMENT VIOLATED at t_sim={e['t_sim_sec']:.1f}: "
            f"census says {e['covered']}/{e['cells_total']} cells covered "
            f"(f={f:.3f}), which caps roi_unknown_fraction at {bound:.6f}, "
            f"but the planner measured {e['roi_unknown_fraction']:.6f}")
    else:
        notes.append(f"{name}: tightest agreement margin {slack:.6f} at "
                     f"t_sim={e['t_sim_sec']:.1f} "
                     f"(f={f:.3f}, cap {bound:.4f}, "
                     f"measured {e['roi_unknown_fraction']:.4f})")

    # --- the commit identity --------------------------------------------
    final_commits = rows[-1]["commits_total"]
    if changed_sum != final_commits:
        fails.append(
            f"{name}: sum(changed) over {len(rows)} rows is {changed_sum} but "
            f"the final commits_total is {final_commits}. In P1 every counted "
            f"change is a commitSelf and nothing else bumps update_id, so "
            f"these must be equal — a gap means a census row was lost or a "
            f"second writer is touching update_id")

    # --- convergence, non-vacuously --------------------------------------
    us = [e["roi_unknown_fraction"] for e in rows
          if e["roi_unknown_fraction"] >= 0.0]
    fs = [e["covered_fraction"] for e in rows
          if e["roi_unknown_fraction"] >= 0.0]
    progress = us[0] - us[-1]
    if progress < MIN_PROGRESS:
        fails.append(
            f"NOT-APPLICABLE {name}: roi_unknown_fraction moved only "
            f"{progress:.4f} ({us[0]:.4f} -> {us[-1]:.4f}) over the run, below "
            f"the {MIN_PROGRESS} this gate needs. Coverage never saturated, so "
            f"the convergence claim was not tested — this is a refusal to "
            f"report a pass, not a pass")
        return None

    if max(e["cells_measured"] for e in rows) == 0:
        fails.append(f"{name}: no cell ever cleared min_observed_columns, so "
                     f"the census had no evidence about any cell all run")
        return None

    if rows[-1]["covered"] == 0:
        # A census that never promotes is TWO different failures wearing the
        # same row, and saying only "did not converge" hands the reader the
        # wrong one half the time. Attribute it.
        thr_cov = params.get("cell_covered_max_unknown")
        best = min(e["cell_unknown_min"] for e in rows
                   if e["cells_measured"] > 0)
        frontier_ever_ok = max(e["cells_frontier_ok"] for e in rows) > 0
        # The best candidate's OWN frontier fraction, at the row where it was
        # the best candidate — so the two numbers in the message describe one
        # cell and a reader can set both thresholds from them at once.
        best_row = min((e for e in rows if e["cells_measured"] > 0),
                       key=lambda e: e["cell_unknown_min"])
        why = (f"(roi_unknown_fraction fell {us[0]:.4f} -> {us[-1]:.4f}; the "
               f"best-observed cell bottomed out at {best:.4f} unknown with "
               f"frontier fraction "
               f"{best_row['cell_frontier_frac_at_best_unknown']:.4f})")
        if thr_cov is not None and best > thr_cov:
            fails.append(
                f"{name}: no cell ever reached COVERED {why}. The best cell on "
                f"the map never came within reach of cell_covered_max_unknown "
                f"= {thr_cov}, so no setting of the frontier veto could have "
                f"promoted anything: the THRESHOLD is unreachable in this "
                f"world, not the census broken. This is a world-calibrated "
                f"knob like done_unknown_fraction — re-scale it for THIS "
                f"world in the launch config (CELL_COVERED_U / "
                f"CELL_EXPLORING_U in sim/run_explo_sim_rviz.sh), not in the "
                f"library default. Fit to the spread across several runs, not "
                f"to one run's plateau, and score it on a run other than the "
                f"ones used to read it off")
        elif not frontier_ever_ok:
            fails.append(
                f"{name}: no cell ever reached COVERED {why}. Cells did clear "
                f"cell_covered_max_unknown = {thr_cov}, but no cell ever "
                f"cleared cell_covered_max_frontier_frac "
                f"= {params.get('cell_covered_max_frontier_frac')} — the "
                f"frontier "
                f"veto is the binding constraint")
        else:
            fails.append(
                f"{name}: no cell ever reached COVERED {why}, yet cells "
                f"cleared BOTH promotion thresholds at some point in the run. "
                f"Both vetoes were satisfiable and nothing was promoted, so "
                f"this is the census itself, not its tuning")
    if fs[-1] <= fs[0]:
        fails.append(f"{name}: covered_fraction did not rise over the run "
                     f"({fs[0]:.4f} -> {fs[-1]:.4f})")

    rho = spearman(us, fs)
    if rho is None:
        fails.append(f"{name}: rank correlation undefined (one of the two "
                     f"measures never moved)")
    elif rho > MAX_RHO:
        fails.append(f"{name}: covered_fraction tracks roi_unknown_fraction "
                     f"only weakly (Spearman rho = {rho:+.3f}, need <= "
                     f"{MAX_RHO}) — the census is not following coverage")
    else:
        notes.append(f"{name}: rho = {rho:+.3f} over {len(us)} rows, "
                     f"roi_unknown {us[0]:.4f} -> {us[-1]:.4f}, "
                     f"covered_fraction {fs[0]:.4f} -> {fs[-1]:.4f}, "
                     f"commits {final_commits} over {nx * ny} cells")
    return rows[0]["grid_hash"]


def check_run(run_dir, fails, notes):
    logs = event_log.robot_logs(run_dir)
    if not logs:
        raise GateError(f"{run_dir}: no *.events.jsonl")
    hashes = {}
    for name, path in sorted(logs.items()):
        h = check_robot(f"{os.path.basename(run_dir)}/{name}", path,
                        fails, notes)
        if h is not None:
            hashes[name] = h
    # The identity every cross-robot mechanism from P2 on is built on.
    if len(set(hashes.values())) > 1:
        fails.append(f"{run_dir}: robots disagree about the cell grid: "
                     + ", ".join(f"{k}=0x{v:08x}" for k, v in
                                 sorted(hashes.items()))
                     + " — the same cell id names different ground on "
                       "different robots")
    elif hashes:
        notes.append(f"{os.path.basename(run_dir)}: grid_hash "
                     f"0x{next(iter(hashes.values())):08x} shared by "
                     f"{len(hashes)} robot(s)")


def main(argv):
    if len(argv) < 2:
        print(__doc__.strip().splitlines()[-3], file=sys.stderr)
        print("usage: gate_p1.py RUN_DIR [RUN_DIR ...]", file=sys.stderr)
        return 2
    fails, notes = [], []
    for d in argv[1:]:
        try:
            check_run(d, fails, notes)
        except (GateError, event_log.EventLogError, KeyError) as exc:
            fails.append(f"{d}: {exc}")
    for n in notes:
        print(f"  {n}")
    if fails:
        print()
        for f in fails:
            print(f"FAIL  {f}")
        print(f"\nP1 GATE: FAIL ({len(fails)} finding(s))")
        return 1
    print("\nP1 GATE: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
