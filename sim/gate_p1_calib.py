#!/usr/bin/env python3
"""Calibrates gate_p1.py: proves each of its guards can actually fail.

Six guards in this repo went inert while still printing PASSes, one of them a
pre-registration. A gate that has never been shown to fail is not evidence, and
"it passed on the smoke run" is exactly the observation an inert gate produces.
So before gate_p1.py's verdict on a real run counts for anything, every check
in it gets a known-answer injection: a defect built to trip that specific
check, with the expected message named up front.

Matching the MESSAGE, not just the exit status, is the point. An injection that
fails the gate for some unrelated reason has not calibrated the guard it was
aimed at — it has only shown the gate can fail somehow, which is the weaker
claim that let the inert guards survive.

The baseline is synthetic by default: a hand-built census stream that satisfies
every identity gate_p1 checks. That is deliberate. A real run's PASS proves the
PLANNER agrees with the gate; a synthetic baseline proves the GATE agrees with
its own stated bound, and it is available before any long run finishes. Pass
--from RUN_DIR to re-run the same injections on top of a real run's params and
rows instead; the two together are the useful pair.

Note the synthetic rows deliberately do NOT couple `changed` to the covered
count the way the C++ does. Coupling them would mean an injection that edits
coverage also edits the commit identity, and every injection would trip two
guards at once — which is precisely the ambiguity this file exists to remove.

Usage:
    ./gate_p1_calib.py [--from RUN_DIR]
Exit status 0 = every guard CALIBRATED, 1 = at least one did not.
"""

import copy
import io
import json
import os
import shutil
import sys
import tempfile
from contextlib import redirect_stdout

import event_log
import gate_p1

NX = NY = 10
N = NX * NY
CELL_SIZE = 10.0
THR = 0.35
GRID_HASH = 0xE26AC4A4
ROWS = 24


def synth_params(seed_hash=GRID_HASH):
    return {
        "cell_world_enable": True,
        "cell_exploring_min_unknown": THR,
        "cell_covered_max_unknown": 0.15,
        "cell_size_m": CELL_SIZE,
        "cell_nx": float(NX),
        "cell_ny": float(NY),
        "cell_grid_hash": float(seed_hash),
        "roi_min_x": -50.0,
        "roi_max_x": 50.0,
        "roi_min_y": -50.0,
        "roi_max_y": 50.0,
    }


def synth_rows(u0=1.0, grid_hash=GRID_HASH):
    """A stream that satisfies every identity gate_p1 asserts.

    covered climbs 0 -> 46 while roi_unknown_fraction falls; the fall is chosen
    to stay under the gate's own cap R*(thr*f + 1 - f) at every row, with real
    margin rather than by a hair, so an injection that fails the agreement
    check has failed it on its own merits and not on rounding.
    """
    rows, commits = [], 0
    for i in range(ROWS):
        covered = 2 * i
        f = covered / N
        exploring = min(30, N - covered)
        changed = 60 if i == 0 else 5
        commits += changed
        rows.append({
            "event": "cell_census",
            "t_sim_sec": 30.0 + 5.0 * i,
            "cells_total": N,
            "unseen": N - covered - exploring,
            "exploring": exploring,
            "covered": covered,
            "exploring_by_others": 0,
            "covered_by_others": 0,
            "covered_fraction": covered / N,
            "roi_unknown_fraction": u0 - 0.7 * f,
            "coverage_source": "scovox",
            "changed": changed,
            "commits_total": commits,
            "cell_size_m": CELL_SIZE,
            "nx": NX,
            "ny": NY,
            "grid_hash": grid_hash,
            "edges_enabled": 116,
            "edges_total": 342,
            # A distribution that comfortably straddles covered_max_unknown, so
            # the reachability diagnosis has a clean baseline to depart from:
            # the best cell is promotable and the median is not.
            "cells_measured": max(1, covered + 10),
            "cells_frontier_ok": max(1, covered + 5),
            "cell_unknown_min": 0.05,
            "cell_unknown_p10": 0.11,
            "cell_unknown_median": 0.48,
            "cell_frontier_frac_min": 0.01,
            "cell_frontier_frac_median": 0.06,
            "cell_frontier_frac_at_best_unknown": 0.02,
        })
    return rows


def synth_run():
    return {"atlas": (synth_params(), synth_rows(1.0)),
            "bestla": (synth_params(), synth_rows(0.98))}


def load_run(run_dir):
    """A real run, reduced to the (params, cell_census rows) the gate reads.

    Everything else in the stream is dropped, so a re-serialised run is not a
    faithful copy of the original — it is a faithful copy of the SLICE gate_p1
    looks at, which is what the injections need to operate on.
    """
    out = {}
    for name, path in sorted(event_log.robot_logs(run_dir).items()):
        evs = event_log.read_events(path)
        start = next(e for e in evs if e["event"] == "run_start")
        rows = [e for e in evs if e["event"] == "cell_census"]
        out[name] = (start.get("params", {}), rows)
    return out


def write_run(run, dest):
    os.makedirs(dest, exist_ok=True)
    for name, (params, rows) in run.items():
        with open(os.path.join(dest, f"{name}.events.jsonl"), "w") as fh:
            fh.write(json.dumps({"event": "run_start", "seq": 0,
                                 "robot": name, "t_sim_sec": 0.0,
                                 "params": params}) + "\n")
            for i, r in enumerate(rows):
                row = dict(r)
                row["seq"] = i + 1
                row["robot"] = name
                fh.write(json.dumps(row) + "\n")
    return dest


def run_gate(run, tmp, tag):
    dest = write_run(run, os.path.join(tmp, tag))
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = gate_p1.main(["gate_p1.py", dest])
    shutil.rmtree(dest, ignore_errors=True)
    return rc, buf.getvalue()


# --- the injections -----------------------------------------------------
#
# Each takes the whole run (robot -> (params, rows)) and breaks exactly one
# thing. The paired string is a fragment of the message the aimed-at guard is
# supposed to emit.

def inj_disabled(run):
    run["atlas"][0]["cell_world_enable"] = False


def inj_too_few_rows(run):
    p, rows = run["atlas"]
    run["atlas"] = (p, rows[:gate_p1.MIN_ROWS - 1])


def inj_no_threshold(run):
    del run["atlas"][0]["cell_exploring_min_unknown"]


def inj_grid_smaller_than_roi(run):
    # Grid unchanged; the ROI grows past it. The census now measures less
    # ground than the number it is being compared against.
    run["atlas"][0].update(roi_min_x=-100.0, roi_max_x=100.0,
                           roi_min_y=-100.0, roi_max_y=100.0)


def inj_geometry_drift(run):
    run["atlas"][1][10]["nx"] = NX + 1


def inj_not_a_partition(run):
    run["atlas"][1][10]["unseen"] += 1


def inj_cells_total_mismatch(run):
    # Partition kept intact, so this can only trip the nx*ny check.
    r = run["atlas"][1][10]
    r["cells_total"] += 1
    r["unseen"] += 1
    r["covered_fraction"] = ((r["covered"] + r["covered_by_others"])
                             / r["cells_total"])


def inj_covered_fraction_wrong(run):
    run["atlas"][1][10]["covered_fraction"] += 0.01


def inj_hearsay(run):
    r = run["atlas"][1][10]
    r["unseen"] -= 3
    r["covered_by_others"] = 3
    r["covered_fraction"] = ((r["covered"] + 3) / r["cells_total"])


def inj_agreement_violated(run):
    # Claim nearly the whole grid is covered while the map still reports most
    # of the ROI unknown. This is the failure the bound exists to catch.
    r = run["atlas"][1][10]
    r["covered"] = 95
    r["exploring"] = 5
    r["unseen"] = 0
    r["covered_fraction"] = 0.95
    r["roi_unknown_fraction"] = 0.90


def inj_no_measurable_coverage(run):
    for r in run["atlas"][1]:
        r["roi_unknown_fraction"] = -1.0


def inj_lost_census_row(run):
    p, rows = run["atlas"]
    run["atlas"] = (p, rows[:10] + rows[11:])


def inj_no_progress(run):
    # Coverage never saturates. Held low enough that the agreement bound is
    # comfortably satisfied, so the only thing wrong is the lack of progress.
    for r in run["atlas"][1]:
        r["roi_unknown_fraction"] = 0.30


def inj_never_covered(run):
    for r in run["atlas"][1]:
        r["exploring"] += r["covered"]
        r["covered"] = 0
        r["covered_fraction"] = 0.0


def inj_weak_correlation(run):
    # Endpoints kept (so the rise check still passes) and the middle reversed,
    # which leaves covered_fraction climbing overall while tracking
    # roi_unknown_fraction the WRONG way for most of the run.
    p, rows = run["atlas"]
    mid = [r["covered"] for r in rows[1:-1]][::-1]
    for r, c in zip(rows[1:-1], mid):
        r["covered"] = c
        r["exploring"] = min(30, r["cells_total"] - c)
        r["unseen"] = r["cells_total"] - c - r["exploring"]
        r["covered_fraction"] = c / r["cells_total"]


def inj_quantiles_unordered(run):
    run["atlas"][1][10]["cell_unknown_p10"] = 0.9  # above the median


def inj_frontier_quantiles_unordered(run):
    run["atlas"][1][10]["cell_frontier_frac_min"] = 0.9


def inj_joint_not_in_marginal(run):
    # The joint reading claims a candidate cleaner than any cell on the map.
    run["atlas"][1][10]["cell_frontier_frac_at_best_unknown"] = 0.0001


def inj_measured_exceeds_grid(run):
    run["atlas"][1][10]["cells_measured"] = N + 1


def inj_frontier_ok_exceeds_measured(run):
    r = run["atlas"][1][10]
    r["cells_frontier_ok"] = r["cells_measured"] + 1


def inj_nothing_measured(run):
    for r in run["atlas"][1]:
        r["cells_measured"] = 0
        r["cells_frontier_ok"] = 0


def _strip_covered(rows):
    for r in rows:
        r["exploring"] += r["covered"]
        r["covered"] = 0
        r["covered_fraction"] = 0.0


def inj_threshold_unreachable(run):
    # Nothing promoted, and the best cell on the map never came near the
    # promotion threshold. The gate must blame the threshold, not the census.
    p, rows = run["atlas"]
    _strip_covered(rows)
    for r in rows:
        r["cell_unknown_min"] = 0.55
        r["cell_unknown_p10"] = 0.60
        r["cell_unknown_median"] = 0.70


def inj_frontier_veto_binding(run):
    # Cells were well within the unknown threshold; the frontier count is what
    # held them back. A gate that blamed the threshold here would send somebody
    # to tune the wrong number.
    p, rows = run["atlas"]
    _strip_covered(rows)
    for r in rows:
        r["cells_frontier_ok"] = 0


def inj_census_itself_broken(run):
    # Both vetoes satisfiable all run, still nothing promoted. This is the one
    # case that is genuinely the census's fault, and it is the case a
    # tuning-flavoured message would bury.
    _strip_covered(run["atlas"][1])


def inj_hash_mismatch(run):
    # One robot's grid names different ground by the same cell id.
    p, rows = run["bestla"]
    p["cell_grid_hash"] = float(GRID_HASH ^ 0xFF)
    for r in rows:
        r["grid_hash"] = GRID_HASH ^ 0xFF


INJECTIONS = [
    ("cell world switched off", inj_disabled, "carries no cell world"),
    ("census barely sampled", inj_too_few_rows, "need >="),
    ("threshold missing from run_start", inj_no_threshold,
     "no cell_exploring_min_unknown"),
    ("grid smaller than the ROI", inj_grid_smaller_than_roi,
     "SMALLER than the ROI"),
    ("grid geometry drifts mid-run", inj_geometry_drift,
     "grid geometry changed mid-run"),
    ("histogram is not a partition", inj_not_a_partition,
     "not a partition"),
    ("cells_total disagrees with nx*ny", inj_cells_total_mismatch,
     "!= nx*ny"),
    ("covered_fraction miscomputed", inj_covered_fraction_wrong,
     "covered_fraction"),
    ("second-hand status in a P1 run", inj_hearsay,
     "second-hand statuses"),
    ("census over-reports coverage", inj_agreement_violated,
     "AGREEMENT VIOLATED"),
    ("coverage never measurable", inj_no_measurable_coverage,
     "no census row carried a measurable"),
    ("a census row was lost", inj_lost_census_row, "sum(changed)"),
    ("run never saturated", inj_no_progress, "NOT-APPLICABLE"),
    ("no cell ever covered", inj_never_covered,
     "no cell ever reached COVERED"),
    ("quantiles out of order", inj_quantiles_unordered,
     "not ordered in [0,1]"),
    ("frontier quantiles out of order", inj_frontier_quantiles_unordered,
     "frontier-fraction quantiles are not ordered"),
    ("joint reading not a real cell", inj_joint_not_in_marginal,
     "not one of the cells it claims"),
    ("more cells measured than exist", inj_measured_exceeds_grid,
     "cells_measured"),
    ("frontier_ok exceeds measured", inj_frontier_ok_exceeds_measured,
     "cells_frontier_ok"),
    ("nothing ever measured", inj_nothing_measured,
     "no evidence about any cell"),
    ("promotion threshold unreachable", inj_threshold_unreachable,
     "THRESHOLD is unreachable"),
    ("frontier veto is what binds", inj_frontier_veto_binding,
     "frontier veto is the binding constraint"),
    ("census broken, not mistuned", inj_census_itself_broken,
     "this is the census itself"),
    ("census does not track coverage", inj_weak_correlation,
     "Spearman rho"),
    ("robots disagree about the grid", inj_hash_mismatch,
     "disagree about the cell grid"),
]


def main(argv):
    src = None
    if len(argv) > 2 and argv[1] == "--from":
        src = argv[2]
    elif len(argv) > 1:
        print("usage: gate_p1_calib.py [--from RUN_DIR]", file=sys.stderr)
        return 2

    base = load_run(src) if src else synth_run()
    origin = src if src else "synthetic baseline"
    print(f"Calibrating gate_p1.py against {origin} "
          f"({len(base)} robot(s), "
          f"{min(len(r) for _, r in base.values())}+ census rows)\n")

    tmp = tempfile.mkdtemp(prefix="gate_p1_calib_")
    bad = []
    try:
        rc, out = run_gate(copy.deepcopy(base), tmp, "baseline")
        if rc != 0:
            print(out)
            print("BASELINE FAILS — every injection below would be "
                  "meaningless, since the gate rejects the clean run too.")
            return 1
        print("  baseline                              PASS (as required)")

        for name, inject, expect in INJECTIONS:
            run = copy.deepcopy(base)
            inject(run)
            rc, out = run_gate(run, tmp, "inj")
            if rc == 0:
                bad.append((name, "gate still PASSED", out))
                verdict = "NOT CALIBRATED — gate passed"
            elif expect not in out:
                bad.append((name, f"failed, but not on {expect!r}", out))
                verdict = "NOT CALIBRATED — wrong guard"
            else:
                verdict = "CALIBRATED"
            print(f"  {name:<38}{verdict}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if bad:
        print()
        for name, why, out in bad:
            print(f"--- {name}: {why}")
            print("    " + "\n    ".join(out.strip().splitlines()))
        print(f"\nCALIBRATION: FAIL ({len(bad)} guard(s) unproven)")
        return 1
    print(f"\nCALIBRATION: PASS ({len(INJECTIONS)} guards, each shown to fail "
          f"on its own defect)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
