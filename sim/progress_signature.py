#!/usr/bin/env python3
"""Do the long runs share a signature, or are they just the tail?

Two candidate explanations, with different consequences for the experiment:

  TAIL      one distribution, some runs unlucky. Then the endpoint is the median
            and the usual completion-time sample sizes apply. (The power tool
            this line used to name, replication.py, is gone; replication_sd.py
            is the surviving one, and its n-per-arm is an order-of-magnitude
            estimate off four pairs on a long-superseded binary. The standing
            result is that ~30 cells/arm is the FLOOR for completion time, not
            a clean 80% -- do not read it as sufficient without re-checking.)
  MODE      a distinct failure the short runs never enter. Then "fraction of runs
            that fail" is a separate, cheaper, and more operationally meaningful
            endpoint than the median -- and the failure itself may be fixable.

Discriminators, all from data already on disk:
  progress curve   time to each unknown-fraction rung. A run that is uniformly
                   slower has a stretched curve; a run that stalls has a knee.
  endgame share    fraction of the run spent after the completion threshold was
                   first crossed. High = the robots found coverage but could not
                   finish (return/sync trouble), not an exploration problem.
  travel           metres driven per unit of coverage gained. High = the robots
                   explored inefficient ground, i.e. they went somewhere worse.
  churn            peer_seen count. High = link thrashing.
"""
import glob
import json
import os

ROOT = os.environ.get("HMR_CAMPAIGN_ROOT", "/tmp/hmr_campaign")
DONE_UNKNOWN = 0.55       # the run's completion criterion


def cell_features(d):
    mf = {}
    try:
        for ln in open(os.path.join(d, "run_manifest.txt"), errors="ignore"):
            if "=" in ln:
                k, _, v = ln.strip().partition("=")
                mf[k] = v
    except OSError:
        return None
    if not mf.get("run_end_t_sim"):
        return None
    T = float(mf["run_end_t_sim"])

    rungs, dist, steps, churn, last_unk = {}, 0.0, 0, 0, 1.0
    t_cross = None
    for rob in ("atlas", "bestla"):
        p = os.path.join(d, f"{rob}.events.jsonl")
        if not os.path.exists(p):
            continue
        for ln in open(p, errors="ignore"):
            try:
                e = json.loads(ln)
            except Exception:
                continue
            ev = e.get("event")
            if ev == "coverage_milestone":
                th = float(e.get("threshold", 0))
                t = float(e.get("t_sim_sec", 0))
                # earliest robot to reach each rung
                if th not in rungs or t < rungs[th]:
                    rungs[th] = t
            elif ev == "step":
                steps += 1
                dist = max(dist, float(e.get("distance_m") or 0))
                u = e.get("unknown_fraction")
                if u is not None:
                    last_unk = min(last_unk, float(u))
                    if float(u) <= DONE_UNKNOWN and t_cross is None:
                        t_cross = float(e.get("t_sim_sec") or 0)
            elif ev == "peer_seen":
                churn += 1

    return {
        "cell": os.path.basename(d),
        "arm": mf.get("reconnect_mode_requested", "?"),
        "T": T,
        "end": mf.get("run_end_reason", "?"),
        "dist": dist,
        "steps": steps,
        "churn": churn,
        "unk": last_unk,
        "t_cross": t_cross,
        "endgame": (T - t_cross) / T if t_cross else None,
        "rungs": rungs,
    }


cells = []
for tag in ("p12", "p13", "p13smoke"):
    for d in sorted(glob.glob(os.path.join(ROOT, tag + "_*"))):
        if os.path.isdir(d) and not d.endswith(".attempts"):
            f = cell_features(d)
            if f:
                cells.append(f)
cells.sort(key=lambda c: c["T"])

print(f"{'cell':<24} {'arm':<11} {'t_done':>7} {'dist_m':>7} {'m/100s':>7} "
      f"{'steps':>6} {'churn':>6} {'unk_end':>8} {'t_.55':>7} {'endgame':>8}")
for c in cells:
    eg = f"{100*c['endgame']:7.0f}%" if c["endgame"] is not None else f"{'-':>8}"
    tc = f"{c['t_cross']:7.0f}" if c["t_cross"] else f"{'-':>7}"
    print(f"{c['cell']:<24} {c['arm']:<11} {c['T']:7.0f} {c['dist']:7.0f} "
          f"{100*c['dist']/c['T']:7.1f} {c['steps']:6d} {c['churn']:6d} "
          f"{c['unk']:8.3f} {tc} {eg}")

LONG = 2000.0
lo = [c for c in cells if c["T"] < LONG]
hi = [c for c in cells if c["T"] >= LONG]
print(f"\n=== SHORT (<{LONG:.0f}s, n={len(lo)}) vs LONG (>={LONG:.0f}s, n={len(hi)}) ===")
med = lambda xs: sorted(xs)[len(xs) // 2] if xs else float("nan")
for name, fn in (("t_done", lambda c: c["T"]),
                 ("distance_m", lambda c: c["dist"]),
                 ("metres per 100 s", lambda c: 100 * c["dist"] / c["T"]),
                 ("steps", lambda c: c["steps"]),
                 ("link churn", lambda c: c["churn"]),
                 ("final unknown frac", lambda c: c["unk"]),
                 ("endgame share", lambda c: c["endgame"] if c["endgame"] is not None else float("nan"))):
    print(f"  {name:<20} short {med([fn(c) for c in lo]):9.2f}   "
          f"long {med([fn(c) for c in hi]):9.2f}")

print("\n=== PROGRESS CURVES: sim seconds to reach each unknown-fraction rung ===")
ths = sorted({t for c in cells for t in c["rungs"]}, reverse=True)
print(f"{'cell':<24} " + " ".join(f"{t:>6.2f}" for t in ths))
for c in cells:
    row = " ".join((f"{c['rungs'][t]:6.0f}" if t in c["rungs"] else "     -") for t in ths)
    print(f"{c['cell']:<24} {row}")
