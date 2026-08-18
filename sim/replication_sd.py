#!/usr/bin/env python3
"""How much does an identical run vary when nothing is changed?

p12 and p13 share seeds and differ only in pursuit_budget_max_sec. The `off` and
`rendezvous` arms never enter PURSUE, so that parameter cannot reach them: their
same-seed cells across the two campaigns are REPEAT RUNS of identical
conditions, on a byte-identical binary (sha de822611e451dd33). The seed pins the
radio fading trace but sim sensor noise is unseeded, so the spread between such a
pair is the experiment's irreducible noise floor.

That floor is the number that decides whether any arm comparison means anything,
and it is measurable here for free -- no extra runs required.
"""
import os
import re

ROOT = os.environ.get("HMR_CAMPAIGN_ROOT", "/tmp/hmr_campaign")


def mf(cell, key):
    p = os.path.join(ROOT, cell, "run_manifest.txt")
    try:
        for ln in open(p, errors="ignore"):
            if ln.startswith(key + "="):
                return ln.strip().split("=", 1)[1]
    except OSError:
        return None


def disc(cell):
    """Disconnected fraction, recomputed here so the two campaigns are scored
    by one code path."""
    import csv
    p = os.path.join(ROOT, cell, "link_states.csv")
    T = mf(cell, "run_end_t_sim")
    if not os.path.exists(p) or not T:
        return None
    T = float(T)
    lk = []
    with open(p) as fh:
        for row in csv.DictReader(fh):
            try:
                lk.append((float(row["t_sim"]), int(float(row["connected"]))))
            except (ValueError, KeyError, TypeError):
                continue
    lk.sort()
    tot, drop, run0 = 0.0, None, None
    for t, c in lk:
        if not c:
            run0 = None
            if drop is None:
                drop = t
        else:
            if run0 is None:
                run0 = t
            if drop is not None and t - run0 >= 10.0:
                if run0 - drop >= 60.0:
                    tot += run0 - drop
                drop = None
    if drop is not None:
        tot += T - drop
    return 100.0 * tot / T


print("=== REPEAT RUNS: same arm, same seed, identical conditions ===")
print(f"{'arm':<11} {'seed':>4} {'p12 t_done':>11} {'p13 t_done':>11} {'delta_s':>8} "
      f"{'delta%':>7} | {'p12 disc%':>10} {'p13 disc%':>10} {'delta_pp':>9}")
dts, dds = [], []
for arm in ("off", "rendezvous"):
    for s in (1, 2):
        a, b = f"p12_{arm}_seed{s}", f"p13_{arm}_seed{s}"
        ta, tb = mf(a, "run_end_t_sim"), mf(b, "run_end_t_sim")
        if not (ta and tb):
            continue
        ta, tb = float(ta), float(tb)
        da, db = disc(a), disc(b)
        d = abs(tb - ta)
        dts.append(d)
        dd = abs(db - da) if (da is not None and db is not None) else None
        if dd is not None:
            dds.append(dd)
        print(f"{arm:<11} {s:>4} {ta:11.0f} {tb:11.0f} {d:8.0f} "
              f"{100.0*d/min(ta,tb):6.0f}% | {da:9.1f}% {db:9.1f}% "
              f"{(f'{dd:8.1f}' if dd is not None else '       -'):>9}")

med = lambda x: sorted(x)[len(x)//2]
print(f"\nnoise floor, completion time : median |delta| = {med(dts):.0f} s "
      f"(range {min(dts):.0f}-{max(dts):.0f})")
print(f"noise floor, disconnected %  : median |delta| = {med(dds):.1f} pp "
      f"(range {min(dds):.1f}-{max(dds):.1f})")

print("\n=== ARM SPREAD IN p13 (n=2), against that floor ===")
for arm in ("off", "rendezvous", "pursuit", "hybrid"):
    ts = [float(mf(f"p13_{arm}_seed{s}", "run_end_t_sim")) for s in (1, 2)]
    ds = [disc(f"p13_{arm}_seed{s}") for s in (1, 2)]
    print(f"{arm:<11} t_done {ts[0]:6.0f} {ts[1]:6.0f}  (within-arm spread "
          f"{abs(ts[0]-ts[1]):5.0f} s)   disc% {ds[0]:5.1f} {ds[1]:5.1f}")

print("\nSample size to detect a completion-time difference d, 80% power, "
      "two-sided 0.05:")
# D = X1 - X2 over repeat runs has variance 2*sigma^2, so median|D| = 0.954*sigma.
# (1.128 is the MEAN|D| constant and would understate the spread.) Four pairs
# make this an order-of-magnitude estimate, not a calibrated one.
sd = med(dts) / 0.954
print(f"  estimated SD ~= {sd:.0f} s")
for d in (300, 500, 800, 1200):
    print(f"  d = {d:5d} s  ->  n ~= {16.0 * (sd / d) ** 2:5.1f} runs per arm")
