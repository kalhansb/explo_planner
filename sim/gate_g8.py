#!/usr/bin/env python3
"""Read a generation-8 campaign against the pre-registered gate.

Descends from gate_g6.py. Checks 1-14 are carried over unchanged in meaning;
15-20 are new and exist to prove that each generation-8 fix ACTUALLY PRODUCED
its field in real data, rather than being present in source and inert at
runtime ([[checks-that-stopped-checking]]).

Two properties every check here must have, and which are asserted by
gate_g8_calib.py:

  1. It prints the quantity it looked at, not just a verdict.
  2. An EMPTY POPULATION reports UNRESOLVED, never PASS. "No home_watchdog
     rows were wrong" and "no home_watchdog rows exist" are different
     statements and the second one is not a pass. Six guards went inert while
     still printing passes the last time this distinction was skipped.
"""
import collections
import csv
import json
import os
import re
import sys

# Overridable so gate_g8_calib.py can point the REAL logic at a synthetic
# campaign of known-wrong cells. The identity in EXPECT is deliberately NOT
# overridable this way — see the note there.
ROOT = os.environ.get("GATE_ROOT", "/home/kalhan/hmr_campaign")
TAG = sys.argv[1] if len(sys.argv) > 1 else "g8r1"
ROBOTS = ["atlas", "bestla"]

# ---------------------------------------------------------------------------
# Declared identity of generation 8 (pre-registration section 32.14).
#
# These are DECLARED CONSTANTS, deliberately not read from the working tree:
# the gate's job is to prove the cells came from the binary the pre-registration
# names, and a gate that reads the identity from whatever happens to be built
# would pass for any binary at all.
# ---------------------------------------------------------------------------
EXPECT = {
    "git_explo_planner": "FILL_ME",
    "git_simple_nav_3d": "c9f83a7",
    "git_scovox": "078d3f7",
    "sha256_explo_planner_node": "FILL_ME",
}

DONE_UNKNOWN_FRACTION = 0.640

LATCH_RE = re.compile(
    r"Exploration complete \[latch\]: ROI unknown fraction ([0-9.]+) <= ([0-9.]+)")

# Check 17. The generation-8 line states the outcome; the event carries the same
# expression's answer. They are computed ONCE and shared, so any disagreement
# here means the sharing was broken, not that the classifier is imperfect.
ENDED_RE = re.compile(
    r"Reconnect manoeuvre ended after ([\d.]+) s sim:\s+"
    r"(reconnected|gave_up|abandoned) \(-> (\w+)\)")
# The generation-7 wording, kept so its ABSENCE can be asserted. A cell carrying
# this line is a cell built from the wrong binary, and check 3 should already
# have caught that; this is the independent second witness.
ENDED_RE_G7 = re.compile(r"Reconnect manoeuvre ended after [\d.]+ s sim \(->")

# Check 20. Every mid-run reconnect line must be one the parser knows. The
# non-dispatch wordings are listed so a genuine dispatch line that stops
# matching is not hidden among them.
MIDRUN_MARKER = "Reconnect (mid-run):"
MIDRUN_KNOWN = (
    "team incomplete",            # dispatch (generation 8 wording)
    "peer silent",                # dispatch (generation 7 wording)
    "attempt budget exhausted",
    "gave up after",
    "standing down",
)

# Check 18. Which way the inequality that fired points, keyed on the test name
# rather than the reason, because the name is what the row carries.
FAILGOAL_DIR = {
    "nav_elapsed_sec":    ">",   # elapsed exceeded the budget
    "rotate_elapsed_sec": ">",   # rotation exceeded its timeout
    "window_progress_m":  "<",   # movement fell short of the minimum
}


def read_lines(p):
    if not os.path.exists(p):
        return []
    return list(open(p, errors="replace"))


def latch_fraction(d, robot):
    for ln in read_lines(os.path.join(d, f"planner_{robot}.log")):
        m = LATCH_RE.search(ln)
        if m:
            return float(m.group(1))
    return None


def manifest(d):
    m = {}
    for ln in read_lines(os.path.join(d, "run_manifest.txt")):
        if "=" in ln:
            k, _, v = ln.strip().partition("=")
            m[k] = v
    return m


def events(d, robot):
    out = []
    for ln in read_lines(os.path.join(d, f"{robot}.events.jsonl")):
        ln = ln.strip()
        if not ln:
            continue
        try:
            out.append(json.loads(ln))
        except Exception:
            pass
    return out


def count_tok(path, tok):
    if not os.path.exists(path):
        return None
    return sum(1 for ln in open(path, errors="replace") if tok in ln)


def recovery_pairing(path):
    if not os.path.exists(path):
        return (None, None, False, "nav log missing")
    seq = []
    for ln in open(path, errors="replace"):
        if "-> recovery:" in ln:
            seq.append("E")
        elif "recovery EXIT:" in ln:
            seq.append("X")
    ent, ext = seq.count("E"), seq.count("X")
    if ent == ext:
        return (ent, ext, True, "paired")
    if ent == ext + 1 and seq and seq[-1] == "E":
        return (ent, ext, True, "one unpaired entry, and it is LAST -> teardown tolerance")
    return (ent, ext, False, "unpaired entry that is not the final line")


if "FILL_ME" in EXPECT.values():
    print("REFUSING TO RUN: the generation-8 identity is not filled in.\n"
          "  A gate that does not know which binary it is gating cannot fail\n"
          "  check 3, and would pass a campaign built from anything.")
    sys.exit(2)

cells = sorted(
    d for d in os.listdir(ROOT)
    if d.startswith(TAG + "_") and os.path.isdir(os.path.join(ROOT, d))
)
if not cells:
    print(f"no cells for tag {TAG}")
    sys.exit(1)

hard_fail, soft, unresolved = [], [], []
agg_recovery_entries = 0
rows = []
nav_fail_by_arm = collections.Counter()
cells_by_arm = collections.Counter()

# Populations for the new checks. Counted so an empty one can be reported as
# UNRESOLVED instead of passing by vacuity.
n_csv_rows = n_hw_fire = n_hw_escape = n_reconnect_end = 0
n_navfail_rows = n_peer_rows = n_runend_rows = n_midrun = 0

for c in cells:
    d = os.path.join(ROOT, c)
    m = manifest(d)
    arm = "hybrid" if "_hybrid_" in c else "off"
    cells_by_arm[arm] += 1
    row = {"cell": c, "arm": arm}

    # 1. run ended by the exploration criterion, not a cap
    reason = m.get("run_end_reason", "MISSING")
    row["end"] = reason
    row["t_sim"] = m.get("run_end_t_sim", "?")
    if reason != "all_done":
        hard_fail.append(f"{c}: run_end_reason={reason} (not all_done)")

    # 2. harness comms gates
    verdict = m.get("run_gates_verdict", "MISSING")
    row["verdict"] = verdict
    if verdict != "CLEAN":
        hard_fail.append(f"{c}: run_gates_verdict={verdict}")

    # 3. provenance: manifest must match the declared generation
    for k, v in EXPECT.items():
        if m.get(k) != v:
            hard_fail.append(f"{c}: {k}={m.get(k)} expected {v}")

    lat, arrived, unknowns, navfails, poseloss = 0, 0, [], 0, 0
    for r in ROBOTS:
        ev = events(d, r)
        if not ev:
            hard_fail.append(f"{c}/{r}: no events")
            continue
        plog = read_lines(os.path.join(d, f"planner_{r}.log"))

        # 3b/3c. baked rev and arm identity
        start = [e for e in ev if e.get("event") == "run_start"]
        if start:
            rev = str(start[0].get("git_rev", "")).replace("-dirty", "")
            if not m.get("git_explo_planner", "").startswith(rev[:7]):
                hard_fail.append(
                    f"{c}/{r}: JSONL git_rev={start[0].get('git_rev')} != "
                    f"manifest {m.get('git_explo_planner')}")
            if "-dirty" in str(start[0].get("git_rev", "")):
                hard_fail.append(f"{c}/{r}: JSONL git_rev is -dirty")
            pr = start[0].get("params", {})
            row.setdefault("mode_req", pr.get("reconnect_mode_requested", pr.get("arm")))
            row.setdefault("rdv", pr.get("rendezvous_enabled"))
            # 18b. the five nav timeout params must be echoed, or the
            # thresholds on nav_goal_failed rows cannot be audited against the
            # configuration that produced them.
            for p in ("goal_rotate_timeout_sec", "nav_speed_estimate_mps",
                      "nav_safety_factor", "nav_min_timeout_sec",
                      "nav_max_timeout_sec"):
                if p not in pr:
                    hard_fail.append(f"{c}/{r}: check 18b — run_start params missing {p}")
            # 18c. done_action must be "idle". The node's own default is
            # "shutdown", so a shared_params.yaml that failed to install leaves
            # every other provenance field in the manifest matching while the
            # planner exits at DONE — which means no homing leg and therefore
            # no mission_end, i.e. the primary endpoint is silently absent for
            # the whole campaign.
            #
            # Read from the run_start params, NOT from the manifest's
            # done_action_in_params: the manifest reports what the YAML on disk
            # says, and this reports what the node actually loaded. They differ
            # in exactly the case worth catching (the node fell back to its own
            # default), which is the case where the manifest looks fine.
            if pr.get("done_action") != "idle":
                hard_fail.append(
                    f"{c}/{r}: check 18c — run_start params done_action="
                    f"{pr.get('done_action')!r}, expected 'idle'; the planner "
                    f"exits at DONE under 'shutdown' and never homes")

        # 4 / 14. exploration ended on the coverage latch, and the recorded
        # fraction is the deciding one
        ec = [e for e in ev if e.get("event") == "exploration_complete"]
        if not ec:
            hard_fail.append(f"{c}/{r}: no exploration_complete")
        else:
            if ec[0].get("reason") != "coverage-latched":
                hard_fail.append(f"{c}/{r}: exploration_complete reason={ec[0].get('reason')}")
            else:
                lat += 1
            u = ec[0].get("unknown_fraction")
            if u is not None:
                unknowns.append(round(float(u), 3))
            lf = latch_fraction(d, r)
            if u is None:
                hard_fail.append(f"{c}/{r}: check 14 — event has no unknown_fraction")
            elif float(u) > DONE_UNKNOWN_FRACTION:
                hard_fail.append(
                    f"{c}/{r}: check 14 — event unknown_fraction={float(u):.6f} "
                    f"> criterion {DONE_UNKNOWN_FRACTION}")
            elif lf is None:
                hard_fail.append(
                    f"{c}/{r}: check 14 — no 'Exploration complete [latch]' line")
            elif abs(float(u) - lf) > 5.0e-4 + 1e-6:
                hard_fail.append(
                    f"{c}/{r}: check 14 — event {float(u):.6f} disagrees with "
                    f"latch line {lf:.3f}")

        # 5. censoring
        mc = [e for e in ev if e.get("event") == "mission_complete"]
        if not mc:
            hard_fail.append(f"{c}/{r}: no mission_complete")
        elif mc[0].get("result") != "arrived":
            hard_fail.append(f"{c}/{r}: mission_complete result={mc[0].get('result')} (CENSORED)")
        else:
            arrived += 1

        # 6 / 18. nav_goal_failed carries the fired inequality, and it HOLDS
        nf = [e for e in ev if e.get("event") == "nav_goal_failed"]
        navfails += len(nf)
        nav_fail_by_arm[arm] += len(nf)
        n_navfail_rows += len(nf)
        for e in nf:
            if "budget_sec" not in e or "pose_stale" not in e:
                hard_fail.append(f"{c}/{r}: nav_goal_failed missing budget_sec/pose_stale")
                break
        for e in nf:
            tn, tv, tt = e.get("test_name"), e.get("test_value"), e.get("test_threshold")
            if tn is None or tv is None or tt is None:
                hard_fail.append(
                    f"{c}/{r}: check 18 — nav_goal_failed[{e.get('reason')}] "
                    f"missing test_name/test_value/test_threshold")
                continue
            d_ = FAILGOAL_DIR.get(tn)
            if d_ is None:
                hard_fail.append(f"{c}/{r}: check 18 — unknown test_name {tn!r}")
            elif d_ == ">" and not float(tv) > float(tt):
                hard_fail.append(
                    f"{c}/{r}: check 18 — {tn}={tv} is NOT > {tt}, yet the goal "
                    f"failed [{e.get('reason')}]")
            elif d_ == "<" and not float(tv) < float(tt):
                hard_fail.append(
                    f"{c}/{r}: check 18 — {tn}={tv} is NOT < {tt}, yet the goal "
                    f"failed [{e.get('reason')}]")

        # 16. home_watchdog: fire rows carry the inequality and it holds;
        # escape-end rows omit it entirely (absence, not a sentinel).
        for e in [x for x in ev if x.get("event") == "home_watchdog"]:
            kind = e.get("kind")
            if kind == "escape-end":
                n_hw_escape += 1
                for f in ("test_delta_m", "test_threshold_m"):
                    if f in e:
                        hard_fail.append(
                            f"{c}/{r}: check 16 — escape-end row carries {f}; it "
                            f"evaluates no inequality and must omit it")
                continue
            n_hw_fire += 1
            td, tt = e.get("test_delta_m"), e.get("test_threshold_m")
            if td is None or tt is None:
                hard_fail.append(
                    f"{c}/{r}: check 16 — home_watchdog[{kind}] missing "
                    f"test_delta_m/test_threshold_m")
            elif not float(td) < float(tt):
                hard_fail.append(
                    f"{c}/{r}: check 16 — {kind} fired but test_delta_m={td} "
                    f"is NOT < test_threshold_m={tt}")

        # 17. reconnect_end outcome agrees with the planner log's stated outcome
        re_ev = [e for e in ev if e.get("event") == "reconnect_end"]
        stated = [m2.group(2) for ln in plog for m2 in [ENDED_RE.search(ln)] if m2]
        g7ish = [ln for ln in plog if ENDED_RE_G7.search(ln)]
        n_reconnect_end += len(re_ev)
        if g7ish:
            hard_fail.append(
                f"{c}/{r}: check 17 — {len(g7ish)} reconnect-ended line(s) in the "
                f"generation-7 wording (no outcome stated)")
        if len(re_ev) != len(stated):
            hard_fail.append(
                f"{c}/{r}: check 17 — {len(re_ev)} reconnect_end event(s) but "
                f"{len(stated)} stated outcome(s) in the log")
        else:
            for i, (e, s) in enumerate(zip(re_ev, stated)):
                if e.get("outcome") != s:
                    hard_fail.append(
                        f"{c}/{r}: check 17 — manoeuvre {i}: event outcome "
                        f"{e.get('outcome')!r} != log {s!r} (they share one "
                        f"expression, so this cannot happen by imprecision)")

        # 19. schema migration actually took effect in the DATA
        peers = [e for e in ev if e.get("event") in ("peer_lost", "peer_seen")]
        n_peer_rows += len(peers)
        for e in peers:
            if "last_contact_age_sec" in e:
                hard_fail.append(
                    f"{c}/{r}: check 19 — {e.get('event')} still carries the "
                    f"removed last_contact_age_sec")
            if "team_incomplete_sec" not in e:
                hard_fail.append(
                    f"{c}/{r}: check 19 — {e.get('event')} missing team_incomplete_sec")
        for e in [x for x in ev if x.get("event") == "run_end"]:
            n_runend_rows += 1
            if "metrics_rows" in e:
                hard_fail.append(f"{c}/{r}: check 19 — run_end carries the old metrics_rows")
            if "metrics_timer_rows" not in e:
                hard_fail.append(f"{c}/{r}: check 19 — run_end missing metrics_timer_rows")

        # 20. every mid-run reconnect line is one the parser knows
        for ln in plog:
            if MIDRUN_MARKER in ln:
                n_midrun += 1
                if not any(k in ln for k in MIDRUN_KNOWN):
                    hard_fail.append(
                        f"{c}/{r}: check 20 — unrecognised mid-run line, the "
                        f"parser would silently undercount it: {ln.strip()[:120]}")

        # 15. selected_score and selected_utility are documented as an EXACT
        # alias. If they ever diverge, every analysis that quotes one while
        # meaning the other is wrong, and nothing else would reveal it.
        csvp = os.path.join(d, f"planner_{r}.csv")
        if os.path.exists(csvp):
            with open(csvp, newline="", errors="replace") as fh:
                for i, rec in enumerate(csv.DictReader(fh)):
                    if "selected_score" not in rec or "selected_utility" not in rec:
                        hard_fail.append(f"{c}/{r}: check 15 — CSV lacks the alias columns")
                        break
                    n_csv_rows += 1
                    try:
                        a, b = float(rec["selected_score"]), float(rec["selected_utility"])
                    except (TypeError, ValueError):
                        continue
                    if a != b:
                        hard_fail.append(
                            f"{c}/{r}: check 15 — row {i}: selected_score={a} != "
                            f"selected_utility={b}; they are documented as an exact alias")
                        break
        else:
            hard_fail.append(f"{c}/{r}: check 15 — planner CSV missing")

        # 7. gate A
        nav = os.path.join(d, f"nav_{r}.log")
        ok = count_tok(nav, "global plan ok:")
        starve = count_tok(nav, "planner starving:")
        if ok is None:
            hard_fail.append(f"{c}/{r}: nav log missing")
        else:
            if ok == 0:
                hard_fail.append(f"{c}/{r}: gate A — no 'global plan ok:' line")
            if starve:
                hard_fail.append(f"{c}/{r}: gate A — {starve} 'planner starving:' line(s)")

        # 8. gate E pairing
        ent, ext, pok, why = recovery_pairing(nav)
        if ent is not None:
            agg_recovery_entries += ent
            if not pok:
                hard_fail.append(f"{c}/{r}: gate E — entries={ent} exits={ext} ({why})")

        poseloss += len([e for e in ev if e.get("event") == "pose_health" and e.get("lost")])

    # 9. deadman / process death
    con = os.path.join(ROOT, c + ".console.log")
    if os.path.exists(con):
        txt = open(con, errors="replace").read()
        for pat, label in (("sim clock frozen", "DEADMAN FIRED"),
                           ("died mid-run", "PROCESS DIED"),
                           ("/clock unreadable", "CLOCK UNREADABLE")):
            if pat in txt:
                hard_fail.append(f"{c}: {label}")

    row.update(latched=lat, arrived=arrived, unknown=unknowns,
               navfail=navfails, poseloss=poseloss)
    if poseloss:
        soft.append(f"{c}: {poseloss} pose-loss episode(s)")
    rows.append(row)

print(f"=== {TAG}: {len(cells)} cells ({dict(cells_by_arm)}) ===\n")
hdr = (f"{'cell':34} {'end':10} {'verdict':8} {'t_sim':>6} {'lat':>3} {'arr':>3} "
       f"{'nf':>3} {'pl':>3}  unknown")
print(hdr)
print("-" * len(hdr))
for r in rows:
    print(f"{r['cell']:34} {r['end']:10} {r['verdict']:8} {r['t_sim']:>6} "
          f"{r['latched']:>3} {r['arrived']:>3} {r['navfail']:>3} {r['poseloss']:>3}  "
          f"{r['unknown']}")

print("\narm identity: " + ", ".join(
    sorted({f"{r['arm']}: mode_req={r.get('mode_req')} rdv={r.get('rdv')}" for r in rows})))
print(f"nav_goal_failed by arm: {dict(nav_fail_by_arm)}")

# Populations examined — printed so a pass can be read as "N rows were checked",
# never as "nothing objected".
print("\npopulations examined by the generation-8 checks:")
for label, n, check in (
        ("CSV rows (alias)",            n_csv_rows,      "15"),
        ("home_watchdog fires",         n_hw_fire,       "16"),
        ("home_watchdog escape-ends",   n_hw_escape,     "16"),
        ("reconnect_end events",        n_reconnect_end, "17"),
        ("nav_goal_failed rows",        n_navfail_rows,  "18"),
        ("peer_lost/peer_seen rows",    n_peer_rows,     "19"),
        ("run_end rows",                n_runend_rows,   "19"),
        ("mid-run reconnect lines",     n_midrun,        "20")):
    mark = "" if n else "   <- EMPTY: check is UNRESOLVED, not passed"
    print(f"  check {check:>2}  {label:28} {n:6d}{mark}")
    if not n:
        unresolved.append(f"check {check}: {label} — zero rows, nothing was tested")

print(f"\ngate E aggregate '-> recovery:' entries: {agg_recovery_entries}")
if agg_recovery_entries == 0:
    print("  -> UNRESOLVED (not a pass): zero entries is what generation 4 produced.")
    unresolved.append("gate E: zero recovery entries")
else:
    print("  -> RESOLVED: the recovery path executed and every entry paired.")

print()
if hard_fail:
    print(f"HARD FAILURES ({len(hard_fail)}):")
    for f in hard_fail:
        print("  " + f)
else:
    print("HARD FAILURES: none")
if unresolved:
    print(f"\nUNRESOLVED ({len(unresolved)}) — these are NOT passes:")
    for u in unresolved:
        print("  " + u)
if soft:
    print(f"\nnotes ({len(soft)}):")
    for s in soft:
        print("  " + s)

sys.exit(1 if hard_fail else 0)
