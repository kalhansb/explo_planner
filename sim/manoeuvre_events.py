#!/usr/bin/env python3
"""One row per reconnection manoeuvre — the mechanism evidence, per event.

The campaign scores runs. A run is a poor unit for judging the manoeuvres: it
holds 0-5 firings, the outcome is settled before any of them fire (section
3.16), and realised severity is an outcome rather than a treatment (3.17). The
manoeuvre itself is the thing under test, so make it the unit of observation.

READ THIS AS DESCRIPTIVE MECHANISM EVIDENCE, NOT AS AN ESTIMATE OF ARM EFFECT.
Events cluster hard in runs (over half of the p3b firings come from two robots),
so the effective sample is the run count, not the event count, and nothing here
is an independent sample. Arms are never compared on these numbers.

Why the log is the source of truth for firings, not the CSV `state` column.

    The metrics CSV samples on a ~5-10 s timer. Manoeuvre episodes are routinely
    shorter than that: p3b_hybrid_seed1 alone contains chases of 7.1, 6.4 and
    3.7 s, and p3b_hybrid_seed2's atlas firing leaves NO manoeuvre row in the
    CSV at all. Counting `state in {PURSUE, RETURN_NAV, RETURN_SYNC}` therefore
    undercounts, and undercounts precisely the FAST reconnections — the ones the
    modes are supposed to be good at. Firings are parsed from the planner log,
    which emits one line per decision.

Four kinds of firing, all of which must be recognised:

    chase          startPursuit armed a trail chase
    meeting_point  hybrid's fallback after a declined or exhausted chase
    anchor_return  rendezvous' return to the last-connected anchor
    hold           pure pursuit's park-and-beacon after a declined chase

Two traps this parser exists to avoid, both verified in the data:

    `Rendezvous: exploration ended [...] with full team present -> DONE` is a
    NON-firing (the manoeuvre correctly declined, explo_planner_node.cpp:2877).
    It pattern-matches any naive grep for "Rendezvous:" and inflates counts.

    `holdForTeam` logs `Rendezvous: waiting for team at the barrier` even in the
    PURE PURSUIT arm (explo_planner_node.cpp:2866-2871). Classifying firings by
    the word "Rendezvous" files pursuit's park-and-beacon under rendezvous. The
    arm comes from the manifest and the kind from the decision line, never from
    the barrier chatter.

Time bases. The CSVs and link trace are in absolute SIM seconds; the planner
log's stamps are SYSTEM clock (rcutils does not follow use_sim_time). They are
reconciled per run: `reconnect_elapsed_sec` counts up from 0 at the arm row, so
any CSV-visible episode pins one log stamp to an exact sim time, and the
"ended after X s sim" line gives an episode's sim duration against its own wall
duration, which is the slope. Runs whose fit is unusable get their events
emitted with t_sim blank and their link-derived columns dropped, rather than
silently misaligned.

The suppression columns are the load-bearing ones. A peer reads missing when the
heartbeat is state-gated (3.8), so a manoeuvre can arm against a teammate that
is in radio range the whole time; it then "reconnects" almost instantly and
flatters every speed statistic. Two independent checks, because either alone is
circumstantial:

    link_up_frac    fraction of the 10 s before the arm in which the emulator
                    had the pair connected. 1.0 means the radio was carrying
                    traffic throughout the window in which the robot decided
                    its teammate was gone.
    peer_suppressed the PEER's own log says its heartbeat was suppressed during
                    that window ("Heartbeat resumed after X s suppressed"). This
                    is read from the other robot's log, since a robot cannot
                    observe its own silence.

Both true is a manoeuvre armed against a healthy, in-range teammate: a planner
artifact, not a reconnection. Link up with no suppression found is unexplained
and flagged separately rather than quietly counted as either.

What is deliberately NOT computed here: divergence drained across the contact.
Under the reliable relay any contact drains essentially the whole backlog, so
the drain measures the preceding outage history — a post-treatment covariate —
and not the manoeuvre. Earliness is what a manoeuvre buys; see map_divergence.py.
"""
import argparse
import csv
import glob
import os
import re
import statistics as st

MANOEUVRE_STATES = {"PURSUE", "RETURN_NAV", "RETURN_SYNC"}

# Decision lines. Each marks one firing; the kind is fixed by which line hit.
# The 2026-08-17 planner renamed the dispatch prefix ("exploration ended" ->
# "dispatched": with the mid-run trigger the manoeuvre no longer implies the
# end of exploration); both spellings are accepted so old campaigns keep
# parsing.
RE_CHASE = re.compile(
    r"Pursuit: (?:exploration ended|dispatched) \[([^\]]*)\], '([^']+)' "
    r"out of comms \(record (\d+)s old[^)]*\) -> chasing")
# Two decline shapes: the staleness-gate veto ("max Ns") and the goal-stale
# cover-check veto ("goal stale ... chase would die mid-trail").
RE_DECLINE = re.compile(
    r"Pursuit: record of '([^']+)' is (\d+)s old "
    r"\((?:max (\d+)s|goal stale)\)")
RE_RETURN = re.compile(
    r"Rendezvous: (?:exploration ended|dispatched) \[([^\]]*)\], "
    r"team incomplete "
    r"\((\d+)/(\d+) peers\) -> returning to (meeting point|last-connected anchor)")
RE_HOLD = re.compile(r"Reconnect: holding for the team at the current pose")
# Mid-run trigger context (2026-08-17). The dispatch marker precedes the
# firing line and tags it; the resume/exhaustion lines are their own events.
RE_MIDRUN_DISPATCH = re.compile(
    r"Reconnect \(mid-run\): peer silent (\d+)s >= \d+s \(attempt (\d+)/(\d+)\)")
RE_MIDRUN_RESUME = re.compile(
    r"Reconnect \(mid-run\): gave up after (\d+)s at the barrier")
RE_MIDRUN_EXHAUSTED = re.compile(
    r"Reconnect \(mid-run\): attempt budget exhausted")
# Hold escalation is a CONTINUATION of the running manoeuvre (reconnect_active_
# persists across the escalated leg), never a second firing.
RE_ESCALATE = re.compile(
    r"-> escalating to the last-connected anchor")
# Outcomes.
RE_REJOIN = re.compile(
    r"(?:Pursuit: (?:team reconnected|chased peer back in comms) mid-chase"
    r"|Rendezvous: team reconnected en route"
    r"|Rendezvous: full team connected)")
RE_ARRIVED = re.compile(r"Rendezvous: reached (last-connected anchor|meeting point)")
RE_UNREACH = re.compile(r"Rendezvous: (meeting point|last-connected anchor) unreachable")
RE_GIVEUP = re.compile(r"max_wait=[\d.]+s reached -> giving up and finishing")
RE_ENDED = re.compile(r"Reconnect manoeuvre ended after ([\d.]+) s sim")
# The non-firing that looks like one.
RE_FULLTEAM = re.compile(r"exploration ended \[[^\]]*\] with full team present")
# Section 3.8: the robot's own heartbeat went quiet while it was busy. Logged on
# RESUME, so the episode it reports is [stamp - dur, stamp].
RE_SUPPRESS = re.compile(r"Heartbeat resumed after ([\d.]+) s suppressed")
RE_STAMP = re.compile(r"^\[[A-Z]+\] \[(\d+\.\d+)\]")
# The time-base anchor: one per exploration step, with a matching LOG_STEP row
# in the CSV carrying that step's sim time.
RE_STEP_LOGGED = re.compile(r"Step (\d+) logged:")


def _f(row, key, default=None):
    try:
        return float(row[key])
    except (ValueError, KeyError, TypeError):
        return default


def read_manifest(run_dir):
    out = {}
    try:
        with open(os.path.join(run_dir, "run_manifest.txt")) as fh:
            for line in fh:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    out[k] = v
    except OSError:
        pass
    return out


def read_planner_csv(path):
    rows = []
    try:
        with open(path) as fh:
            for r in csv.DictReader(fh):
                t = _f(r, "sim_time_sec")
                if t is None:
                    continue
                try:
                    step = int(r.get("step", ""))
                except (ValueError, TypeError):
                    step = None
                rows.append({
                    "t": t,
                    "step": step,
                    "state": (r.get("state") or "").strip(),
                    "elapsed": _f(r, "reconnect_elapsed_sec", -1.0),
                    "dist": _f(r, "distance_traveled", 0.0),
                    "unk": _f(r, "unknown_fraction", -1.0),
                })
    except OSError:
        return []
    rows.sort(key=lambda r: r["t"])
    return rows


def csv_episodes(rows):
    """Manoeuvre episodes visible in the CSV: (t_arm, t_last, dist0, dist1).

    t_arm is reconstructed as sim_time - reconnect_elapsed_sec rather than taken
    from the first sampled row, because the sampler lands somewhere inside the
    episode, not on its start. The counter is cleared by transitionTo before the
    exit row is written, so that row reads -1 and is not part of the episode.
    """
    eps, cur = [], None
    for r in rows:
        if r["state"] in MANOEUVRE_STATES:
            if cur is None:
                cur = {"rows": [], "t_arm": None}
            cur["rows"].append(r)
            if r["elapsed"] is not None and r["elapsed"] >= 0:
                cand = r["t"] - r["elapsed"]
                cur["t_arm"] = cand if cur["t_arm"] is None else min(cur["t_arm"], cand)
        elif cur is not None:
            eps.append(cur)
            cur = None
    if cur is not None:
        eps.append(cur)
    out = []
    for e in eps:
        rs = e["rows"]
        out.append({
            "t_arm": e["t_arm"] if e["t_arm"] is not None else rs[0]["t"],
            "t_last": rs[-1]["t"],
            "dist0": rs[0]["dist"],
            "dist1": rs[-1]["dist"],
            "n_rows": len(rs),
        })
    return out


def parse_log(path, steps=None):
    """Ordered manoeuvre events from one planner log, stamped in WALL seconds.

    `steps`, when given, is filled with {step_index: wall_stamp} for the
    time-base fit.
    """
    events = []
    pending_decline = None
    pending_midrun = False
    try:
        fh = open(path, errors="replace")
    except OSError:
        return events
    with fh:
        for line in fh:
            m = RE_STAMP.match(line)
            if not m:
                continue
            w = float(m.group(1))
            if steps is not None:
                ms = RE_STEP_LOGGED.search(line)
                if ms:
                    steps.setdefault(int(ms.group(1)), w)
                    continue
            if RE_FULLTEAM.search(line):
                events.append({"w": w, "type": "no_fire"})
                continue
            m2 = RE_MIDRUN_DISPATCH.search(line)
            if m2:
                # Context marker: the fire line that follows carries the kind;
                # this tags it as a mid-run (vs terminal) dispatch.
                pending_midrun = True
                events.append({"w": w, "type": "midrun_dispatch",
                               "silent": float(m2.group(1)),
                               "attempt": int(m2.group(2))})
                continue
            m2 = RE_MIDRUN_RESUME.search(line)
            if m2:
                events.append({"w": w, "type": "resumed",
                               "waited": float(m2.group(1))})
                continue
            if RE_MIDRUN_EXHAUSTED.search(line):
                events.append({"w": w, "type": "midrun_exhausted"})
                continue
            m2 = RE_DECLINE.search(line)
            if m2:
                # Not a firing on its own: the fallback line that follows is.
                pending_decline = {"stale": float(m2.group(2)),
                                   "max": (float(m2.group(3))
                                           if m2.group(3) else None)}
                events.append({"w": w, "type": "decline",
                               "peer": m2.group(1), "stale": float(m2.group(2))})
                continue
            m2 = RE_CHASE.search(line)
            if m2:
                events.append({"w": w, "type": "fire", "kind": "chase",
                               "reason": m2.group(1), "peer": m2.group(2),
                               "stale": float(m2.group(3)), "declined": False,
                               "midrun": pending_midrun})
                pending_decline = None
                pending_midrun = False
                continue
            # Escalation must be tested BEFORE RE_RETURN: the escalated leg
            # also logs a "dispatched [hold-escalate] ... returning to
            # last-connected anchor" line, which would otherwise double-count
            # the manoeuvre as a second firing.
            if RE_ESCALATE.search(line):
                events.append({"w": w, "type": "escalate"})
                continue
            m2 = RE_RETURN.search(line)
            if m2:
                if m2.group(1) == "hold-escalate":
                    events.append({"w": w, "type": "escalate_leg"})
                    continue
                kind = ("meeting_point" if m2.group(4) == "meeting point"
                        else "anchor_return")
                events.append({"w": w, "type": "fire", "kind": kind,
                               "reason": m2.group(1), "peer": "",
                               "stale": pending_decline["stale"] if pending_decline else None,
                               "declined": pending_decline is not None,
                               "midrun": pending_midrun})
                pending_decline = None
                pending_midrun = False
                continue
            if RE_HOLD.search(line):
                events.append({"w": w, "type": "fire", "kind": "hold",
                               "reason": "", "peer": "",
                               "stale": pending_decline["stale"] if pending_decline else None,
                               "declined": pending_decline is not None,
                               "midrun": pending_midrun})
                pending_decline = None
                pending_midrun = False
                continue
            if RE_REJOIN.search(line):
                events.append({"w": w, "type": "rejoin"})
                continue
            m2 = RE_ARRIVED.search(line)
            if m2:
                events.append({"w": w, "type": "arrived", "where": m2.group(1)})
                continue
            if RE_UNREACH.search(line):
                events.append({"w": w, "type": "unreachable"})
                continue
            if RE_GIVEUP.search(line):
                events.append({"w": w, "type": "gaveup"})
                continue
            m2 = RE_ENDED.search(line)
            if m2:
                events.append({"w": w, "type": "ended", "dur": float(m2.group(1))})
                continue
            m2 = RE_SUPPRESS.search(line)
            if m2:
                events.append({"w": w, "type": "suppressed", "dur": float(m2.group(1))})
    return events


def fire_durations(events):
    """Logged sim duration of each firing, positionally aligned with the fires.

    None where the manoeuvre never ended (still running at the horizon), which
    is itself a result: pure pursuit's hold has no timeout.
    """
    out = []
    for i, e in enumerate(events):
        if e["type"] != "fire":
            continue
        dur = None
        for j in range(i + 1, len(events)):
            if events[j]["type"] == "ended":
                dur = events[j]["dur"]
                break
            if events[j]["type"] == "fire":
                break
        out.append(dur)
    return out


def align_episodes(durations, episodes, tol=25.0):
    """Match CSV episodes to logged firings by DURATION, order-preserving.

    Counting alone is not enough to pair them: the metrics timer misses short
    episodes, so a robot with three firings and one episode has three candidate
    pairings and no way to choose between them by order. Duration decides. The
    CSV span is a LOWER bound on the true duration (the last sampled row sits
    somewhere before the end), so the cost is asymmetric — an episode may not be
    longer than the firing it is matched to, beyond sampling jitter.

    Returns {episode_index: fire_index} for matches within tolerance.
    """
    nf, ne = len(durations), len(episodes)
    if ne == 0 or ne > nf:
        return {}

    def cost(i, j):
        d, span = durations[i], episodes[j]["t_last"] - episodes[j]["t_arm"]
        if d is None:
            return 5.0                    # unknown: allowed, mildly penalised
        if span - d > 12.0:               # episode longer than the firing
            return 1e6
        return abs(d - span)

    INF = float("inf")
    best = [[INF] * (ne + 1) for _ in range(nf + 1)]
    back = [[None] * (ne + 1) for _ in range(nf + 1)]
    for i in range(nf + 1):
        best[i][0] = 0.0
    for i in range(1, nf + 1):
        for j in range(1, min(i, ne) + 1):
            skip = best[i - 1][j]
            take = best[i - 1][j - 1] + cost(i - 1, j - 1)
            if take <= skip:
                best[i][j], back[i][j] = take, "take"
            else:
                best[i][j], back[i][j] = skip, "skip"
    if best[nf][ne] >= 1e6:
        return {}
    out, i, j = {}, nf, ne
    while j > 0 and i > 0:
        if back[i][j] == "take":
            out[j - 1] = i - 1
            i, j = i - 1, j - 1
        else:
            i -= 1
    # Drop individually bad matches even when the total was acceptable.
    return {j: i for j, i in out.items() if cost(i, j) <= tol}


def fit_wall_to_sim(per_robot):
    """Affine wall->sim for one RUN, least squares over step anchors.

    The map belongs to the run, not the robot: both planners log against the
    same system clock and observe the same /clock, so anchors pool.

    The anchors are the per-step log lines. `Step N logged: ...` is emitted once
    per exploration step and the CSV writes a matching LOG_STEP row carrying
    that step's sim time, which gives 55-75 (wall, sim) pairs spread across the
    whole run for every robot — independent of whether any manoeuvre happened.
    That independence is the point: a run whose only firings were too short to
    be sampled still gets a time base, and the earlier scheme (pair firings to
    CSV manoeuvre episodes by duration) could not place those runs at all.

    Returns (slope, offset, n_anchors, residual_s).
    """
    xs, ys = [], []
    for robot in per_robot.values():
        for step, w in robot["steps"].items():
            t = robot["step_times"].get(step)
            if t is not None:
                xs.append(w)
                ys.append(t)
    if len(xs) < 3:
        return 1.0, None, len(xs), None
    n = len(xs)
    mx, my = st.mean(xs), st.mean(ys)
    denom = sum((a - mx) ** 2 for a in xs)
    if denom <= 0:
        return 1.0, None, n, None
    slope = sum((a - mx) * (b - my) for a, b in zip(xs, ys)) / denom
    offset = my - slope * mx
    resid = max(abs(slope * a + offset - b) for a, b in zip(xs, ys))
    return slope, offset, n, resid


RE_RELAY = re.compile(r"relay totals: (\d+) delivered, (\d+) dropped")


def read_relay_totals(run_dir):
    """[(wall, delivered, dropped)] from the emulator's 10 s counter dump.

    "Connected" only means bandwidth > 0. Best-effort intents can still be
    dropped on an up link — the airtime pool is shared across every link, so a
    map drain starves the heartbeat exactly during the reconnect-detection
    window (section 8). Delivery rate distinguishes a robot that heard nothing
    because the radio was down from one that heard nothing because the channel
    was busy carrying somebody else's map.
    """
    out = []
    try:
        fh = open(os.path.join(run_dir, "comms.log"), errors="replace")
    except OSError:
        return out
    with fh:
        for line in fh:
            m = RE_RELAY.search(line)
            if not m:
                continue
            w = re.search(r"\[(\d+\.\d+)\]", line)
            if w:
                out.append((float(w.group(1)), int(m.group(1)), int(m.group(2))))
    out.sort(key=lambda x: x[0])
    return out


def delivery_rate(totals, w, window=20.0):
    """(delivered/s, dropped/s) over the `window` wall seconds before w."""
    seg = [r for r in totals if w - window <= r[0] <= w]
    if len(seg) < 2:
        return None, None
    dt = seg[-1][0] - seg[0][0]
    if dt <= 0:
        return None, None
    return ((seg[-1][1] - seg[0][1]) / dt, (seg[-1][2] - seg[0][2]) / dt)


def read_link(run_dir):
    """[(t_sim, connected, distance_m, trees)] with the startup window masked.

    Rows written before both poses have landed carry zeroed physics and read
    connected=0; counting them fabricates an outage at t=0 (section 5.3).
    path_loss_db == 0 is that signature.
    """
    out = []
    try:
        with open(os.path.join(run_dir, "link_states.csv")) as fh:
            for r in csv.DictReader(fh):
                pl = _f(r, "path_loss_db", 0.0)
                if pl is None or pl <= 0.0:
                    continue
                t = _f(r, "t_sim")
                c = _f(r, "connected")
                if t is None or c is None:
                    continue
                out.append((t, c > 0.5, _f(r, "distance_m", float("nan")),
                            _f(r, "trees_on_link", float("nan"))))
    except OSError:
        return []
    out.sort(key=lambda x: x[0])
    return out


def link_at(link, t):
    prev = None
    for row in link:
        if row[0] > t:
            break
        prev = row
    return prev


def link_up_fraction(link, t, window=10.0):
    """Share of the `window` seconds before t in which the pair was connected.

    A single sample is the wrong test. The connect decision carries hysteresis
    (3-of-8 samples, ~0.6 s) and the heartbeat is 1 Hz, so a link that came up
    a moment ago has not yet delivered anything and a robot may legitimately
    still read its teammate as missing. A window asks the question that matters:
    had the radio been carrying traffic long enough for a beacon to arrive?
    """
    vals = [c for tt, c, _, _ in link if t - window <= tt <= t]
    return (sum(1 for c in vals if c) / len(vals)) if vals else None


def next_rising_edge(link, t):
    """First time after t at which the link comes up (down -> up)."""
    prev = None
    for row in link:
        if row[0] < t:
            prev = row
            continue
        if prev is not None and not prev[1] and row[1]:
            return row[0]
        prev = row
    return None


def dist_at(rows, t):
    prev = None
    for r in rows:
        if r["t"] > t:
            break
        prev = r
    return prev["dist"] if prev else None


def analyse_run(run_dir):
    man = read_manifest(run_dir)
    name = os.path.basename(os.path.normpath(run_dir))
    per_robot = {}
    for p in sorted(glob.glob(os.path.join(run_dir, "planner_*.csv"))):
        robot = os.path.basename(p)[len("planner_"):-len(".csv")]
        rows = read_planner_csv(p)
        log = os.path.join(run_dir, f"planner_{robot}.log")
        steps = {}
        events = parse_log(log, steps)
        # sim time of each LOG_STEP row, keyed by step index
        step_times = {r["step"]: r["t"] for r in rows
                      if r["state"] == "LOG_STEP" and r["step"] is not None}
        per_robot[robot] = {"rows": rows, "episodes": csv_episodes(rows),
                            "events": events, "steps": steps,
                            "step_times": step_times}
    if not per_robot:
        return None

    slope, offset, n_off, resid = fit_wall_to_sim(per_robot)
    for R in per_robot.values():
        fires = [e for e in R["events"] if e["type"] == "fire"]
        if fires and R["episodes"]:
            R["match"] = align_episodes(fire_durations(R["events"]), R["episodes"])
    link = read_link(run_dir)
    relay = read_relay_totals(run_dir)

    def to_sim(w):
        return None if offset is None else slope * w + offset

    # Suppression episodes per robot, in sim time. A robot cannot observe its
    # own silence, so these are looked up on the PEER when a firing is judged.
    suppress = {}
    for robot, R in per_robot.items():
        iv = []
        for e in R["events"]:
            if e["type"] == "suppressed":
                t_end = to_sim(e["w"])
                if t_end is not None:
                    iv.append((t_end - e["dur"], t_end))
        suppress[robot] = iv

    def peer_suppressed(robot, t, window=10.0):
        """Was any OTHER robot's heartbeat suppressed in [t-window, t]?"""
        if t is None:
            return ""
        for other, iv in suppress.items():
            if other == robot:
                continue
            for lo, hi in iv:
                if hi >= t - window and lo <= t:
                    return 1
        return 0

    out = []
    for robot, R in sorted(per_robot.items()):
        ev = R["events"]
        fire_idx = -1
        for i, e in enumerate(ev):
            if e["type"] == "fire":
                fire_idx += 1
            if e["type"] != "fire":
                continue
            t_arm = to_sim(e["w"])
            # Walk forward to the resolution of THIS firing. The LAST decisive
            # event before the next firing wins, not the first: an escalated
            # hold can arrive at the anchor and STILL give up later, and a
            # mid-run barrier can arrive and still resume — "arrived_waiting"
            # is only the outcome when nothing further resolved it. escalate /
            # escalate_leg / midrun context rows are continuations, never
            # resolutions.
            outcome, t_out, dur = "open_at_horizon", None, None
            arrived_at = None
            for j in range(i + 1, len(ev)):
                nxt = ev[j]
                if nxt["type"] == "fire":
                    break
                if nxt["type"] == "rejoin":
                    outcome, t_out = "reconnected", to_sim(nxt["w"])
                    break
                if nxt["type"] == "resumed":
                    outcome, t_out = "resumed_exploring", to_sim(nxt["w"])
                    break
                if nxt["type"] == "gaveup":
                    outcome, t_out = "gave_up", to_sim(nxt["w"])
                    break
                if nxt["type"] == "arrived":
                    arrived_at = to_sim(nxt["w"])
                    continue
                if nxt["type"] == "unreachable":
                    outcome, t_out = "unreachable", to_sim(nxt["w"])
                    break
            if outcome == "open_at_horizon" and arrived_at is not None:
                outcome, t_out = "arrived_waiting", arrived_at
            for j in range(i + 1, len(ev)):
                if ev[j]["type"] == "ended":
                    dur = ev[j]["dur"]
                    break
                if ev[j]["type"] == "fire":
                    break

            lk = link_at(link, t_arm) if (link and t_arm is not None) else None
            upf = (link_up_fraction(link, t_arm)
                   if (link and t_arm is not None) else None)
            dlv, drp = delivery_rate(relay, e["w"]) if relay else (None, None)
            d0 = dist_at(R["rows"], t_arm) if t_arm is not None else None
            d1 = (dist_at(R["rows"], t_out) if t_out is not None
                  else (R["rows"][-1]["dist"] if R["rows"] else None))
            out.append({
                "run": name,
                "arm": man.get("reconnect_mode_requested", "?"),
                "seed": man.get("seed", "?"),
                "tx_power_dbm": man.get("tx_power_dbm", "?"),
                "gates": man.get("run_gates_verdict", ""),
                "end_reason": man.get("run_end_reason", ""),
                "robot": robot,
                "kind": e["kind"],
                "midrun": int(bool(e.get("midrun"))),
                "after_decline": int(bool(e.get("declined"))),
                "staleness_s": e.get("stale"),
                "t_arm_sim": t_arm,
                "link_up_at_arm": ("" if lk is None else int(lk[1])),
                "link_up_frac_10s": ("" if upf is None else round(upf, 2)),
                "delivered_per_s": ("" if dlv is None else round(dlv, 2)),
                "dropped_per_s": ("" if drp is None else round(drp, 2)),
                "peer_suppressed": peer_suppressed(robot, t_arm),
                "separation_m_at_arm": ("" if lk is None else lk[2]),
                "trees_on_link_at_arm": ("" if lk is None else lk[3]),
                "outcome": outcome,
                "t_outcome_sim": t_out,
                "dt_to_outcome_s": (None if (t_arm is None or t_out is None)
                                    else t_out - t_arm),
                "manoeuvre_dur_s": dur,
                "dist_travelled_m": (None if (d0 is None or d1 is None) else d1 - d0),
                "csv_visible": int(fire_idx in set(R.get("match", {}).values())),
            })
    declines = sum(1 for R in per_robot.values()
                   for e in R["events"] if e["type"] == "decline")
    no_fire = sum(1 for R in per_robot.values()
                  for e in R["events"] if e["type"] == "no_fire")
    return {
        "name": name, "events": out, "declines": declines, "no_fire": no_fire,
        "fit": (slope, offset, n_off, resid),
        "usable": man.get("run_end_reason", "") != "",
        "gates": man.get("run_gates_verdict", ""),
        "tx": man.get("tx_power_dbm", "?"),
        "arm": man.get("reconnect_mode_requested", "?"),
    }


FIELDS = ["run", "arm", "seed", "tx_power_dbm", "gates", "end_reason", "robot",
          "kind", "midrun", "after_decline", "staleness_s", "t_arm_sim",
          "link_up_at_arm",
          "link_up_frac_10s", "delivered_per_s", "dropped_per_s",
          "peer_suppressed", "separation_m_at_arm",
          "trees_on_link_at_arm", "outcome", "t_outcome_sim", "dt_to_outcome_s",
          "manoeuvre_dur_s", "dist_travelled_m", "csv_visible"]


def fmt(v, nd=1):
    if v is None or v == "":
        return "--"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--root", default="/tmp/hmr_campaign")
    ap.add_argument("--csv", default="", help="write the event table here")
    ap.add_argument("--include-unfinished", action="store_true",
                    help="also emit runs with no run_end_reason (interrupted)")
    args = ap.parse_args()

    runs, skipped = [], []
    for name in sorted(os.listdir(args.root)):
        d = os.path.join(args.root, name)
        if not (os.path.isdir(d) and
                os.path.exists(os.path.join(d, "run_manifest.txt"))):
            continue
        r = analyse_run(d)
        if not r:
            continue
        if not r["usable"] and not args.include_unfinished:
            skipped.append(r["name"])
            continue
        runs.append(r)

    if skipped:
        print("interrupted (no run_end_reason), excluded: " + ", ".join(skipped))
        print()

    events = [e for r in runs for e in r["events"]]
    if not events:
        print("no manoeuvre firings found")
        return 0

    print(f"{'run':<24}{'rob':<7}{'kind':<14}{'mid':>4}{'stale':>7}{'t_arm':>8}"
          f"{'up10s':>7}{'supp':>5}{'sep_m':>7}{'trees':>6}{'outcome':>17}"
          f"{'dt':>7}{'dist':>7}{'csv':>4}")
    print("-" * 125)
    for e in events:
        print(f"{e['run']:<24}{e['robot']:<7}{e['kind']:<14}"
              f"{e['midrun']:>4}"
              f"{fmt(e['staleness_s'], 0):>7}{fmt(e['t_arm_sim'], 0):>8}"
              f"{fmt(e['link_up_frac_10s'], 2):>7}{fmt(e['peer_suppressed']):>5}"
              f"{fmt(e['separation_m_at_arm']):>7}"
              f"{fmt(e['trees_on_link_at_arm'], 2):>6}{e['outcome']:>17}"
              f"{fmt(e['dt_to_outcome_s'], 0):>7}"
              f"{fmt(e['dist_travelled_m'], 0):>7}{e['csv_visible']:>4}")

    # --- summaries, stratified by transmit power ---------------------------
    # NEVER pool across tx: the powers are different link budgets, and section
    # 3.19 rules the whole severity ladder they came from inadmissible anyway.
    print("\n=== firings by kind, stratified by tx power ===")
    by_tx = {}
    for e in events:
        by_tx.setdefault(e["tx_power_dbm"], []).append(e)
    for tx, es in sorted(by_tx.items()):
        kinds = {}
        for e in es:
            kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
        runs_here = len({e["run"] for e in es})
        robots_here = len({(e["run"], e["robot"]) for e in es})
        print(f"tx={tx:<7} events={len(es):<4} in {runs_here} run(s), "
              f"{robots_here} robot(s):  "
              + ", ".join(f"{k}={v}" for k, v in sorted(kinds.items())))

    def outcome_table(es, indent="    "):
        by_kind = {}
        for e in es:
            by_kind.setdefault(e["kind"], []).append(e)
        for kind, ks in sorted(by_kind.items()):
            oc = {}
            for e in ks:
                oc[e["outcome"]] = oc.get(e["outcome"], 0) + 1
            dts = [e["dt_to_outcome_s"] for e in ks
                   if e["dt_to_outcome_s"] is not None and e["outcome"] == "reconnected"]
            dt_s = (f"  reconnect dt: median {st.median(dts):.0f}s "
                    f"({min(dts):.0f}-{max(dts):.0f})" if dts else "")
            print(f"{indent}{kind:<14} n={len(ks):<3} "
                  + ", ".join(f"{k}={v}" for k, v in sorted(oc.items())) + dt_s)

    print("\n=== outcome by kind (within tx stratum) ===")
    for tx, es in sorted(by_tx.items()):
        print(f"tx={tx}")
        outcome_table(es)

    # --- the validity check that matters most -----------------------------
    print("\n=== validity: was the teammate actually unreachable at arm time? ===")
    known = [e for e in events if e["link_up_frac_10s"] != ""]
    print(f"firings with a resolvable arm time: {len(known)}/{len(events)}")
    genuine, artifact, unexplained = [], [], []
    for e in known:
        if e["link_up_frac_10s"] < 0.5:
            genuine.append(e)
        elif e["peer_suppressed"] == 1:
            artifact.append(e)
        else:
            unexplained.append(e)
    print(f"  genuine outage   (link down >50% of the 10 s before arming): "
          f"{len(genuine)}")
    print(f"  planner artifact (link up AND peer heartbeat suppressed):    "
          f"{len(artifact)}")
    print(f"  unexplained      (link up, no suppression found):            "
          f"{len(unexplained)}")
    for label, group in (("ARTIFACT", artifact), ("UNEXPLAINED", unexplained)):
        for e in group:
            print(f"    [{label}] {e['run']:<24}{e['robot']:<7}{e['kind']:<14}"
                  f"up={fmt(e['link_up_frac_10s'], 2)} "
                  f"dlv={fmt(e['delivered_per_s'], 1)}/s "
                  f"drp={fmt(e['dropped_per_s'], 1)}/s sep="
                  f"{fmt(e['separation_m_at_arm'])} m  {e['outcome']} "
                  f"dt={fmt(e['dt_to_outcome_s'], 0)}s")
    if artifact or unexplained:
        print("  A firing with the link up armed against a teammate that was in"
              "\n  range. The heartbeat is state-gated (3.8), so a planner busy in"
              "\n  PLAN goes quiet and its teammate declares it missing under a"
              "\n  healthy link. These 'reconnect' almost instantly and flatter"
              "\n  every speed statistic; they are not reconnections.")

    if genuine:
        print("\n=== outcome by kind, GENUINE OUTAGES ONLY ===")
        print("The table above pools real reconnections with firings against an"
              "\nin-range teammate. This one keeps only firings where the link was"
              "\ndown for most of the 10 s before arming. It is the honest one.")
        by_tx_g = {}
        for e in genuine:
            by_tx_g.setdefault(e["tx_power_dbm"], []).append(e)
        for tx, es in sorted(by_tx_g.items()):
            print(f"tx={tx}")
            outcome_table(es)

    # --- clustering, stated rather than buried ----------------------------
    print("\n=== degenerate firings: no path was planned or driven ===")
    deg = [e for e in events
           if e["manoeuvre_dur_s"] is not None and e["manoeuvre_dur_s"] < 5.0
           and (e["dist_travelled_m"] is None or e["dist_travelled_m"] < 1.0)]
    print(f"{len(deg)}/{len(events)} firings ended within 5 s having travelled "
          f"under 1 m:")
    for e in deg:
        print(f"    {e['run']:<24}{e['robot']:<7}{e['kind']:<14}"
              f"dur={fmt(e['manoeuvre_dur_s'])}s dist="
              f"{fmt(e['dist_travelled_m'])} m  stale="
              f"{fmt(e['staleness_s'], 0)}s")
    if deg:
        print("  These arm and dissolve before the robot moves. The test that ARMS"
              "\n  the manoeuvre (peer missing) and the test that ENDS it (team"
              "\n  present) do not agree within a single cycle, so the manoeuvre"
              "\n  fires and is cancelled on the next tick. p3b_hybrid_seed2 has"
              "\n  one that logs 'ended after 0.0 s sim'. Counting these as"
              "\n  reconnections is what makes the chase look instant: no trail was"
              "\n  followed, no waypoint reached, no path planned. Any evaluation of"
              "\n  the reconnection PATH PLANNING must exclude them.")

    print("\n=== clustering (why event count is not sample size) ===")
    per_robot = {}
    for e in events:
        per_robot[(e["run"], e["robot"])] = per_robot.get((e["run"], e["robot"]), 0) + 1
    top = sorted(per_robot.items(), key=lambda kv: -kv[1])
    print(f"{len(events)} events from {len(per_robot)} robot-runs in "
          f"{len({e['run'] for e in events})} runs")
    print("  most productive: " + ", ".join(
        f"{r}/{rob}={n}" for (r, rob), n in top[:4]))
    print("  Events within a robot-run share a trajectory, a map and a link"
          "\n  history. Treat the run as the unit; do not fit a regression"
          "\n  across events as though they were independent.")

    print("\n=== log/CSV disagreement (why firings are parsed from the log) ===")
    inv = [e for e in events if not e["csv_visible"]]
    print(f"firings with NO manoeuvre row in the metrics CSV: {len(inv)}/{len(events)}")
    for e in inv:
        print(f"    {e['run']:<24}{e['robot']:<7}{e['kind']:<14}"
              f"dur={fmt(e['manoeuvre_dur_s'])}s")
    if inv:
        print("  The CSV timer cannot see these. A state-column census undercounts"
              "\n  firings, and undercounts the FAST ones preferentially.")

    print("\n=== non-firings and declines (context, not events) ===")
    for r in runs:
        if r["declines"] or r["no_fire"]:
            print(f"    {r['name']:<24} declined chases={r['declines']}  "
                  f"'full team present -> DONE'={r['no_fire']}")
    print("  'full team present -> DONE' is the manoeuvre correctly NOT firing"
          "\n  (explo_planner_node.cpp:2877). It is not a manoeuvre.")

    # --- the number the design question actually turns on -----------------
    deg_set = {id(e) for e in deg}
    usable = [e for e in genuine
              if id(e) not in deg_set and e["kind"] != "hold"]
    print("\n=== how much evidence about the PATH PLANNING is on disk? ===")
    print(f"  {len(events):>3} firings parsed from the logs")
    print(f"  {len(genuine):>3} armed during a genuine outage")
    print(f"  {len(genuine) - len([e for e in genuine if id(e) in deg_set]):>3} "
          f"of those were not degenerate")
    print(f"  {len(usable):>3} of those drove a route (holds park by design and "
          f"plan nothing)")
    print(f"      across {len({e['run'] for e in usable})} runs, "
          f"{len({(e['run'], e['robot']) for e in usable})} robot-runs")
    if usable:
        dd = [e["dist_travelled_m"] for e in usable
              if e["dist_travelled_m"] is not None]
        tt = [e["dt_to_outcome_s"] for e in usable
              if e["dt_to_outcome_s"] is not None]
        if dd:
            print(f"      distance driven: {min(dd):.0f}-{max(dd):.0f} m")
        if tt:
            print(f"      time to outcome: {min(tt):.0f}-{max(tt):.0f} s")
    print("  That is the whole evidence base for the manoeuvre path planning in"
          "\n  every campaign run recorded so far. It is not enough to rank three"
          "\n  modes, and no amount of re-analysis of these runs will make it so.")

    print("\n=== time-base fit quality (per run) ===")
    for r in runs:
        slope, offset, n_off, resid = r["fit"]
        if not r["events"]:
            continue
        if offset is None:
            print(f"    {r['name']:<24} NO ANCHOR — sim times unavailable, "
                  f"link columns dropped")
        else:
            print(f"    {r['name']:<24} slope={slope:.6f}  step anchors={n_off:<4}"
                  f"max residual={'--' if resid is None else f'{resid:.2f} s'}")

    if args.csv:
        with open(args.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            for e in events:
                w.writerow({k: ("" if e.get(k) is None else e.get(k)) for k in FIELDS})
        print(f"\nevent table -> {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
