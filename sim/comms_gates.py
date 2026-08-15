#!/usr/bin/env python3
"""Bring-up and run-time gates for the comms/reconnection experiment.

Every failure these catch is silent: the run completes, the CSVs fill, the plots
look reasonable, and the numbers are wrong. A leaked topic makes an outage arm
behave like the control while still reporting outages; a QoS mismatch makes a
relayed stream vanish entirely, which reads as "the link was always down"; an
overflow drops map deltas the receiver never learns it is missing; a dead odom
source freezes a link's state — possibly at *connected* — for the rest of the
run. None of them raise an error anywhere.

Two modes:

  check   one-shot, after bring-up. Runs leakage + QoS + a first overflow read.
          Exit 0 = clear, 1 = a gate failed, 2 = the check could not be run
          (which is itself a failure — an unrunnable gate is not a passed gate).

  watch   run-time monitor, until SIGINT/SIGTERM. Polls overflow and the odom
          watchdog. Appends findings to --report and exits non-zero if any gate
          tripped, so the harness can invalidate the run at teardown.

Usage:
  comms_gates.py check --robots atlas,bestla [--report FILE] [--allow rviz,foo]
  comms_gates.py watch --robots atlas,bestla [--report FILE] [--odom-timeout 10]

Requires a sourced ROS 2 environment and the same ROS_DOMAIN_ID as the run.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time


class _Stop(Exception):
    """Raised from the SIGTERM handler so watch mode exits through its finally."""


def _raise_stop(signum, frame):
    raise _Stop()

# Nodes permitted to subscribe to a cross-robot topic. The §5.1 merge-attribution
# oracle REQUIRES bagging cross-robot topics, so the recorder is not a leak;
# neither is a visualiser, which consumes but never feeds a planner. Anything
# else holding a cross-robot subscription is routing real data around the
# emulator.
DEFAULT_ALLOW = ["rosbag", "rviz", "transform_listener", "_ros2cli"]

# Topic suffixes that must cross the emulator when COMMS=1. A subscriber on the
# pre-relay copy of one of these is the leak that silently disables the
# experiment.
GATED_SUFFIXES = ["scovox_node/scovox_bin", "exploration/intents"]


def run(cmd, timeout=15):
    """Run a command, returning stdout ('' on any failure). Never raises.

    A '' return is genuinely ambiguous — the topic may not exist, or the CLI may
    have timed out on a slow box — so callers must NOT silently `continue` past
    it and then count the topic as inspected. Use run_checked() where the
    difference between "clean" and "never looked" matters, which is most places:
    "N topics clean" over a sweep that inspected none of them is the failure
    mode this whole script exists to prevent.
    """
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           stdin=subprocess.DEVNULL)
        return p.stdout if p.returncode == 0 else ""
    except (subprocess.TimeoutExpired, OSError):
        return ""


def run_checked(cmd, timeout=15):
    """(stdout, err) — err is None on success, else a short reason string."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           stdin=subprocess.DEVNULL)
        if p.returncode != 0:
            tail = (p.stderr or "").strip().splitlines()
            return "", f"exit {p.returncode}: {tail[-1] if tail else 'no stderr'}"
        return p.stdout, None
    except subprocess.TimeoutExpired:
        return "", f"timed out after {timeout}s"
    except OSError as e:
        return "", f"could not exec: {e}"


# Sim-time safety factor for CLI reads that wait on a sim-clock publisher.
#
# /hmr_comms_sim/stats is published on a ROS timer at stats_period_s (10.0) with
# use_sim_time, so its WALL period is 10 / RTF seconds. A fixed wall timeout
# therefore turns real-time factor into a gate verdict: below RTF ~0.5 a 20 s
# timeout expires before the next sample and a healthy run is reported invalid
# every poll. The gate's job is to detect overflow, not slowness, so the wait is
# sized for a slow box and the "could not read" case is reported as UNRUNNABLE
# (exit 2) rather than as a failed gate.
STATS_WALL_TIMEOUT_S = 90


class Report:
    def __init__(self, path):
        self.path = path
        self.failures = []
        self.notes = []
        # Gates that could not be evaluated. Kept separate from failures so a
        # slow or half-up graph does not read as a tripped gate, and separate
        # from notes so it never reads as a pass either.
        self.unrunnables = []

    def fail(self, gate, msg):
        self.failures.append((gate, msg))
        print(f"[GATE FAIL] {gate}: {msg}", flush=True)

    def unrunnable(self, gate, msg):
        self.unrunnables.append((gate, msg))
        print(f"[GATE ????] {gate}: {msg}", flush=True)

    def note(self, gate, msg):
        self.notes.append((gate, msg))
        print(f"[gate ok]   {gate}: {msg}", flush=True)

    def flush(self):
        if not self.path:
            return
        with open(self.path, "a") as f:
            for g, m in self.notes:
                f.write(f"PASS\t{g}\t{m}\n")
            for g, m in self.unrunnables:
                f.write(f"UNRUN\t{g}\t{m}\n")
            for g, m in self.failures:
                f.write(f"FAIL\t{g}\t{m}\n")


# ---------------------------------------------------------------------------
# Gate 1 — leakage
# ---------------------------------------------------------------------------

def gate_leakage(robots, allow, rep):
    """No node outside the emulator may subscribe to another robot's raw stream.

    `ros2 topic info -v` lists subscribers per topic with their node names. For
    each robot's gated topics, every subscriber must be either hmr_comms_sim (the
    relay itself), a node in the same robot's namespace (its own data), or
    allowlisted.
    """
    checked = 0
    for r in robots:
        for suffix in GATED_SUFFIXES:
            topic = f"/{r}/{suffix}"
            out, err = run_checked(["ros2", "topic", "info", "-v", topic])
            if err or not out:
                # NOT a silent continue. An uninspectable gated topic is an
                # unrunnable gate, and an unrunnable gate is not a passed gate.
                rep.fail("leakage",
                         f"could not inspect {topic} ({err or 'empty output'})"
                         f" — cannot certify it is not leaking")
                continue
            checked += 1
            for node, ns in parse_endpoints(out, "Subscription"):
                full = f"{ns.rstrip('/')}/{node}".replace("//", "/")
                if "hmr_comms_sim" in node:
                    continue
                if any(a in node or a in full for a in allow):
                    continue
                # A robot reading its own stream is not a cross-robot path.
                if ns.strip("/").split("/")[:1] == [r]:
                    continue
                rep.fail("leakage",
                         f"{full} subscribes to {topic} — cross-robot data "
                         f"bypassing the emulator")

    # The shared bus, explicitly. This one bypasses the emulator by construction
    # rather than by accident: it is a single global topic, so nothing can sit
    # between two robots on it. Under COMMS=1 the planners must have moved off
    # it, and any remaining subscriber means at least one planner did not.
    out = run(["ros2", "topic", "info", "-v", "/exploration/intents"])
    if out:
        subs = [n for n, _ in parse_endpoints(out, "Subscription")
                if not any(a in n for a in allow)]
        if subs:
            rep.fail("leakage",
                     f"/exploration/intents still has subscribers {subs} — the "
                     f"shared intent bus cannot be gated, so no outage can ever "
                     f"make a peer read as missing")
        else:
            rep.note("leakage", "/exploration/intents has no live subscribers")

    if checked == 0:
        rep.fail("leakage", "no gated topics found at all — checked nothing; "
                            "is the stack up and ROS_DOMAIN_ID correct?")
    elif not rep.failures:
        rep.note("leakage", f"{checked} gated topic(s) clean")


def parse_blocks(info_out):
    """Parse `ros2 topic info -v` into a list of endpoint dicts.

    Each endpoint is a block of the form

        Node name: dscovox_node
        Node namespace: /atlas
        Topic type: scovox_msgs/msg/ScovoxMapBinary
        Endpoint type: SUBSCRIPTION
        GID: ...
        QoS profile:
          Reliability: RELIABLE
          Durability: VOLATILE
          ...

    Keyed off "Endpoint type" rather than the "Publisher count:" /
    "Subscription count:" headers, because the count headers are positional and
    the endpoint field is stated per block.
    """
    blocks, cur = [], None
    for line in info_out.splitlines():
        s = line.strip()
        if s.startswith("Node name:"):
            if cur:
                blocks.append(cur)
            cur = {"node": s.split(":", 1)[1].strip(), "ns": "/",
                   "kind": "", "reliability": "", "durability": ""}
        elif cur is None:
            continue
        elif s.startswith("Node namespace:"):
            cur["ns"] = s.split(":", 1)[1].strip()
        elif s.startswith("Endpoint type:"):
            cur["kind"] = s.split(":", 1)[1].strip().upper()
        elif s.startswith("Reliability:"):
            cur["reliability"] = s.split(":", 1)[1].strip().upper()
        elif s.startswith("Durability:"):
            cur["durability"] = s.split(":", 1)[1].strip().upper()
    if cur:
        blocks.append(cur)
    return blocks


def parse_endpoints(info_out, kind):
    """(node, namespace) pairs for endpoints of `kind` ("Subscription"/"Publisher")."""
    want = "SUBSCRIPTION" if kind.lower().startswith("sub") else "PUBLISHER"
    return [(b["node"], b["ns"]) for b in parse_blocks(info_out)
            if b["kind"] == want]


# ---------------------------------------------------------------------------
# Gate 2 — QoS match
# ---------------------------------------------------------------------------

def gate_qos(robots, rep):
    """Relayed topics must have a compatible publisher/subscriber QoS pair.

    An incompatible pair does not error anywhere — the subscription simply never
    matches, the stream is silently absent, and the run reads as a permanent
    outage on that link. The emulator mirrors the source publisher's reliability
    so the mismatch should not arise today; this exists because that is a
    property of the current code, not a guarantee, and the failure is
    indistinguishable from a result.
    """
    rx = [t for t in run(["ros2", "topic", "list"]).split()
          if "/rx/" in t]
    if not rx:
        rep.fail("qos", "no /rx/ topics — the relay never formed")
        return
    bad = 0
    for topic in rx:
        out = run(["ros2", "topic", "info", "-v", topic])
        if not out:
            continue
        blocks = parse_blocks(out)
        pubs = [b for b in blocks if b["kind"] == "PUBLISHER"]
        subs = [b for b in blocks if b["kind"] == "SUBSCRIPTION"]
        if not pubs:
            # This is the case the module docstring names first — "a QoS
            # mismatch makes a relayed stream vanish entirely, which reads as
            # 'the link was always down'" — and it was being skipped. A topic
            # that exists with a subscriber and NO publisher is exactly a relay
            # the emulator never formed: the consumer is waiting on a name
            # nothing writes to, and every downstream metric records a
            # permanent outage that the radio model never produced.
            bad += 1
            rep.fail("qos",
                     f"{topic}: NO publisher (subscribers: "
                     f"{[s['node'] for s in subs] or 'none'}) — the relay never "
                     f"formed; this stream reads as a permanent outage")
            continue
        if not subs:
            # A relay nobody listens to is not a QoS fault, but it is still a
            # stream going nowhere — the merger or planner was pointed at the
            # wrong topic name, which reads as a permanent outage.
            rep.fail("qos", f"{topic}: relay has NO subscriber — nothing is "
                            f"consuming this link's traffic")
            bad += 1
            continue
        for p in pubs:
            for s in subs:
                # Incompatible iff publisher BEST_EFFORT and subscriber RELIABLE.
                if "BEST_EFFORT" in p["reliability"] and "RELIABLE" in s["reliability"]:
                    bad += 1
                    rep.fail("qos",
                             f"{topic}: publisher {p['reliability']} vs "
                             f"{s['ns']}/{s['node']} {s['reliability']} — "
                             f"incompatible, this stream is never delivered")
                # Transient-local subscriber against a volatile publisher also
                # fails to match.
                if ("VOLATILE" in p["durability"]
                        and "TRANSIENT_LOCAL" in s["durability"]):
                    bad += 1
                    rep.fail("qos",
                             f"{topic}: publisher VOLATILE vs "
                             f"{s['ns']}/{s['node']} TRANSIENT_LOCAL — "
                             f"incompatible durability")
    if bad == 0:
        rep.note("qos", f"{len(rx)} relay topic(s) QoS-compatible")


# ---------------------------------------------------------------------------
# Gate 2b — the expected relay set exists
# ---------------------------------------------------------------------------

def expected_relays(robots):
    """Every relay topic a COMMS=1 run must have, as /<rx>/rx/<tx>/<suffix>."""
    out = []
    for rx in robots:
        for tx in robots:
            if tx == rx:
                continue
            for suffix in GATED_SUFFIXES:
                out.append(f"/{rx}/rx/{tx}/{suffix}")
    return out


def gate_relay_set(robots, rep):
    """Assert the relays that MUST exist do exist.

    gate_qos only inspects whichever /rx/ topics happen to be present, so it is
    structurally unable to see a relay that was never created — and a missing
    relay is the most consequential failure in the stack: the consumer waits on
    a name nothing publishes, every downstream metric records a permanent
    outage, and the arm silently becomes "no comms at all" instead of "comms
    modelled by the radio". The emulator discovers sources by a 1 Hz poll of the
    topic list, so a typo in coord_intent_pub_topic or a peer_bin_topic_pattern
    that resolved wrong produces exactly this and nothing logs it.
    """
    present = set(run(["ros2", "topic", "list"]).split())
    missing = [t for t in expected_relays(robots) if t not in present]
    if missing:
        rep.fail("relay_set",
                 f"expected relay topic(s) absent: {missing} — the emulator "
                 f"never formed these links; the streams they carry will read "
                 f"as a permanent outage for the whole run")
        return False
    rep.note("relay_set",
             f"all {len(expected_relays(robots))} expected relay topic(s) present")
    return True


# ---------------------------------------------------------------------------
# Gate 3 — reliable-queue overflow
# ---------------------------------------------------------------------------

def read_stats(timeout=STATS_WALL_TIMEOUT_S):
    """Latest /hmr_comms_sim/stats payload as a dict, or None."""
    out = run(["ros2", "topic", "echo", "--once", "--field", "data",
               "/hmr_comms_sim/stats"], timeout=timeout)
    if not out:
        return None
    for line in out.splitlines():
        line = line.strip().strip("'\"")
        if line.startswith("{"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                return None
    return None


def gate_overflow(rep, stats=None):
    """drop_overflow > 0 on any link invalidates the run.

    The reliable queue evicts oldest at 64 MiB per directional link. Those are
    map deltas, and a dropped delta permanently holes the receiver's merged map:
    the receiver cannot detect the gap, and total_observed_voxels — the primary
    endpoint's input — is quietly wrong for the rest of the run.
    """
    s = stats if stats is not None else read_stats()
    if s is None:
        # Unrunnable, not failed. /hmr_comms_sim/stats is on a SIM-clock timer,
        # so on a slow box its wall period stretches by 1/RTF; reporting that as
        # a tripped gate would invalidate healthy runs for being slow.
        rep.unrunnable("overflow",
                       f"no /hmr_comms_sim/stats within "
                       f"{STATS_WALL_TIMEOUT_S}s wall — overflow NOT checked "
                       f"(is the emulator up? is RTF very low?)")
        return None
    tripped = False
    for link in s.get("links", []):
        if link.get("drop_overflow", 0) > 0:
            tripped = True
            rep.fail("overflow",
                     f"{link.get('from')}->{link.get('to')}: drop_overflow="
                     f"{link['drop_overflow']} — the receiver's merged map is "
                     f"missing voxels; RUN INVALID")
    if not tripped:
        rep.note("overflow", f"{len(s.get('links', []))} link(s), no overflow")
    return s


# ---------------------------------------------------------------------------
# Gate 4 — odom watchdog
# ---------------------------------------------------------------------------

def gate_odom(robots, rep, timeout_s):
    """Every robot's pose source must still be publishing.

    The emulator's has_pose_ latches true on the first message and is never
    cleared, so if an odom source dies mid-run the link keeps being evaluated
    at the last known separation — frozen, possibly at *connected*, for the
    remainder of the run, with no warning and no drop statistic.
    """
    dead = []
    for r in robots:
        topic = f"/{r}/odom_ground_truth"
        out = run(["ros2", "topic", "echo", "--once", "--field",
                   "header.stamp.sec", topic], timeout=timeout_s)
        if not out.strip():
            dead.append(topic)
    if dead:
        rep.fail("odom", f"no message within {timeout_s}s on {dead} — the "
                         f"emulator will hold those links at their last "
                         f"evaluated state for the rest of the run")
        return False
    rep.note("odom", f"{len(robots)} pose source(s) live")
    return True


# ---------------------------------------------------------------------------
# Gate 5 — the independent variable actually varied
# ---------------------------------------------------------------------------

def outage_seen(stats):
    """True if any link has ever refused traffic for being disconnected."""
    if not stats:
        return False
    return any(link.get("drop_disconnected", 0) > 0
               for link in stats.get("links", []))


def gate_outage_occurred(rep, ever_seen, polls):
    """A COMMS=1 run in which the link never dropped is a control run.

    Nothing else in the stack checks this. The radio model is monotone in
    separation and tree count, so if the two robots simply never got far enough
    apart, the emulator relays everything, no peer ever reads MISSING, no
    reconnect manoeuvre fires, and the result is a complete, plausible dataset
    in which the treatment arm and the control arm are the same experiment.
    drop_disconnected is the direct evidence: it increments once per message
    refused because the link was down.
    """
    if ever_seen:
        rep.note("outage", "link outage(s) observed (drop_disconnected > 0)")
        return True
    rep.fail("outage",
             f"NO link outage in {polls} poll(s): drop_disconnected == 0 on "
             f"every link for the whole run. The comms arm never differed from "
             f"the control — check separation vs tx_power_dbm before using "
             f"this run for anything")
    return False


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["check", "watch"])
    ap.add_argument("--robots", required=True,
                    help="comma-separated robot names")
    ap.add_argument("--report", default="",
                    help="append PASS/FAIL lines here")
    ap.add_argument("--allow", default="",
                    help="extra comma-separated node-name substrings exempt "
                         "from the leakage gate")
    ap.add_argument("--period", type=float, default=30.0,
                    help="watch mode poll period, seconds")
    ap.add_argument("--odom-timeout", type=float, default=10.0)
    # The outage gate is the one gate that is wrong for a deliberate control
    # run. Phase 1 of the plan runs the control THROUGH the emulator at high
    # tx_power_dbm — so the relay path (extra hop, delay_ms, rx QoS) is present
    # but the link never drops, which is the intended condition, not a defect.
    # Left on, every control run would end flagged as "a comms arm that never
    # differed from the control", which is exactly backwards, and a campaign
    # whose baseline arm always reports FAIL trains the reader to ignore the
    # gate on the arms where it matters. Off, the gate still runs and still
    # reports what it saw — it just does not make the run a failure, and an
    # outage under a control label is worth knowing about either way.
    ap.add_argument("--expect-outage", choices=["yes", "no"], default="yes",
                    help="watch mode: 'no' for a control arm run at a "
                         "tx_power_dbm where the link is meant to stay up")
    args = ap.parse_args()

    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    allow = DEFAULT_ALLOW + [a.strip() for a in args.allow.split(",") if a.strip()]
    rep = Report(args.report)

    if not run(["ros2", "topic", "list"]):
        rep.fail("setup", "`ros2 topic list` returned nothing — no ROS graph "
                          "visible (wrong ROS_DOMAIN_ID, or nothing running)")
        rep.flush()
        return 2

    if args.mode == "check":
        gate_leakage(robots, allow, rep)
        gate_relay_set(robots, rep)
        gate_qos(robots, rep)
        gate_overflow(rep)
        gate_odom(robots, rep, args.odom_timeout)
        rep.flush()
        if rep.failures:
            return 1
        # An unrunnable gate is not a passed gate (see the module docstring).
        return 2 if rep.unrunnables else 0

    # watch
    print(f"[gates] watching every {args.period:.0f}s "
          f"(pid {os.getpid()}); Ctrl-C to stop", flush=True)
    tripped = False
    polls = 0
    ever_outage = False
    # SIGINT must be registered EXPLICITLY, and the reason is not obvious.
    #
    # The harness starts this watcher as a background job of a non-interactive
    # shell, and POSIX says such a job inherits SIGINT (and SIGQUIT) as SIG_IGN.
    # CPython honours an inherited SIG_IGN: it installs its default
    # KeyboardInterrupt handler only when the disposition it inherits is not
    # already "ignore". So the `except KeyboardInterrupt` below was unreachable —
    # the process ignored SIGINT outright. Teardown then waited 60 s and
    # escalated to SIGKILL, which cannot be caught, so the summary was never
    # written. signal.signal() here overrides the inherited SIG_IGN, which is
    # the whole fix; verified by sending SIGINT to the process group before and
    # after (before: still alive, empty report / after: "stopped; polls=2" and
    # both summary lines present).
    #
    # Measured cost of the bug: all 10 runs on disk logged "gateswatch ignored
    # SIGINT" and not one wrote a `watch` or `outage` line. The only run that
    # ever produced a run-time verdict was one killed with SIGTERM by hand.
    #
    # What that cost: gate_outage_occurred is the check that the INDEPENDENT
    # VARIABLE ACTUALLY VARIED — that a COMMS arm's link really dropped. It has
    # therefore never adjudicated a real run, while runs carried
    # run_gates_verdict=CLEAN earned entirely by bring-up gates. Same lesson as
    # plan §3.14 for the third time: a validity gate that cannot fail the run is
    # a log message.
    signal.signal(signal.SIGINT, _raise_stop)
    signal.signal(signal.SIGTERM, _raise_stop)
    try:
        while True:
            time.sleep(args.period)
            polls += 1
            r2 = Report(args.report)
            stats = gate_overflow(r2)
            ever_outage = ever_outage or outage_seen(stats)
            gate_odom(robots, r2, args.odom_timeout)
            if r2.failures:
                tripped = True
            # Flush failures AND unrunnables. Previously only failing polls
            # wrote anything, so an empty report was indistinguishable between
            # "clean for the whole run" and "the watcher died at t=0".
            if r2.failures or r2.unrunnables:
                r2.flush()
    except (KeyboardInterrupt, _Stop):
        pass
    summary = Report(args.report)
    if args.expect_outage == "yes":
        if not gate_outage_occurred(summary, ever_outage, polls):
            tripped = True
    elif ever_outage:
        # Not a failure, but it means the "control" was degraded, so the
        # unknown floor and makespan it produces are not a clean reference.
        summary.note("outage",
                     "link outage(s) observed in a run declared outage-free "
                     "(--expect-outage no) — tx_power_dbm may be too low for a "
                     "control arm; the floor/makespan from this run are not a "
                     "clean reference")
    else:
        summary.note("outage",
                     f"no link outage in {polls} poll(s), as expected for a "
                     f"control arm (--expect-outage no)")
    if tripped:
        summary.fail("watch", f"gates tripped during the run ({polls} polls)")
    else:
        summary.note("watch", f"{polls} polls, no gate tripped")
    summary.flush()
    print(f"[gates] stopped; polls={polls} tripped={tripped}", flush=True)
    return 1 if tripped else 0


if __name__ == "__main__":
    sys.exit(main())
