#!/usr/bin/env python3
# Moved comments: docs/sim_notes/comms_gates_notes.md
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

# Nodes allowed to subscribe to a cross-robot topic: the bag recorder and
# visualisers are not leaks. Any other cross-robot subscriber routes real data
# around the emulator. (notes: gates-leakage-allowlist)
DEFAULT_ALLOW = ["rosbag", "rviz", "transform_listener", "_ros2cli"]

# Suffixes that must cross the emulator when COMMS=1. List only streams every
# COMMS=1 run carries: gate_relay_set fails on a missing relay, so conditional
# streams go in via --gated-extra. (notes: gates-gated-suffixes)
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


def topic_list(attempts=3):
    """(set of topic names, err) — ONE listing, shared by every gate that needs it.

    THREE THINGS THIS FIXES, all of them the same mistake in different places.

    (1) A FAILED LISTING USED TO BE A FAIL, NOT AN UNRUN. `ros2 topic list` was
    called through run(), which collapses a non-zero exit, a timeout and a stale
    ros2 daemon into the same "". Three call sites then read that "" as an
    OBSERVATION: the setup precondition reported "no ROS graph visible", gate_qos
    reported "no /rx/ topics — the relay never formed", and gate_relay_set
    reported every single expected relay absent. All three are fail(), and a
    non-zero FAIL count scores the whole cell INVALID. So a CLI that timed out on
    a loaded box — or a stale daemon, which is the documented way this happens
    here and cannot be cleared with SIGTERM — threw away a perfectly good run and
    did it with a message asserting something about the radio that was never
    observed. That is the exact inversion the STATS_WALL_TIMEOUT_S note and the
    gate_qos UNRUN block already argue against; those two were fixed and these
    three were not.

    The distinction that matters is not "empty vs non-empty", it is "the command
    worked and the graph is empty" (evidence: FAIL) against "the command did not
    work" (no evidence: UNRUN). Only run_checked can tell those apart, so err is
    returned rather than swallowed and callers are expected to branch on it.

    (2) RETRIED, for the same reason gate_qos retries its per-topic reads: one
    timeout while a sim saturates the box says nothing. Only a listing that stays
    unreadable is worth a verdict.

    (3) ONE LISTING, NOT THREE. The three call sites each ran their own, seconds
    apart, so they could legitimately disagree — a relay appearing between the
    setup check and gate_relay_set made the expected-set gate fail against a
    graph that no longer existed by the time gate_qos looked. Sharing the read
    makes the three gates' verdicts refer to the same observed graph.
    """
    out, err = "", None
    for _ in range(max(1, attempts)):
        out, err = run_checked(["ros2", "topic", "list"])
        if err is None:
            break
    if err is not None:
        return set(), err
    return set(out.split()), None


# Wall wait for /hmr_comms_sim/stats, published every stats_period_s of sim time
# (wall period stretches by 1/RTF). Sized for a slow box; a read that times out
# is UNRUNNABLE, not a failed gate. (notes: gates-stats-wall-timeout)
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

    # The global /exploration/* buses cannot be gated; under COMMS=1 any
    # subscriber means a planner did not move off them. Driven off
    # GATED_SUFFIXES so --gated-extra streams are checked too.
    # (notes: gates-leakage-shared-bus)
    for suffix in GATED_SUFFIXES:
        if not suffix.startswith("exploration/"):
            continue
        bus = f"/{suffix}"
        out, err = run_checked(["ros2", "topic", "info", "-v", bus])
        if err is not None or not out:
            # A non-zero exit is the normal COMMS=1 case (the bus does not
            # exist) and is noted; a timeout means this half of the gate did not
            # run and is UNRUNNABLE. Never a silent skip.
            # (notes: gates-shared-bus-unread)
            if err and "timed out" in err:
                rep.unrunnable("leakage",
                               f"could not read {bus} ({err}) — cannot certify "
                               f"the shared bus has no subscribers")
            else:
                rep.note("leakage", f"{bus} does not exist ({err or 'no output'})")
            continue
        subs = [n for n, _ in parse_endpoints(out, "Subscription")
                if not any(a in n for a in allow)]
        if subs:
            rep.fail("leakage",
                     f"{bus} still has subscribers {subs} — a shared "
                     f"broadcast bus cannot be gated, so no outage can ever "
                     f"make a peer read as missing on this stream")
        else:
            rep.note("leakage", f"{bus} has no live subscribers")

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

def gate_qos(robots, rep, present):
    """Relayed topics must have a compatible publisher/subscriber QoS pair.

    An incompatible pair does not error anywhere — the subscription simply never
    matches, the stream is silently absent, and the run reads as a permanent
    outage on that link. The emulator mirrors the source publisher's reliability
    so the mismatch should not arise today; this exists because that is a
    property of the current code, not a guarantee, and the failure is
    indistinguishable from a result.
    """
    # Uses the caller's listing; main() returns before the gates if topic_list()
    # fails, so an empty rx here is a real observation.
    # (notes: gates-qos-shared-listing)
    rx = [t for t in present if "/rx/" in t]
    if not rx:
        rep.fail("qos", "no /rx/ topics — the relay never formed")
        return
    bad = 0
    unreadable = []          # (topic, reason) for topics whose QoS never loaded
    for topic in rx:
        # Read through run_checked, up to 3 attempts: one timeout on a loaded
        # box is not evidence. Unreadable topics are recorded with their reason,
        # never skipped. (notes: gates-qos-retried-read)
        out, err = "", None
        for attempt in range(3):
            out, err = run_checked(["ros2", "topic", "info", "-v", topic])
            if err is None and out:
                break
        if err is not None or not out:
            unreadable.append((topic, err or "empty output, exit 0"))
            continue
        blocks = parse_blocks(out)
        pubs = [b for b in blocks if b["kind"] == "PUBLISHER"]
        subs = [b for b in blocks if b["kind"] == "SUBSCRIPTION"]
        if not pubs:
            # No publisher on a relay topic means the emulator never formed the
            # relay; downstream it reads as a permanent outage, so it fails the
            # gate. (notes: gates-qos-no-publisher)
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
    # An unread topic is UNRUNNABLE, not FAIL: it is no evidence of a QoS
    # defect. UNRUN scores SUSPECT, and qos is not in gate_g8's
    # REPORT_ONLY_GATES, so it still surfaces. The note reports checked out of
    # len(rx). (notes: gates-qos-denominator)
    checked = len(rx) - len(unreadable)
    if unreadable:
        shown = ", ".join(f"{t} ({why})" for t, why in unreadable[:3])
        rep.unrunnable("qos",
                       f"QoS UNREAD for {len(unreadable)}/{len(rx)} relay "
                       f"topic(s) after 3 attempts each: {shown}"
                       f"{' ...' if len(unreadable) > 3 else ''}. These topics "
                       f"are UNCHECKED, which is not the same as compatible")
    if bad == 0 and checked:
        rep.note("qos", f"{checked}/{len(rx)} relay topic(s) inspected and "
                        f"QoS-compatible")


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


def gate_relay_set(robots, rep, present):
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
    # Uses the caller's listing so this gate and gate_qos judge the same
    # observed graph. (notes: gates-relay-set-shared-listing)
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


# Counters both link gates read with a default of 0: drop_overflow (overflow)
# and drop_disconnected (outage). stats_schema_problem asserts they exist so a
# renamed field cannot read as a clean zero. (notes: gates-required-link-keys)
REQUIRED_LINK_KEYS = ("drop_overflow", "drop_disconnected")


def stats_schema_problem(s):
    """Why this stats message cannot decide the link gates, or None if it can.

    THE DENOMINATOR IS THE POINT. An empty `links` array is not "no overflow",
    it is "nothing was examined", and the two used to print the same PASS line
    (`0 link(s), no overflow`). The emulator publishing a well-formed message
    that describes no links at all is a real state — a mis-parsed roster, a
    robot list that never expanded — and it is indistinguishable, in the report,
    from a healthy two-robot run whose queues never overflowed.
    """
    links = s.get("links")
    if links is None:
        return ("the stats message carries no `links` field at all — the "
                "emulator's schema changed and neither the overflow gate nor "
                "the outage gate has anything to read")
    if not links:
        return ("the stats message describes ZERO links. Nothing was examined, "
                "which is not the same as nothing being wrong: with no link "
                "the overflow counter cannot trip and the outage counter "
                "cannot rise, so both gates would pass on an empty denominator")
    missing = {}
    for link in links:
        for k in REQUIRED_LINK_KEYS:
            if k not in link:
                missing.setdefault(k, []).append(
                    f"{link.get('from', '?')}->{link.get('to', '?')}")
    if missing:
        return ("the stats links are missing counter(s) the gates read as "
                "zero: " + "; ".join(
                    f"{k} absent on {len(v)} link(s) ({', '.join(v[:3])})"
                    for k, v in sorted(missing.items())))
    return None


def gate_overflow(rep, stats=None):
    """drop_overflow > 0 on any link invalidates the run.

    The reliable queue evicts oldest at 64 MiB per directional link. Those are
    map deltas, and a dropped delta permanently holes the receiver's merged map:
    the receiver cannot detect the gap, and total_observed_voxels — the primary
    endpoint's input — is quietly wrong for the rest of the run.

    Returns the stats dict on success, and None when the run cannot be judged
    from it — INCLUDING when the message arrived but says nothing usable. The
    caller must treat None as "no reading", never as "a reading of zero": the
    watch loop's `outage_seen(stats)` reads the same dict, so a schema break
    here is a schema break there.
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
    # Checked before the loop: an empty links list or a missing counter would
    # otherwise fall through to the no-overflow note. Returning None also
    # withholds the dict from the outage gate.
    # (notes: gates-overflow-schema-first)
    problem = stats_schema_problem(s)
    if problem is not None:
        rep.unrunnable("overflow",
                       f"/hmr_comms_sim/stats arrived but cannot be read: "
                       f"{problem} — overflow NOT checked, and the outage gate "
                       f"reads the same message")
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
    """True if any link has ever refused traffic for being disconnected.

    A False here means "this message showed no outage", NOT "this message was
    readable and showed no outage" — an empty links list and a link dict with
    no drop_disconnected key both return False. Callers must gate on
    `stats_schema_problem` (gate_overflow does, and withholds the dict when it
    fails) before letting a False accumulate into `ever_seen`.
    """
    if not stats:
        return False
    return any(link.get("drop_disconnected", 0) > 0
               for link in stats.get("links", []))


def gate_outage_occurred(rep, ever_seen, polls):
    """A COMMS=1 run in which NO link ever dropped is a control run.

    Nothing else in the stack checks this. The radio model is monotone in
    separation and tree count, so if the robots simply never got far enough
    apart, the emulator relays everything, no peer ever reads MISSING, no
    reconnect manoeuvre fires, and the result is a complete, plausible dataset
    in which the treatment arm and the control arm are the same experiment.
    drop_disconnected is the direct evidence: it increments once per message
    refused because the link was down.

    IT IS AN ANY(), OVER ALL N*(N-1) DIRECTIONAL LINKS, and until 2026-09-18
    this docstring said "the two robots" as though there were one pair. At N=2
    the two readings coincide; at N=3 there are 6 links and at N=4 there are 12,
    and ONE of them dropping once passes the gate for the whole run. So a PASS
    means "the independent variable was varied SOMEWHERE in the fleet", not
    "every pair was exercised" and not "this robot was ever isolated" — an N=4
    run where robot 3 drifted off alone and robots 0-2 stayed in a tight
    triangle all run passes exactly like one where everybody separated. The
    all-pairs statement is not recoverable from here; it is in link_states.csv,
    which is per-link. Read it there before claiming a pair was exercised.

    The FAIL direction is the sound one and is the direction that matters: zero
    drops across every link really does mean nothing was varied anywhere.
    """
    if ever_seen:
        rep.note("outage", "link outage(s) observed (drop_disconnected > 0)")
        return True
    # Zero polls means nobody looked (watcher killed early or every read
    # failed): UNRUNNABLE, not a FAIL that blames the radio.
    # (notes: gates-outage-zero-polls)
    if polls <= 0:
        rep.unrunnable("outage",
                       "NOBODY LOOKED: 0 usable poll(s) of "
                       "/hmr_comms_sim/stats for the whole run, so whether the "
                       "link ever dropped is unknown — this says nothing about "
                       "the radio. Check that the emulator stayed up and that "
                       "the watcher was not killed at t=0")
        return False
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
    # Extra gated suffixes for streams only some runs carry (e.g. TeamWorld, off
    # unless team_world_hz > 0, or gen 34's team beacon). The caller that turns
    # a stream on is the one that passes it here. (notes: gates-gated-extra)
    ap.add_argument("--gated-extra", default="",
                    help="comma-separated extra topic suffixes to treat as "
                         "must-cross-the-emulator (e.g. "
                         "exploration/team_world when team_world_hz > 0, "
                         "exploration/team_beacon for the gen-34 node)")
    # no is for a control arm run through the emulator at a tx_power_dbm where
    # the link should stay up: the outage gate still runs and reports, but
    # cannot fail the run. (notes: gates-expect-outage-control)
    ap.add_argument("--expect-outage", choices=["yes", "no"], default="yes",
                    help="watch mode: 'no' for a control arm run at a "
                         "tx_power_dbm where the link is meant to stay up")
    args = ap.parse_args()

    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    allow = DEFAULT_ALLOW + [a.strip() for a in args.allow.split(",") if a.strip()]
    # Mutated in place, deliberately: gate_leakage and expected_relays both read
    # the module global, and threading the list through them instead would leave
    # two call sites that could be updated apart. Every gate must see the same
    # set or a topic could be required to exist and not checked for leaks.
    for s in (x.strip().strip("/") for x in args.gated_extra.split(",")):
        if s and s not in GATED_SUFFIXES:
            GATED_SUFFIXES.append(s)
    rep = Report(args.report)

    # Read the graph ONCE, here, and hand it to the gates that need it.
    present, list_err = topic_list()
    if list_err is not None:
        # UNRUN, not FAIL: the graph could not be read, so nothing may be
        # asserted about the radio. The caller only tests rc == 0; downstream
        # this banks SUSPECT instead of INVALID. (notes: gates-topic-list-unrun)
        rep.unrunnable("setup",
                       f"`ros2 topic list` did not run ({list_err}) after 3 "
                       f"attempts — the ROS graph could not be read, so no gate "
                       f"below could be evaluated. This is NOT a finding about "
                       f"the run; a stale ros2 daemon does exactly this")
        rep.flush()
        return 2
    if not present:
        rep.fail("setup", "`ros2 topic list` returned nothing — no ROS graph "
                          "visible (wrong ROS_DOMAIN_ID, or nothing running)")
        rep.flush()
        return 2

    if args.mode == "check":
        gate_leakage(robots, allow, rep)
        gate_relay_set(robots, rep, present)
        gate_qos(robots, rep, present)
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
    # polls counts loop iterations; usable_polls counts polls where
    # gate_overflow returned a dict (stats arrived and passed
    # stats_schema_problem). The outage gate is judged on usable_polls.
    # (notes: gates-usable-polls)
    usable_polls = 0
    ever_outage = False
    # Register SIGINT explicitly: as a background job of a non-interactive shell
    # this process inherits SIGINT as ignored, so KeyboardInterrupt never fires
    # and the summary is never written. (notes: gates-watch-sigint)
    signal.signal(signal.SIGINT, _raise_stop)
    signal.signal(signal.SIGTERM, _raise_stop)
    try:
        while True:
            time.sleep(args.period)
            polls += 1
            r2 = Report(args.report)
            stats = gate_overflow(r2)
            if stats is not None:
                usable_polls += 1
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
    tripped = watch_summary(summary, args.expect_outage, ever_outage,
                            polls, usable_polls, tripped)
    summary.flush()
    print(f"[gates] stopped; polls={polls} tripped={tripped}", flush=True)
    return 1 if tripped else 0


def watch_summary(summary, expect_outage, ever_outage, polls, usable_polls,
                  tripped):
    """The watcher's closing verdict. Returns the final `tripped`.

    A FUNCTION AND NOT A TAIL OF main() FOR ONE REASON: everything decided here
    is decided from five scalars, and while it sat inline no known-answer case
    could reach it without a ROS graph, an emulator and a real run. That is the
    condition every inert guard in `checks-that-stopped-checking` was in. See
    comms_gates_calib.py — the zero-poll branches below are the ones that used
    to read as all-clears, so they are the ones that must stay provably live.
    """
    if expect_outage == "yes":
        # Trip on a new failure, not on the False return: gate_outage_occurred
        # returns False both when the link never dropped and when nobody looked,
        # and only the first is a failure. (notes: gates-trip-on-failure)
        before = len(summary.failures)
        gate_outage_occurred(summary, ever_outage, usable_polls)
        if len(summary.failures) > before:
            tripped = True
    elif ever_outage:
        # Not a failure, but it means the "control" was degraded, so the
        # unknown floor and makespan it produces are not a clean reference.
        summary.note("outage",
                     "link outage(s) observed in a run declared outage-free "
                     "(--expect-outage no) — tx_power_dbm may be too low for a "
                     "control arm; the floor/makespan from this run are not a "
                     "clean reference")
    elif usable_polls <= 0:
        # Zero usable polls on a control arm is no reading, not the expected
        # no-outage reading, so it is UNRUNNABLE.
        # (notes: gates-control-zero-polls)
        summary.unrunnable("outage",
                           "0 usable poll(s) of /hmr_comms_sim/stats, so the "
                           "control arm's link was never actually observed to "
                           "stay up — this is not the expected reading, it is "
                           "no reading")
    else:
        summary.note("outage",
                     f"no link outage in {usable_polls} usable poll(s), as "
                     f"expected for a control arm (--expect-outage no)")
    if tripped:
        summary.fail("watch", f"gates tripped during the run ({polls} polls)")
    elif polls <= 0:
        # The teardown treats a comms_gates.txt with no watch line as SUSPECT.
        # On zero polls write an UNRUN watch line: the teardown finds it and
        # still lands SUSPECT, never an all-clear.
        # (notes: gates-watch-line-teardown)
        summary.unrunnable("watch",
                           "the watcher completed 0 poll(s) — it was killed or "
                           "died before its first period elapsed, so NO gate "
                           "supervised this run at run time. Not an all-clear")
    else:
        summary.note("watch",
                     f"{polls} polls ({usable_polls} with readable stats), "
                     f"no gate tripped")
    return tripped


if __name__ == "__main__":
    sys.exit(main())
