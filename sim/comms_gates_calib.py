#!/usr/bin/env python3
# Moved comments: docs/sim_notes/comms_gates_calib_notes.md
"""Calibrates the link gates in comms_gates.py against known answers.

THE DEFECT CLASS THIS FILE EXISTS FOR IS THE EMPTY DENOMINATOR. Every gate here
decides a run by counting something and finding zero, and until 2026-09-18 not
one of them could tell "I looked and there was nothing wrong" apart from "there
was nothing to look at":

  * gate_overflow looped over `links` and, on an empty list, printed the PASS
    `0 link(s), no overflow`;
  * both counters were read with `.get(name, 0)`, so a renamed field read as a
    clean zero on every link of every run — forever, silently;
  * gate_outage_occurred blamed the radio ("check separation vs tx_power_dbm")
    for a FAIL it reached with `polls == 0`, i.e. for the watcher having died;
  * the watch summary wrote `0 polls, no gate tripped`, which satisfies the
    teardown's "is there a `watch` line" check — so a watcher killed at t=0
    banked the cell CLEAN on the strength of a sentence meaning the opposite.

These are the same shape as the six guards in `checks-that-stopped-checking`:
each kept printing a PASS after it stopped being able to fail. So each new
refusal gets a known-answer case here, and — the half that matters more — so
does the healthy reading beside it, because a guard that refuses EVERYTHING is
just as inert as one that refuses nothing, and is much easier to ship.

The gates are driven directly as functions against hand-built stats dicts. No
ROS graph, no emulator, no run directory: `read_stats` is the only thing that
touches the outside world, and every gate under test takes the parsed dict as
an argument precisely so it can be calibrated without one.

Usage:
    ./comms_gates_calib.py
Exit status 0 = every case matched its known answer, 1 = at least one did not.
"""

import sys

import comms_gates as cg


def link(a="r1", b="r2", **kw):
    """A well-formed stats link. Keyword args override or, with None, DELETE.

    Deleting is the whole reason this helper exists rather than a literal: the
    schema break being calibrated is a field going ABSENT, and a fixture that
    can only set fields to zero cannot express it — which is exactly how the
    real `.get(name, 0)` defect stayed invisible.
    """
    d = {"from": a, "to": b, "drop_overflow": 0, "drop_disconnected": 0,
         "delivered": 10}
    for k, v in kw.items():
        if v is None:
            d.pop(k, None)
        else:
            d[k] = v
    return d


def stats(*links, **kw):
    d = {"links": list(links), "t_sec": 12.0}
    for k, v in kw.items():
        if v is None:
            d.pop(k, None)
        else:
            d[k] = v
    return d


HEALTHY = stats(link("r1", "r2"), link("r2", "r1"))
OVERFLOWED = stats(link("r1", "r2", drop_overflow=7), link("r2", "r1"))
DROPPED = stats(link("r1", "r2", drop_disconnected=31), link("r2", "r1"))


# --- the cases -------------------------------------------------------------
# Each case carries an expected verdict and a substring its message must
# contain, so a gate that refuses for an unrelated reason does not pass.
# (notes: gates-calib-case-substring)

def run_overflow(s):
    rep = cg.Report("")
    ret = cg.gate_overflow(rep, stats=s)
    return rep, ret


def verdict(rep):
    if rep.failures:
        return "FAIL", rep.failures[0][1]
    if rep.unrunnables:
        return "UNRUN", rep.unrunnables[0][1]
    if rep.notes:
        return "PASS", rep.notes[0][1]
    return "SILENT", ""


CASES = []


def case(name, want, want_sub, fn):
    CASES.append((name, want, want_sub, fn))


# --- gate_overflow: the healthy readings it must still accept ---------------

case("a clean two-link run still passes", "PASS", "2 link(s), no overflow",
     lambda: verdict(run_overflow(HEALTHY)[0]))

case("a real overflow still fails", "FAIL", "drop_overflow=7",
     lambda: verdict(run_overflow(OVERFLOWED)[0]))

case("a dropped link is not an overflow", "PASS", "no overflow",
     lambda: verdict(run_overflow(DROPPED)[0]))

# --- gate_overflow: the empty denominators ---------------------------------

case("zero links is not zero overflow", "UNRUN", "describes ZERO links",
     lambda: verdict(run_overflow(stats())[0]))

case("no links field at all is refused", "UNRUN", "no `links` field",
     lambda: verdict(run_overflow(stats(link(), links=None))[0]))

case("a renamed drop_overflow is refused", "UNRUN",
     "drop_overflow absent on 1 link(s)",
     lambda: verdict(run_overflow(
         stats(link("r1", "r2", drop_overflow=None), link("r2", "r1")))[0]))

case("a renamed drop_disconnected is refused by the SAME gate", "UNRUN",
     "drop_disconnected absent on 2 link(s)",
     lambda: verdict(run_overflow(
         stats(link("r1", "r2", drop_disconnected=None),
               link("r2", "r1", drop_disconnected=None)))[0]))

case("the refusal names the outage gate as collateral", "UNRUN",
     "the outage gate reads the same message",
     lambda: verdict(run_overflow(stats())[0]))

# THE DICT IS WITHHELD, NOT JUST REPORTED ON. gate_overflow's return value is
# what the watch loop counts as a usable poll and hands to outage_seen, so a
# schema break that was reported but still returned would keep feeding the
# outage gate the very message this one just declared unreadable.

case("an unreadable message is withheld from the caller", "withheld",
     "withheld",
     lambda: ("withheld", "withheld") if run_overflow(stats())[1] is None
     else ("returned", "the dict was handed on anyway"))

case("a readable message IS returned to the caller", "returned", "returned",
     lambda: ("returned", "returned") if run_overflow(HEALTHY)[1] is HEALTHY
     else ("withheld", "a healthy message was withheld"))

# --- outage_seen -----------------------------------------------------------

case("outage_seen sees a real drop", "yes", "yes",
     lambda: ("yes", "yes") if cg.outage_seen(DROPPED) else ("no", "missed it"))

case("outage_seen is False on a healthy link", "no", "no",
     lambda: ("no", "no") if not cg.outage_seen(HEALTHY) else ("yes", "spurious"))

case("outage_seen is False on None (and must not be trusted alone)",
     "no", "no",
     lambda: ("no", "no") if not cg.outage_seen(None) else ("yes", "spurious"))


# --- gate_outage_occurred --------------------------------------------------

def run_outage(ever, polls):
    rep = cg.Report("")
    cg.gate_outage_occurred(rep, ever, polls)
    return verdict(rep)


case("an observed outage passes", "PASS", "outage(s) observed",
     lambda: run_outage(True, 40))

case("no outage over real polls still FAILS the arm", "FAIL",
     "NO link outage in 40 poll(s)",
     lambda: run_outage(False, 40))

case("no outage over ZERO polls is unrunnable, not a failed link", "UNRUN",
     "NOBODY LOOKED",
     lambda: run_outage(False, 0))

case("the zero-poll refusal does not send the reader to tx_power_dbm",
     "clean", "clean",
     lambda: ("clean", "clean")
     if "tx_power_dbm" not in run_outage(False, 0)[1]
     else ("misdirected", "blames the radio for a dead watcher"))

case("a negative poll count cannot reach the FAIL text", "UNRUN",
     "NOBODY LOOKED",
     lambda: run_outage(False, -1))


# --- watch_summary ---------------------------------------------------------
# run_explo_sim_rviz.sh treats a comms_gates.txt with no line whose second
# field is watch as SUSPECT. PASS clears the run; UNRUN keeps the token but
# lands SUSPECT. Both are asserted below. (notes: gates-calib-watch-token)

def run_watch(expect, ever, polls, usable, tripped_in=False):
    rep = cg.Report("")
    out = cg.watch_summary(rep, expect, ever, polls, usable, tripped_in)
    lines = ([("FAIL", g, m) for g, m in rep.failures]
             + [("UNRUN", g, m) for g, m in rep.unrunnables]
             + [("PASS", g, m) for g, m in rep.notes])
    return out, lines


def watch_line(expect, ever, polls, usable, tripped_in=False):
    _, lines = run_watch(expect, ever, polls, usable, tripped_in)
    for tok, gate, msg in lines:
        if gate == "watch":
            return tok, msg
    return "ABSENT", "no `watch` line at all — the teardown reads this as SUSPECT"


def outage_line(expect, ever, polls, usable):
    _, lines = run_watch(expect, ever, polls, usable)
    for tok, gate, msg in lines:
        if gate == "outage":
            return tok, msg
    return "ABSENT", "no `outage` line"


case("a healthy treated run closes PASS/watch", "PASS", "no gate tripped",
     lambda: watch_line("yes", True, 40, 40))

case("the healthy watch line reports both counts", "PASS",
     "40 polls (38 with readable stats)",
     lambda: watch_line("yes", True, 40, 38))

case("a tripped run closes FAIL/watch", "FAIL", "gates tripped during the run",
     lambda: watch_line("yes", True, 40, 40, tripped_in=True))

case("a watcher that never polled is UNRUN, not an all-clear", "UNRUN",
     "completed 0 poll(s)",
     lambda: watch_line("yes", False, 0, 0))

case("the zero-poll watch line still carries the `watch` token so the "
     "teardown finds it", "UNRUN", "Not an all-clear",
     lambda: watch_line("yes", False, 0, 0))

case("a treated run with no outage trips the watcher", "trips", "trips",
     lambda: ("trips", "trips") if run_watch("yes", False, 40, 40)[0]
     else ("clears", "a control-in-disguise closed clean"))

case("zero usable polls does NOT trip — it refuses", "clears", "clears",
     lambda: ("clears", "clears") if not run_watch("yes", False, 0, 0)[0]
     else ("trips", "a dead watcher was reported as a tripped gate"))

# The control arm (--expect-outage no), where "no outage" is the EXPECTED
# reading and therefore where an unreadable run is hardest to tell from a
# healthy one.

case("a control arm with a quiet link passes", "PASS",
     "no link outage in 40 usable poll(s)",
     lambda: outage_line("no", False, 40, 40))

case("a control arm that dropped is noted, not failed", "PASS",
     "declared outage-free",
     lambda: outage_line("no", True, 40, 40))

case("a control arm with no usable polls claims nothing", "UNRUN",
     "it is no reading",
     lambda: outage_line("no", False, 40, 0))

case("polls without readable stats are not counted as evidence", "UNRUN",
     "0 usable poll(s)",
     lambda: outage_line("yes", False, 240, 0))


def main():
    bad = []
    for name, want, want_sub, fn in CASES:
        try:
            got, msg = fn()
        except Exception as e:                       # noqa: BLE001
            got, msg = "EXCEPTION", f"{type(e).__name__}: {e}"
        if got != want:
            bad.append((name, f"want {want} got {got}", msg))
            mark = f"FAIL  want {want} got {got}"
        elif want_sub not in msg:
            bad.append((name, f"{got}, but not on {want_sub!r}", msg))
            mark = f"FAIL  {got}, wrong reason"
        else:
            mark = f"ok    {got}"
        print(f"  {mark:<28}{name}")

    if bad:
        print()
        for name, why, msg in bad:
            print(f"--- {name}: {why}")
            print(f"    got: {msg}")
        print(f"\n{len(bad)} FAILURE(S) of {len(CASES)} known-answer cases")
        return 1
    print(f"\nALL PASS ({len(CASES)} known-answer cases)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
