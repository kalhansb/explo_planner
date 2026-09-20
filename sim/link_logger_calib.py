#!/usr/bin/env python3
"""Known-answer calibration for link_logger.resolve_stride().

Same discipline as team_convergence_calib.py and for the same reason: this
repo has watched guards go inert while still printing passes
([[checks-that-stopped-checking]]). resolve_stride is exactly that kind of
guard -- link_logger.py's header comment says the column count is read off the
message layout rather than assumed because a half-built workspace would
otherwise write "a CSV whose columns are silently shifted by one every row --
the one failure mode worth code to avoid". A guard written for that has to be
tested against payloads that can be read two ways, not just well-formed ones.

WHAT THIS CAUGHT (2026-09-18). The original resolve_stride tried candidates in
the order (declared, 10, 9) and took the first that divided the payload. Two
consequences it never stated:

  * A payload that divides by BOTH widths -- any multiple of 90, e.g. 10 legacy
    pairs of 9 columns, which is N=5 -- resolved to 10 by falling through to
    the preference order. That is not a reading of the payload, it is the
    assumption the function exists to avoid making, restored silently.
  * A declared width this script does not know (a future generation) was
    IGNORED rather than refused: the loop skipped it and guessed 10, i.e. read
    the payload at a width its publisher had just said it was not using.

And the fix's own first draft crashed: `declared` is publisher-supplied and 0
is its default-constructed value, so the remainder in the error message was a
division by zero -- inside the subscription callback, which would have taken
the logging node down rather than skipped one message. That case is below.

resolve_stride touches only self.get_logger(), so it is called unbound here
against a stub. This exercises the REAL function from the real file; a
reimplementation of its logic would calibrate nothing.

Usage:  link_logger_calib.py
Exit:   0 every case behaved as declared
        1 at least one did not
"""

import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "link_logger.py")

_spec = importlib.util.spec_from_file_location("link_logger", SRC)
link_logger = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(link_logger)

FIELDS = link_logger.FIELDS
FIELDS_LEGACY = link_logger.FIELDS_LEGACY


class _Log:
    def __init__(self, sink):
        self.sink = sink

    def error(self, m):
        self.sink.append(("error", m))

    def info(self, m):
        self.sink.append(("info", m))

    def warn(self, m):
        self.sink.append(("warn", m))


class _Stub:
    """Just enough LinkLogger for resolve_stride to run."""

    def __init__(self):
        self.msgs = []

    def get_logger(self):
        return _Log(self.msgs)


class _Dim:
    def __init__(self, size):
        self.size = size


class _Msg:
    def __init__(self, n, declared=None):
        self.data = [0.0] * n
        self.layout = type("L", (), {})()
        self.layout.dim = [] if declared is None else [_Dim(0), _Dim(declared)]


fails = 0


def case(label, n, declared, want_stride, want_log=None, want_silent=False):
    """want_log is asserted on the CONCATENATED log, not on stdout: a gate that
    returns the right answer for a reason it no longer states has still
    changed, and the next reader of these messages is a human debugging a
    misaligned CSV at 2am."""
    global fails
    stub = _Stub()
    got = link_logger.LinkLogger.resolve_stride(stub, _Msg(n, declared))
    logged = " | ".join(m for _lvl, m in stub.msgs)
    if got != want_stride:
        print("FAIL %-50s stride=%r want=%r  log=%s"
              % (label, got, want_stride, logged or "(silent)"))
        fails += 1
        return
    if want_silent and stub.msgs:
        print("FAIL %-50s expected no log, got: %s" % (label, logged))
        fails += 1
        return
    if want_log is not None and want_log not in logged:
        print("FAIL %-50s right answer, wrong reason: wanted %r, log=%s"
              % (label, want_log, logged or "(silent)"))
        fails += 1
        return
    print("ok   %-50s -> %r" % (label, got))


# --- 1. the declaration is authoritative when this script can use it.
# hmr_comms_sim_node.cpp sets layout.dim[1].size = kLinkStateCols, so this is
# the path every real cell takes.
case("N=2 (1 pair) x 10, declared", 10, FIELDS, FIELDS)
case("N=3 (3 pairs) x 10, declared", 30, FIELDS, FIELDS)
case("N=4 (6 pairs) x 10, declared", 60, FIELDS, FIELDS)
case("a legacy emulator, declared 9", 54, FIELDS_LEGACY, FIELDS_LEGACY,
     "half-built")

# --- 2. the ambiguous payload, RESOLVED by the declaration. This is the whole
# point of publishing dim[1].size and BOTH readings of the same 90 values have
# to be honoured, or the declaration is decoration.
case("90 values declared 9  -> 10 legacy pairs", 90, FIELDS_LEGACY,
     FIELDS_LEGACY)
case("90 values declared 10 -> 9 modern pairs", 90, FIELDS, FIELDS)

# --- 3. a declaration this script cannot use is a REFUSAL, not a fallback.
case("declared 11 (a generation we don't know)", 66, 11, None,
     "did not declare")
case("declared 10 but 65 values (layout lies)", 65, FIELDS, None,
     "remainder of 5")
# The crash case. 0 is Float64MultiArray's default-constructed dim size, so an
# unset layout reaches the error path as a zero divisor.
case("declared 0 (dim present, size unset)", 60, 0, None, "not one of them")

# --- 4. with no declaration, divisibility -- but only where it is decisive.
case("no layout, 60 values (only 10 fits)", 60, None, FIELDS)
case("no layout, 54 values (only 9 fits)", 54, None, FIELDS_LEGACY,
     "half-built")
case("no layout, 61 values (neither fits)", 61, None, None,
     "not a whole number")

# --- 5. THE CASE THAT MUST REFUSE. A tie is missing information, not a vote
# for the newer width. If this one ever starts returning 10 again, the guard
# has gone back to assuming the workspace is not half-built.
case("no layout, 90 values (BOTH fit)", 90, None, None,
     "divides evenly by BOTH")
case("no layout, 180 values (BOTH fit)", 180, None, None,
     "divides evenly by BOTH")

# --- 6. an empty payload decides nothing and says nothing. Every candidate
# divides zero, so a tie-refusal here would be both wrong and once-per-message
# noisy; returning None leaves self.stride unset so the NEXT message decides.
case("empty payload, no layout", 0, None, None, want_silent=True)
case("empty payload, declared 10", 0, FIELDS, None, want_silent=True)

print()
if fails:
    print("%d case(s) did not behave as declared" % fails)
    sys.exit(1)
print("ALL PASS")
