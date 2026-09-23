#!/usr/bin/env python3
# Moved comments: docs/sim_notes/gate_g8_notes.md
"""Read a generation-8 campaign against the pre-registered gate.

Descends from gate_g6.py, which no longer exists in this tree -- the lineage is
history, not a file to consult. Checks 1-14 are carried over unchanged in meaning;
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

Usage:  gate_g8.py [TAG]            default TAG g8r1
Env:    GATE_ROOT                   campaign root (default /home/kalhan/hmr_campaign)
        GATE_IDENTITY               path to the declared-identity file
        GATE_EXPECT_<key>           override a single declared identity field
        GATE_CELLS_PER_ARM          pre-registered cells per arm (default 30)
        GATE_ARMS                   pre-registered arms, comma-separated
                                    (default "hybrid,off")
        GATE_CONTROL_ARMS           arms that run with the manoeuvre disabled
                                    (default "off")
        GATE_SCHEMA_VERSION         event-log schema to require (default 11;
                                    pass the banked generation's own number to
                                    re-gate an older campaign, e.g. 5 for a
                                    gen-16 one or 3 for a gen-9 one)
Exit:   0  no hard failures, every population non-empty
        1  hard failures, or no cells for the tag
        2  the generation identity was never declared
        3  no hard failures, but at least one population was UNRESOLVED

Exit 3 exists because property 2 above was true of the printed output and
false of the exit status: a wrapper branching on $? read "nothing was tested"
as "everything passed", which is the exact failure this gate exists to catch.
"""
import collections
import csv
import glob
import json
import os
import re
import sys

# Overridable so gate_g8_calib.py can point the REAL logic at a synthetic
# campaign of known-wrong cells. The identity in EXPECT is deliberately NOT
# overridable this way — see the note there.
ROOT = os.environ.get("GATE_ROOT", "/home/kalhan/hmr_campaign")
if len(sys.argv) > 2 or (len(sys.argv) == 2 and sys.argv[1].startswith("-")):
    # Not argparse, because the only positional is a tag and the rest of the
    # interface is environment. But silently scoring a campaign tagged "--help"
    # (which finds zero cells and exits 1) is a confusing way to answer a
    # request for help, and silently ignoring argv[2:] is worse.
    print(__doc__)
    sys.exit(0 if sys.argv[1] in ("-h", "--help") else 2)
TAG = sys.argv[1] if len(sys.argv) > 1 else "g8r1"
# The two-robot default, kept ONLY as the last resort for a cell whose manifest
# names no team. Every real cell now gets its roster from robots_of() below;
# this constant is what a pre-team_robot_names cell falls back to, and it is
# reported when it is used rather than applied silently.
ROBOTS_FALLBACK = ["atlas", "bestla"]

# Gates in comms_gates.txt that are readings with no pass threshold. Used by
# check 2 only: their UNRUN lines alone do not hard-fail a cell. Add a name only
# if that gate's own header says it has no pass criterion.
# (notes: gate-report-only-gates)
REPORT_ONLY_GATES = frozenset({"map_agree"})


def robots_of(m, d):
    """This cell's roster, from its own manifest.

    Returns (names, source). The gate used to iterate a module-level
    ROBOTS = ["atlas", "bestla"] over every cell of every campaign, which is
    wrong in both directions at N>=3: husky and skadi were never opened, so
    checks 3b/3c/3e/3f/3g/18b/18c ran on two thirds of a three-robot team and
    reported a pass over the third, and a campaign that renamed its robots
    would hard-fail "no events" on cells that were perfectly good.

    `robots=` is written by run_campaign.sh from the value actually launched, so
    it is the witness. `team_robot_names` is the same list as the node saw it
    and is used as the cross-check; a disagreement between the two is reported
    by check 3k rather than resolved here.
    """
    raw = m.get("robots")
    if raw:
        names = [t.strip() for t in raw.split(",") if t.strip()]
        if names:
            return names, "manifest robots="
    # Older manifests wrote only the node-side list, as a JSON array.
    raw = m.get("team_robot_names")
    if raw:
        try:
            names = [str(t) for t in json.loads(raw)]
        except ValueError:
            names = [t.strip().strip('"[] ') for t in raw.split(",")]
        names = [n for n in names if n]
        if names:
            return names, "manifest team_robot_names="
    # Last resort, used only when the manifest names no team: the event logs on
    # disk. Never the primary source, since a robot whose log is missing would
    # silently drop out of the expectation.
    # (notes: gate-roster-from-logs-last-resort)
    found = sorted(os.path.basename(q)[:-len(".events.jsonl")]
                   for q in glob.glob(os.path.join(d, "*.events.jsonl")))
    if found:
        return found, "event logs on disk (NOT a manifest expectation)"
    return list(ROBOTS_FALLBACK), "hardcoded fallback"


# Cell-directory suffixes the binary never stamps; compare run_start against the
# stripped form. RUNTIME suffixes are pooled out of arm; DESIGN suffixes are
# stripped for policy_arm only, kept in arm, and checked by 3i.
# (notes: gate-runtime-vs-design-suffixes)
RUNTIME_SUFFIXES = ("_seek",)
# (regex on the suffix, manifest key it must agree with, human name, unit)
# The unit is carried here rather than written into the message, because one
# message now serves every dimension and "TTL 120 m" is worse than no unit at
# all.
DESIGN_SUFFIXES = (
    (re.compile(r"^(.*)_r([0-9]{1,3})$"), "coord_claim_radius_override",
     "claim radius", "m"),
    (re.compile(r"^(.*)_ttl([0-9]{1,4})$"), "alloc_peer_pos_max_age_sec",
     "allocator peer-position TTL", "s"),
)


def parse_cell_name(cell, tag):
    """Split a cell directory name into the three things the gate needs.

    Returns (arm, policy_arm, claims, leftover) where

      arm        the CELL's identity: what to count, what GATE_ARMS must have
                 declared, which column of the design this is.
      policy_arm what the BINARY can stamp: the reconnect policy alone, every
                 suffix removed. Anything compared against run_start or against
                 the m-tare vocabulary uses this.
      claims     {manifest_key: (value_as_float, human_name, unit)} for each design
                 suffix present, to be checked against the manifest.
      leftover   the arm with every RECOGNISED suffix stripped, for the caller
                 to sanity-check. Equal to policy_arm; returned separately so
                 the caller need not re-derive it.

    ONE FUNCTION, called once, because the previous inline version stripped
    _seek and then matched _r<N> with a single anchored regex -- so a name
    carrying BOTH a radius and a TTL, which is every ts1b cell
    (`ts1b_n3_mtare_hybrid_r40_ttl0_seed1`), matched neither. The consequences
    were not subtle: policy_arm came out as the whole middle token, check 3e
    hard-failed "directory says mtare_hybrid_r40_ttl0 but params say
    mtare_hybrid" on every cell, 3g declared the arm unstampable by any binary,
    3i hard-failed "directory names no claim radius" against a manifest that
    recorded 40.0, and check 21 found zero cells in either declared arm. 76 hard
    failures on a campaign with nothing wrong with it. Stripping in a LOOP until
    nothing matches makes suffix ORDER irrelevant, which is the property the
    single-pass version lacked.
    """
    mid = cell[len(tag) + 1:] if cell.startswith(tag + "_") else cell
    arm = mid.rsplit("_seed", 1)[0] if "_seed" in mid else mid
    for suf in RUNTIME_SUFFIXES:
        while arm.endswith(suf):
            arm = arm[:-len(suf)]
    claims = {}
    policy_arm = arm
    changed = True
    while changed:
        changed = False
        for rx, key, human, unit in DESIGN_SUFFIXES:
            mm = rx.match(policy_arm)
            if mm:
                # First match wins per key: a name repeating a dimension is
                # malformed, and quietly keeping the inner one would file the
                # cell under a level it did not run.
                claims.setdefault(key, (float(mm.group(2)), human, unit))
                policy_arm = mm.group(1)
                changed = True
    return arm, policy_arm, claims, policy_arm


def unrecognised_suffix(dir_policy, stamped):
    """The one shape of 3e disagreement that is a GATE gap, not a run defect.

    Returns the leftover token when the directory's policy is exactly the
    binary's stamp plus a trailing `_<something>` this file does not know about,
    and None otherwise.

    The distinction matters because the two failures need opposite responses. A
    directory reading `mtare_off` against a stamp of `mtare_hybrid` is an
    assignment error and must stop the campaign. A directory reading
    `mtare_hybrid_ttl0` against a stamp of `mtare_hybrid` is a correctly-run
    cell carrying a knob added after this gate was last edited — the cells are
    fine and the TABLE above is out of date. Reporting the second as a
    provenance hard failure is how a gate teaches operators to ignore it.

    Deliberately one-directional: a stamp LONGER than the directory name is not
    a missing suffix, it is the binary claiming an arm the directory does not,
    and that stays a hard failure.
    """
    if not dir_policy or not stamped:
        return None
    if dir_policy == stamped or not dir_policy.startswith(stamped + "_"):
        return None
    return dir_policy[len(stamped) + 1:]


# ---------------------------------------------------------------------------
# Declared, never read from the working tree. The explo_planner rev and node
# sha256 cannot be literals (this file is in the commit they name); they come
# from TAG.identity.txt under GATE_ROOT, env overrides for calibration.
# (notes: gate-declared-identity-out-of-band)
# ---------------------------------------------------------------------------
# The simple_nav_3d binary that actually ran. Declared like the other four, but
# REQUIRED only where the campaign can answer -- see the self-calibrating block
# after cell enumeration for why it is not simply added to EXPECT above.
NAV_BIN_KEY = "sha256_simple_nav_planner_node"
# Its three siblings. They move as a unit with it, so a hash that moves for one
# and not the others is a partial install; that is checked per-cell and needs no
# declaration, because the defect is internal disagreement, not a wrong value.
NAV_BIN_SIBLINGS = ("sha256_simple_nav_costmap_node",
                    "sha256_simple_nav_controller_node",
                    "sha256_simple_nav_navigator_node")


def _declared_identity():
    """The build-dependent identity fields, from env or the identity file."""
    out = {}
    path = os.environ.get("GATE_IDENTITY",
                          os.path.join(ROOT, f"{TAG}.identity.txt"))
    if os.path.exists(path):
        for line in open(path):
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            # An empty right-hand side is not a declaration. Skipping it leaves
            # the key at FILL_ME, so the refusal below points at the truncated
            # identity file instead of every cell failing.
            # (notes: gate-identity-empty-value)
            if v.strip():
                out[k.strip()] = v.strip()
    # Env wins, so the calibration can override a real identity file if one
    # happens to exist in its synthetic root.
    for k in ("git_explo_planner", "git_simple_nav_3d", "git_scovox",
              "sha256_explo_planner_node", NAV_BIN_KEY):
        if os.environ.get(f"GATE_EXPECT_{k}"):
            out[k] = os.environ[f"GATE_EXPECT_{k}"]
    return out


_decl = _declared_identity()
# All four are declared, none is a literal. Do not hardcode a rev here: a pin
# only a source edit can move hard-fails check 3 on every cell of a later
# generation. (notes: gate-no-literal-revs)
EXPECT = {
    "git_explo_planner": _decl.get("git_explo_planner", "FILL_ME"),
    "git_simple_nav_3d": _decl.get("git_simple_nav_3d", "FILL_ME"),
    "git_scovox": _decl.get("git_scovox", "FILL_ME"),
    "sha256_explo_planner_node": _decl.get("sha256_explo_planner_node",
                                           "FILL_ME"),
}

# Deliberately NOT folded into EXPECT here. EXPECT's members are unconditional,
# and this one is conditional on the campaign carrying the key at all; it joins
# EXPECT below, once the cells have been read.
EXPECT_NAV = _decl.get(NAV_BIN_KEY)

# ==================================================================
# check 3l: the configuration that must not vary silently
# ==================================================================
# Sensor and candidate params, never a treatment, so variation means a stale
# yaml, a stray override or pooled runs. CFG_HARD: constant campaign-wide.
# CFG_SOFT: may differ between arms (reported), never within one (hard).
# (notes: gate-3l-config-hard-soft)
CFG_HARD = ("fov_hfov", "fov_vfov", "fov_h_rays", "fov_v_rays",
            "fov_min_range", "fov_max_range", "fov_is_omnidirectional",
            "candidate_n_yaw", "candidate_enable_polar")
CFG_SOFT = ("exploitation_enabled", "coord_claim_radius_m")
CFG_KEYS = CFG_HARD + CFG_SOFT


def cfg_val(v):
    """Canonical text for a config value, so FORMAT is never a finding.

    96 and 96.0 are the same ray count, and true and True are the same flag.
    Comparing json.dumps output directly would have reported both pairs as a
    campaign that mixes two sensor models -- an unsatisfiable hard failure on a
    correct campaign, which is the exact trap check 3 had for git_simple_nav_3d
    and the exact precedent the radio-regime check already records ("the harness
    writes 30.0, an older manifest may say 30, and they are the same radio").

    bool FIRST: in Python bool is a subclass of int, so a value-ordered test
    renders True as "1" and silently makes a flag indistinguishable from a
    count of one.
    """
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return repr(float(v))
    return json.dumps(v)

DONE_UNKNOWN_FRACTION = 0.640
# Event-log schema pin for check 3d. Re-gate a banked campaign by passing its
# own number in GATE_SCHEMA_VERSION, never by editing the pin:
# (notes: gate-schema-pin-override)
#
#   GATE_SCHEMA_VERSION=3 GATE_MIDRUN_SILENCE=240 GATE_GEN9_PARAMS=0 \
#       ./gate_g8.py g8r1
#
# The pin must move with the binary: check 3d is an exact equality.
# kSchemaVersion in experiment_log.hpp is the authority on what each version
# means; do not re-derive that list here. (notes: gate-schema-version-history)
SCHEMA_VERSION = int(os.environ.get("GATE_SCHEMA_VERSION", "11"))
# Check 3f treatment config. Every sub-check is overridable, so the check can be
# calibrated on a known-answer case; re-gating a banked campaign needs both
# overrides: (notes: gate-3f-treatment-overrides)
#
#   GATE_MIDRUN_SILENCE=240 GATE_GEN9_PARAMS=0 ./gate_g8.py g8r1
#
# which is the only invocation that scores an old campaign on its own terms.
MIDRUN_SILENCE_EXPECT = float(os.environ.get("GATE_MIDRUN_SILENCE", "90"))
LINK_DOWN_CONFIRM_EXPECT = float(
    os.environ.get("GATE_LINK_DOWN_CONFIRM", "0"))
# 0 relaxes the three fields generation 9 introduced — link_gate_configured,
# its runtime witness, and reconnect_link_down_confirm_sec. Nothing older emits
# them, so demanding them of a banked campaign tests the binary's age rather
# than the run's validity.
GEN9_PARAMS_REQUIRED = os.environ.get("GATE_GEN9_PARAMS", "1") != "0"
# The runtime half can be dropped on its own, for a campaign whose console logs
# were not kept. The param half stays on: it costs nothing to read.
LINK_GATE_LIVE_REQUIRED = (
    GEN9_PARAMS_REQUIRED
    and os.environ.get("GATE_LINK_GATE_LIVE", "1") != "0")

# check 3m: the ENDPOINT is declared exactly once
#
# At most one mission_complete row per robot-run.
# run_end.mission_completes_suppressed must be 0; mission_return_reentries may
# be nonzero. GATE_ENDPOINT_FIELDS off relaxes only the fields' existence.
# (notes: gate-3m-endpoint-once)
ENDPOINT_FIELDS_REQUIRED = os.environ.get("GATE_ENDPOINT_FIELDS", "1") != "0"


# Radio regime for check 3j: three manifest fields (trunk cost, clear-lane
# reach, tx power) behind every connectivity result. Re-score a banked campaign
# by declaring its regime in the invocation, never by editing the pin:
# (notes: gate-3j-radio-regime-pin)
#
#   GATE_TREE_ATTEN=11.98 GATE_MAX_RANGE=none ./gate_g8.py cr5
#
# "none" is how the absence of a key is declared, because absence is a VALUE
# here, not a gap: a manifest with no max_range_m line was written by a harness
# that had no radio horizon, and that is a fact about the run, not a missing
# measurement.
def _regime_val(raw):
    """One radio-regime field, normalised so 30 and 30.0 are one value.

    An unparseable value keeps its raw spelling rather than becoming 0.0 —
    float() on a truncated line would otherwise raise, and a permissive parse
    would silently make every non-number equal to every other non-number.
    """
    if raw is None:
        return "<absent>"
    try:
        return f"{float(raw):g}"
    except (TypeError, ValueError):
        return str(raw)


def _regime_expect(env_key, default):
    v = os.environ.get(env_key, default).strip()
    if v.lower() in ("", "none", "absent"):
        return "<absent>"
    return _regime_val(v)


COMMS_REGIME_KEYS = ("tree_attenuation_db", "max_range_m", "tx_power_dbm")
COMMS_REGIME_EXPECT = (
    _regime_expect("GATE_TREE_ATTEN", "70.0"),
    _regime_expect("GATE_MAX_RANGE", "30.0"),
    _regime_expect("GATE_TX_POWER", "30.0"),
)
# The UNIFORMITY half — one regime per campaign — has no escape hatch and is
# not meant to have one. Two regimes in one directory is not a campaign scored
# against the wrong pin, it is two experiments, and there is no invocation that
# makes that all right.
COMMS_REGIME_EXPECT_REQUIRED = os.environ.get("GATE_COMMS_REGIME", "1") != "0"

# The pre-registered shape of the campaign, checked so that a run which died
# part-way cannot be scored as if it were the whole thing. run_campaign.sh is
# seed-major, so an early abort is systematically arm-unbalanced rather than
# randomly so — the truncated dataset is biased, not merely small.
EXPECT_CELLS_PER_ARM = int(os.environ.get("GATE_CELLS_PER_ARM", "30"))
# The pre-registered arms, declared rather than inferred from the directories,
# so check 21 can notice a truncated campaign. Entries are stripped: a stray
# space would declare a phantom arm. (notes: gate-expect-arms-declared)
EXPECT_ARMS = tuple(
    a for a in (s.strip()
                for s in os.environ.get("GATE_ARMS", "hybrid,off").split(","))
    if a)

# Control arms run with reconnect_enabled=false; every other arm must have it
# enabled. The default is off alone: an allocator-on control such as mtare_off
# must be declared in GATE_CONTROL_ARMS at scoring time.
# (notes: gate-control-arms-default)
CONTROL_ARMS = frozenset(
    a for a in (s.strip()
                for s in os.environ.get("GATE_CONTROL_ARMS", "off").split(","))
    if a)

# A control arm must match an expected arm, or the reconnect policy of one, so
# suffixed designs need not repeat their suffixes. A token matching neither
# exempts nothing and aborts. (notes: gate-orphan-control-arms)
_expect_policies = {a: parse_cell_name(a, TAG)[1] for a in EXPECT_ARMS}
_control_targets = set(EXPECT_ARMS) | set(_expect_policies.values())
_orphan_controls = CONTROL_ARMS - _control_targets
if _orphan_controls:
    sys.stderr.write(
        f"FATAL: GATE_CONTROL_ARMS names {sorted(_orphan_controls)}, which "
        f"matches neither an arm in GATE_ARMS {list(EXPECT_ARMS)} nor the "
        f"reconnect policy of one ({sorted(set(_expect_policies.values()))}). "
        f"A control arm that is not an expected arm exempts nothing.\n")
    sys.exit(2)
if not (CONTROL_ARMS & _control_targets):
    sys.stderr.write(
        f"FATAL: none of the expected arms {list(EXPECT_ARMS)} is a control "
        f"arm. Every arm would be scored as treated and the campaign has no "
        f"baseline.\n")
    sys.exit(2)

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
    "gate says stay",             # P4 §3.6 value gate suppressed the dispatch
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


_LIVE_CACHE = {}
# The planner's stdout+stderr, per robot. run_explo_sim_rviz.sh's start() sends
# each node to "$OUTDIR/planner_$r.log"; $ROOT/<cell>.console.log holds only the
# two HARNESS scripts' own log() output and never a line the node emitted.
#
# Do not read the console log instead: it exists, so a missing token there reads
# as absent rather than unanswerable. (notes: gate-planner-log-not-console)
PLANNER_LOG = "planner_{robot}.log"


def planner_log_has_live(cell, robot):
    """Check 3f's runtime half, memoised per (CELL, ROBOT).

    Returns True (the line is there), False (the log is there and it is not),
    or None (no planner log, so the question is unanswerable).

    Per robot, not per cell: each node writes its own log and each node runs its
    own veto, so one robot's live witness says nothing about the other's.
    """
    key = (cell, robot)
    if key in _LIVE_CACHE:
        return _LIVE_CACHE[key]
    p = os.path.join(ROOT, cell, PLANNER_LOG.format(robot=robot))
    if not os.path.exists(p):
        _LIVE_CACHE[key] = None
    else:
        with open(p, errors="replace") as fh:
            _LIVE_CACHE[key] = any("link_gate_live:" in ln for ln in fh)
    return _LIVE_CACHE[key]


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
    missing = sorted(k for k, v in EXPECT.items() if v == "FILL_ME")
    print("REFUSING TO RUN: the binary identity is not declared.\n"
          "  A gate that does not know which binary it is gating cannot fail\n"
          "  check 3, and would pass a campaign built from anything.\n"
          f"  Undeclared: {', '.join(missing)}\n"
          f"  Write them to {os.path.join(ROOT, TAG + '.identity.txt')} as\n"
          "  key=value lines, or set GATE_EXPECT_<key> in the environment.")
    sys.exit(2)

# A cell is TAG_<arm>_seed<N>. The .attempts siblings run_campaign.sh keeps for
# redone cells are excluded by name only; any other non-matching directory stays
# enumerated and hard-fails. (notes: gate-cell-enumeration-attempts)
CELL_RE = re.compile(r"^" + re.escape(TAG) + r"_.+_seed\d+$")
_all_dirs = sorted(
    d for d in os.listdir(ROOT)
    if d.startswith(TAG + "_") and os.path.isdir(os.path.join(ROOT, d))
)
cells = [d for d in _all_dirs if not d.endswith(".attempts")]
_attempts = [d for d in _all_dirs if d.endswith(".attempts")]
_malformed = [d for d in cells if not CELL_RE.match(d)]
if not cells:
    print(f"no cells for tag {TAG}")
    sys.exit(1)

# ==================================================================
# D2 witness: the simple_nav_3d BINARY (check 3, extra key)
# ==================================================================
# Pins the simple_nav_3d binary, which no source rev witnesses. Key in no cell:
# one INFO line. In any cell: it must be declared or the gate refuses to run,
# and cells lacking it hard-fail. (notes: gate-nav-binary-witness)
_nav_present = [c for c in cells
                if manifest(os.path.join(ROOT, c)).get(NAV_BIN_KEY)]
NAV_WITNESS_NOTE = None
if not _nav_present:
    NAV_WITNESS_NOTE = (
        f"INFO: no cell carries {NAV_BIN_KEY}, so this campaign predates the "
        f"nav-binary witness. Check 3 pins the explo_planner binary and three "
        f"source revisions; it CANNOT tell whether simple_nav_3d was rebuilt, "
        f"so any D2 (goal-snap) behaviour claimed from these cells rests on "
        f"the operator's word, not on this gate.")
elif EXPECT_NAV is None:
    print(f"REFUSING TO RUN: {len(_nav_present)}/{len(cells)} cells record "
          f"{NAV_BIN_KEY},\n"
          "  so this campaign CAN say which simple_nav_3d binary ran and the\n"
          "  declaration does not. D2 (the goal-snap append) lives in that\n"
          "  binary and in no other provenance field here: git_simple_nav_3d\n"
          "  moves with the source tree whether or not colcon ran.\n"
          f"  Undeclared: {NAV_BIN_KEY}\n"
          f"  Write it to {os.path.join(ROOT, TAG + '.identity.txt')} as a\n"
          f"  key=value line, or set GATE_EXPECT_{NAV_BIN_KEY}.")
    sys.exit(2)
else:
    EXPECT[NAV_BIN_KEY] = EXPECT_NAV

# ==================================================================
# The M-TARE feature vector each arm token is DEFINED to carry (check 3g).
# ==================================================================
# mtare_off runs the silence gate on purpose: it dispatches nothing.
# mtare_rendezvous and mtare_hybrid share a vector (3e separates them by mode).
# Every stampable arm, _mdp included, needs a row or 3g hard-fails it.
# (notes: gate-mtare-arm-stack-table)
MTARE_ARM_STACK = {
    "mtare_off": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": False,
        "rendezvous_schedule_enable": False, "pursuit_predictor=mdp": False},
    "mtare_pursuit": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": True,
        "rendezvous_schedule_enable": False, "pursuit_predictor=mdp": False},
    "mtare_pursuit_mdp": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": True,
        "rendezvous_schedule_enable": False, "pursuit_predictor=mdp": True},
    "mtare_rendezvous": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": True,
        "rendezvous_schedule_enable": True, "pursuit_predictor=mdp": False},
    "mtare_hybrid": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": True,
        "rendezvous_schedule_enable": True, "pursuit_predictor=mdp": False},
    "mtare_hybrid_mdp": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": True,
        "rendezvous_schedule_enable": True, "pursuit_predictor=mdp": True},
}
# The only m-tare arm token a pre-P5 binary could stamp. Any other on a pre-P5
# cell means the directory was renamed after the fact.
# (notes: gate-mtare-arms-pre-p5)
MTARE_ARMS_PRE_P5 = {"mtare_hybrid"}
# And which a P5-but-pre-P6 binary could: everything except the two `_mdp`
# names, which did not exist until the predictor did. Derived from the table
# rather than written out, so adding a row cannot leave this list behind.
MTARE_ARMS_PRE_P6 = {a for a in MTARE_ARM_STACK if not a.endswith("_mdp")}


def mtare_expect(arm, gen_p5, gen_p6):
    """The feature vector `arm` is defined to carry, or None if a binary of
    this generation cannot stamp that arm at all.

    `gen_p5`/`gen_p6` select the vocabulary AND the vector width: a pre-P5
    binary emits no rendezvous_schedule_enable and a pre-P6 one no
    pursuit_predictor, so asking about either would compare against a key the
    observed vector does not have. Handled by generation for the reason spelt
    out at check 3g: dropping the key outright would certify a P6 `_mdp` cell
    that ran the trail, and hard-failing its absence would make the gate unable
    to score any campaign banked before P6."""
    if arm.startswith("mtare_"):
        if not gen_p5 and arm not in MTARE_ARMS_PRE_P5:
            return None
        if not gen_p6 and arm not in MTARE_ARMS_PRE_P6:
            return None
        want = MTARE_ARM_STACK.get(arm)
        if want is None:
            return None
        want = dict(want)
    else:
        # An untreated arm carries none of the features, and this direction
        # decides the comparison: a control cell that ran the allocator is a
        # treated cell in the control column. (notes: gate-untreated-arm-vector)
        want = {k: False for k in MTARE_ARM_STACK["mtare_hybrid_mdp"]}
    if not gen_p5:
        want.pop("rendezvous_schedule_enable", None)
    if not gen_p6:
        want.pop("pursuit_predictor=mdp", None)
    return want


hard_fail, soft, unresolved = [], [], []
agg_recovery_entries = 0
rows = []
nav_fail_by_arm = collections.Counter()
cells_by_arm = collections.Counter()
# The same cells counted by reconnect POLICY, with design suffixes stripped.
# Only check 21 reads it, and only to tell a genuinely malformed campaign apart
# from a correct one whose GATE_ARMS declaration predates a second factor.
cells_by_policy = collections.Counter()
policy_of_arm = {}
comms_regime_by_cell = {}

# Populations for the new checks. Counted so an empty one can be reported as
# UNRESOLVED instead of passing by vacuity.
n_csv_rows = n_hw_fire = n_hw_escape = n_reconnect_end = 0
n_navfail_rows = n_peer_rows = n_runend_rows = n_midrun = 0
n_disp_rows = n_runstart_rows = n_console_logs = 0
# Check 3m populations: robot-runs whose mission_complete row count was checked
# (any generation) and those carrying the run_end counters. Many of the first
# with none of the second must read UNRESOLVED. (notes: gate-3m-populations)
n_3m_rowcount = n_3m_fields = 0
# Check 3n's population: robot-runs whose planner_<robot>.log could be read at
# all. The duplicate-run_end counter has NO path into the jsonl (see the block
# at check 3n for why), so a missing planner log is not a partial reading —
# it is the check not running, and it has to say so.
n_3n_plogs = 0
# Robot-runs where the once-per-run latch actually refused something, listed
# rather than counted so the report can name them. Non-empty is HEALTH.
n_3m_reentry_runs = []
# Check 3f runs on the treated arms only, so its population is not cells_by_arm
# and an off-only campaign leaves it at zero legitimately. Both are reported.
# Counted in ROBOT-RUNS, like the other run_start populations, because that is
# the loop it lives in.
n_3f_runs = n_3f_live_checked = 0
# Check 3g populations, counted and reported separately: a zero in either means
# no cell was certified treated (or untreated), which must not read as a pass.
# (notes: gate-3g-populations)
n_3g_treated = n_3g_control = 0
# Check 3h's populations: how many robot-runs came from each binary generation,
# and which cells they were, so a mixed campaign can name the cells rather than
# just the counts.
n_3g_gen_p5 = n_3g_gen_pre = 0
_gen_cells_p5, _gen_cells_pre = set(), set()
# The same populations at the P6 boundary, witnessed separately: a cell can
# carry rendezvous_schedule_enable without pursuit_predictor, so P5 presence
# says nothing about which chase mtare_hybrid means.
# (notes: gate-3g-p6-populations)
n_3g_gen_p6 = n_3g_gen_pre6 = 0
_gen_cells_p6, _gen_cells_pre6 = set(), set()
# check 3l. The sensor/candidate configuration, collected per robot-run and
# asserted campaign-wide below. value -> {(cell, arm, robot)}, per param.
_cfg_seen = collections.defaultdict(lambda: collections.defaultdict(set))
_live_reported = {}

for c in cells:
    d = os.path.join(ROOT, c)
    m = manifest(d)
    # arm is the cell's identity (what to count, what GATE_ARMS declares);
    # policy_arm is what the binary stamps, every suffix removed. Compare
    # run_start and the m-tare vocabulary against policy_arm; 3i checks the
    # suffixes. (notes: gate-arm-vs-policy-arm)
    arm, policy_arm, dir_claims, _leftover = parse_cell_name(c, TAG)
    cells_by_arm[arm] += 1
    cells_by_policy[policy_arm] += 1
    policy_of_arm[arm] = policy_arm
    row = {"cell": c, "arm": arm}

    # Control-ness belongs to the reconnect policy, not a design level. Either
    # the full identity or policy_arm may be declared in GATE_CONTROL_ARMS; the
    # identity is tried first, so a design can control one level only.
    # (notes: gate-control-by-policy)
    is_control = arm in CONTROL_ARMS or policy_arm in CONTROL_ARMS

    # This cell's roster, from its own manifest rather than a module constant.
    # See robots_of(): iterating a hardcoded pair over an N>=3 campaign checked
    # two thirds of each team and reported a pass over the rest.
    cell_robots, roster_src = robots_of(m, d)
    row["n_robots"] = len(cell_robots)
    row["roster"] = ",".join(cell_robots)
    if roster_src == "hardcoded fallback":
        unresolved.append(
            f"{c}: manifest names no team (no robots=, no team_robot_names), "
            f"so the roster fell back to the hardcoded {ROBOTS_FALLBACK}. "
            f"Every per-robot check below ran over that guess; if the cell had "
            f"a third robot, nothing here looked at it")
    elif roster_src.startswith("event logs"):
        unresolved.append(
            f"{c}: manifest names no team, so the roster was read off the "
            f"*.events.jsonl files present ({','.join(cell_robots)}). A robot "
            f"whose log is missing entirely is invisible to that reading — it "
            f"drops out of the expectation instead of failing check 3a")

    # 3k. The two roster witnesses must agree: robots= is written by
    # run_campaign.sh from what it launched, team_robot_names by the node. A
    # disagreement means the per-robot checks iterate the wrong set.
    # (notes: gate-3k-roster-witnesses)
    _m_robots = [t.strip() for t in (m.get("robots") or "").split(",")
                 if t.strip()]
    _m_team = m.get("team_robot_names")
    if _m_robots and _m_team:
        try:
            _team = [str(t) for t in json.loads(_m_team)]
        except ValueError:
            _team = [t.strip().strip('"[] ') for t in _m_team.split(",")]
        _team = [t for t in _team if t]
        if _team and sorted(_team) != sorted(_m_robots):
            hard_fail.append(
                f"{c}: check 3k — the manifest's two rosters disagree: "
                f"robots={_m_robots} but team_robot_names={_team}. One of them "
                f"names a team that did not run")

    # 3i. EVERY DESIGN LEVEL THE DIRECTORY NAME ADVERTISES MUST BE THE ONE THE
    # RUN ACTUALLY USED.
    #
    # Each design level the directory claims must match its manifest key,
    # written at launch; iterated over DESIGN_SUFFIXES. An absent key is
    # forgiven only if the directory claims nothing; a suffixed cell without one
    # hard-fails. (notes: gate-3i-design-level-witness)
    _levels = []
    for _rx, _key, _human, _unit in DESIGN_SUFFIXES:
        _claim = dir_claims.get(_key)
        _man = m.get(_key)
        _lvl = f"{_claim[0]:g} {_unit}" if _claim is not None else None
        if _lvl:
            _levels.append(f"{_human}={_lvl}")
        if _claim is None:
            if _man not in (None, "none"):
                hard_fail.append(
                    f"{c}: check 3i — directory names no {_human} but the "
                    f"manifest says {_key}={_man!r}. The cell ran a pinned "
                    f"value under a name that promises the yaml default")
        elif _man is None:
            hard_fail.append(
                f"{c}: check 3i — directory says the {_human} was "
                f"{_lvl}, but the manifest records no {_key} at all, so "
                f"nothing here can confirm it. Score this campaign with a "
                f"harness that writes the key")
        elif _man == "none" or float(_man) != _claim[0]:
            hard_fail.append(
                f"{c}: check 3i — directory says {_human} {_lvl} but "
                f"the manifest says {_key}={_man!r}. The cell is filed under a "
                f"level it did not run")
    row["levels"] = "; ".join(_levels) if _levels else "-"

    # 3j, first half: record this cell's radio regime. Compared campaign-wide
    # after the loop rather than cell-by-cell, because the failure is a
    # RELATION between cells — one cell at 11.98 dB is a valid run, and only
    # becomes a defect next to a cell at 70.0 under the same tag.
    comms_regime_by_cell[c] = tuple(
        _regime_val(m.get(k)) for k in COMMS_REGIME_KEYS)

    # 1. run ended by the exploration criterion, not a cap
    reason = m.get("run_end_reason", "MISSING")
    row["end"] = reason
    row["t_sim"] = m.get("run_end_t_sim", "?")
    if reason != "all_done":
        hard_fail.append(f"{c}: run_end_reason={reason} (not all_done)")

    # 2. harness comms gates
    #
    # Anything but CLEAN hard-fails, except a SUSPECT whose UNRUN lines in
    # comms_gates.txt all name REPORT_ONLY_GATES: that is UNRESOLVED. A SUSPECT
    # with no UNRUN lines (watcher never reported) hard-fails.
    # (notes: gate-2-suspect-report-only)
    verdict = m.get("run_gates_verdict", "MISSING")
    row["verdict"] = verdict
    if verdict != "CLEAN":
        _unrun = []
        for _ln in read_lines(os.path.join(d, "comms_gates.txt")):
            if _ln.startswith("UNRUN"):
                _f = _ln.split("\t")
                _unrun.append(_f[1].strip() if len(_f) > 1 else "?")
        if (verdict == "SUSPECT" and _unrun
                and all(g in REPORT_ONLY_GATES for g in _unrun)):
            unresolved.append(
                f"{c}: check 2 — run_gates_verdict=SUSPECT, but every UNRUN "
                f"line in comms_gates.txt names a report-only gate "
                f"({', '.join(sorted(set(_unrun)))}), which has no pass "
                f"threshold and cannot invalidate a run. Not scored as a "
                f"failure; read the file before using this cell")
        else:
            hard_fail.append(
                f"{c}: run_gates_verdict={verdict}"
                + (f" (UNRUN: {', '.join(sorted(set(_unrun)))})" if _unrun
                   else " (no UNRUN lines — the run-time watcher never "
                        "reported, so the outage gate itself is missing)"))

    # 3. provenance: manifest must match the declared generation
    for k, v in EXPECT.items():
        if m.get(k) != v:
            hard_fail.append(f"{c}: {k}={m.get(k)} expected {v}")

    # 3a'. The nav binary, two ways. First, split provenance: a cell with no
    # NAV_BIN_KEY while sibling cells carry one hard-fails, since the harness
    # changed mid-campaign. (notes: gate-3a-nav-split-provenance)
    if _nav_present and not m.get(NAV_BIN_KEY):
        hard_fail.append(
            f"{c}: no {NAV_BIN_KEY} in the manifest, but "
            f"{len(_nav_present)}/{len(cells)} sibling cells have one — the "
            f"harness changed mid-campaign, so this cell cannot be pooled "
            f"with them")

    # Second, a partial install: the four nav executables are installed as a
    # unit, so any the manifest records as missing hard-fails; no declaration is
    # needed. (notes: gate-3a-nav-partial-install)
    _nav_missing = sorted(k for k in (NAV_BIN_KEY,) + NAV_BIN_SIBLINGS
                          if m.get(k) == "missing")
    if _nav_missing:
        hard_fail.append(
            f"{c}: {', '.join(_nav_missing)}=missing — the simple_nav_3d "
            f"install tree was incomplete when this cell ran")

    lat, arrived, unknowns, navfails, poseloss = 0, 0, [], 0, 0
    for r in cell_robots:
        ev = events(d, r)
        if not ev:
            hard_fail.append(
                f"{c}/{r}: no events (roster from {roster_src}: "
                f"{','.join(cell_robots)})")
            continue
        plog = read_lines(os.path.join(d, f"planner_{r}.log"))

        # 3b/3c. baked rev and arm identity
        #
        # git_rev is read from run_start.params, not the top level of run_start,
        # where it is never written. A missing run_start hard-fails rather than
        # skipping 3b, 18b and 18c. (notes: gate-3b-git-rev-in-params)
        start = [e for e in ev if e.get("event") == "run_start"]
        if not start:
            hard_fail.append(
                f"{c}/{r}: no run_start event — checks 3b, 18b and 18c cannot "
                f"run on this robot, and a head-truncated log is not a pass")
        else:
            n_runstart_rows += 1
            pr = start[0].get("params", {})
            # 3l. Collect the configuration this robot-run resolved, from
            # run_start.params and not the yaml: overrides beat the yaml, and
            # the 0-means-auto knobs are rewritten at construction.
            # (notes: gate-3l-resolved-config)
            for _k in CFG_KEYS:
                if _k in pr:
                    _cfg_seen[_k][cfg_val(pr[_k])].add((c, arm, r))
            raw_rev = str(pr.get("git_rev", ""))
            rev = raw_rev.replace("-dirty", "")
            if not raw_rev:
                hard_fail.append(
                    f"{c}/{r}: check 3b — run_start params carry no git_rev, "
                    f"so the baked rev cannot be compared to the manifest")
            elif not m.get("git_explo_planner", "").startswith(rev[:7]):
                hard_fail.append(
                    f"{c}/{r}: check 3b — JSONL git_rev={raw_rev} != manifest "
                    f"{m.get('git_explo_planner')}")
            if "-dirty" in raw_rev:
                hard_fail.append(f"{c}/{r}: check 3c — JSONL git_rev is -dirty")
            # 3d. The schema stamp. The pin is SCHEMA_VERSION above, movable
            # from the command line when scoring a banked campaign.
            # (notes: gate-3d-schema-stamp)
            sv = start[0].get("schema_version", pr.get("schema_version"))
            if sv != SCHEMA_VERSION:
                hard_fail.append(
                    f"{c}/{r}: check 3d — schema_version={sv!r}, expected "
                    f"{SCHEMA_VERSION}")
            row.setdefault("mode_req", pr.get("arm"))
            # Either spelling: reconnect_enabled, or rendezvous_enabled in older
            # cells. The node stamps both now.
            # (notes: gate-reconnect-enabled-spelling)
            have_rdv = pr.get("reconnect_enabled")
            if have_rdv is None:
                have_rdv = pr.get("rendezvous_enabled")
            row.setdefault("rdv", have_rdv)
            # 3e. The treatment must be confirmed from run_start, not taken from
            # the directory name. Compared against policy_arm: the node stamps
            # the bare reconnect policy, and 3i holds the suffixes against the
            # manifest. (notes: gate-3e-arm-stamp)
            want_rdv = not is_control
            _stamp = pr.get("arm")
            _unrec = unrecognised_suffix(policy_arm, _stamp)
            _lookup_arm = _stamp if _unrec else policy_arm
            if _stamp is not None and _stamp != policy_arm and not _unrec:
                hard_fail.append(
                    f"{c}/{r}: check 3e — directory says arm={arm} "
                    + (f"(policy {policy_arm}) " if policy_arm != arm else "")
                    + f"but run_start params say arm={_stamp!r}")
            elif _unrec:
                unresolved.append(
                    f"{c}/{r}: check 3e — directory says arm={arm}, the binary "
                    f"stamped arm={_stamp!r}, and the difference is the single "
                    f"trailing token _{_unrec} that this gate does not know. "
                    f"That is a gap in THIS FILE, not evidence about the run: "
                    f"add _{_unrec} to RUNTIME_SUFFIXES if it is a switch no "
                    f"campaign contrasts, or to DESIGN_SUFFIXES with the "
                    f"manifest key that witnesses it if it is an independent "
                    f"variable. Until then the cell's level of that knob is "
                    f"unchecked and check 21 cannot see whether both levels ran")
            if have_rdv is not None and bool(have_rdv) != want_rdv:
                hard_fail.append(
                    f"{c}/{r}: check 3e — arm={arm} but reconnect_enabled="
                    f"{have_rdv!r}")
            # 3g. THE M-TARE ARM MUST WITNESS ITS WHOLE STACK, NOT ONE BIT.
            #
            # The mtare_ prefix is an OR over features, and cell_world_enable
            # and team_world_hz rename nothing, so the name proves little: the
            # whole vector is checked from params, in both directions; the
            # control direction decides. (notes: gate-3g-whole-stack)
            _mt_want = arm.startswith("mtare_")
            _mt_raw = {k: pr.get(k) for k in
                       ("cell_world_enable", "team_world_hz",
                        "global_alloc_enable", "reconnect_gate")}
            _mt_absent = sorted(k for k, v in _mt_raw.items() if v is None)
            # rendezvous_schedule_enable dates the cell: absent means a pre-P5
            # binary, held to the pre-P5 arm vocabulary, not failed. Absence of
            # the other four cannot be certified. Mixing generations fails 3h.
            # (notes: gate-3g-p5-dating)
            _p5_raw = pr.get("rendezvous_schedule_enable")
            _gen_p5 = _p5_raw is not None
            if _gen_p5:
                n_3g_gen_p5 += 1
                _gen_cells_p5.add(c)
            else:
                n_3g_gen_pre += 1
                _gen_cells_pre.add(c)
            # pursuit_predictor dates the cell the same way: absent means a
            # pre-P6 binary. Never default it to trail, the value a P6 _mdp cell
            # must not have. (notes: gate-3g-p6-dating)
            _p6_raw = pr.get("pursuit_predictor")
            _gen_p6 = _p6_raw is not None
            if _gen_p6:
                n_3g_gen_p6 += 1
                _gen_cells_p6.add(c)
            else:
                n_3g_gen_pre6 += 1
                _gen_cells_pre6.add(c)
            # The node refuses to start on any third spelling (it throws), so a
            # value outside the pair cannot have come from a run — it means
            # this param dump is not what the binary wrote, and every feature
            # read out of it is suspect, not just this one.
            if _gen_p6 and _p6_raw not in ("trail", "mdp"):
                hard_fail.append(
                    f"{c}/{r}: check 3g — pursuit_predictor={_p6_raw!r} is "
                    f"neither 'trail' nor 'mdp'. The node throws on any other "
                    f"value, so this run_start was not written by a run of it")
            # Pre-P5 implies pre-P6: the predictor shipped after the scheduler
            # and no binary has ever had one without the other. A cell claiming
            # otherwise is a dump assembled from two sources, which invalidates
            # the dating both checks rest on.
            if _gen_p6 and not _gen_p5:
                hard_fail.append(
                    f"{c}/{r}: check 3g — run_start carries pursuit_predictor "
                    f"(P6) but no rendezvous_schedule_enable (pre-P5). No "
                    f"binary was ever built that way, so this param dump does "
                    f"not date to any single generation")
            if _mt_absent:
                # The node emits all four unconditionally, so absence is not a
                # default — it is a cell written by a binary predating the
                # stack, whose arm cannot be certified in either direction.
                hard_fail.append(
                    f"{c}/{r}: check 3g — run_start params carry no "
                    f"{', '.join(_mt_absent)}. The node emits every one of "
                    f"them unconditionally, so this cell came from a binary "
                    f"predating the M-TARE stack and arm={arm} cannot be "
                    f"certified either way")
            else:
                try:
                    _hz_on = float(_mt_raw["team_world_hz"]) > 0.0
                except (TypeError, ValueError):
                    _hz_on = None
                if _hz_on is None:
                    hard_fail.append(
                        f"{c}/{r}: check 3g — team_world_hz="
                        f"{_mt_raw['team_world_hz']!r} is not a number, so "
                        f"whether the exchange ran is unknown")
                else:
                    # team_world_hz IS the switch (0 = off), not merely a rate,
                    # which is why it is compared against 0 and not recorded.
                    _feat = {
                        "cell_world_enable": bool(_mt_raw["cell_world_enable"]),
                        "team_world_hz>0": _hz_on,
                        "global_alloc_enable":
                            bool(_mt_raw["global_alloc_enable"]),
                        "reconnect_gate=info":
                            _mt_raw["reconnect_gate"] == "info",
                    }
                    if _gen_p5:
                        _feat["rendezvous_schedule_enable"] = bool(_p5_raw)
                    if _gen_p6:
                        # Compared against the string, not truthiness: this
                        # param is a NAME (`trail` or `mdp`) and both spellings
                        # are non-empty, so bool() would read every cell as the
                        # model arm.
                        _feat["pursuit_predictor=mdp"] = (_p6_raw == "mdp")
                    # Looked up by _lookup_arm: policy_arm, since
                    # MTARE_ARM_STACK has no room for knob suffixes, or the
                    # binary's own stamp when 3e found an unknown suffix, so the
                    # stack is still checked. (notes: gate-3g-lookup-arm)
                    _want = mtare_expect(_lookup_arm, _gen_p5, _gen_p6)
                    if _want is None:
                        if not _gen_p5:
                            _gen_name, _known = "pre-P5", MTARE_ARMS_PRE_P5
                        elif not _gen_p6:
                            _gen_name, _known = "P5-but-pre-P6", MTARE_ARMS_PRE_P6
                        else:
                            _gen_name, _known = "P6-or-later", set(MTARE_ARM_STACK)
                        hard_fail.append(
                            f"{c}/{r}: check 3g — arm={_lookup_arm} is not an arm a "
                            f"{_gen_name} binary "
                            f"can stamp (known: "
                            f"{sorted(_known)}"
                            f"). The directory name and the binary disagree "
                            f"about which experiment this is")
                    else:
                        _wrong = [k for k, v in _feat.items() if v != _want[k]]
                        if _wrong:
                            hard_fail.append(
                                f"{c}/{r}: check 3g — arm={arm} but "
                                + ", ".join(f"{k}={_feat[k]} "
                                            f"(want {_want[k]})"
                                            for k in _wrong)
                                + "; the arm token names the whole stack")
                    if _mt_want:
                        n_3g_treated += 1
                    else:
                        n_3g_control += 1

            # 3f. The treatment must have been able to fire. Read from run_start
            # params (what the node was handed), not the manifest (what the
            # launcher intended). Applies to every non-control arm, not to one
            # arm by name. (notes: gate-3f-treatment-possible)
            if not is_control:
                n_3f_runs += 1

                def _num(key):
                    """Parsed float, or None if absent/unparseable.

                    float() on a non-numeric value used to raise straight out
                    of the gate: a malformed param killed the run instead of
                    reporting a hard failure, which is the one outcome a gate
                    must never have.
                    """
                    v = pr.get(key)
                    if v is None:
                        return None
                    try:
                        return float(v)
                    except (TypeError, ValueError):
                        hard_fail.append(
                            f"{c}/{r}: check 3f — {key}={v!r} is not a number")
                        return None

                sil = _num("reconnect_midrun_silence_sec")
                if pr.get("reconnect_midrun_silence_sec") is None:
                    hard_fail.append(
                        f"{c}/{r}: check 3f — run_start params carry no "
                        f"reconnect_midrun_silence_sec, so it cannot be shown "
                        f"the mid-run trigger was even reachable")
                elif sil is not None \
                        and abs(sil - MIDRUN_SILENCE_EXPECT) > 1e-6:
                    hard_fail.append(
                        f"{c}/{r}: check 3f — reconnect_midrun_silence_sec="
                        f"{sil}, expected {MIDRUN_SILENCE_EXPECT}")

                # link_gate_configured only means the two topic names were set.
                # Liveness is the one-shot link_gate_live: line the node logs on
                # the first usable link-states sample; its presence is asserted,
                # not a warning's absence. (notes: gate-3f-link-gate-live)
                if not GEN9_PARAMS_REQUIRED:
                    pass
                elif pr.get("link_gate_configured") is None:
                    hard_fail.append(
                        f"{c}/{r}: check 3f — run_start params carry no "
                        f"link_gate_configured; a pre-generation-9 binary "
                        f"cannot be gated as generation 9")
                elif not bool(pr.get("link_gate_configured")):
                    hard_fail.append(
                        f"{c}/{r}: check 3f — link_gate_configured=false, so "
                        f"the link veto never ran while the mid-run clock was "
                        f"at {sil}s (below the ~180s suppression tail)")
                elif LINK_GATE_LIVE_REQUIRED:
                    live = planner_log_has_live(c, r)
                    if live is None:
                        if not _live_reported.get((c, r)):
                            _live_reported[(c, r)] = True
                            unresolved.append(
                                f"check 3f: {c}/{r} — no planner log at "
                                f"{c}/{PLANNER_LOG.format(robot=r)}, so 'did "
                                f"the link veto actually go live' could not be "
                                f"answered")
                    else:
                        n_3f_live_checked += 1
                        if not live:
                            hard_fail.append(
                                f"{c}/{r}: check 3f — link_gate_configured=true "
                                f"but no 'link_gate_live:' line: the topics "
                                f"were named and nothing ever arrived, so the "
                                f"veto was absent for the whole run")

                dc = _num("reconnect_link_down_confirm_sec") \
                    if GEN9_PARAMS_REQUIRED else None
                if not GEN9_PARAMS_REQUIRED:
                    pass
                elif pr.get("reconnect_link_down_confirm_sec") is None:
                    hard_fail.append(
                        f"{c}/{r}: check 3f — run_start params carry no "
                        f"reconnect_link_down_confirm_sec")
                elif dc is not None \
                        and abs(dc - LINK_DOWN_CONFIRM_EXPECT) > 1e-6:
                    hard_fail.append(
                        f"{c}/{r}: check 3f — reconnect_link_down_confirm_sec="
                        f"{dc}, expected {LINK_DOWN_CONFIRM_EXPECT}")
            # 18b. the five nav timeout params must be echoed, or the
            # thresholds on nav_goal_failed rows cannot be audited against the
            # configuration that produced them.
            for p in ("goal_rotate_timeout_sec", "nav_speed_estimate_mps",
                      "nav_safety_factor", "nav_min_timeout_sec",
                      "nav_max_timeout_sec"):
                if p not in pr:
                    hard_fail.append(f"{c}/{r}: check 18b — run_start params missing {p}")
            # 18c. done_action must be idle; under the node default shutdown the
            # planner exits at DONE and never homes. Read from run_start params,
            # not done_action_in_params, which reports the yaml, not what the
            # node loaded. (notes: gate-18c-done-action-idle)
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

        # 3m. the endpoint is declared EXACTLY once — see the pin block above
        # for the generation-8 defect this exists for.
        n_3m_rowcount += 1
        if len(mc) > 1:
            hard_fail.append(
                f"{c}/{r}: check 3m — {len(mc)} mission_complete rows, "
                f"expected at most 1. The writer latch added in generation 9 "
                f"makes a second row impossible, so this robot-run came from "
                f"an older binary than the manifest claims, or the latch was "
                f"removed. Its homing_duration_sec/homing_distance_m restart "
                f"from zero on the second row and its run contains two homing "
                f"budgets")
        _end = [e for e in ev if e.get("event") == "run_end"]
        _sup = _end[-1].get("mission_completes_suppressed") if _end else None
        _ree = _end[-1].get("mission_return_reentries") if _end else None
        if _sup is None or _ree is None:
            # Absence is reported, never inferred as zero. Both fields are
            # written unconditionally by logRunEnd, so one present and the
            # other missing is itself a finding — hence the is-None test on
            # each rather than on the pair.
            if ENDPOINT_FIELDS_REQUIRED:
                hard_fail.append(
                    f"{c}/{r}: check 3m — run_end carries no "
                    f"{'mission_completes_suppressed' if _sup is None else ''}"
                    f"{'/' if _sup is None and _ree is None else ''}"
                    f"{'mission_return_reentries' if _ree is None else ''}. "
                    f"Generations 8 and 9 both stamp schema_version 4, so "
                    f"between those two their absence is the only witness "
                    f"that this is a pre-generation-9 binary. Score a banked "
                    f"campaign with GATE_ENDPOINT_FIELDS=0")
        else:
            n_3m_fields += 1
            if int(_sup) > 0:
                hard_fail.append(
                    f"{c}/{r}: check 3m — run_end says "
                    f"mission_completes_suppressed={_sup}. logMissionComplete "
                    f"has ONE call site (finishMissionReturn) and the "
                    f"mission_return_done_ latch stands in front of it, so a "
                    f"suppressed row means finishMissionReturn re-entered "
                    f"WITHOUT passing startReturnHome — a path the node guard "
                    f"cannot see. The kept row describes the FIRST attempt "
                    f"while the run contains two")
            # mission_return_reentries is not failed on: the latch refusing a
            # late second homing request is the fix working. Collected for the
            # population report instead. (notes: gate-3m-reentries-not-failed)
            if int(_ree) > 0:
                n_3m_reentry_runs.append(f"{c}/{r}={_ree}")

        # 3n. run_end is written exactly once — the OTHER half of the endpoint,
        # and the only one that cannot be read from the jsonl.
        #
        # Read from the planner log, not the jsonl: a trailing event would break
        # run_end-is-last and make event_log.py drop the cell. Both markers
        # (run-time WARN, destructor ERROR) are matched; either is the finding.
        # (notes: gate-3n-planner-log-witness)
        _plog_path = os.path.join(d, f"planner_{r}.log")
        if not os.path.exists(_plog_path):
            unresolved.append(
                f"check 3n: {c}/{r} — no planner_{r}.log, so the duplicate "
                f"run_end check did not run on this robot-run. The counter has "
                f"no path into the jsonl, so there is no fallback reading")
        else:
            n_3n_plogs += 1
            _dup = [ln for ln in plog
                    if "run_end was already written" in ln
                    or "SUPPRESSED duplicate run_end" in ln]
            if _dup:
                hard_fail.append(
                    f"{c}/{r}: check 3n — the planner log reports a SUPPRESSED "
                    f"duplicate run_end. The node reached a terminal state "
                    f"twice and the file keeps the FIRST ending only, so this "
                    f"run's completion stamp, milestones_reached and every "
                    f"other endpoint metric describe one of two endings and "
                    f"the run is not scoreable as it stands: "
                    f"{_dup[0].strip()[:220]}")

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
        #
        # Assert each field against the writer that emits it:
        # team_incomplete_sec only on reconnect_dispatch rows (PeerEvent has
        # none), and peer rows must no longer carry last_contact_age_sec.
        # (notes: gate-19-field-writers)
        peers = [e for e in ev if e.get("event") in ("peer_lost", "peer_seen")]
        n_peer_rows += len(peers)
        for e in peers:
            if "last_contact_age_sec" in e:
                hard_fail.append(
                    f"{c}/{r}: check 19 — {e.get('event')} still carries the "
                    f"removed last_contact_age_sec")
        disp = [e for e in ev if e.get("event") == "reconnect_dispatch"]
        n_disp_rows += len(disp)
        for e in disp:
            if "team_incomplete_sec" not in e:
                hard_fail.append(
                    f"{c}/{r}: check 19 — reconnect_dispatch missing "
                    f"team_incomplete_sec, the left-hand side of the "
                    f"inequality the mid-run trigger evaluated")
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
    #
    # A missing console log is UNRESOLVED, not a skip: that cell is the one most
    # likely to have had a process die. (notes: gate-9-console-log-absent)
    con = os.path.join(ROOT, c + ".console.log")
    if os.path.exists(con):
        n_console_logs += 1
        txt = open(con, errors="replace").read()
        for pat, label in (("sim clock frozen", "DEADMAN FIRED"),
                           ("died mid-run", "PROCESS DIED"),
                           ("/clock unreadable", "CLOCK UNREADABLE")):
            if pat in txt:
                hard_fail.append(f"{c}: {label}")
    else:
        unresolved.append(
            f"check 9: {c} — no console log at {os.path.basename(con)}, so the "
            f"deadman/process-death check did not run on this cell")

    row.update(latched=lat, arrived=arrived, unknown=unknowns,
               navfail=navfails, poseloss=poseloss)
    if poseloss:
        soft.append(f"{c}: {poseloss} pose-loss episode(s)")
    rows.append(row)

print(f"=== {TAG}: {len(cells)} cells ({dict(cells_by_arm)}) ===\n")
# Cell column width comes from the data, not a literal.
# (notes: gate-table-cell-width)
_cw = max([len("cell")] + [len(r["cell"]) for r in rows])
hdr = (f"{'cell':{_cw}} {'end':10} {'verdict':8} {'t_sim':>6} {'N':>2} "
       f"{'lat':>3} {'arr':>3} {'nf':>3} {'pl':>3}  unknown")
print(hdr)
print("-" * len(hdr))
for r in rows:
    print(f"{r['cell']:{_cw}} {r['end']:10} {r['verdict']:8} {r['t_sim']:>6} "
          f"{r.get('n_robots', '?'):>2} "
          f"{r['latched']:>3} {r['arrived']:>3} {r['navfail']:>3} {r['poseloss']:>3}  "
          f"{r['unknown']}")

print("\narm identity: " + ", ".join(
    sorted({f"{r['arm']}: mode_req={r.get('mode_req')} rdv={r.get('rdv')}" for r in rows})))
print(f"nav_goal_failed by arm: {dict(nav_fail_by_arm)}")
# Report-only, and therefore INFO and not UNRUN: it states what this gate did
# NOT check, which is a different thing from a check that failed to run.
if NAV_WITNESS_NOTE:
    print("\n" + NAV_WITNESS_NOTE)

# ==================================================================
# 3h. ONE BINARY GENERATION PER CAMPAIGN.
# ==================================================================
# Same claim as 3b, read off behaviour instead of a label.
# rendezvous_schedule_enable (P5) and pursuit_predictor (P6) are separate
# boundaries; mixing either pools two meanings of the arm names.
# (notes: gate-3h-one-generation)
if n_3g_gen_p5 and n_3g_gen_pre:
    hard_fail.append(
        f"check 3h — this campaign mixes binary generations: "
        f"{n_3g_gen_p5} robot-run(s) in {len(_gen_cells_p5)} cell(s) emit "
        f"rendezvous_schedule_enable (P5 or later) and {n_3g_gen_pre} in "
        f"{len(_gen_cells_pre)} cell(s) do not (pre-P5). The arm tokens do not "
        f"mean the same thing across that boundary. "
        f"pre-P5 e.g. {sorted(_gen_cells_pre)[:3]}, "
        f"P5+ e.g. {sorted(_gen_cells_p5)[:3]}")
elif n_3g_gen_p5 or n_3g_gen_pre:
    print(f"binary generation: "
          f"{'P5+' if n_3g_gen_p5 else 'pre-P5'} "
          f"({n_3g_gen_p5 or n_3g_gen_pre} robot-runs, uniform)")
if n_3g_gen_p6 and n_3g_gen_pre6:
    hard_fail.append(
        f"check 3h — this campaign mixes binary generations at the P6 "
        f"boundary: {n_3g_gen_p6} robot-run(s) in {len(_gen_cells_p6)} cell(s) "
        f"emit pursuit_predictor (P6 or later) and {n_3g_gen_pre6} in "
        f"{len(_gen_cells_pre6)} cell(s) do not (pre-P6). `mtare_pursuit` and "
        f"`mtare_hybrid` name a trail chase on one side of that line and are "
        f"ambiguous between trail and model on the other. "
        f"pre-P6 e.g. {sorted(_gen_cells_pre6)[:3]}, "
        f"P6+ e.g. {sorted(_gen_cells_p6)[:3]}")
elif n_3g_gen_p6 or n_3g_gen_pre6:
    print(f"binary generation (predictor): "
          f"{'P6+' if n_3g_gen_p6 else 'pre-P6'} "
          f"({n_3g_gen_p6 or n_3g_gen_pre6} robot-runs, uniform)")

# ==================================================================
# 3l. ONE SENSOR AND CANDIDATE CONFIGURATION PER CAMPAIGN.
# ==================================================================
# Same shape as 3h and 3j: refuses to pool two sensor models, which a
# params-file change creates without moving any other field read here. Prints
# the configuration even when uniform. (notes: gate-3l-one-sensor-config)
_cfg_split_hard, _cfg_split_soft, _cfg_uniform = [], [], []
for _k in CFG_KEYS:
    _vals = _cfg_seen.get(_k)
    if not _vals:
        # Absent from every run_start. Not a failure on its own -- a banked
        # campaign predates the key -- but emphatically not a pass either, so
        # it is named in the INFO block below rather than skipped in silence.
        _cfg_uniform.append(f"{_k}=<not recorded>")
        continue
    if len(_vals) == 1:
        _cfg_uniform.append(f"{_k}={next(iter(_vals))}")
        continue
    # Split. Which kind?
    _by_arm = collections.defaultdict(set)
    for _v, _where in _vals.items():
        for (_c, _a, _r) in _where:
            _by_arm[_a].add(_v)
    _within = sorted(a for a, vs in _by_arm.items() if len(vs) > 1)
    _detail = "; ".join(
        f"{_v} in {len(_w)} robot-run(s) e.g. {sorted(_w)[0][0]}"
        for _v, _w in sorted(_vals.items()))
    if _within or _k in CFG_HARD:
        _cfg_split_hard.append(
            f"check 3l — {_k} is not constant across this campaign: {_detail}."
            + (f" It varies WITHIN arm(s) {_within}, which is never a design: "
               f"cells of one arm ran different configurations."
               if _within else
               " This is a sensor/candidate parameter, not a treatment, so "
               "arms differing in it are not comparable on any endpoint."))
    else:
        _cfg_split_soft.append(
            f"check 3l — {_k} differs BETWEEN arms ({_detail}), which is a "
            f"legitimate design if it is this campaign's independent variable "
            f"and a confound if it is not. Not scored as a failure; confirm "
            f"against the campaign's intent before pooling")
hard_fail.extend(_cfg_split_hard)
unresolved.extend(_cfg_split_soft)
print("\nconfiguration (check 3l, from run_start after resolution):")
for _line in _cfg_uniform:
    print(f"  {_line}")

# sha256_shared_params must be constant within an arm. Between arms it may
# differ, since a per-arm yaml knob is varied by giving the arm its own params
# file. (notes: gate-3l-params-file-hash)
_ph = collections.defaultdict(set)
for _c in cells:
    _v = manifest(os.path.join(ROOT, _c)).get("sha256_shared_params")
    if _v:
        _ph[parse_cell_name(_c, TAG)[0]].add(_v)
_ph_split = sorted(a for a, vs in _ph.items() if len(vs) > 1)
if _ph_split:
    hard_fail.append(
        f"check 3l — sha256_shared_params is not constant within arm(s) "
        f"{_ph_split}: cells of one arm ran different params files")
elif _ph:
    _allv = {v for vs in _ph.values() for v in vs}
    print(f"  sha256_shared_params={sorted(_allv)}"
          + ("" if len(_allv) == 1 else "  <- differs between arms"))

# ==================================================================
# 3j. ONE RADIO REGIME PER CAMPAIGN, AND IT MUST BE THE DECLARED ONE.
# ==================================================================
# Refuses to pool two radio regimes: every connectivity result depends on them.
# Read off the manifest, written at launch from the emulator's values; never
# inferred from observed dropouts, which the regime explains.
# (notes: gate-3j-one-radio-regime)
def _regime_str(t):
    return ", ".join(f"{k}={v}" for k, v in zip(COMMS_REGIME_KEYS, t))


if comms_regime_by_cell:
    _by_regime = collections.defaultdict(list)
    for _c, _t in comms_regime_by_cell.items():
        _by_regime[_t].append(_c)
    if len(_by_regime) > 1:
        _detail = "; ".join(
            f"[{_regime_str(t)}] in {len(cs)} cell(s) e.g. {sorted(cs)[:2]}"
            for t, cs in sorted(_by_regime.items()))
        hard_fail.append(
            f"check 3j — this campaign mixes {len(_by_regime)} radio regimes: "
            f"{_detail}. A trunk does not cost the same on both sides, so the "
            f"outage measurements are not comparable and the arm means "
            f"something different in each. There is no override for this")
    elif COMMS_REGIME_EXPECT_REQUIRED:
        _got = next(iter(_by_regime))
        if _got != COMMS_REGIME_EXPECT:
            hard_fail.append(
                f"check 3j — this campaign ran [{_regime_str(_got)}] but was "
                f"scored expecting [{_regime_str(COMMS_REGIME_EXPECT)}]. The "
                f"shipped regime moved on 2026-09-03 (trunks 11.98 -> 70.0 dB, "
                f"a 30 m horizon added), so a campaign from before it is "
                f"re-scored on its own terms by declaring the regime in the "
                f"invocation: GATE_TREE_ATTEN=11.98 GATE_MAX_RANGE=none "
                f"./gate_g8.py {TAG}")
        else:
            print(f"radio regime: {_regime_str(_got)} "
                  f"({len(comms_regime_by_cell)} cells, uniform, as declared)")
    else:
        print(f"radio regime: {_regime_str(next(iter(_by_regime)))} "
              f"({len(comms_regime_by_cell)} cells, uniform; "
              f"GATE_COMMS_REGIME=0, so NOT compared against a declaration)")

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
        ("reconnect_dispatch rows",     n_disp_rows,     "19"),
        ("run_end rows",                n_runend_rows,   "19"),
        ("run_start rows",              n_runstart_rows, "3b"),
        ("mission_complete row counts", n_3m_rowcount,   "3m"),
        ("endpoint-counter witnesses",  n_3m_fields,     "3m"),
        ("planner logs read",           n_3n_plogs,      "3n"),
        ("treated-arm run_start rows",  n_3f_runs,       "3f"),
        ("link-gate live witnesses",    n_3f_live_checked, "3f"),
        ("m-tare features certified ON", n_3g_treated,   "3g"),
        ("m-tare features certified OFF", n_3g_control,  "3g"),
        ("binary-generation witnesses", n_3g_gen_p5 + n_3g_gen_pre, "3h"),
        ("predictor-gen witnesses",     n_3g_gen_p6 + n_3g_gen_pre6, "3h"),
        ("radio-regime manifests",      len(comms_regime_by_cell), "3j"),
        ("console logs present",        n_console_logs,   "9"),
        ("mid-run reconnect lines",     n_midrun,        "20")):
    mark = "" if n else "   <- EMPTY: check is UNRESOLVED, not passed"
    # The one population that is allowed to be legitimately empty, and only
    # because the operator switched the sub-check off by hand. Everything else
    # empty means the check ran against nothing and must not read as a pass.
    if not n and label == "link-gate live witnesses" \
            and not LINK_GATE_LIVE_REQUIRED:
        mark = ("   <- sub-check disabled by GATE_GEN9_PARAMS"
                "/GATE_LINK_GATE_LIVE")
        print(f"  check {check:>2}  {label:28} {n:6d}{mark}")
        continue
    # Same exemption, same reasoning, for 3m's generation-9 half: an operator
    # re-scoring a banked campaign switched it off on purpose. The row-count
    # half above has NO exemption, because every generation ever written can be
    # asked how many mission_complete rows it holds.
    if not n and label == "endpoint-counter witnesses" \
            and not ENDPOINT_FIELDS_REQUIRED:
        print(f"  check {check:>2}  {label:28} {n:6d}"
              f"   <- sub-check disabled by GATE_ENDPOINT_FIELDS")
        continue
    # 3g's populations are exempt only when GATE_ARMS declares no arm of that
    # kind; a campaign that declared both and produced only one stays
    # UNRESOLVED. (notes: gate-3g-population-exemption)
    if not n and check == "3g":
        _any_mtare = any(a.startswith("mtare_") for a in EXPECT_ARMS)
        _all_mtare = all(a.startswith("mtare_") for a in EXPECT_ARMS)
        _exempt = (not _any_mtare) if label.endswith("ON") else _all_mtare
        if _exempt:
            print(f"  check {check:>2}  {label:28} {n:6d}"
                  f"   <- GATE_ARMS {list(EXPECT_ARMS)} declares no "
                  f"{'m-tare' if label.endswith('ON') else 'non-m-tare'} arm, "
                  f"so this population cannot exist")
            continue
    print(f"  check {check:>2}  {label:28} {n:6d}{mark}")
    if not n:
        unresolved.append(f"check {check}: {label} — zero rows, nothing was tested")

# Check 3m, the mixture half, not relaxable by GATE_ENDPOINT_FIELDS: some
# robot-runs carrying the counters and some not is two binary generations in one
# directory. (notes: gate-3m-mixture-half)
if n_3m_fields and n_3m_fields < n_3m_rowcount:
    hard_fail.append(
        f"check 3m — {n_3m_fields} of {n_3m_rowcount} robot-runs carry the "
        f"generation-9 endpoint counters and {n_3m_rowcount - n_3m_fields} do "
        f"not, so this campaign MIXES binary generations. GATE_ENDPOINT_FIELDS "
        f"does not relax this: it declares a campaign to predate the fields, "
        f"and this one does not predate them uniformly")

# Check 3m, the health half: the zero case is printed too, in words, since a
# guard that reports only when it fires looks compiled out. A zero is not
# failed: short runs may never latch coverage after homing.
# (notes: gate-3m-health-half)
if n_3m_fields:
    if n_3m_reentry_runs:
        print(f"\ncheck 3m — the once-per-run mission-return latch REFUSED a "
              f"later homing request in {len(n_3m_reentry_runs)} of "
              f"{n_3m_fields} robot-runs (this is the fix working, not a "
              f"fault):")
        for _w in n_3m_reentry_runs[:12]:
            print(f"    {_w}")
        if len(n_3m_reentry_runs) > 12:
            print(f"    ... and {len(n_3m_reentry_runs) - 12} more")
    else:
        print(f"\ncheck 3m — the once-per-run mission-return latch refused "
              f"nothing across {n_3m_fields} robot-runs. The field IS present, "
              f"so this is a measured zero and not an absent check; on ts1b "
              f"the generation-8 defect it guards showed up in 16 robot-runs, "
              f"so a campaign-wide zero is worth confirming rather than "
              f"assuming.")

# check 3n, printed unconditionally for the same reason 3m's health half is: a
# check whose healthy output is an empty list is indistinguishable from a check
# that did not run. The denominator is stated, never inferred.
_n_3n_dups = len([h for h in hard_fail if "check 3n" in h])
if _n_3n_dups:
    print(f"\ncheck 3n — {_n_3n_dups} of {n_3n_plogs} robot-runs wrote a "
          f"duplicate run_end (listed in the hard failures above).")
else:
    print(f"\ncheck 3n — no duplicate run_end in {n_3n_plogs} robot-run(s). "
          f"This is a measured zero: the planner logs were read and searched "
          f"for both markers. Robot-runs with no planner log are listed as "
          f"UNRESOLVED rather than counted here. Base rate is 0 in 3342 banked "
          f"robot-runs, so a zero here confirms nothing new — it only keeps "
          f"the generation-9 suppression from hiding the event entirely.")

print(f"\ngate E aggregate '-> recovery:' entries: {agg_recovery_entries}")
if agg_recovery_entries == 0:
    print("  -> UNRESOLVED (not a pass): zero entries is what generation 4 produced.")
    unresolved.append("gate E: zero recovery entries")
else:
    print("  -> RESOLVED: the recovery path executed and every entry paired.")

# 21. The campaign must have its pre-registered shape; seed-major ordering makes
# an early abort arm-unbalanced. Both lists are printed, since both are
# overridable from the environment. (notes: gate-21-campaign-shape)
# Reported, not silent: a redo is legitimate, but the number of them is a
# property of the campaign an operator should see next to the cell counts —
# REDOs correlate with arm, and an arm that needed five of them is not the
# same evidence as one that needed none.
if _attempts:
    print(f"\nretained failed attempts (excluded from the cell count): "
          f"{len(_attempts)} — {sorted(_attempts)}")
if _malformed:
    hard_fail.append(
        f"check 21 — {len(_malformed)} director(y/ies) under {TAG}_ do not "
        f"parse as <TAG>_<arm>_seed<N>: {sorted(_malformed)}")
# Classified by the same rule the cell loop uses (identity or policy), so the
# printed split cannot disagree with the split the checks actually applied.
_declared_control = [a for a in EXPECT_ARMS
                     if a in CONTROL_ARMS or _expect_policies[a] in CONTROL_ARMS]
print(f"\narms expected: {list(EXPECT_ARMS)} "
      f"(control: {_declared_control} from GATE_CONTROL_ARMS="
      f"{sorted(CONTROL_ARMS)}, treated: "
      f"{[a for a in EXPECT_ARMS if a not in _declared_control]})")
print(f"campaign shape: {dict(cells_by_arm)} "
      f"(pre-registered {EXPECT_CELLS_PER_ARM} per arm)")
# When GATE_ARMS matches the campaign's reconnect policies but every directory
# adds a design suffix, say so: the declaration is one factor short. Still a
# hard failure, since only full identities show both levels ran.
# (notes: gate-21-declaration-one-factor-short)
if (set(cells_by_arm) != set(EXPECT_ARMS)
        and set(cells_by_policy) == set(EXPECT_ARMS)
        and any(a != p for a, p in policy_of_arm.items())):
    hard_fail.append(
        f"check 21 — GATE_ARMS names the reconnect policies "
        f"{sorted(EXPECT_ARMS)}, which match this campaign exactly, but every "
        f"cell directory carries a design suffix on top of them "
        f"({', '.join(sorted(cells_by_arm))}). The cells are not mislabelled "
        f"and the campaign is not malformed — the DECLARATION is one factor "
        f"short. Re-declare GATE_ARMS with the full cell identities so this "
        f"check can see whether BOTH levels of that factor ran; that is the "
        f"failure it exists for, and it is invisible at the policy level. The "
        f"arm lines below are consequences of this one")
for a in EXPECT_ARMS:
    if cells_by_arm.get(a, 0) != EXPECT_CELLS_PER_ARM:
        hard_fail.append(
            f"check 21 — arm {a} has {cells_by_arm.get(a, 0)} cells, "
            f"pre-registered {EXPECT_CELLS_PER_ARM}. Set GATE_CELLS_PER_ARM to "
            f"score a deliberately partial campaign; do not score it silently")
extra = set(cells_by_arm) - set(EXPECT_ARMS)
if extra:
    hard_fail.append(f"check 21 — unexpected arm(s) {sorted(extra)}")

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

# Three states, three exit codes: 1 hard failures, 3 unresolved (not a failure;
# read the UNRESOLVED list), 0 clean. A campaign with an off arm legitimately
# has empty reconnect populations, so 3 is expected. (notes: gate-exit-codes)
if hard_fail:
    sys.exit(1)
sys.exit(3 if unresolved else 0)
