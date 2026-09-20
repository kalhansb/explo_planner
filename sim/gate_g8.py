#!/usr/bin/env python3
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

# Gates in comms_gates.txt that are READINGS, not thresholds: they describe the
# run, they cannot condemn it. Used by check 2 only, and only to keep a line
# such a gate was never entitled to write from failing the whole cell. Adding a
# name here is a claim that the gate has no pass criterion — check the gate's
# own header before doing it.
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
    # Last resort: whatever event logs are on disk. This is DELIBERATELY not
    # the primary source -- deriving the roster from the files present makes
    # "a robot's log is missing" unobservable, since the missing robot simply
    # drops out of the expectation. It is used only when the manifest names no
    # team at all, and the caller says so in its output.
    found = sorted(os.path.basename(q)[:-len(".events.jsonl")]
                   for q in glob.glob(os.path.join(d, "*.events.jsonl")))
    if found:
        return found, "event logs on disk (NOT a manifest expectation)"
    return list(ROBOTS_FALLBACK), "hardcoded fallback"


# Suffixes a cell directory may carry that the BINARY never stamps into
# run_start's `arm`. run_campaign.sh strips each before setting RECONNECT_MODE,
# so a directory carrying them holds a node that stamped the bare policy name,
# and anything compared against run_start has to compare the stripped form.
#
# Two kinds, and the difference decides whether the suffix survives into the
# cell's IDENTITY:
#
#   RUNTIME  — a switch that is not a design dimension of any campaign using it.
#              Pooled away entirely: `arm` loses it, so cells with and without
#              it count into the same bucket.
#   DESIGN   — an independent variable. Stripped for the comparison against the
#              binary (policy_arm) but KEPT in `arm`, because folding r10 and
#              r40 into one bucket would leave check 21 unable to notice that
#              half a design never ran, which is the precise failure check 21
#              exists for. Each also gets a manifest cross-check (3i, 3k2), on
#              the same argument as 3e one level down: the analysis reads the
#              level off the directory name, so a launcher bug that runs the
#              other level silently swaps the design's two columns.
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
# Declared identity of generation 8 (pre-registration section 32.14).
#
# These are DECLARED CONSTANTS, deliberately not read from the working tree:
# the gate's job is to prove the cells came from the binary the pre-registration
# names, and a gate that reads the identity from whatever happens to be built
# would pass for any binary at all.
#
# Two of them CANNOT be literals here, for a reason that is structural and not
# a matter of taste. The manifest's git_explo_planner is `git rev-parse HEAD`
# at run time, and the JSONL's git_rev is baked into the binary at CMake
# configure time — so both name the commit the campaign was built from. This
# file is IN that commit. A file cannot contain its own commit hash, and the
# binary's sha256 has the same problem, since the rev string is compiled into
# it: filling either literal changes the tree, which changes the commit, which
# changes both values again. There is no fixed point.
#
# So they are declared OUT OF BAND, in a file written once at campaign launch
# and living outside git:
#
#     $GATE_ROOT/<TAG>.identity.txt      (key=value, one per line)
#
# That preserves the property that matters — the gate is told what to expect by
# something it does not compute — while being physically possible. The
# pre-registration records the same four values, and the identity file being
# outside git is not a weakness here: it is written BEFORE the first cell runs
# and any later edit is visible in its mtime against the campaign's own cells.
#
# Env overrides exist for the calibration harness, which must point the real
# logic at synthetic cells of known identity.
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
            # An empty right-hand side is not a declaration. Accepting it put
            # "" into EXPECT, which is not FILL_ME, so the refusal below did
            # not fire and every cell instead hard-failed with "expected "
            # and nothing after it — pointing the operator at the cells when
            # the fault is a truncated identity file.
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
# ALL FOUR are declared, none is a literal. git_simple_nav_3d and git_scovox
# used to be hardcoded here ("c9f83a7" and "078d3f7"), which was invisible for as
# long as the gate was only ever pointed at generation-8 cells, where those revs
# were in fact correct. The moment simple_nav_3d changed -- which it does in
# generation 9, for D2/D2b -- EVERY cell hard-failed check 3 on a non-defect, and
# nothing in the identity file or the environment could clear it. An operator
# facing a gate that cannot be satisfied does one of two things, and both are
# worse than the bug: abort a good campaign, or learn to skim past check-3 lines.
# Check 3 is the one that catches real cross-generation pooling, so teaching the
# reader to ignore it is the expensive failure. A pin that only a source edit can
# move is a pin that gets moved under time pressure, months later, by someone who
# just wants the gate to go green -- which is exactly how the eight guards in
# `checks-that-stopped-checking` went inert while still printing PASSes.
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
# The sensor model and the candidate generator. None of these is an arm's
# treatment in any campaign this gate scores, so variation in them is not a
# design -- it is a stale installed shared_params.yaml, a -p override that
# reached some cells and not others, or two invocations pooled.
#
# It needs its own check because the existing provenance cannot see it.
# sha256_shared_params moves if the yaml changes, but it is in no EXPECT set
# and nothing compares it; and it says the file differed, never WHICH value
# differed -- the same complaint the harness itself writes three times over the
# *_in_params keys. git_explo_planner does not move for a params change at all.
#
# CFG_HARD are campaign-wide constants. CFG_SOFT have been independent
# variables before (cr2 varied the claim radius by arm), so between-arm
# variation in them is a legitimate design and is reported, not failed --
# but WITHIN one arm they are still constants, and that half stays hard.
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
# The event-log schema this gate scores against (check 3d). Overridable for the
# same reason as everything else in this block: a banked generation-9 campaign
# was written at schema 3 and is still perfectly valid on its own terms, so
# re-gating it must be possible WITHOUT editing the pin. Editing the pin is how
# a gate stops checking; passing GATE_SCHEMA_VERSION=3 says out loud, in the
# invocation, which generation is being scored.
#
#   GATE_SCHEMA_VERSION=3 GATE_MIDRUN_SILENCE=240 GATE_GEN9_PARAMS=0 \
#       ./gate_g8.py g8r1
#
# Bumped to 4 for the M-TARE evolution. The v4 additions are all default-off, so
# a v4 binary run at shipped defaults is behaviourally the same planner as v3 —
# but "the same behaviour" is a claim for equiv_gate.py to prove per phase, not
# something this file may assume. Here the stamp stays what it has always been:
# a one-field tripwire on mixing generations in one campaign directory.
#
# Bumped to 5 on 2026-09-16 (generation 10), when the rendezvous appointment
# stopped being derived independently on each robot and became a (cell,
# interval) pair exchanged over TeamWorld and committed by unanimity. Unlike the
# v4 bump this one is BEHAVIOURAL and not default-off: the rendezvous and hybrid
# arms of a v5 campaign are running a different mechanism from the same-named
# arms of a v4 one. The pin must move with the binary — check 3d is an exact
# equality, so a stale pin here rejects every cell of the current generation
# while the calibrator, which pins the gate's env to its own constant, goes on
# reporting PASS against a schema nothing writes.
#
# THE HISTORY ABOVE STOPPED AT v5 UNTIL 2026-09-18 while the pin below read 8,
# which is the exact failure this block was written to prevent, committed in the
# block itself. The three missing bumps:
#
#   v6 (generation 17) — MEANING-ONLY. `RendezvousAgreedEvent::t_meet_sec` went
#     from "the next meeting instant" to the phase offset of a repeating
#     interval. Same field, same type, same name, different quantity. Nothing
#     rejects a v6 file read as v5; it just answers a different question.
#   v7 (generation 19) — MEANING-ONLY. A new `rendezvous_outcome` label, so the
#     set of values a reader must switch on grew without the key changing.
#   v8 (generation 23, today) — three meaning changes and three additive
#     fields. See the block comment on the schema constant in experiment_log.hpp
#     (`kSchemaVersion`), which is the authority; do not re-derive the list here
#     and do not let the two drift again.
#
# v9 (generation 25, 2026-09-18) moved the same day as the header — MEANING-
# ONLY: the arming floor behind `t_meet_sec` went from now-plus-notice to bare
# now, and arrived=false outcome rows split into barrier conversions vs
# strandings. kSchemaVersion in experiment_log.hpp remains the authority.
#
# A MEANING-ONLY BUMP IS STILL A BUMP, and those are the ones that get skipped:
# nothing fails to parse, so the pressure to record them is entirely absent, and
# a reader pooling v6 with v5 gets numbers rather than an error. The pin exists
# to make that pooling impossible inside one campaign directory. It cannot do
# that job if the reason for a version is only in the commit that raised it.
#
# v10 (generation 29) and v11 (generation 31) moved with the header and are
# recorded there, not here — the paragraph above already says the authority is
# `kSchemaVersion` and that this list must not be re-derived. v11 is the first
# VOCABULARY widening since v4: `appointment_leg`.
SCHEMA_VERSION = int(os.environ.get("GATE_SCHEMA_VERSION", "11"))
# Generation-9 treatment configuration (check 3f). Overridable so the check can
# be calibrated against a known-answer case — a gate that has never been shown
# to FAIL on a bad input is not evidence of anything, which is the lesson the
# 3b/3c git_rev bug taught this file.
#
# EVERY sub-check is overridable, not just the clock. An earlier draft exposed
# only GATE_MIDRUN_SILENCE, and the amendment claimed check 3f "goes silent when
# told to expect 240". It does not: on g8r1 that override takes 138 hard
# failures to 92, not to 0, because the three params generation 9 introduced
# have no escape and no generation-8 binary can ever carry them. Re-gating a
# banked campaign needs GATE_GEN9_PARAMS=0 as well:
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
# Generation 8 shipped a DONE -> RETURN_HOME re-entry: a run that had already
# homed could be sent home a second time by the late coverage latch, because the
# only guard tested `state_ == State::RETURN_HOME` and the second request
# arrives from DONE. On ts1b that produced 16 robot-runs carrying two
# mission_complete rows — 1/3/12 at N=2/3/4, all in the treatment arm and none
# in any control. Two rows is two homing budgets, a refilled escape ladder, and
# homing_duration_sec/homing_distance_m restarting from zero on the second one.
#
# Generation 9 closes it in two places and this check reads both:
#
#   * the node latch (mission_return_done_) refuses the second homing LEG. What
#     it refused rides out on run_end.mission_return_reentries, which this check
#     does NOT require to be zero — a late coverage latch legitimately asks, and
#     the refusal is the fix working.
#   * the writer latch (ExperimentLog::logMissionComplete) refuses the second
#     ROW, counting it into run_end.mission_completes_suppressed. That one MUST
#     be zero: with one call site inside finishMissionReturn and the node latch
#     in front of it, a suppressed row means finishMissionReturn re-entered
#     without passing startReturnHome — a path the node guard cannot see.
#
# Requiring the fields to EXIST is the other half, and the more important one.
# Generations 8 and 9 both stamp schema_version 4, so check 3d cannot separate
# THOSE two, and a zero read off a field that was never written is the exact
# shape of a check that stopped checking. (Generation 10 stamps 5 and IS
# separable by the stamp; this argument is about the 8/9 pair only, which is
# what GATE_ENDPOINT_FIELDS=0 exists for.)
# GATE_ENDPOINT_FIELDS=0 scores a banked pre-generation-9
# campaign on its own terms; it relaxes only the existence requirement, and the
# "at most one row" rule below still runs, because that one is readable on every
# generation ever written.
ENDPOINT_FIELDS_REQUIRED = os.environ.get("GATE_ENDPOINT_FIELDS", "1") != "0"


# The radio regime (check 3j). Three manifest fields decide what "the link
# dropped" MEANS on a run: how much a trunk costs, how far a trunk-free link
# reaches, and how much power it started with. Every result this project
# reports about connectivity is conditional on them.
#
# They moved on 2026-09-03. tree_attenuation_db went from 11.98 dB — the IEEE
# 9260568 per-trunk fit, under which a single trunk almost never dropped a link
# — to 70.0, where any trunk in the Fresnel zone is fatal; max_range_m appeared
# at the same time and bounds a CLEAR lane at 30 m, which nothing did before.
# cr3/cr4/cr5 and everything earlier ran the old radio. Pooling across that is
# forbidden, and until this check existed nothing read the fields: the manifest
# recorded the difference and every analysis averaged over it.
#
# Overridable in the same spirit as GATE_SCHEMA_VERSION above, and for the same
# reason: a banked campaign is perfectly valid on its own terms and must be
# re-scorable WITHOUT editing the pin, by saying which regime it ran out loud
# in the invocation —
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
# The arms the campaign was pre-registered with. Was the literal pair
# ("hybrid", "off") in check 21, which made every campaign that is not
# hybrid-vs-off fail as "unexpected arm(s)" — including off vs mtare_hybrid,
# which is the same experiment with a different treatment. Declared rather than
# inferred from the directories on disk: inferring it would make check 21
# incapable of noticing the truncation it exists to catch, since a campaign that
# only ever ran one arm would "expect" exactly that arm.
# `.strip()`: these arrive as a shell string, and `GATE_ARMS="off, mtare_hybrid"`
# would otherwise declare an arm named " mtare_hybrid" that no cell can ever
# match — check 21 then reports the real arm as unexpected AND the phantom as
# missing, and the campaign fails for a space.
EXPECT_ARMS = tuple(
    a for a in (s.strip()
                for s in os.environ.get("GATE_ARMS", "hybrid,off").split(","))
    if a)

# The control arm, and the ONLY arm that runs with reconnect_enabled=false.
# Everything else — hybrid, pursuit, rendezvous and the three treated
# `mtare_*` tokens — reaches the manoeuvre and must have it enabled. Check 3e used to spell this as
# `arm == "hybrid"`, which asserted reconnect_enabled=False for a pursuit or
# rendezvous cell and would have hard-failed a correct run of either.
#
# The default is `off` ALONE. It briefly shipped as "off,mtare_off" on the
# theory that a future allocator-on/reconnect-off control would want the same
# exemption, and that was a pre-authorised hole: `mtare_off` is a name the node
# can already stamp (its arm string is "mtare_" + "off" whenever the allocator
# is on and rendezvous is not), so the day that token is added to the runner's
# case list, a cell carrying a decision-changing treatment would arrive with
# check 3f skipped and 3e satisfied — every assertion about its treatment
# exempted in advance, by a default nobody had to type. An arm earns the
# exemption by being declared at scoring time, not by being guessed at here.
#
# THAT DAY HAS ARRIVED and the default has deliberately not moved. `mtare_off`
# is now a real runner token — the control cell of the §3.6.1 factorial, which
# carries P1-P3 and neither reconnect mechanism. Scoring that campaign
# therefore requires GATE_CONTROL_ARMS="mtare_off" to be typed out, and the
# check below refuses a control arm that no cell is running, so the typing
# cannot be wrong in the quiet direction.
CONTROL_ARMS = frozenset(
    a for a in (s.strip()
                for s in os.environ.get("GATE_CONTROL_ARMS", "off").split(","))
    if a)

# A control arm nobody is running is a control arm that exempts nothing, and it
# is the exact shape a typo takes: GATE_CONTROL_ARMS=of leaves check 3f demanding
# reconnect config from the `off` cells, which fails loudly, but
# GATE_CONTROL_ARMS=off with GATE_ARMS naming no `off` arm fails silently — the
# exemption sits there unused while the campaign has no control at all.
#
# Matched at BOTH levels, because control-ness is a property of the reconnect
# policy and GATE_ARMS carries full cell identities: a two-factor campaign whose
# arms are `mtare_off_r40_ttl0` and `mtare_hybrid_r40_ttl0` has a control, and
# `GATE_CONTROL_ARMS=mtare_off` names it correctly. Refusing that spelling would
# force the operator to repeat every design suffix in a second variable, where a
# copy that drifts from the first is silent. The typo protection is unchanged:
# a token matching neither an expected arm nor the policy of one still exempts
# nothing and still aborts.
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
# THE FIRST DRAFT OF THIS CHECK READ THE CONSOLE LOG. It would have hard-failed
# every treated robot-run of every real campaign -- and not as an UNRESOLVED
# skip, because that file exists (check 9 depends on it), so the probe returned
# "log present, token absent" rather than "cannot answer". The calibration
# fixture planted the token in the same wrong file, so the known-answer case
# agreed with the bug and reported ALL PASS. A probe and its calibration written
# from the same wrong assumption check nothing [[checks-that-stopped-checking]];
# this one is now pinned to the path start() actually writes.
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

# Anchored on a trailing _seed<N>, the way event_log.py and modes_compare.py
# already spell it. `startswith(TAG + "_")` alone also swept up the
# `<cell>.attempts/` directories run_campaign.sh creates beside a cell it is
# redoing (it keeps the failed attempt's evidence rather than deleting it).
# Those parse to a perfectly good arm name — "mtare_hybrid_seed7.attempts"
# rsplits to "mtare_hybrid" — so every redone cell was counted TWICE in
# check 21 and then hard-failed a second time for having no manifest and no
# event log. A redo is not a defect and REDOs correlate with arm, so this
# inflated one arm's count on exactly the campaigns that needed scoring most,
# and the operator's only escapes were to raise GATE_CELLS_PER_ARM or delete
# the evidence — each of which disables a check.
#
# Excluded by NAME, not by shape: dropping everything that fails to match
# `_seed<N>$` would also drop a genuinely malformed cell directory, and a
# malformed cell is something this gate must shout about, not skip. So
# `.attempts` — the one sibling the harness is known to create — is removed
# explicitly, and anything else that does not parse is still enumerated and
# still hard-fails below on its missing manifest.
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
# The four keys in EXPECT pin one binary and three SOURCE trees. Generation 9
# puts a behavioural change -- D2, the goal-snap append -- in a binary that none
# of them covers. git_simple_nav_3d moves with the source tree, so it moves
# whether or not colcon ran; a source rev cannot witness a rebuild. That left
# D2's one failure mode (edited, not rebuilt: the node on the wire is still
# generation 8 while every provenance field says generation 9) invisible to
# every check in this file.
#
# The requirement is self-calibrating rather than unconditional, and that is the
# load-bearing part. Cells banked before the harness emitted the key cannot
# answer, and a key that hard-fails 80 good cells with no way to clear it is a
# key that gets deleted by the next person under time pressure -- which is the
# bug this gate just had for git_simple_nav_3d, and the shape of every guard in
# `checks-that-stopped-checking`. So ask the campaign which case it is:
#
#   key in NO cell    -> the campaign predates it. One INFO line saying D2 is
#                        unwitnessed here. Not a failure, and not silence.
#   key in SOME cells -> the harness changed mid-campaign. Hard-fail the cells
#                        that lack it; a split provenance is a real defect
#                        ([[commit-mid-campaign-splits-provenance]]).
#   key in EVERY cell, undeclared -> refuse. The campaign CAN answer and the
#                        operator has not said what the answer should be, which
#                        is exactly the FILL_ME case for the other four.
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
# Until P5 this was a single boolean — every feature ON for an `mtare_*` arm,
# every feature OFF otherwise — because there was exactly one m-tare arm and it
# carried the whole stack. The doc §3.6.1 factorial ends that: its four arms
# share the P1-P3 stack (cell world, exchange, allocator) and differ precisely
# in the two reconnect MECHANISMS, so "all features on" would reject three of
# the four cells of the design this check exists to certify.
#
# `mtare_off` carries reconnect_gate=silence on purpose and not as an
# oversight: the gate can only suppress a dispatch, and that arm makes none, so
# `info` there would pin an inert knob. What is emphatically NOT inert in it is
# the allocator, which is why the factorial's control is `mtare_off` and not
# plain `off` — a plain-`off` control would confound the two mechanisms under
# test with P3.
#
# `mtare_rendezvous` and `mtare_hybrid` have identical vectors. They are
# separated by reconnect_mode / reconnect_enabled, which check 3e already
# compares against the directory name; this table is about the stack, not the
# mode.
#
# P6 ADDED A SIXTH FEATURE AND TWO ARM NAMES, and the table has to carry both
# or it rejects the campaign it exists to certify. `pursuit_predictor` selects
# how a chase is AIMED — `trail` drives at the peer's last declared goal,
# `mdp` at a modelled intercept — so `mtare_hybrid` and `mtare_hybrid_mdp` are
# two treatments, exactly as `mtare_pursuit` and `mtare_hybrid` are. The node
# stamps the suffix itself (explo_planner_node.cpp: `if (pursuit_predictor_mdp_)
# arm += "_mdp"`), so the directory name and the stamp agree and check 3e is
# silent — which is precisely why this table not knowing the token is
# dangerous rather than noisy: without the two `_mdp` rows every cell of the
# two chasing arms hard-failed 3g as "not an arm any binary can stamp", and
# the four-arm ts4 design is half `_mdp`.
#
# The suffix is a FEATURE here and not merely a name, for the same reason
# every other row is a vector: `mtare_hybrid_mdp` carrying pursuit_predictor=
# trail is the treatment arm silently running its own control, and nothing
# else in this gate looks at that param.
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
# Which m-tare arm tokens a PRE-P5 binary could have stamped. Only one: the
# three factorial names did not exist, and the runner refused every route to
# `mtare_off` (the arm-name/arm-stamp check rejected a plain token with the
# allocator on, and RECONNECT_GATE=info with RECONNECT_MODE=off was fatal). So
# seeing one of them on a pre-P5 cell means the directory was renamed after the
# fact, not that the binary ran that arm.
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
        # An untreated arm carries none of it, in either generation. Checked in
        # this direction too, and this is the direction that decides the
        # comparison: a control cell that somehow ran the allocator is a
        # treated cell sitting in the control column, and no amount of care in
        # the treated arm compensates for that.
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
# check 3m's two populations, counted separately because they answer different
# questions: how many robot-runs had their mission_complete ROW COUNT checked
# (every generation can be), and how many carried the generation-9 run_end
# counters at all (only generation 9 can be). A campaign where the second is 0
# while the first is large is a pre-generation-9 binary wearing a generation-9
# manifest, and it must read as UNRESOLVED rather than as a pass.
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
# Check 3g's two populations, counted separately and reported separately. An
# off-vs-mtare_hybrid campaign must produce BOTH: a zero in the first means no
# cell was ever certified as treated, a zero in the second means no cell was
# ever certified as untreated, and either alone would let the check read as a
# pass over the arm it never looked at.
n_3g_treated = n_3g_control = 0
# Check 3h's populations: how many robot-runs came from each binary generation,
# and which cells they were, so a mixed campaign can name the cells rather than
# just the counts.
n_3g_gen_p5 = n_3g_gen_pre = 0
_gen_cells_p5, _gen_cells_pre = set(), set()
# The same populations one phase further out. P6 is a SECOND generation
# boundary and needs its own witness rather than riding on P5's: a binary can
# have the scheduler and not the predictor (every cell banked between P5 and
# P6 is one), so `rendezvous_schedule_enable` present says nothing about
# whether `mtare_hybrid` on this cell means the trail chase or the modelled
# one — which is the same pooling 3h refuses at P5, one name further down.
n_3g_gen_p6 = n_3g_gen_pre6 = 0
_gen_cells_p6, _gen_cells_pre6 = set(), set()
# check 3l. The sensor/candidate configuration, collected per robot-run and
# asserted campaign-wide below. value -> {(cell, arm, robot)}, per param.
_cfg_seen = collections.defaultdict(lambda: collections.defaultdict(set))
_live_reported = {}

for c in cells:
    d = os.path.join(ROOT, c)
    m = manifest(d)
    # Cell identity. ONE parser, because a name can carry several suffixes and
    # their ORDER is not fixed: `ts1b_n3_mtare_hybrid_r40_ttl0_seed1` and
    # `at2_mtare_hybrid_ttl180_r40_seed2` are the same two design dimensions
    # written the other way round. See parse_cell_name() for what the previous
    # single-pass regex did to both of them (76 hard failures on a clean
    # campaign).
    #
    # Two names, and the distinction is load-bearing. `arm` is the CELL's
    # identity: what to count, what GATE_ARMS must have declared, which column
    # of the design this is. `policy_arm` is what the BINARY can stamp: the
    # reconnect policy alone, every runtime and design suffix removed, because
    # run_campaign.sh strips them before setting RECONNECT_MODE and the node
    # therefore stamps the bare policy name. Anything compared against
    # run_start or against the m-tare vocabulary uses policy_arm; everything
    # else uses arm. Comparing the full directory identity against the node's
    # stamp would hard-fail every correctly-run cr2, at2 and ts1b cell; what
    # the suffixes themselves claim is not dropped, it is held against the
    # manifest by check 3i below.
    arm, policy_arm, dir_claims, _leftover = parse_cell_name(c, TAG)
    cells_by_arm[arm] += 1
    cells_by_policy[policy_arm] += 1
    policy_of_arm[arm] = policy_arm
    row = {"cell": c, "arm": arm}

    # Control-ness is a property of the reconnect POLICY, not of a design
    # level: `mtare_off_r40_ttl0` is the control column of a two-factor design,
    # and a claim radius does not make it a treated cell. Testing membership on
    # the full identity alone would have read every suffixed control cell as
    # treated — check 3e would then demand reconnect_enabled=True of a correct
    # `off` run and hard-fail it, and check 3g would run the treated-arm stack
    # assertions over the control. Either spelling may be declared in
    # GATE_CONTROL_ARMS; the full identity is tried first so a design that
    # deliberately controls only ONE level of a factor can still say so.
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

    # 3k. the two roster witnesses must agree. run_campaign.sh writes `robots=`
    # from the value it launched; the node writes team_robot_names from the
    # list it was configured with. They are independent, so a disagreement
    # means one of the two is describing a team that did not run, and the
    # per-robot checks below are iterating the wrong set either way.
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
    # Same argument as 3e one level down. The analysis reads r10-vs-r40 and
    # ttl0-vs-ttl180 off the directory, so a launcher bug or a leaked
    # COORD_CLAIM_R that runs 10 m inside an _r40_ directory silently swaps the
    # design's two columns and every downstream number is attributed to the
    # wrong level. The manifest is written at launch from the value actually
    # passed, so it is the witness.
    #
    # Iterated over DESIGN_SUFFIXES rather than written out once for the claim
    # radius, so adding a dimension to that table adds its check with it. The
    # hand-written version covered the radius only, which is why ts1b's _ttl0
    # — a level that is the DEFAULT since 2026-09-05 and is therefore written
    # on almost every new cell — went unchecked.
    #
    # An absent key is only forgiven when the directory claims nothing either:
    # a campaign predating the override wrote no such line and its unsuffixed
    # cells really did run the yaml default. A suffixed directory with no line
    # to check it against is unverifiable, and unverifiable is not a pass.
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
    # NOT relaxed: anything other than CLEAN is a hard failure, because the
    # verdict is the harness's own statement that it could not certify the run.
    # The one exception is narrow and provable, and it exists because a check
    # that documents itself as report-only was failing whole campaigns.
    #
    # run_explo_sim_rviz.sh derives SUSPECT from `grep -c "^UNRUN"` over
    # comms_gates.txt, with no notion of which gate wrote the line. map_agree is
    # a REPORT-ONLY reading — it has no pass threshold and its own header says
    # so — but until 2026-09-15 it emitted UNRUN whenever it could not compute
    # (which, being hardcoded to two planner CSVs, was EVERY N>=3 cell). One
    # informational line it was never entitled to write therefore turned into
    # SUSPECT, and SUSPECT turned into a hard failure here: every cell of every
    # N>=3 campaign, rejected for a gate that does not gate.
    #
    # So a SUSPECT is re-read against the file it came from. If it holds UNRUN
    # lines and EVERY one of them names a report-only gate, the cell is
    # UNRESOLVED — not a pass, because the harness still declined to certify it
    # and an operator must look; not a hard failure, because nothing that can
    # invalidate a run is among the reasons. Any other shape stays a hard
    # failure, including a SUSPECT with no UNRUN lines at all: that is the
    # "watcher never reported" path, where the missing line is the outage gate
    # itself and absence of failures is emphatically not a pass.
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

    # 3a'. the nav binary, two ways the EXPECT loop above cannot cover.
    #
    # First: split provenance. EXPECT only carries NAV_BIN_KEY when EVERY cell
    # has it, so a campaign where only some do would otherwise drop the check
    # for the cells that lack it -- silently, and on exactly the cells whose
    # provenance is in question. A harness that changed mid-campaign splits the
    # run into two generations ([[commit-mid-campaign-splits-provenance]]).
    if _nav_present and not m.get(NAV_BIN_KEY):
        hard_fail.append(
            f"{c}: no {NAV_BIN_KEY} in the manifest, but "
            f"{len(_nav_present)}/{len(cells)} sibling cells have one — the "
            f"harness changed mid-campaign, so this cell cannot be pooled "
            f"with them")

    # Second: a partial install. The four nav executables are built and
    # installed as a unit; the campaign-wide value is pinned by declaration,
    # but internal disagreement is a defect on its own terms and needs no
    # declaration to detect. "missing" means the executable was not there when
    # the manifest was written, which for a run that produced nav logs means
    # the install tree and the running node had already diverged.
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
        # git_rev is NOT a top-level key of run_start. The node writes it with
        # addParamStr (explo_planner_node.cpp:3493, the EXPLO_PLANNER_GIT_REV
        # branch; :3495 is the "unknown" fallback), so it lands in
        # run_start.params.git_rev. Reading it at the top level yielded "" on
        # every real cell, and `anything.startswith("")` is True, so BOTH this
        # check and the -dirty witness below passed unconditionally — verified
        # by planting params.git_rev="deadbee-dirty" against a manifest saying
        # 322b6fc and getting "HARD FAILURES: none". A stale rev is exactly the
        # generation mix this gate exists to catch, so this was the check
        # failing at its one job while reporting a pass.
        #
        # A missing run_start is a hard failure rather than a skip. It used to
        # be `if start:` with no else, which meant a robot log truncated at the
        # head got none of 3b, 18b or 18c — and that is the robot-run whose
        # provenance you most want checked, not least.
        start = [e for e in ev if e.get("event") == "run_start"]
        if not start:
            hard_fail.append(
                f"{c}/{r}: no run_start event — checks 3b, 18b and 18c cannot "
                f"run on this robot, and a head-truncated log is not a pass")
        else:
            n_runstart_rows += 1
            pr = start[0].get("params", {})
            # 3l. Collect the configuration this robot-run actually resolved.
            # Read from run_start.params and not from the yaml, because the
            # yaml is a request: -p overrides beat it, and the "0 means auto"
            # knobs are rewritten at construction (coord_claim_radius_m=0.0
            # resolves to fov_max_range, which this generation doubles). The
            # node logs these AFTER resolution, so this is the outcome.
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
            # 3d. the schema stamp. The identity in section 32.14 pinned schema
            # 3, the binary writes it (experiment_log.cpp:287) and every reader
            # downstream keys off it, but nothing here read it — so the one
            # field that catches a generation mix at a glance was unchecked.
            # The pin now lives in SCHEMA_VERSION above, where an operator
            # scoring a banked campaign can move it from the command line
            # instead of from a diff nobody reviews.
            sv = start[0].get("schema_version", pr.get("schema_version"))
            if sv != SCHEMA_VERSION:
                hard_fail.append(
                    f"{c}/{r}: check 3d — schema_version={sv!r}, expected "
                    f"{SCHEMA_VERSION}")
            row.setdefault("mode_req", pr.get("arm"))
            # Either spelling: `reconnect_enabled` since the 2026-09-03 rename,
            # `rendezvous_enabled` in every cell up to and including cr5. The
            # node stamps both now; reading both keeps one gate over a mixed
            # set of campaigns.
            have_rdv = pr.get("reconnect_enabled")
            if have_rdv is None:
                have_rdv = pr.get("rendezvous_enabled")
            row.setdefault("rdv", have_rdv)
            # 3e. the treatment variable must not come from the directory name
            # alone. Everything downstream keys the arm off the cell directory,
            # so a launcher bug or a leaked environment variable that runs the
            # off configuration in a _hybrid_ directory corrupts the assignment
            # silently and the gate would have said CLEAN. mode_req/rdv were
            # parsed and PRINTED as an informational line; printing is not
            # checking.
            #
            # Compared against policy_arm, not arm: the node stamps the
            # reconnect policy, and per-cell runtime suffixes like _r<N> are
            # stripped by run_campaign.sh before it ever sees one. Comparing the
            # full directory identity here would hard-fail every correctly-run
            # cr2 cell. What the suffix itself claims is not dropped — check 3i
            # above holds it against the manifest.
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
            # The node reconstitutes the arm itself and prefixes `mtare_` when
            # `global_alloc_enable_ || reconnect_gate_info_ ||
            # rendezvous_schedule_enable_` — an OR. So the name `mtare_hybrid`
            # proves at least ONE of P3, P4 and P5 was live and never all
            # three, and check 3e, which compares that name against the stamp
            # it came from, is satisfied by a cell running a third of the
            # treatment. The arm's definition is a feature VECTOR; nothing
            # above asks about the vector.
            #
            # Two of them are worse than ambiguous, they are invisible.
            # cell_world_enable and team_world_hz rename NOTHING, so a cell
            # that ran no census or no exchange still carries an mtare_hybrid
            # directory name, an mtare_hybrid stamp, and passes 3e — and the
            # launcher guard that would have caught it cannot see an ambient
            # export either. These params are what the node was actually
            # handed, and they are the only record that says so.
            #
            # Checked in BOTH directions, and the control direction is the one
            # that decides the comparison. An `off` cell that somehow ran the
            # allocator is a treated cell sitting in the control column, and no
            # amount of care in the treated arm compensates for that.
            _mt_want = arm.startswith("mtare_")
            _mt_raw = {k: pr.get(k) for k in
                       ("cell_world_enable", "team_world_hz",
                        "global_alloc_enable", "reconnect_gate")}
            _mt_absent = sorted(k for k, v in _mt_raw.items() if v is None)
            # P5 added a FIFTH feature, and its param's absence does not mean
            # what the other four's does. Those four have been emitted
            # unconditionally since P4, so a cell missing one predates the
            # whole stack and cannot be certified at all. rendezvous_schedule_
            # enable instead DATES the cell: absent means a pre-P5 binary,
            # which is a perfectly legitimate thing to score — campaign mh1 is
            # one — but a different generation, in which the token
            # `mtare_hybrid` does not name the same arm it names after P5
            # (there was no appointment to switch on, so hybrid's fallback
            # destination was the midpoint).
            #
            # Handled by GENERATION rather than by exemption, because the two
            # loosenings on offer are both the failure this file exists to
            # prevent: dropping the fifth key from the vector would certify a
            # P7 mtare_hybrid cell that never armed an appointment, and
            # hard-failing its absence would make the gate unable to score any
            # campaign banked before today. So each cell is dated, a pre-P5
            # cell is held to the pre-P5 arm vocabulary, and a campaign that
            # mixes the two generations is rejected outright below (3h) — that
            # is pooling across binary generations, which is already forbidden.
            _p5_raw = pr.get("rendezvous_schedule_enable")
            _gen_p5 = _p5_raw is not None
            if _gen_p5:
                n_3g_gen_p5 += 1
                _gen_cells_p5.add(c)
            else:
                n_3g_gen_pre += 1
                _gen_cells_pre.add(c)
            # P6's discriminator, read the same way and for the same reason.
            # `pursuit_predictor` is emitted unconditionally by any binary that
            # has one, so its absence dates the cell rather than defaulting it
            # — and defaulting it to `trail` is exactly the wrong reflex here,
            # because that is the value a P6 `_mdp` cell must NOT have.
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
                    # policy_arm again: MTARE_ARM_STACK is keyed by the reconnect
                    # policy vocabulary, which has no room for a per-cell knob
                    # suffix. A radius does not change which m-tare features the
                    # arm promises, so mtare_hybrid_r10 must be looked up as
                    # mtare_hybrid or the whole stack assertion is skipped in
                    # favour of a spurious "not an arm this binary can stamp".
                    #
                    # _lookup_arm, not policy_arm, so a cell carrying a suffix
                    # this file has not learnt yet is still held to its stack:
                    # 3e has already reported the unknown token as UNRESOLVED,
                    # and falling back to the binary's own stamp checks the
                    # five features against the arm the binary says it ran
                    # rather than skipping the assertion entirely.
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

            # 3f. THE TREATMENT MUST HAVE BEEN ABLE TO HAPPEN. This is the
            # check g8r1 needed and did not have. That campaign was launched,
            # ran to completion, and passed every gate here — while the mid-run
            # clock sat at 240 s against an outage distribution whose p90 is
            # 52-111 s and only 0.00-3.34 % of whose outages reach 240 at all,
            # so the trigger expired in 3 of 23 hybrid cells and 87 %
            # of the treated arm was behaviourally the control. Nothing was
            # broken; the treatment was configured out of existence, and no
            # check asked whether it could fire at all.
            #
            # Read from run_start params, not the manifest. The manifest
            # records what the launcher INTENDED; these are what the node was
            # actually handed, and generation 9's whole provenance argument is
            # that only the second one explains a run.
            #
            # Applied to every arm EXCEPT `off`, not to "hybrid" by name: `off`
            # disables the manoeuvre on purpose and a check that fires on the
            # control every time is one nobody reads, but any other arm —
            # pursuit, rendezvous, a future one — reaches the same trigger and
            # has the same way of being configured out of existence. Naming the
            # one arm would have exempted the rest by accident.
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

                # CONFIGURED vs LIVE — two questions, and only asking the first
                # is how g8r1 passed. link_gate_configured is computed at
                # startRun from two topic NAMES being non-empty, so it is true
                # whenever the launcher typed them; it cannot know whether a
                # sample ever arrived, because nothing has been delivered yet
                # when it is written. A dead emulator therefore satisfies it
                # while the veto is absent for the whole run — the exact
                # configuration this generation exists to prevent.
                #
                # The runtime half is the one-shot "link_gate_live:" line the
                # node emits from its link-states subscription on the first
                # usable sample. (Deliberately not from linkGateReady(), which
                # only runs when something consults the gate: a run whose team
                # never went silent would emit nothing and be scored a failure.)
                # PRESENCE is asserted, not absence of the not-usable warning:
                # a run whose logging broke would pass an absence test
                # ([[nav-global-planner-never-planned]]).
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
            # mission_return_reentries is NOT failed on. The late coverage
            # latch legitimately asks to home a second time and the latch
            # refusing it is the fix working, so a non-zero value here is
            # evidence of health, not of a defect. It is collected for the
            # population report instead, where a campaign-wide zero can be seen
            # and questioned.
            if int(_ree) > 0:
                n_3m_reentry_runs.append(f"{c}/{r}={_ree}")

        # 3n. run_end is written exactly once — the OTHER half of the endpoint,
        # and the only one that cannot be read from the jsonl.
        #
        # Why this check lives in the planner log rather than beside 3m's two
        # run_end counters. `dup_run_ends_` can only become non-zero AFTER the
        # first run_end row is already on disk, so a `dup_run_ends` field
        # written beside mission_completes_suppressed would read 0 on every run
        # ever, including the runs it exists to catch — a field observed only
        # at zero because it is structurally unable to be anything else. And
        # appending a trailing `run_end_duplicate` event is worse than useless:
        # it breaks "run_end is the last line" and the
        # last-line-seq == events_written-1 invariant, and event_log.py's
        # `complete = (written == len(evs))` would then mark the run TRUNCATED
        # and drop the cell — destroying the evidence in the act of recording
        # it.
        #
        # So the witness is the planner's own stderr and this is its reader.
        # TWO markers, written by different code at different times: the WARN
        # fires during the run on the first suppressed attempt
        # (experiment_log.cpp:692), the destructor ERROR fires at close with
        # the total (experiment_log.cpp:73). Either alone is the finding, and
        # both are matched because the destructor runs during rclcpp teardown
        # and its output is the less certain of the two to reach the file.
        #
        # THE BASE RATE, so a clean reading is not over-read. The suppression
        # is new in generation 9; under generation 8 a second terminal state
        # would have written a second run_end ROW, and across the whole bank —
        # 3342 robot-runs, every campaign — there are ZERO files with more than
        # one. So this failure mode has never once been observed, and a future
        # "0 duplicates" is consistent with a working guard AND with a guard
        # that could never have fired. What the check actually buys is that the
        # generation-9 suppression, which makes the event invisible in the
        # jsonl for the first time, cannot make it invisible everywhere.
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
        # Assert each field against the writer that actually emits it. An
        # earlier version of this check demanded team_incomplete_sec on every
        # peer_lost/peer_seen row, which no binary has ever written: PeerEvent
        # has no such member and the only writer is logReconnectDispatch
        # (experiment_log.cpp:463, inside the reconnect_dispatch block at
        # :431-478). Run against a real cell it produced 34 hard
        # failures on clean data, and it survived calibration only because the
        # fixture manufactured the field the binary does not write — the
        # fixture asserting the gate's belief instead of the binary's output.
        # That is why the calibration is now seeded from a real cell.
        #
        # What generation 8 actually did to the peer rows was REMOVE
        # last_contact_age_sec. That half is the real migration witness.
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
    # The absent-file case is UNRESOLVED, not a skip. This check used to sit
    # under a bare `if os.path.exists(con):` with no else, so deleting or
    # failing to write the console log turned the whole process-death check off
    # and printed nothing at all — and a cell whose console log never got
    # written is precisely the cell most likely to have had a process die.
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
# Width from the data, not a literal. A fixed 34 was two characters short of
# `ts1b_n3_mtare_hybrid_r40_ttl0_seed1` — every row of every two-factor campaign
# overflowed its column and pushed the rest of the line out of alignment, which
# is how a table stops being read.
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
# Check 3b already pins one git_rev across the campaign, which catches this
# whenever the revision is recorded and readable. This is the same claim read
# off the BEHAVIOUR instead of off the provenance string, and it is worth
# having twice because the two fail differently: 3b compares a label, 3h
# compares what the binary actually emitted.
#
# The P5 param is the discriminator. A binary that has the scheduled
# rendezvous emits `rendezvous_schedule_enable` in every run_start, on or off;
# one that predates it emits nothing. So a campaign holding both kinds of cell
# pooled two generations under one set of arm names — and the arm names are
# exactly what does not survive that: `mtare_hybrid` means "chase, falling back
# to the midpoint" on one side of P5 and "chase, falling back to the agreed
# cell" on the other. Analysed together they are one arm run twice with two
# different treatments in it.
#
# P6 IS A SECOND SUCH BOUNDARY AND IS CHECKED SEPARATELY, not folded into the
# first. The two are not the same question: every cell banked between the two
# phases emits rendezvous_schedule_enable and no pursuit_predictor, so it is
# uniform by the P5 test while `mtare_hybrid` on it means the trail chase and
# `mtare_hybrid` on a P6 cell means the modelled one. Pooling those is the
# identical failure one name further down, and the P5 test cannot see it.
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
# Same shape as 3h and 3j, one layer further down: 3h refuses to pool two
# binaries, 3j two radios, this refuses to pool two sensor models. The
# generation-9 lidar FOV change (full azimuth, 96 rays, 20 m) is a params-file
# change, and a params-file change moves NO field this gate previously read --
# git_explo_planner does not move for it, and sha256_shared_params moves but is
# compared to nothing. A stale installed yaml therefore produced a cell that ran
# the generation-8 sensor under a manifest that said generation 9 everywhere.
#
# And it prints the configuration whether or not it varies, because the reason
# this check exists is that "D1 was in effect" was an assumption with no witness
# in the run record. An operator reading a green gate should be able to see the
# FOV the campaign actually ran, not infer it from the yaml in the source tree.
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

# The params file is the other half of what the node ran, and until now the
# gate never looked at it. Within an arm it is a constant; between arms it can
# legitimately differ, because varying a yaml-only knob per arm inside one
# invocation is done by giving the arm its own params file
# ([[per-cell-knobs-ride-the-arm-suffix]]).
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
# Same shape as 3h one layer down: 3h refuses to pool two binaries, this
# refuses to pool two radios. Every connectivity result — deep_outage_sec, the
# dropout count, the %-time-connected mediator, and through it the completion
# time — is conditional on how much a trunk costs and how far a clear lane
# reaches. Two regimes under one tag is not a noisier campaign, it is two
# experiments whose arm means something different in each.
#
# Read off the MANIFEST, which is written at launch from the values actually
# passed to the emulator. The alternative — inferring the regime from observed
# dropouts — cannot work: dropout rate is exactly the thing the regime is
# supposed to explain, so inferring one from the other would make the check
# agree with itself.
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
    # 3g's two populations are exempt only when the PRE-REGISTRATION says the
    # arm does not exist. A hybrid-vs-off campaign has no m-tare arm and must
    # not be marked unresolved for the absence of one; an all-m-tare campaign
    # has no untreated arm and likewise. What is never exempt is a campaign
    # that declared both and produced only one — that is the check failing to
    # look at half the design, and it must stay UNRESOLVED.
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

# check 3m, the mixture half. NOT relaxable by GATE_ENDPOINT_FIELDS, because
# the flag says "this campaign predates the fields" and a campaign where SOME
# robot-runs carry them and some do not is not that campaign — it is two binary
# generations inside one directory, which is never a design. Found by the
# negative control for the relaxation itself: with the flag set and the fields
# stripped from one arm only, the gate reported a non-zero witness count and
# read as though everything had been checked.
if n_3m_fields and n_3m_fields < n_3m_rowcount:
    hard_fail.append(
        f"check 3m — {n_3m_fields} of {n_3m_rowcount} robot-runs carry the "
        f"generation-9 endpoint counters and {n_3m_rowcount - n_3m_fields} do "
        f"not, so this campaign MIXES binary generations. GATE_ENDPOINT_FIELDS "
        f"does not relax this: it declares a campaign to predate the fields, "
        f"and this one does not predate them uniformly")

# check 3m, the health half. Printed unconditionally — including the zero case,
# in words — because a guard that reports only when it fires is
# indistinguishable from a guard that was compiled out. The fix is not FAILED
# on a zero: a campaign of short runs may genuinely never latch coverage after
# homing. It is reported so the zero has to be looked at rather than inferred.
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

# 21. the campaign is the shape it was pre-registered as. Nothing above counts
# cells, so a campaign that died after three of them scored CLEAN and invited
# analysis of a truncated, arm-unbalanced dataset. Seed-major ordering makes an
# early abort systematically unbalanced, so "small" here also means "biased".
# Both lists are printed because both are overridable from the environment, and
# an override is invisible in the output it changes: a reader seeing "campaign
# shape: {...} CLEAN" cannot tell whether check 21 compared against the
# pre-registered pair or against whatever GATE_ARMS happened to be exported in
# that shell.
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
# One specific disagreement deserves its own sentence, because the generic
# messages below describe it as a launcher bug when it is a declaration that is
# one factor short. If the declared list matches the campaign's reconnect
# POLICIES exactly, and the only difference is that every directory carries a
# design suffix on top of them, then no cell is mislabelled: GATE_ARMS was
# written for a one-factor design and the campaign ran a two-factor one. It
# stays a hard failure — the count check is exactly what notices that only one
# level of the new factor ran, and it cannot notice that at the policy level.
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

# Three states, three exit codes. UNRESOLVED is the gate's designed answer for
# an empty population — the property the module docstring pins — and it was
# invisible to $?, so any wrapper branching on the exit code read "nothing was
# tested" as "everything passed". That is the failure this gate was written to
# prevent, reproduced in the gate's own interface.
#
# 3 rather than 1 because unresolved is not a failure: it is a claim that part
# of the gate had no data to run on, which the operator must read and judge. A
# campaign with the off arm in it will legitimately have empty reconnect
# populations, so 3 is expected and is not an error — it means "read the
# UNRESOLVED list before believing this".
if hard_fail:
    sys.exit(1)
sys.exit(3 if unresolved else 0)
