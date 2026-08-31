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

Usage:  gate_g8.py [TAG]            default TAG g8r1
Env:    GATE_ROOT                   campaign root (default /home/kalhan/hmr_campaign)
        GATE_IDENTITY               path to the declared-identity file
        GATE_EXPECT_<key>           override a single declared identity field
        GATE_CELLS_PER_ARM          pre-registered cells per arm (default 30)
        GATE_ARMS                   pre-registered arms, comma-separated
                                    (default "hybrid,off")
        GATE_CONTROL_ARMS           arms that run with the manoeuvre disabled
                                    (default "off")
        GATE_SCHEMA_VERSION         event-log schema to require (default 4;
                                    pass 3 to re-gate a banked gen-9 campaign)
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
ROBOTS = ["atlas", "bestla"]

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
# pre-registration records the same two values, and the identity file being
# outside git is not a weakness here: it is written BEFORE the first cell runs
# and any later edit is visible in its mtime against the campaign's own cells.
#
# Env overrides exist for the calibration harness, which must point the real
# logic at synthetic cells of known identity.
# ---------------------------------------------------------------------------
def _declared_identity():
    """The two build-dependent identity fields, from env or the identity file."""
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
    for k in ("git_explo_planner", "sha256_explo_planner_node"):
        if os.environ.get(f"GATE_EXPECT_{k}"):
            out[k] = os.environ[f"GATE_EXPECT_{k}"]
    return out


_decl = _declared_identity()
EXPECT = {
    "git_explo_planner": _decl.get("git_explo_planner", "FILL_ME"),
    "git_simple_nav_3d": "c9f83a7",
    "git_scovox": "078d3f7",
    "sha256_explo_planner_node": _decl.get("sha256_explo_planner_node",
                                           "FILL_ME"),
}

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
SCHEMA_VERSION = int(os.environ.get("GATE_SCHEMA_VERSION", "4"))
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

# The control arm, and the ONLY arm that runs with rendezvous_enabled=false.
# Everything else — hybrid, pursuit, rendezvous and the three treated
# `mtare_*` tokens — reaches the manoeuvre and must have it enabled. Check 3e used to spell this as
# `arm == "hybrid"`, which asserted rendezvous_enabled=False for a pursuit or
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
_orphan_controls = CONTROL_ARMS - set(EXPECT_ARMS)
if _orphan_controls:
    sys.stderr.write(
        f"FATAL: GATE_CONTROL_ARMS names {sorted(_orphan_controls)}, which "
        f"is not in GATE_ARMS {list(EXPECT_ARMS)}. A control arm that is not "
        f"an expected arm exempts nothing.\n")
    sys.exit(2)
if not (CONTROL_ARMS & set(EXPECT_ARMS)):
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
    print("REFUSING TO RUN: the generation-8 identity is not declared.\n"
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
# separated by reconnect_mode / rendezvous_enabled, which check 3e already
# compares against the directory name; this table is about the stack, not the
# mode.
MTARE_ARM_STACK = {
    "mtare_off": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": False,
        "rendezvous_schedule_enable": False},
    "mtare_pursuit": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": True,
        "rendezvous_schedule_enable": False},
    "mtare_rendezvous": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": True,
        "rendezvous_schedule_enable": True},
    "mtare_hybrid": {
        "cell_world_enable": True, "team_world_hz>0": True,
        "global_alloc_enable": True, "reconnect_gate=info": True,
        "rendezvous_schedule_enable": True},
}
# Which m-tare arm tokens a PRE-P5 binary could have stamped. Only one: the
# three factorial names did not exist, and the runner refused every route to
# `mtare_off` (the arm-name/arm-stamp check rejected a plain token with the
# allocator on, and RECONNECT_GATE=info with RECONNECT_MODE=off was fatal). So
# seeing one of them on a pre-P5 cell means the directory was renamed after the
# fact, not that the binary ran that arm.
MTARE_ARMS_PRE_P5 = {"mtare_hybrid"}


def mtare_expect(arm, gen_p5):
    """The feature vector `arm` is defined to carry, or None if a binary of
    this generation cannot stamp that arm at all.

    `gen_p5` selects the vocabulary AND the vector width: a pre-P5 binary emits
    no rendezvous_schedule_enable, so asking about it would compare against a
    key the observed vector does not have."""
    if arm.startswith("mtare_"):
        if not gen_p5 and arm not in MTARE_ARMS_PRE_P5:
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
        want = {k: False for k in MTARE_ARM_STACK["mtare_hybrid"]}
    if not gen_p5:
        want.pop("rendezvous_schedule_enable", None)
    return want


hard_fail, soft, unresolved = [], [], []
agg_recovery_entries = 0
rows = []
nav_fail_by_arm = collections.Counter()
cells_by_arm = collections.Counter()

# Populations for the new checks. Counted so an empty one can be reported as
# UNRESOLVED instead of passing by vacuity.
n_csv_rows = n_hw_fire = n_hw_escape = n_reconnect_end = 0
n_navfail_rows = n_peer_rows = n_runend_rows = n_midrun = 0
n_disp_rows = n_runstart_rows = n_console_logs = 0
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
_live_reported = {}

for c in cells:
    d = os.path.join(ROOT, c)
    m = manifest(d)
    # Parse the arm out of the flat cell name <TAG>_<arm>_seed<N> rather than
    # asking "is 'hybrid' in it, else off". That binary form silently relabels
    # every OTHER arm as the control: a _pursuit_ or _rendezvous_ cell landed in
    # the `off` bucket, where check 3e then compared run_start's arm=pursuit
    # against a directory reading of "off" and hard-failed a correct run — or,
    # worse, where a genuinely-off cell and a pursuit cell get pooled in the
    # per-arm counts. Unrecognised shapes fall back to the whole middle token,
    # which check 3e then catches against the node's own answer.
    #
    # The trailing _seek is a RUNTIME switch, not an arm. run_campaign.sh strips
    # it before setting RECONNECT_MODE (see its cell_mode/cell_seek block), so a
    # <TAG>_hybrid_seek_seed<N> directory holds a node that stamped arm=hybrid.
    # Reading the directory literally invents an arm the node never reports:
    # check 3e then hard-fails "directory says hybrid_seek, params say hybrid"
    # on a perfectly good cell, check 21 finds an unexpected arm, and check 3f
    # would run the treated-arm assertions over an off_seek control. Such cells
    # are banked (ds1_hybrid_seek_seed1..4), so this is not hypothetical.
    _mid = c[len(TAG) + 1:]
    arm = _mid.rsplit("_seed", 1)[0] if "_seed" in _mid else _mid
    if arm.endswith("_seek"):
        arm = arm[:-len("_seek")]
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
        #
        # git_rev is NOT a top-level key of run_start. The node writes it with
        # addParamStr (explo_planner_node.cpp:2520), so it lands in
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
            # 3, the binary writes it (experiment_log.cpp:265) and every reader
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
            row.setdefault("rdv", pr.get("rendezvous_enabled"))
            # 3e. the treatment variable must not come from the directory name
            # alone. Everything downstream keys the arm off the cell directory,
            # so a launcher bug or a leaked environment variable that runs the
            # off configuration in a _hybrid_ directory corrupts the assignment
            # silently and the gate would have said CLEAN. mode_req/rdv were
            # parsed and PRINTED as an informational line; printing is not
            # checking.
            want_rdv = arm not in CONTROL_ARMS
            if pr.get("arm") is not None and pr.get("arm") != arm:
                hard_fail.append(
                    f"{c}/{r}: check 3e — directory says arm={arm} but "
                    f"run_start params say arm={pr.get('arm')!r}")
            if pr.get("rendezvous_enabled") is not None \
                    and bool(pr.get("rendezvous_enabled")) != want_rdv:
                hard_fail.append(
                    f"{c}/{r}: check 3e — arm={arm} but rendezvous_enabled="
                    f"{pr.get('rendezvous_enabled')!r}")
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
                    _want = mtare_expect(arm, _gen_p5)
                    if _want is None:
                        hard_fail.append(
                            f"{c}/{r}: check 3g — arm={arm} is not an arm a "
                            f"{'P5-or-later' if _gen_p5 else 'pre-P5'} binary "
                            f"can stamp (known: "
                            f"{sorted(MTARE_ARM_STACK if _gen_p5 else MTARE_ARMS_PRE_P5)}"
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
            if arm not in CONTROL_ARMS:
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
        # (experiment_log.cpp:436). Run against a real cell it produced 34 hard
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
        ("treated-arm run_start rows",  n_3f_runs,       "3f"),
        ("link-gate live witnesses",    n_3f_live_checked, "3f"),
        ("m-tare features certified ON", n_3g_treated,   "3g"),
        ("m-tare features certified OFF", n_3g_control,  "3g"),
        ("binary-generation witnesses", n_3g_gen_p5 + n_3g_gen_pre, "3h"),
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
                  f"   <- no such arm in GATE_ARMS {list(EXPECT_ARMS)}")
            continue
    print(f"  check {check:>2}  {label:28} {n:6d}{mark}")
    if not n:
        unresolved.append(f"check {check}: {label} — zero rows, nothing was tested")

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
print(f"\narms expected: {list(EXPECT_ARMS)} "
      f"(control: {sorted(CONTROL_ARMS)}, treated: "
      f"{[a for a in EXPECT_ARMS if a not in CONTROL_ARMS]})")
print(f"campaign shape: {dict(cells_by_arm)} "
      f"(pre-registered {EXPECT_CELLS_PER_ARM} per arm)")
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
