#!/usr/bin/env python3
"""Default-off equivalence gate (docs/mtare_evolution_plan.md §6).

Every M-TARE phase lands param-gated default-off: at shipped defaults the new
binary must be the SAME PLANNER as its parent commit. This file is how that
claim is checked, per phase, before anything measured on the new binary is
believed.

What it does NOT compare, and why
---------------------------------
Not event sequences, not row counts, not the latch step, not completion time,
not distance. The sim is nondeterministic run-to-run under a fixed seed — the
same seed has produced 1803 s and 856 s — so any gate built on those quantities
has exactly two possible fates: it fails on honest same-binary pairs until
somebody widens the tolerance, and then it passes everything forever. A gate
that cannot fail is worse than no gate, because it is mistaken for evidence.

What it compares instead is the structure a run CANNOT vary by chance:

  1. schema_version               same within a side; child >= parent
  2. the run_start param dump     no param lost, no shared param re-valued,
                                  every param NEW in the child sitting at the
                                  default compiled into the child's own source
  3. the set of event kinds       nothing outside the declared pre-v4
                                  vocabulary may appear at defaults
  4. the armed comms-gate names   the same checks were armed on both sides
  5. the manifest arm block       the two campaigns were configured alike

Each of those is a property of the CONFIGURATION, not of how the run happened
to unfold, so a same-binary pair passes deterministically and a phase that
switched something on fails deterministically.

Where the defaults come from
----------------------------
Not from a list kept in this file. From `dp("name", default)` in the planner's
own source — the same trick test_experiment_log and test_failed_goal_blacklist
use, for the same reason: a hand-copied expectation drifts from the binary the
first time somebody adds a param and forgets this file exists.

A default this file cannot parse is a HARD FAILURE, never a skip. That is the
whole reason the parser is allowed to be simple: it does not have to understand
every C++ initialiser, it only has to refuse to guess.

Populations
-----------
An empty side is UNRESOLVED, not PASS. "I found no cells to compare" and "I
compared them and they matched" are different answers and this file will not
conflate them.

Usage:  equiv_gate.py PARENT_ROOT CHILD_ROOT
        Each root is a campaign directory of cell subdirectories, or one cell
        directory. Cells are matched by nothing — the comparison is over the
        UNION of each side, since seeds and cell names differ between runs.
Env:    EQUIV_NODE_SRC   explo_planner_node.cpp (default: alongside this file)
        EQUIV_LOG_HPP    experiment_log.hpp     (default: alongside this file)
        EQUIV_ALLOW_NEW_KINDS   comma-separated event kinds permitted to appear
                                in the child. For the phase that turns a
                                mechanism ON, so the same file can score a
                                treatment arm; NEVER for a defaults run.
Exit:   0  equivalent
        1  a difference that defeats the default-off claim
        2  usage, or a default the parser refused to guess at
        3  a population was empty — nothing was actually compared
"""

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..", "explo_planner")
NODE_SRC = os.environ.get(
    "EQUIV_NODE_SRC", os.path.join(PKG, "src", "explo_planner_node.cpp"))
LOG_HPP = os.environ.get(
    "EQUIV_LOG_HPP",
    os.path.join(PKG, "include", "explo_planner", "experiment_log.hpp"))

# Run_start fields that are dumped as params but are DERIVED rather than
# declared: addParamNum/addParamStr writes them, so no dp("name", default) call
# exists to read a default from. The value here is what the derivation yields
# when its inputs are at THEIR defaults — i.e. the unconfigured state a legacy
# run shows.
#
# This is the one hand-maintained list in this file, so it is arranged to fail
# loudly rather than silently: it is consulted only for params dp() did not
# supply, and a param in neither place is a hard failure. Forgetting to add an
# entry cannot make the gate quietly accept something.
DERIVED_DEFAULTS = {
    "robot_id": -1.0,    # fleet_.self_id with team_robot_names empty
    "team_hash": 0.0,    # fleet_.team_hash likewise
}

# Params whose value is EXPECTED to differ between two campaigns and says
# nothing about the binary's behaviour. Kept deliberately short: every entry is
# a check being given up, so each one names the reason it is not evidence.
VOLATILE_PARAMS = {
    "git_rev",       # the point of the exercise is that these differ
    "build_stamp",
    "output_csv",    # contains the cell directory name
    "experiment_log_path",
    "robot_name",    # compared per robot, not across robots
}

# Manifest keys that vary per run by construction. Everything NOT listed here is
# treated as part of the arm configuration and must match, so a key added to the
# manifest later is compared by default rather than ignored by default.
VOLATILE_MANIFEST = {
    "started_utc", "host", "outdir", "ign_partition", "seed",
    "run_end_reason", "run_end_t_sim", "run_gates_verdict",
    "comms_trees_loaded", "targets", "experiment_log",
}
VOLATILE_MANIFEST_PREFIXES = ("git_", "sha256_", "mtime_")

# Manifest keys a phase ADDS to the record. A phase that ships a subsystem
# default-off still writes that subsystem's whole knob block to the manifest,
# deliberately: "this run had the knob and it was 0" and "this run predates the
# knob" are different facts, and within one binary generation the node hash
# cannot separate them. run_explo_sim_rviz.sh says so at the write site. So the
# child's manifest legitimately carries keys the parent's does not, and a plain
# set-difference reads EVERY phase boundary as a configuration difference —
# which is the fastest way to get an equivalence gate switched off.
#
# The permission granted here is not "new keys are fine". It is the rule block 2
# already applies to params, transposed: a new key is inert only if the switch
# that would make it matter is present and OFF. "At its compiled default" is the
# wrong test for these, because they are HARNESS variables and the harness
# overrides several on purpose — CELL_COVERED_U ships 0.55 against a library
# default of 0.15 (a world-calibrated knob, cf. done_unknown_fraction) and is
# still inert while cell_world=0.
#
# Each entry is  gate key -> (off value, keys it gates). Hand-maintained, and
# arranged to fail loudly like DERIVED_DEFAULTS above: a new key in neither the
# gate-key position nor some group's dependent set is a hard failure, so
# forgetting an entry cannot make the gate quietly accept a live knob.
GATED_MANIFEST_GROUPS = {
    "cell_world": ("0", {
        "cell_size_m", "cell_census_period_s",
        "cell_covered_max_unknown", "cell_exploring_min_unknown",
        "cell_covered_max_frontier_frac",
        # The roster is passed to the node only inside the CELL_WORLD=1 branch,
        # so with the gate off it reaches nothing: robot_id and team_hash stay
        # at the unconfigured -1/0 that DERIVED_DEFAULTS records. That is not
        # assumed here — block 2 compares them, and would fail first.
        "team_robot_names",
    }),
    "team_world": ("0", {
        # team_world_hz is written to the manifest as 1.0 even when the exchange
        # is off, because the launcher's TEAM_WORLD_HZ default is 1.0 and the
        # manifest records the harness variable, not what reached the node. The
        # PARAM dump is the other way round — the -p is passed only inside the
        # TEAM_WORLD=1 branch, so with the switch off the node logs the compiled
        # 0.0 and block 2 above compares it against dp("team_world_hz", 0.0).
        # Both facts are wanted: block 2 proves the binary ran at its default,
        # this block proves the harness knob that would have changed that was
        # off. Neither substitutes for the other, and the pair is why the two
        # values may legitimately disagree here.
        "team_world_hz",
    }),
}


class GateError(Exception):
    pass


# ----------------------------------------------------------------------------
# The declared event vocabulary, read from the writer's header
# ----------------------------------------------------------------------------

def declared_vocabulary(path):
    """Return (legacy_kinds, v4_kinds) from experiment_log.hpp.

    The split is kFirstV4EventKind, which the header maintains next to the list
    itself and a unit test pins against the writer's source. This file therefore
    never holds an opinion about which kinds are new — it asks the binary.
    """
    try:
        src = open(path, errors="replace").read()
    except OSError as e:
        raise GateError(f"cannot read the event vocabulary: {e}")
    m = re.search(r"kEventKinds\[\]\s*=\s*\{(.*?)\}\s*;", src, re.S)
    if not m:
        raise GateError(f"{path}: no kEventKinds[] initialiser found")
    kinds = re.findall(r'"([^"]+)"', m.group(1))
    b = re.search(r"kFirstV4EventKind\s*=\s*(\d+)", src)
    if not b:
        raise GateError(f"{path}: no kFirstV4EventKind found")
    cut = int(b.group(1))
    if not 0 < cut <= len(kinds):
        raise GateError(
            f"{path}: kFirstV4EventKind={cut} is outside kEventKinds "
            f"({len(kinds)} entries) — the boundary and the list disagree")
    return set(kinds[:cut]), set(kinds[cut:])


# ----------------------------------------------------------------------------
# Compiled parameter defaults, read from the node's own source
# ----------------------------------------------------------------------------

UNPARSEABLE = object()


def _default_expr(src, i):
    """Return the raw default expression of the dp( call whose '(' is at i."""
    depth, j = 0, i
    while j < len(src):
        if src[j] == "(":
            depth += 1
        elif src[j] == ")":
            depth -= 1
            if depth == 0:
                break
        j += 1
    else:
        return None
    inner = src[i + 1:j]
    # Split on the top-level comma only: std::clamp(x, lo, hi) as a default has
    # commas of its own, and splitting naively would hand back a fragment that
    # then "parses" as a number.
    depth, cut = 0, None
    for k, ch in enumerate(inner):
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        elif ch == "," and depth == 0:
            cut = k
            break
    if cut is None:
        return None
    return inner[cut + 1:].strip()


def _literal(expr):
    """Interpret a C++ default expression, or UNPARSEABLE. Never guesses."""
    e = " ".join(expr.split())
    m = re.fullmatch(r'std::string\s*\(\s*"((?:[^"\\]|\\.)*)"\s*\)', e)
    if m:
        return m.group(1).encode().decode("unicode_escape")
    if re.fullmatch(r'"((?:[^"\\]|\\.)*)"', e):
        return e[1:-1].encode().decode("unicode_escape")
    # An empty vector reaches the log as the empty joined string (the node
    # joins with commas before writing it). A NON-empty vector default would
    # need the same join and there is none in the tree, so it is left
    # unparseable rather than half-handled.
    if re.fullmatch(r"std::vector\s*<\s*std::string\s*>\s*\{\s*\}", e):
        return ""
    if e in ("true", "false"):
        return e == "true"
    m = re.fullmatch(r"[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?[fF]?", e)
    if m:
        return float(e.rstrip("fF"))
    return UNPARSEABLE


def source_defaults(path):
    """Map param name -> compiled default (or UNPARSEABLE) from dp(...) calls."""
    try:
        src = open(path, errors="replace").read()
    except OSError as e:
        raise GateError(f"cannot read the param defaults: {e}")
    out = {}
    for m in re.finditer(r'\bdp\s*\(\s*"([A-Za-z0-9_]+)"\s*,', src):
        name = m.group(1)
        open_paren = src.index("(", m.start())
        expr = _default_expr(src, open_paren)
        out[name] = UNPARSEABLE if expr is None else _literal(expr)
    if not out:
        raise GateError(f"{path}: no dp(\"name\", default) calls found — the "
                        f"param block was renamed and this gate went blind")
    return out


# ----------------------------------------------------------------------------
# Reading a side
# ----------------------------------------------------------------------------

def _read_manifest(path):
    out = {}
    if not os.path.exists(path):
        return out
    for ln in open(path, errors="replace"):
        ln = ln.strip()
        if not ln or ln.startswith("#") or "=" not in ln:
            continue
        k, _, v = ln.partition("=")
        out[k.strip()] = v.strip()
    return out


def _cell_dirs(root):
    if not os.path.isdir(root):
        raise GateError(f"not a directory: {root}")
    if any(f.endswith(".events.jsonl") for f in os.listdir(root)):
        return [root]                                   # a single cell
    return [os.path.join(root, d) for d in sorted(os.listdir(root))
            if os.path.isdir(os.path.join(root, d))]


def read_side(root, label):
    """Collect the run-invariant structure of every robot-run under root."""
    runs = []
    for d in _cell_dirs(root):
        manifest = _read_manifest(os.path.join(d, "run_manifest.txt"))
        gates = set()
        gpath = os.path.join(d, "comms_gates.txt")
        if os.path.exists(gpath):
            for ln in open(gpath, errors="replace"):
                parts = ln.split("\t")
                if len(parts) >= 2:
                    gates.add(parts[1].strip())
        for f in sorted(os.listdir(d)):
            if not f.endswith(".events.jsonl"):
                continue
            robot = f[:-len(".events.jsonl")]
            params, schema, kinds = None, None, set()
            for ln in open(os.path.join(d, f), errors="replace"):
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                kind = e.get("event")
                if kind:
                    kinds.add(kind)
                if kind == "run_start" and params is None:
                    params = e.get("params", {})
                    schema = e.get("schema_version")
            if params is None:
                # Not a skip: a robot-run with no run_start has no param dump,
                # so its side of the comparison is missing rather than equal.
                raise GateError(
                    f"{label}: {os.path.basename(d)}/{robot} has no run_start "
                    f"— its configuration cannot be compared")
            runs.append({"cell": os.path.basename(d), "robot": robot,
                         "params": params, "schema": schema, "kinds": kinds,
                         "manifest": manifest, "gates": gates})
    return runs


# ----------------------------------------------------------------------------
# The comparison
# ----------------------------------------------------------------------------

def _same_value(a, b):
    if isinstance(a, bool) != isinstance(b, bool):
        return False
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) \
            and not isinstance(a, bool):
        return abs(float(a) - float(b)) <= 1e-9 * max(1.0, abs(float(a)))
    return a == b


def _agreed(runs, pick, what, label, fails):
    """The value of `pick` across a side, or a failure if the side disagrees."""
    seen = {}
    for r in runs:
        seen.setdefault(repr(pick(r)), []).append(f"{r['cell']}/{r['robot']}")
    if len(seen) > 1:
        detail = "; ".join(f"{v} in {len(c)} run(s) e.g. {c[0]}"
                           for v, c in sorted(seen.items()))
        fails.append(f"{label}: {what} is not constant within the side — "
                     f"{detail}. The side is not one configuration, so there "
                     f"is nothing for the other side to be equivalent to.")
        return None
    return pick(runs[0])


def compare(parent, child, legacy_kinds, v4_kinds, defaults, allow_new):
    fails, notes = [], []

    # 1. schema -------------------------------------------------------------
    ps = _agreed(parent, lambda r: r["schema"], "schema_version", "parent",
                 fails)
    cs = _agreed(child, lambda r: r["schema"], "schema_version", "child", fails)
    if ps is not None and cs is not None:
        if cs < ps:
            fails.append(f"schema_version went backwards: child {cs} < parent "
                         f"{ps}")
        else:
            notes.append(f"schema_version parent {ps} -> child {cs}")

    # 2. the param dump -----------------------------------------------------
    pp = {k: v for r in parent for k, v in r["params"].items()}
    for r in parent:                       # a side must agree with itself
        for k, v in r["params"].items():
            if k not in VOLATILE_PARAMS and not _same_value(pp[k], v):
                fails.append(f"parent: param {k} differs between runs "
                             f"({pp[k]!r} vs {v!r}) — the side is not one "
                             f"configuration")
    cp = {k: v for r in child for k, v in r["params"].items()}
    for r in child:
        for k, v in r["params"].items():
            if k not in VOLATILE_PARAMS and not _same_value(cp[k], v):
                fails.append(f"child: param {k} differs between runs "
                             f"({cp[k]!r} vs {v!r}) — the side is not one "
                             f"configuration")

    for k in sorted(set(pp) - set(cp)):
        fails.append(f"param {k!r} present in the parent and gone in the "
                     f"child: a removed knob is a behaviour change even when "
                     f"nothing else moved")
    for k in sorted(set(pp) & set(cp)):
        if k in VOLATILE_PARAMS:
            continue
        if not _same_value(pp[k], cp[k]):
            fails.append(f"param {k!r} re-valued: parent {pp[k]!r} -> child "
                         f"{cp[k]!r}. Whatever else the two campaigns differ "
                         f"in, it is not only the binary.")
    new = sorted(set(cp) - set(pp))
    for k in new:
        if k not in defaults and k not in DERIVED_DEFAULTS:
            fails.append(
                f"param {k!r} is new in the child but no dp(\"{k}\", ...) call "
                f"was found in {os.path.basename(NODE_SRC)} and it is not a "
                f"known derived field — this gate cannot tell whether it is at "
                f"its default. Add a dp() default or an entry in "
                f"DERIVED_DEFAULTS; do not delete this check.")
            continue
        d = defaults.get(k, DERIVED_DEFAULTS.get(k))
        if d is UNPARSEABLE:
            # Exit 2, not 1: the phase may well be fine. What failed is this
            # file's ability to say so, and that is not a verdict.
            raise GateError(
                f"cannot parse the compiled default of new param {k!r} from "
                f"{os.path.basename(NODE_SRC)}. Refusing to assume it is at "
                f"its default; teach _literal() this initialiser form.")
        if not _same_value(d, cp[k]):
            fails.append(
                f"new param {k!r} is NOT at its default: the child logged "
                f"{cp[k]!r}, the source compiles in {d!r}. This run is a "
                f"treatment arm, not a defaults run, and proves nothing about "
                f"default-off equivalence.")
    if new:
        notes.append(f"{len(new)} new param(s) at defaults: "
                     f"{', '.join(new)}")

    # 3. the event vocabulary ----------------------------------------------
    # Against the DECLARED legacy set, not against what the parent runs happened
    # to emit. A rare kind the parent never reached (nav_goal_failed, say) is
    # run-to-run variation, and failing on it would make the gate fail honest
    # pairs until somebody disabled it.
    child_kinds = set().union(*(r["kinds"] for r in child))
    parent_kinds = set().union(*(r["kinds"] for r in parent))
    undeclared = child_kinds - legacy_kinds - v4_kinds
    if undeclared:
        fails.append(f"child emits event kind(s) {sorted(undeclared)} that "
                     f"experiment_log.hpp does not declare at all")
    opted_in = sorted((child_kinds & v4_kinds) - allow_new)
    if opted_in:
        fails.append(
            f"child emits opt-in event kind(s) {opted_in} at defaults. Either "
            f"a new mechanism is on when it should not be, or its event is "
            f"written outside its own param gate.")
    if allow_new & child_kinds:
        notes.append(f"permitted new kind(s) present: "
                     f"{sorted(allow_new & child_kinds)} "
                     f"(EQUIV_ALLOW_NEW_KINDS — this is NOT a defaults run)")
    missing = sorted((parent_kinds & legacy_kinds) - child_kinds)
    if missing:
        # Not a failure: whether a run reaches nav_goal_failed is exactly the
        # kind of thing the nondeterminism moves. Reported so a WHOLE category
        # vanishing is at least visible to a reader.
        notes.append(f"kind(s) in the parent and not the child: {missing} "
                     f"(expected under run-to-run variation; investigate only "
                     f"if a mechanism's entire vocabulary disappeared)")

    # 4. armed gates --------------------------------------------------------
    pg = set().union(*(r["gates"] for r in parent)) if parent else set()
    cg = set().union(*(r["gates"] for r in child)) if child else set()
    if pg and cg and pg != cg:
        fails.append(f"different comms gates armed: parent-only "
                     f"{sorted(pg - cg)}, child-only {sorted(cg - pg)}")
    elif not pg or not cg:
        notes.append("comms_gates.txt absent on at least one side; the armed-"
                     "gate set was NOT compared")

    # 5. the manifest arm block --------------------------------------------
    pm = {k: v for r in parent for k, v in r["manifest"].items()}
    cm = {k: v for r in child for k, v in r["manifest"].items()}

    def arm_keys(m):
        return {k for k in m if k not in VOLATILE_MANIFEST
                and not k.startswith(VOLATILE_MANIFEST_PREFIXES)}

    if not pm or not cm:
        notes.append("run_manifest.txt absent on at least one side; the arm "
                     "configuration was NOT compared")
    else:
        # Which group each dependent key belongs to, and whether that group's
        # switch is present and off ON THE CHILD SIDE. Read once, so a key is
        # never excused by a switch that is itself absent.
        gated_by = {d: g for g, (_, deps) in GATED_MANIFEST_GROUPS.items()
                    for d in deps}
        inert = set()
        for g, (off, deps) in GATED_MANIFEST_GROUPS.items():
            if cm.get(g) == off:
                inert |= deps
        relaxed = []
        for k in sorted(arm_keys(pm) | arm_keys(cm)):
            a, b = pm.get(k, "<absent>"), cm.get(k, "<absent>")
            if a == b:
                continue
            if k in pm and k not in cm:
                # Never relaxed, in either direction of the phase sequence. A
                # key that stops being written is a hole in the record, and a
                # hole cannot be distinguished from agreement by reading it.
                fails.append(f"manifest {k}: the parent recorded {a!r} and the "
                             f"child does not record it at all — the harness "
                             f"stopped describing part of the arm; an absent "
                             f"key is not an unchanged one")
            elif k in pm:
                fails.append(f"manifest {k}: parent {a!r} != child {b!r} — the "
                             f"two campaigns were not configured alike")
            elif k in GATED_MANIFEST_GROUPS:
                off = GATED_MANIFEST_GROUPS[k][0]
                if b == off:
                    relaxed.append(k)
                else:
                    fails.append(
                        f"manifest {k}: new in the child at {b!r}, and its "
                        f"declared off value is {off!r}. The phase's own switch "
                        f"is ON — this is a treatment arm, not a defaults run, "
                        f"and it proves nothing about default-off equivalence.")
            elif k in gated_by:
                g = gated_by[k]
                if k in inert:
                    relaxed.append(k)
                else:
                    fails.append(
                        f"manifest {k}: new in the child ({b!r}) and declared "
                        f"as gated by {g}, but the child's {g} is "
                        f"{cm.get(g, '<absent>')!r}, not "
                        f"{GATED_MANIFEST_GROUPS[g][0]!r}. Nothing here shows "
                        f"the knob is inert, so it is read as live.")
            else:
                fails.append(
                    f"manifest {k}: new in the child ({b!r}) and not declared "
                    f"in GATED_MANIFEST_GROUPS, so this gate cannot tell "
                    f"whether it is inert. Declare it under the switch that "
                    f"gates it — adding the switch to the manifest if the "
                    f"harness does not write one — and do not delete this "
                    f"check.")
        if relaxed:
            notes.append(f"{len(relaxed)} new manifest key(s) recording a "
                         f"subsystem that is OFF: {', '.join(relaxed)}")
    return fails, notes


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip())
        return 2
    allow_new = {s.strip() for s in
                 os.environ.get("EQUIV_ALLOW_NEW_KINDS", "").split(",")
                 if s.strip()}
    try:
        legacy_kinds, v4_kinds = declared_vocabulary(LOG_HPP)
        defaults = source_defaults(NODE_SRC)
        parent = read_side(argv[1], "parent")
        child = read_side(argv[2], "child")
    except GateError as e:
        print(f"REFUSING TO SCORE: {e}")
        return 2

    print(f"parent {argv[1]}: {len(parent)} robot-run(s)")
    print(f"child  {argv[2]}: {len(child)} robot-run(s)")
    if not parent or not child:
        # The failure mode this project keeps rediscovering: a gate that finds
        # nothing to check and prints a pass.
        print("\nUNRESOLVED  a side is empty; NOTHING was compared. This is "
              "not a pass.")
        return 3

    try:
        fails, notes = compare(parent, child, legacy_kinds, v4_kinds, defaults,
                               allow_new)
    except GateError as e:
        print(f"REFUSING TO SCORE: {e}")
        return 2

    for n in notes:
        print(f"  note  {n}")
    for f in fails:
        print(f"  FAIL  {f}")
    print()
    if fails:
        print(f"{len(fails)} DIFFERENCE(S) — the default-off equivalence claim "
              f"is not supported")
        return 1
    print("EQUIVALENT  at defaults, on every run-invariant this gate can see")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
