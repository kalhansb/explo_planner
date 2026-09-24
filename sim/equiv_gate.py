#!/usr/bin/env python3
# Moved comments: docs/sim_notes/equiv_gate_notes.md
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
  3b. the shape of each kind      a kind both sides emit carries the same
                                  top-level fields, at the same JSON types.
                                  Names alone are not the schema: a child that
                                  keeps every kind and rewrites its contents is
                                  a writer change, and checking 3 without 3b
                                  passes it clean.
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

Two kinds of param defeat that read, and both are registered rather than
guessed at. A DERIVED field has no dp() call at all, so DERIVED_DEFAULTS
supplies it — as a constant, or as a function of the same run's other params
where one is available. An AUTO-SENTINEL field has a dp() call whose second
argument is not a default: the node reads "<= 0" as "resolve this from another
knob" and does so in the constructor, before the dump is written, so no
at-defaults run ever logs it. Testing those against the sentinel would fail
every honest run; AUTO_RESOLVED_PARAMS marks them UNRESOLVED instead, which is
the truthful answer and not a pass.

Populations
-----------
An empty side is UNRESOLVED, not PASS. "I found no cells to compare" and "I
compared them and they matched" are different answers and this file will not
conflate them.

Usage:  equiv_gate.py PARENT_ROOT CHILD_ROOT
        Each root is a campaign directory of cell subdirectories, or one cell
        directory. Cells are matched by nothing — the comparison is over the
        UNION of each side, since seeds and cell names differ between runs.
        Gen 33 only: a cell whose manifest names another node (gen 34's
        records node=gen34) is refused. This gate reads gen 33's node source
        and vocabulary; gen-34 cells are checked by gen34_check.py.
Env:    EQUIV_NODE_SRC   gen 33's explo_planner_node.cpp (default: ../backup/gen33/)
        EQUIV_LOG_HPP    gen 33's experiment_log.hpp (default: ../backup/gen33/)
        EQUIV_ALLOW_NEW_KINDS   comma-separated event kinds permitted to appear
                                in the child. For the phase that turns a
                                mechanism ON, so the same file can score a
                                treatment arm; NEVER for a defaults run.
        EQUIV_ALLOW_NEW_FIELDS  comma-separated `kind.field` names permitted to
                                appear on a kind both sides emit (check 3b).
                                Same rule: for a phase that deliberately widens
                                an existing event, NEVER for a defaults run.
Exit:   0  equivalent
        1  a difference that defeats the default-off claim
        2  usage, a default the parser refused to guess at, or a cell that is
           not gen 33's
        3  something was not actually compared: a population was empty, or an
           invariant could not be checked (a param whose compiled "default" is
           an AUTO sentinel, or a derived one whose inputs the run did not log)
"""

import json
import math
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..", "explo_planner")
# Gen 33's node, backed up unbuilt when gen 34 took the name (2026-09-23).
NODE_SRC = os.environ.get(
    "EQUIV_NODE_SRC", os.path.join(HERE, "..", "backup", "gen33",
                                   "explo_planner_node.cpp"))
# Its event vocabulary, backed up beside it: the current header is gen 34's.
LOG_HPP = os.environ.get(
    "EQUIV_LOG_HPP", os.path.join(HERE, "..", "backup", "gen33",
                                  "experiment_log.hpp"))

# Dumped params with no dp() call. Each value is what the derivation yields at
# default inputs, or a callable on the child's param dump. Used only when dp()
# has none; a param in neither place is a hard failure.
# (notes: derived-defaults-registry)
DERIVED_DEFAULTS = {
    "robot_id": -1.0,    # fleet_.self_id with team_robot_names empty
    "team_hash": 0.0,    # fleet_.team_hash likewise
    # Mirrors the node's test in explo_planner_node.cpp: omnidirectional if hfov
    # is within one ray step (hfov/h_rays) of 2*pi. Derived from the same run's
    # fov_hfov and fov_h_rays, not pinned True.
    # (notes: derived-fov-omnidirectional)
    "fov_is_omnidirectional": lambda cp: (
        float(cp["fov_hfov"]) >= 2.0 * math.pi - (
            float(cp["fov_hfov"]) / float(cp["fov_h_rays"])
            if float(cp.get("fov_h_rays") or 0) > 0 else 0.0)
    ) if "fov_hfov" in cp and "fov_h_rays" in cp else UNDERIVABLE,
}

# The dp() second argument is an AUTO sentinel (<= 0) the node resolves from the
# named knob in its constructor, before the dump is written. Listed params
# report UNRESOLVED (exit 3), never pass. (notes: auto-resolved-params)
AUTO_RESOLVED_PARAMS = {
    "coord_claim_radius_m":         "fov_max_range",
    "coord_vantage_claim_radius_m": "vantage_visited_tol_m (not logged)",
    "cost_grid_radius_cap_m":       "candidate_max_radius + 2.0 (not logged)",
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

# Gate key -> (off value, dependent keys). A key new in the child is relaxed
# only if its switch is present in the child at the off value (a harness value,
# not a compiled default). A new key declared nowhere fails.
# (notes: gated-manifest-groups)
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
        # The manifest records the launcher's TEAM_WORLD_HZ (default 1.0) even
        # with team_world=0; the node gets the param only when TEAM_WORLD=1, so
        # check 2 compares its compiled 0.0. The two values may differ.
        # (notes: manifest-team-world-hz)
        "team_world_hz",
    }),
    # Gate keys with no dependents: the runner passes no other knob for these
    # subsystems. If it ever does, add those knobs to the set. Register a key
    # here in the same commit that adds it to the manifest.
    # (notes: manifest-p3-p4-gate-keys)
    "global_alloc": ("0", set()),
    "reconnect_gate": ("silence", set()),
    # One gate key, no dependents: the runner passes rendezvous_schedule_enable
    # alone and leaves the scheduler's tuning knobs at their compiled defaults.
    # (notes: manifest-rendezvous-schedule)
    "rendezvous_schedule": ("0", set()),
    # Off value is the word trail (the shipped aimer), not 0;
    # equiv_gate_calib.py pins both directions. Do not soften the hard failure
    # for undeclared keys: it is what catches a missed registration.
    # (notes: manifest-pursuit-predictor-off)
    "pursuit_predictor": ("trail", set()),
    # Off value is the word none, not 0.0: the runner writes none and passes no
    # claim-radius param when COORD_CLAIM_R is unset, while the node reads a
    # radius <= 0 as AUTO. (notes: manifest-claim-radius-override)
    "coord_claim_radius_override": ("none", set()),
    # Witnesses of the installed params file: each off value is the shipped yaml
    # value (coord_claim_radius_m 10.0, global_alloc_comms_mask false) and must
    # track the yaml. A missing params file fails here.
    # (notes: manifest-params-file-witnesses)
    "coord_claim_radius_m_in_params": ("10.0", set()),
    "global_alloc_comms_mask_in_params": ("false", set()),
}

# Keys that restate a key both sides record (key -> source key), e.g. the roster
# from scenario. Relaxed only when the source key is present and equal on both
# sides; no off value exists, so not a gated group.
# (notes: manifest-derived-keys)
DERIVED_MANIFEST = {
    "robots": "scenario",
    "n_robots": "scenario",
}

# Pure renames of a source key. Relaxed only if the source agrees across sides
# AND the child's alias equals its own source, so the gate checks, not trusts,
# that run_explo_sim_rviz.sh writes both from one variable.
# (notes: manifest-alias-keys)
ALIAS_MANIFEST = {
    "reconnect_enabled": "rendezvous_enabled",   # renamed 2026-09-03
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
# A DERIVED_DEFAULTS callable returns this when the run did not log the inputs
# it derives from. Distinct from UNPARSEABLE: that one means this file cannot
# read the source, which is a refusal to score (exit 2); this one means the
# LOG lacks the inputs, which is an honest "not checkable here" (exit 3).
UNDERIVABLE = object()


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


def _float32(x):
    """x as a C++ static_cast<float> leaves it, widened back to a double."""
    return struct.unpack("f", struct.pack("f", x))[0]


def source_defaults(path):
    """Map param name -> compiled default (or UNPARSEABLE) from dp(...) and
    dp_f(...) calls. dp_f declares a double and narrows it to float, and the
    run logs the float widened back, so its defaults are narrowed here too."""
    try:
        src = open(path, errors="replace").read()
    except OSError as e:
        raise GateError(f"cannot read the param defaults: {e}")
    out = {}
    for m in re.finditer(r'\bdp(_f)?\s*\(\s*"([A-Za-z0-9_]+)"\s*,', src):
        name = m.group(2)
        open_paren = src.index("(", m.start())
        expr = _default_expr(src, open_paren)
        v = UNPARSEABLE if expr is None else _literal(expr)
        if m.group(1) and isinstance(v, float):
            v = _float32(v)
        out[name] = v
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
        # The node key arrived with gen 34, so an absent one is gen 33.
        node = manifest.get("node", "gen33")
        if node != "gen33":
            raise GateError(
                f"{label}: {os.path.basename(d)} was run by node={node}. This "
                f"gate reads gen 33's node source and event vocabulary, so it "
                f"cannot score that cell; check gen-34 cells with "
                f"gen34_check.py")
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
            shape = {}
            for ln in open(os.path.join(d, f), errors="replace"):
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                kind = e.get("event")
                if kind:
                    kinds.add(kind)
                    # Top-level keys only, and the JSON type of each. run_start's
                    # nested `params` is compared field-by-field in check 2
                    # already; re-flattening it here would report every param
                    # twice and bury the one thing this check adds.
                    s = shape.setdefault(kind, {})
                    for k, v in e.items():
                        s.setdefault(k, set())
                        if v is not None:
                            s[k].add(type(v).__name__)
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
                         "shape": shape,
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


def compare(parent, child, legacy_kinds, v4_kinds, defaults, allow_new,
            allow_fields=frozenset()):
    fails, notes, unresolved = [], [], []

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
        # Checked BEFORE the dp() lookup, because these params do have a dp()
        # call — its second argument is just a sentinel rather than a default,
        # so the lookup would succeed and then compare against the wrong thing.
        if k in AUTO_RESOLVED_PARAMS:
            unresolved.append(
                f"param {k!r} is new in the child at {cp[k]!r}, and its dp() "
                f"second argument is an AUTO sentinel, not a default: the node "
                f"resolves it from {AUTO_RESOLVED_PARAMS[k]} before the param "
                f"dump is written. This gate cannot certify it from the log, "
                f"and will not pretend the sentinel is the expectation.")
            continue
        if k not in defaults and k not in DERIVED_DEFAULTS:
            fails.append(
                f"param {k!r} is new in the child but no dp(\"{k}\", ...) call "
                f"was found in {os.path.basename(NODE_SRC)} and it is not a "
                f"known derived field — this gate cannot tell whether it is at "
                f"its default. Add a dp() default or an entry in "
                f"DERIVED_DEFAULTS; do not delete this check.")
            continue
        d = defaults.get(k, DERIVED_DEFAULTS.get(k))
        if callable(d):
            d = d(cp)
            if d is UNDERIVABLE:
                unresolved.append(
                    f"param {k!r} is new in the child at {cp[k]!r} and is "
                    f"DERIVED, but this run did not log the params it derives "
                    f"from, so there is nothing to check it against.")
                continue
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
    # Only the ones actually CHECKED. Counting the auto-resolved and underivable
    # ones here would report them as "at defaults" in the same breath the
    # unresolved list says they could not be checked.
    checked_new = [k for k in new if k not in AUTO_RESOLVED_PARAMS]
    if checked_new:
        notes.append(f"{len(checked_new)} new param(s) at defaults: "
                     f"{', '.join(checked_new)}")

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

    # Check 3 compares only kind names; this compares each shared kind's
    # top-level fields and JSON types. A field new in the child fails, one
    # missing from the child is a note. Unions are per side, never per run.
    # (notes: check-3b-event-shape)
    def merge_shape(side):
        out = {}
        for r in side:
            for kind, fields in r["shape"].items():
                dst = out.setdefault(kind, {})
                for k, types in fields.items():
                    dst.setdefault(k, set()).update(types)
        return out

    pshape, cshape = merge_shape(parent), merge_shape(child)
    new_fields, gone_fields, retyped = [], [], []
    for kind in sorted(set(pshape) & set(cshape)):
        pf, cf = pshape[kind], cshape[kind]
        new_fields += [f"{kind}.{k}" for k in sorted(set(cf) - set(pf))]
        gone_fields += [f"{kind}.{k}" for k in sorted(set(pf) - set(cf))]
        for k in sorted(set(pf) & set(cf)):
            # Empty means "seen, but only ever null", which is not a type
            # disagreement with anything.
            if pf[k] and cf[k] and pf[k] != cf[k]:
                retyped.append(f"{kind}.{k} ({'/'.join(sorted(pf[k]))} -> "
                               f"{'/'.join(sorted(cf[k]))})")
    blocked = [f for f in new_fields if f not in allow_fields]
    if blocked:
        fails.append(
            f"child writes field(s) {blocked} on an event kind both sides "
            f"emit, and no parent robot-run wrote them. The event vocabulary "
            f"is unchanged but its contents are not, so this is a writer "
            f"change at defaults. Permit with EQUIV_ALLOW_NEW_FIELDS if the "
            f"phase is meant to add them.")
    if retyped:
        fails.append(f"field(s) changed JSON type between the binaries: "
                     f"{retyped}. Every downstream parse of them is now "
                     f"reading a different thing.")
    if gone_fields:
        notes.append(f"field(s) the parent wrote and the child did not: "
                     f"{gone_fields} (a conditional field on a branch the "
                     f"child's runs did not reach looks exactly like this; "
                     f"a field DELETED from the writer does too)")
    if allow_fields & set(new_fields):
        notes.append(f"permitted new field(s) present: "
                     f"{sorted(allow_fields & set(new_fields))} "
                     f"(EQUIV_ALLOW_NEW_FIELDS — this is NOT a defaults run)")

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
        relaxed, derived_ok = [], []
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
            elif k in ALIAS_MANIFEST:
                s = ALIAS_MANIFEST[k]
                sp, sc = pm.get(s, "<absent>"), cm.get(s, "<absent>")
                if not (sp == sc and s in pm and s in cm):
                    fails.append(
                        f"manifest {k}: new in the child ({b!r}) and declared "
                        f"as derived from {s}, but {s} is {sp!r} on the parent "
                        f"and {sc!r} on the child. It restates a key the two "
                        f"sides do not share, so it is read as a difference of "
                        f"its own.")
                elif b != sc:
                    fails.append(
                        f"manifest {k}: declared a rename of {s}, but the child "
                        f"writes {k}={b!r} and {s}={sc!r} in the SAME manifest. "
                        f"One run cannot have had the switch both ways: the "
                        f"harness has stopped writing the two spellings from "
                        f"one value, and this relaxation is no longer safe.")
                else:
                    derived_ok.append(k)
            elif k in DERIVED_MANIFEST:
                s = DERIVED_MANIFEST[k]
                sp, sc = pm.get(s, "<absent>"), cm.get(s, "<absent>")
                if sp == sc and s in pm and s in cm:
                    derived_ok.append(k)
                else:
                    fails.append(
                        f"manifest {k}: new in the child ({b!r}) and declared "
                        f"as derived from {s}, but {s} is {sp!r} on the parent "
                        f"and {sc!r} on the child. It restates a key the two "
                        f"sides do not share, so it is read as a difference of "
                        f"its own.")
            else:
                fails.append(
                    f"manifest {k}: new in the child ({b!r}) and not declared "
                    f"in GATED_MANIFEST_GROUPS, so this gate cannot tell "
                    f"whether it is inert. Declare it under the switch that "
                    f"gates it — adding the switch to the manifest if the "
                    f"harness does not write one — or, if it only restates a "
                    f"key both sides already record, in DERIVED_MANIFEST (or "
                    f"ALIAS_MANIFEST, if it is a rename carrying that key's "
                    f"identical value). Do not delete this check.")
        if relaxed:
            notes.append(f"{len(relaxed)} new manifest key(s) recording a "
                         f"subsystem that is OFF: {', '.join(relaxed)}")
        if derived_ok:
            notes.append(f"{len(derived_ok)} new manifest key(s) restating a "
                         f"key both sides record: {', '.join(derived_ok)}")
    return fails, notes, unresolved


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip())
        return 2
    allow_new = {s.strip() for s in
                 os.environ.get("EQUIV_ALLOW_NEW_KINDS", "").split(",")
                 if s.strip()}
    allow_fields = {s.strip() for s in
                    os.environ.get("EQUIV_ALLOW_NEW_FIELDS", "").split(",")
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
        fails, notes, unresolved = compare(
            parent, child, legacy_kinds, v4_kinds, defaults, allow_new,
            allow_fields)
    except GateError as e:
        print(f"REFUSING TO SCORE: {e}")
        return 2

    for n in notes:
        print(f"  note  {n}")
    for u in unresolved:
        print(f"  UNRESOLVED  {u}")
    for f in fails:
        print(f"  FAIL  {f}")
    print()
    if fails:
        # Reported first, and alone, when both are present: an UNRESOLVED line
        # is a question, a FAIL is an answer, and the answer already settles
        # the verdict. Printing them both above is deliberate -- the reader
        # still needs to know what went unchecked before acting on the FAILs.
        print(f"{len(fails)} DIFFERENCE(S) — the default-off equivalence claim "
              f"is not supported")
        return 1
    if unresolved:
        # Exit 3, same as the empty-side case: a wrapper branching on the exit
        # status must not read could-not-check as a pass.
        # (notes: exit-3-unresolved)
        print(f"UNRESOLVED  no differences found, but {len(unresolved)} "
              f"invariant(s) could NOT be checked. This is not a pass.")
        return 3
    print("EQUIVALENT  at defaults, on every run-invariant this gate can see")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
