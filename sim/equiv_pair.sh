#!/usr/bin/env bash
# Builds and runs the two sides equiv_gate.py compares.
#
# Owns checkout, build, run, restore and compare, both sides from one workspace
# built in turn. Refuses a dirty tree (untracked files too), a parent not an
# ancestor of HEAD, and a binary whose baked git_rev mismatches.
# (notes: pair-owns-whole-pair)
#
# Usage:
#   ./equiv_pair.sh PARENT_REV
# Env:
#   OUT=<dir>          where the two run roots go (default: mktemp -d)
#   DURATION_S=<n>     sim seconds per side (default 180 — this compares
#                      configuration, not outcomes, so it wants a run long
#                      enough to emit run_start and a manifest, no longer)
#   KEEP=1             keep OUT on success
#
# Exit: 0 equivalent, 1 not equivalent, 2 usage/precondition/provenance.
# Moved comments: docs/sim_notes/equiv_pair_notes.md

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$(cd "$HERE/.." && pwd)"          # the explo_planner submodule root
WS="$(cd "$SRC/../.." && pwd)"         # the colcon workspace

PARENT_REV="${1:-}"
[ -n "$PARENT_REV" ] || { echo "usage: equiv_pair.sh PARENT_REV" >&2; exit 2; }

DURATION_S="${DURATION_S:-180}"
OUT="${OUT:-$(mktemp -d -t equiv_pair_XXXXXX)}"
mkdir -p "$OUT"

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { echo "ERROR: $*" >&2; exit 2; }

# --- preconditions ---------------------------------------------------------

git -C "$SRC" rev-parse --git-dir >/dev/null 2>&1 || die "$SRC is not a git repo"

DIRT="$(git -C "$SRC" status --porcelain)"
if [ -n "$DIRT" ]; then
  echo "ERROR: explo_planner tree is dirty. The child side would be labelled" >&2
  echo "       with a commit it was not built from, and a PASS would certify" >&2
  echo "       a rev nobody tested. Commit or remove:" >&2
  echo "$DIRT" | sed 's/^/       /' >&2
  exit 2
fi

CHILD_REV="$(git -C "$SRC" rev-parse HEAD)"
PARENT_SHA="$(git -C "$SRC" rev-parse --verify "${PARENT_REV}^{commit}")" \
  || die "no such rev: $PARENT_REV"
[ "$PARENT_SHA" != "$CHILD_REV" ] && : || die "parent and child are the same commit"
git -C "$SRC" merge-base --is-ancestor "$PARENT_SHA" "$CHILD_REV" \
  || die "$PARENT_REV is not an ancestor of HEAD ($CHILD_REV) — 'equivalent to
       its parent' is a claim about one lineage, and these are two branches"

# equiv_gate.py scores gen 33's cells only: it reads gen 33's node source and
# vocabulary. A child whose run script pins another node is refused here,
# before two builds and two runs the gate would then refuse to score. The
# parent is an ancestor, so it cannot be the newer node.
NODE_PIN="$(sed -n 's/^NODE=//p' "$HERE/run_explo_sim_rviz.sh" | head -1)"
if [ -n "$NODE_PIN" ] && [ "$NODE_PIN" != "gen33" ]; then
  die "HEAD's run script pins NODE=$NODE_PIN; equiv_gate.py scores gen-33
       cells only. Check gen-34 cells with gen34_check.py"
fi

# Restore the child rev no matter how this exits, including on Ctrl-C. Leaving
# the tree detached at the parent is how a later session ends up measuring the
# old binary and not noticing.
BRANCH="$(git -C "$SRC" symbolic-ref --quiet --short HEAD || echo "")"
restore() {
  local rc=$?
  if [ -n "$BRANCH" ]; then
    git -C "$SRC" checkout --quiet "$BRANCH" 2>/dev/null || true
  else
    git -C "$SRC" checkout --quiet "$CHILD_REV" 2>/dev/null || true
  fi
  local now; now="$(git -C "$SRC" rev-parse HEAD)"
  if [ "$now" != "$CHILD_REV" ]; then
    echo "ERROR: could not restore the tree to $CHILD_REV (now at $now)." >&2
    echo "       Fix this before building anything else in this workspace." >&2
    rc=2
  fi
  exit $rc
}
trap restore EXIT INT TERM

# --- one side --------------------------------------------------------------

run_side() {
  local side="$1" rev="$2" short
  short="$(git -C "$SRC" rev-parse --short "$rev")"
  log "=== $side: $short ==="

  git -C "$SRC" checkout --quiet --detach "$rev"

  # --packages-up-to because explo_planner_msgs must build first; Release to
  # match the build type the workspace already carries, since a flip would
  # change the binary for reasons unrelated to the phase.
  # (notes: pair-build-flags)
  ( set +u
    source /opt/ros/humble/setup.bash
    cd "$WS"
    colcon build --packages-up-to explo_planner \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release
  ) >"$OUT/build_$side.log" 2>&1 \
    || { echo "build failed, tail of $OUT/build_$side.log:" >&2
         tail -30 "$OUT/build_$side.log" >&2; exit 2; }

  # Unsets the treatment knobs so both sides run shipped defaults. Keep it in
  # step with every phase: a leaked knob reaches both sides equally and yields a
  # false EQUIVALENT. Other run knobs still inherit; use a clean shell.
  # (notes: pair-unset-treatment-knobs)
  ( unset CELL_WORLD CELL_SIZE_M CELL_CENSUS_S COMMS LINK_GATE EXPLOIT \
          RECONNECT_MODE TEAM_WORLD TEAM_WORLD_HZ GLOBAL_ALLOC RECONNECT_GATE \
          RENDEZVOUS_SCHEDULE PURSUIT_PREDICTOR COORD_CLAIM_R
    export RVIZ=0 DURATION_S="$DURATION_S" OUTDIR="$OUT/$side/cell_001"
    "$HERE/run_explo_sim_rviz.sh"
  ) >"$OUT/run_$side.log" 2>&1 \
    || { echo "run failed, tail of $OUT/run_$side.log:" >&2
         tail -30 "$OUT/run_$side.log" >&2; exit 2; }

  # Provenance: what did the binary that actually ran think it was built from?
  # This is read out of the run's own event log, not out of git, so a build that
  # silently did not happen cannot pass.
  local baked
  baked="$(python3 - "$OUT/$side/cell_001" <<'PY'
import glob, json, os, sys
paths = sorted(glob.glob(os.path.join(sys.argv[1], "*.events.jsonl")))
if not paths:
    print("NO-EVENT-LOG"); raise SystemExit
with open(paths[0]) as fh:
    print(json.loads(fh.readline()).get("params", {}).get("git_rev", "ABSENT"))
PY
)"
  if [ "$baked" != "$short" ]; then
    echo "ERROR: $side ran a binary stamped '$baked', expected '$short'." >&2
    echo "       Either the rebuild did not happen or it did not take. The" >&2
    echo "       two sides may be the same binary, which would PASS this" >&2
    echo "       gate for the one reason that proves nothing." >&2
    exit 2
  fi
  log "$side ok — binary stamped $baked"
}

run_side parent "$PARENT_SHA"
run_side child  "$CHILD_REV"

# --- compare ---------------------------------------------------------------

log "=== equiv_gate: parent $(git -C "$SRC" rev-parse --short "$PARENT_SHA") vs child $(git -C "$SRC" rev-parse --short "$CHILD_REV") ==="
set +e
python3 "$HERE/equiv_gate.py" "$OUT/parent" "$OUT/child"
GATE_RC=$?
set -e

if [ "$GATE_RC" = "0" ] && [ "${KEEP:-0}" != "1" ]; then
  rm -rf "$OUT"
else
  log "artifacts kept in $OUT"
fi
exit "$GATE_RC"
