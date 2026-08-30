#!/usr/bin/env bash
# Builds and runs the two sides equiv_gate.py compares.
#
# docs/mtare_evolution_plan.md §6 requires every phase to land default-off: at
# shipped defaults the new binary must be the same planner as its parent
# commit. equiv_gate.py scores that claim, but it can only score runs somebody
# produced, and producing them by hand is where the claim quietly rots — one
# side built from a tree that was not the rev it is labelled with, or a run
# launched with a treatment env var still exported from the last shell, and the
# gate compares two things that are not what the report says they are.
#
# So this script owns the whole pair: checkout, build, run, restore, compare.
# Both sides come out of ONE workspace built sequentially, because that is the
# workspace the harness sources and a second install space would be a different
# dependency set pretending to be a control.
#
# What it refuses to do, and why each refusal matters:
#
#   * run on a dirty explo_planner tree. The child side is labelled with a
#     commit; if uncommitted work is in the tree, the binary is not that commit,
#     and the resulting PASS certifies a rev that was never tested. Untracked
#     files are equally disqualifying — they survive `git checkout` and would
#     sit in the parent's tree too.
#   * accept a parent that is not an ancestor of HEAD. "Equivalent to its
#     parent" is a claim about a specific lineage; two unrelated revs can be
#     equivalent and still say nothing about what this phase changed.
#   * believe a build happened. After each run it reads the `git_rev` the
#     binary BAKED IN at compile time back out of the run's own event log and
#     requires it to match the rev that was checked out. A skipped or failed
#     rebuild leaves the previous binary in place, and two runs of the same
#     binary are the most convincing false PASS this gate can produce.
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

  # A stale binary is the failure mode this whole exercise is blind to, so the
  # build is not allowed to be a no-op that succeeded quietly: --packages-up-to,
  # because explo_planner_msgs must build first, and the same Release type the
  # workspace already carries, because a build-type flip would change the
  # binary for a reason that has nothing to do with the phase.
  ( set +u
    source /opt/ros/humble/setup.bash
    cd "$WS"
    colcon build --packages-up-to explo_planner \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release
  ) >"$OUT/build_$side.log" 2>&1 \
    || { echo "build failed, tail of $OUT/build_$side.log:" >&2
         tail -30 "$OUT/build_$side.log" >&2; exit 2; }

  # No cell-world env vars, and none of the other treatment knobs: the whole
  # point is the SHIPPED defaults. Unset rather than trust the caller's shell.
  ( unset CELL_WORLD CELL_SIZE_M CELL_CENSUS_S COMMS LINK_GATE EXPLOIT
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
