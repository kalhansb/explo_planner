#!/usr/bin/env bash
# Phase 5 — perfect comms vs realistic comms, with NOTHING detuned.
#
# The question: does a realistic radio differ measurably from perfect comms?
#
# Both conditions run the shipped 30 dBm radio, arms off, no bags. p5perfect has
# no emulator; p5real runs through it with expect-outage 0, since whether the
# link ever drops is the question. (notes: phase5-perfect-vs-real-arms)
#
# Read out with:
#   sim/map_divergence.py                 (the metric that separated conditions)
#   sim/analyze_runs.py --threshold 0.55
# NOT set -e: run_campaign.sh returns non-zero when any cell fails, and that
# must not stop the other arm from running. (notes: phase5-not-set-e)
# Moved comments: docs/sim_notes/run_phase5_notes.md
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${ROOT:-/tmp/hmr_campaign}"
SEEDS="${SEEDS:-1,2,3}"
DUR="${DUR:-5800}"

log() { echo "[phase5] $*"; }

log "=== p5perfect: no emulator (--comms 0) ==="
"$HERE/run_campaign.sh" --root "$ROOT" --tag p5perfect --comms 0 \
    --arms off --seeds "$SEEDS" --duration "$DUR" --record 0 \
  || log "p5perfect had failed cell(s) — continuing to the realistic arm anyway"

log "=== p5real: emulator at the shipped 30 dBm (--comms 1) ==="
"$HERE/run_campaign.sh" --root "$ROOT" --tag p5real --comms 1 --tx 30.0 \
    --expect-outage 0 --arms off --seeds "$SEEDS" --duration "$DUR" --record 0 \
  || log "p5real had failed cell(s)"

log "=== done — read out with map_divergence.py ==="
"$HERE/map_divergence.py" "$ROOT"/p5perfect_* "$ROOT"/p5real_* || true
