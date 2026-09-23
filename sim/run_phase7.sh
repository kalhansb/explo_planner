#!/usr/bin/env bash
# Phase 7 -- the reconnection-mode pilot, in the DENSE forest.
#
# Compares the four reconnection modes in the dense stand, where outages last
# minutes; tx_power_dbm stays at the shipped 30.0 on every robot.
# (notes: phase7-dense-forest-pilot)
#
# A bare re-run uses today's harness defaults, not those p7modes ran under. To
# reproduce those cells export MIDRUN_SILENCE=0 RECONNECT_RELEASE_CONFIRM=0
# PURSUIT_STALENESS=180 HOLD_ESCALATE=0; all are in the manifest.
# (notes: phase7-reproducing-p7modes)
#
# 12 cells at roughly 45-55 min each is a 10-hour job. run_campaign.sh is
# resumable and orders cells seed-major, so an interruption leaves a COMPLETE
# paired block at every seed it reached rather than a matrix missing one arm.
#
# Read out with:
#   sim/manoeuvre_events.py  <-- the primary readout: scores FIRINGS, not runs
#   sim/comms_metrics.py --a '...off*' --b '...hybrid*'
#   sim/analyze_runs.py --threshold 0.55
#
# NOT set -e; see run_phase5.sh. A failed cell is a cell to re-run, not a reason
# to abandon the rest of the matrix.
# Moved comments: docs/sim_notes/run_phase7_notes.md
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${ROOT:-/tmp/hmr_campaign}"
SEEDS="${SEEDS:-1,2,3}"
ARMS="${ARMS:-off,rendezvous,pursuit,hybrid}"
DUR="${DUR:-5800}"
SC="${SC:-flatforest_dense_2robot_lidar.yaml}"

log() { echo "[phase7] $*"; }

log "=== p7modes: dense forest, shipped 30 dBm radio, four reconnection modes ==="
log "arms=$ARMS seeds=$SEEDS scenario=$SC"
"$HERE/run_campaign.sh" --root "$ROOT" --tag p7modes --comms 1 --tx 30.0 \
    --scenario "$SC" --expect-outage 1 --arms "$ARMS" --seeds "$SEEDS" \
    --duration "$DUR" --record 0 \
  || log "p7modes had failed cell(s) -- the completed cells are still analysable"

log "=== firings (the readout that matters: per event, not per run) ==="
"$HERE/manoeuvre_events.py" "$ROOT"/p7modes_* || true
log "=== divergence across the four modes ==="
"$HERE/map_divergence.py" "$ROOT"/p7modes_* || true
