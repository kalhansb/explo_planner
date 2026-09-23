#!/usr/bin/env bash
# Phase 4: span comms severity instead of assuming it.
#
# Runs only the -6 and -22 dBm strata; the -14 dBm stratum already exists from
# earlier cells. Same seeds throughout so the fade realisations pair across
# strata. (notes: phase4-severity-strata)
# Moved comments: docs/sim_notes/run_phase4_notes.md
set -u

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${ROOT:-/tmp/hmr_campaign}"
ARMS="${ARMS:-off,rendezvous,pursuit,hybrid}"
SEEDS="${SEEDS:-1,2}"
DURATION="${DURATION:-5800}"
RECORD="${RECORD:-2}"
MIN_FREE_GB="${MIN_FREE_GB:-8}"

free_gb() { df -BG --output=avail /tmp | tail -1 | tr -dc '0-9'; }

for spec in "p4mild:-6.0" "p4harsh:-22.0"; do
  tag="${spec%%:*}"
  tx="${spec##*:}"

  avail=$(free_gb)
  if [ "$avail" -lt "$MIN_FREE_GB" ]; then
    echo "[phase4] ABORT before $tag: only ${avail}GB free on /tmp (need ${MIN_FREE_GB}GB)" >&2
    exit 1
  fi
  echo "[phase4] === $tag at tx=${tx} dBm (${avail}GB free) ==="

  "$SIM_DIR/run_campaign.sh" \
    --root "$ROOT" --tag "$tag" \
    --arms "$ARMS" --seeds "$SEEDS" \
    --tx "$tx" --expect-outage 1 --done-unknown 0.55 \
    --duration "$DURATION" --record "$RECORD"
  rc=$?
  echo "[phase4] $tag finished rc=$rc"
done

echo "[phase4] all strata done; -14 stratum is the existing p3b_* cells"
