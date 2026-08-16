#!/usr/bin/env bash
# Phase 4: span comms severity instead of assuming it.
#
# §3.17 measured realised outage duty from 0.426 to 0.867 across cells that all
# ran at the single calibrated power -- nearly the whole range that 20 dB of
# transmit power was used to control. Fixing the power therefore does not fix
# the treatment. This block spans power deliberately so that completeness can
# be fitted against realised pre-treatment duty per arm, which is the only way
# to read policy out of a covariate the robots partly choose for themselves.
#
# Three strata: -6 dBm (calibration duty ~0.47), -14 (~0.61), -22 (~0.80).
# The -14 stratum ALREADY EXISTS as the p3b pilot -- 4 arms x 2 seeds, all
# CLEAN -- so only the outer two are run here. Same seeds throughout so the
# fade realisations pair across strata.
#
# T=5800 s follows §2.5's 3x rule against the solo makespan of 3880 s measured
# in p1solo2; the 3600 s used earlier is what censored B0b.
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
