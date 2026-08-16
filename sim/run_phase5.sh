#!/usr/bin/env bash
# Phase 5 — perfect comms vs realistic comms, with NOTHING detuned.
#
# The question: does a realistic radio differ measurably from perfect comms?
#
# Both conditions run the SHIPPED radio (tx_power_dbm = 30.0) on both robots.
# Transmit power is fixed hardware, identical on every robot, and no field
# experiment can turn it down; every earlier "severity level" in this plan was
# produced by moving it, which is why section 3.19 withdrew that whole ladder.
# The only difference between the two arms here is whether the message-level
# radio emulator is in the path at all:
#
#   p5perfect  --comms 0   no emulator. One broadcast domain, every delta
#                          reaches both robots. This is the honest way to say
#                          "assume comms never fails" — not a magic 160 dBm
#                          radio, which describes nothing a robot could carry.
#   p5real     --comms 1   the emulator at 30 dBm: relay hop, delay, airtime
#                          budget, fading, and disconnection whenever the link
#                          budget actually fails.
#
# --expect-outage 0 on the realistic arm. Whether the honest radio ever drops
# in THIS world is the question being asked, so the gate must not fail a run
# for answering "no". Section 3.19's arithmetic predicts it will not: at 30 dBm
# with 88 stems/ha over the +/-50 m ROI the link sits near 31 dB SNR, the top
# tier, and reaching the 2 dB cutoff at 50 m would need roughly 3.8 trunks on
# the path — about 240 stems/ha. If that prediction holds, the two arms will
# be indistinguishable, and the finding is that THE WORLD cannot exercise the
# radio: the next experiment is a denser forest, not another metric.
#
# arms=off throughout: the reconnection manoeuvres are disabled, so nothing
# here is confounded by policy. Robots still re-merge opportunistically.
#
# --record 0: no bags. This comparison needs the planner CSVs and the link
# trace only, and /tmp was at 97% when this was written. A Phase 5 oracle
# re-merge needs bags and must be its own run.
#
# Read out with:
#   sim/map_divergence.py                 (the metric that separated conditions)
#   sim/analyze_runs.py --threshold 0.55
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${ROOT:-/tmp/hmr_campaign}"
SEEDS="${SEEDS:-1,2,3}"
DUR="${DUR:-5800}"

log() { echo "[phase5] $*"; }

log "=== p5perfect: no emulator (--comms 0) ==="
"$HERE/run_campaign.sh" --root "$ROOT" --tag p5perfect --comms 0 \
    --arms off --seeds "$SEEDS" --duration "$DUR" --record 0

log "=== p5real: emulator at the shipped 30 dBm (--comms 1) ==="
"$HERE/run_campaign.sh" --root "$ROOT" --tag p5real --comms 1 --tx 30.0 \
    --expect-outage 0 --arms off --seeds "$SEEDS" --duration "$DUR" --record 0

log "=== done — read out with map_divergence.py ==="
"$HERE/map_divergence.py" "$ROOT"/p5perfect_* "$ROOT"/p5real_* || true
