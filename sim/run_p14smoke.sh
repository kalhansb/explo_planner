#!/usr/bin/env bash
# p14 smoke: 2 cells at DONE_UNKNOWN=0.60, before committing to the 40-cell run.
#
# Runs the same two cells as the earlier smoke (hybrid:1, pursuit:3), so the
# comparison is paired; only the criterion differs. It checks what rescoring
# cannot: the comms gates, and that treated cells still dispatch.
# (notes: p14smoke-what-it-tests)
#
# PASS CRITERIA, all four required before the 40-cell run:
#   (a) both cells reach all_done on the criterion, not the duration cap
#   (b) gates CLEAN
#   (c) at least one dispatch per cell, and hybrid:1 admits a chase
#   (d) t_done within ~200 s of the rescored prediction -- hybrid:1 -> 902 s,
#       pursuit:3 -> 801 s. This is the one that validates the rescoring method
#       itself, and so validates using 0.60 on the 25 cells already collected.
#       ~200 s is the run-to-run SD at this criterion, so a miss inside it is
#       noise and a large miss means rescoring does not predict a real run.
#
# NO colcon build before or during this, and none between here and the full
# campaign: every cell must share one binary.
#
# NOT -u: /opt/ros/humble/setup.bash reads AMENT_TRACE_SETUP_FILES unguarded.
# Moved comments: docs/sim_notes/run_p14smoke_notes.md
set -eo pipefail

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i 'conda' | paste -sd:)
unset PYTHONPATH CONDA_PREFIX
source /opt/ros/humble/setup.bash

cd /home/kalhan/Projects/hmr_explo_ws/hmr_explo/ws/src/explo_planner

exec env SCENARIO=flatforest_dense_2robot_lidar.yaml DONE_UNKNOWN=0.60 \
  bash sim/run_campaign.sh \
    --root /tmp/hmr_campaign \
    --tag p14smoke \
    --cells "hybrid:1,pursuit:3" \
    --duration 5400 \
    --tx 30.0 \
    --record 2 \
    --env "FRONTIER_ONLY=1 PURSUIT_BUDGET_MAX=2400"
