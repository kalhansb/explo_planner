#!/usr/bin/env bash
# p14 full: 4 arms x 10 seeds = 40 cells at DONE_UNKNOWN=0.60.
#
# Runs to exploration completion; duration 5400 is only a cap. Cells run
# seed-major, so stopping early leaves complete paired blocks. No colcon build
# while this runs: every cell must share one binary.
# (notes: p14-criterion-and-campaign)
#
# Gated on sim/run_p14smoke.sh passing all four of its criteria first.
#
# NOT -u: /opt/ros/humble/setup.bash reads AMENT_TRACE_SETUP_FILES unguarded.
# Moved comments: docs/sim_notes/run_p14_full_notes.md
set -eo pipefail

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i 'conda' | paste -sd:)
unset PYTHONPATH CONDA_PREFIX
source /opt/ros/humble/setup.bash

cd /home/kalhan/Projects/hmr_explo_ws/hmr_explo/ws/src/explo_planner

exec env SCENARIO=flatforest_dense_2robot_lidar.yaml DONE_UNKNOWN=0.60 \
  bash sim/run_campaign.sh \
    --root /tmp/hmr_campaign \
    --tag p14 \
    --arms "off,rendezvous,pursuit,hybrid" \
    --seeds "1,2,3,4,5,6,7,8,9,10" \
    --duration 5400 \
    --tx 30.0 \
    --record 2 \
    --env "FRONTIER_ONLY=1 PURSUIT_BUDGET_MAX=2400"
