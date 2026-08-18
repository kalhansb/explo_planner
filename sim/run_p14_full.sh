#!/usr/bin/env bash
# p14 full: 4 arms x 10 seeds = 40 cells at DONE_UNKNOWN=0.60.
#
# WHY 0.60 RATHER THAN THE 0.55 USED BY p12/p13. At 0.55 the run-to-run SD of
# completion time is ~920 s, larger than any arm difference observed, so n=10
# could only resolve a ~1150 s effect -- the campaign could not have answered
# its own question. 0.55 sits ~0.045 above the lowest unknown fraction ever
# reached here (0.5053), on a part of the curve whose marginal cost has already
# risen ~4x, so completion time was partly measuring how long a run takes to
# scrape past a threshold near the floor. At 0.60 the SD is ~198 s and the
# detectable difference at n=10 falls to ~248 s. See criterion_choice.py; the
# guard against over-raising it is in there too (at 0.70 the SD is better still
# and 71% of treated runs finish before any manoeuvre fires, measuring nothing).
#
# ORDER IS SEED-MAJOR (run_campaign.sh:78): every arm at seed 1 before any arm
# at seed 2. Stopping this campaign at any point therefore leaves a COMPLETE
# paired block for the seeds it reached, which is analysable on its own. That is
# the whole reason it is safe to queue 40 cells overnight rather than 6.
#
# Runs to exploration completion, not a truncated window. 5400 is a cap, not a
# target: the deepest 0.60 crossing in 25 collected cells is ~1961 s, so the cap
# is 2.7x the worst case and exists only to bound a pathological cell.
#
# EVERYTHING ELSE IS IDENTICAL TO p13: same scenario, same binary, tx 30.0,
# duration 5400, record 2, FRONTIER_ONLY=1, PURSUIT_BUDGET_MAX=2400. The
# criterion is the only thing that moved.
#
# NO colcon build while this runs. Every cell must share one binary, or an arm
# difference is confounded with a code change. Nothing needs one: no .cpp or
# .hpp is newer than the built node (2026-08-17 19:08), and the install tree is
# --symlink-install so yaml/config edits are already live.
#
# Gated on sim/run_p14smoke.sh passing all four of its criteria first.
#
# NOT -u: /opt/ros/humble/setup.bash reads AMENT_TRACE_SETUP_FILES unguarded.
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
