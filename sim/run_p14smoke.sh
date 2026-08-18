#!/usr/bin/env bash
# p14 smoke: 2 cells at DONE_UNKNOWN=0.60, before committing to the 40-cell run.
#
# Everything except the criterion is byte-identical to p13: same scenario, same
# binary (built 2026-08-17 19:08, older than every p12 and p13 cell -- the first
# was written 2026-08-17 23:40 -- and nothing has been rebuilt since), tx 30.0,
# duration 5400, record 2, FRONTIER_ONLY=1, PURSUIT_BUDGET_MAX=2400. So any
# difference is the criterion and nothing else.
#
# 2026-08-06 15:15 appears on install/.../explo_planner_node but that is the
# SYMLINK's own mtime, not the binary's: --symlink-install creates the link once
# and later builds rewrite the target in place, leaving the link untouched.
# Follow it to build/ before dating a build.
#
# Same two cells as p13smoke -- hybrid:1 and pursuit:3 -- so the comparison is
# paired rather than against an arm median. hybrid also exercises both halves of
# the manoeuvre (chase, then the meeting-point fallback) in one cell.
#
# Runs to exploration completion, not a truncated window: 5400 is the cap, and a
# healthy cell terminates on the criterion long before reaching it.
#
# WHAT THIS IS ACTUALLY TESTING. criterion_choice.py picked 0.60 by RESCORING
# already-collected runs, which is valid for completion time but assumes the run
# is otherwise unchanged. It is not: at 0.60 a cell stops ~30% earlier, and two
# things could break that rescoring cannot see.
#
#   1. COMMS GATES. A gate can fail because the link never dropped hard enough.
#      A shorter run accumulates less outage, so the gates are the live risk
#      here -- not the planner. This is the check that decides whether 0.60 is
#      usable at all.
#   2. TREATMENT ENGAGEMENT. A mid-run manoeuvre needs 240 s of continuous peer
#      silence to arm. At 0.60, 4 of 17 treated cells finish before their first
#      dispatch (3 of 17 at 0.55). Both smoke cells must still dispatch, or the
#      arms are not being contrasted.
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
