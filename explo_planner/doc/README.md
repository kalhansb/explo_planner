# explo_planner documentation

Detailed documentation for the `explo_planner` package. Package overview,
build and parameters are in [`../README.md`](../README.md); everything here is
either an operating procedure or a design record.

## Start here

| I want to… | Read |
|------------|------|
| Run a two-robot trial on real hardware | [field_trial_manual.md](field_trial_manual.md) |
| Dry-run exploration off a bag first | [dscovox_exploration_run.md](dscovox_exploration_run.md) |
| Dry-run the vantage ring and target queue | [dscovox_exploitation_run.md](dscovox_exploitation_run.md) |
| Understand why something behaves oddly | [limitations.md](limitations.md) |

## Index

**Operating**

- [field_trial_manual.md](field_trial_manual.md) — two-robot field trial:
  bring-up order, per-robot configuration, coordination behaviour, what to
  watch, panic stops, data offload.
- [dscovox_exploration_run.md](dscovox_exploration_run.md) — single-robot
  exploration over a bag replay, step by step.
- [dscovox_exploitation_run.md](dscovox_exploitation_run.md) — the same
  pipeline with the target queue switched on, plus the live tree detector.

**Design and planning**

- [exploitation_plan.md](exploitation_plan.md) — design record for the
  perceptive-exploitation overlay: vantage selection, target lifecycle, the
  EXPLOIT state machine.
- [nav2_exploration_goal_plan.md](nav2_exploration_goal_plan.md) — a fixed
  ten-goal Nav2 plan on already-driven ground, for validating navigation
  independently of the planner. Figure:
  `nav2_exploration_plan_topdown.png`.

**Known issues**

- [limitations.md](limitations.md) — deliberate simplifications and
  known-but-unfixed rough edges, each with the scenario, why it is acceptable
  today, and how it could be fixed. Read §3 before changing a vantage
  parameter on one robot only.

## Related

- [`../config/exploration_params.yaml`](../config/exploration_params.yaml) —
  every parameter, commented with the reasoning behind each field default.
- [SCovox user manual](../../../scovox/docs/user_manual.md) — the mapping and
  fusion layer the planner reads from.
- `doc/experiment_script_forest_inspection.md` (hmr_explo workspace) — the
  campaign design the field trial manual operationalises.
