# explo_planner

Next-Best-View exploration and perceptive exploitation for multi-robot forest
inspection, in ROS 2.

A robot picks viewpoints by **expected information gain** over a
[SCovox](https://github.com/kalhansb/scovox) Beta-conjugate occupancy map,
drives to them through the navigation stack, and logs per-step metrics. When a
tree target arrives it switches to **exploitation**: it circles the trunk at
occlusion-free vantage points and dwells at each so the rosbag captures
overlapping close-range views. Multiple robots deconflict their goals through a
MinPos intent table, yield to each other when their paths cross, and return to
a rendezvous anchor when comms drop.

## Contents

| Path | What it is |
|------|------------|
| [`explo_planner/`](explo_planner/) | The planner package — nodes, config, launch files, tests. Start at its [README](explo_planner/README.md). |
| [`explo_planner_msgs/`](explo_planner_msgs/) | `TreeTarget` and `RobotIntent` message definitions. |
| [`docs/`](docs/) | The **[user manual](docs/user_manual.md)** — two-robot field trial: bring-up order, per-robot configuration, what to watch, panic stops, data offload. |

## Documentation

| I want to… | Read |
|------------|------|
| Understand how the planner works | [explo_planner/README.md](explo_planner/README.md) |
| Run a two-robot trial on real hardware | [docs/user_manual.md](docs/user_manual.md) |
| Dry-run exploration off a bag first | [explo_planner/doc/dscovox_exploration_run.md](explo_planner/doc/dscovox_exploration_run.md) |
| Dry-run the vantage ring and target queue | [explo_planner/doc/dscovox_exploitation_run.md](explo_planner/doc/dscovox_exploitation_run.md) |
| Tune anything | [explo_planner/config/exploration_params.yaml](explo_planner/config/exploration_params.yaml) — every parameter, commented with the reasoning behind each field default |
| Understand why something behaves oddly | [explo_planner/doc/limitations.md](explo_planner/doc/limitations.md) |

The field-trial **campaign design** — area of operations, run matrix, target
list, metrics — lives in the parent
[hmr_explo](https://github.com/kalhansb/hmr_explo) workspace at
`doc/experiment_script_forest_inspection.md`. The manual here is the operating
procedure for it and references its numbers rather than repeating them.

## Build

An `ament_cmake` package that depends on SCovox (`scovox_core`, `scovox_msgs`),
so build it in a workspace overlaying SCovox:

```bash
cd <ws>
colcon build --packages-up-to explo_planner
source install/setup.bash
```

`--packages-up-to`, not `--packages-select` — `explo_planner_msgs` is a sibling
package that has to build first.

## License

BSD-3-Clause.
