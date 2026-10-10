# explo_planner

Next-Best-View exploration and perceptive exploitation for multi-robot forest
inspection, in ROS 2.

A robot picks viewpoints by **expected information gain** over a
[SCovox](https://github.com/kalhansb/scovox) Beta-conjugate occupancy map,
drives to them through the navigation stack, and logs per-step metrics. When a
tree target arrives it switches to **exploitation**: it circles the trunk at
occlusion-free vantage points and dwells at each so the rosbag captures
overlapping close-range views. Robots deconflict their goals through a MinPos
intent table and yield to each other when their paths cross. When exploration
ends they return to their start points and wait there for any teammate still
out of comms.

## Contents

| Path | What it is |
|------|------------|
| [`explo_planner/`](explo_planner/) | The planner package: nodes, config, launch files, tests. Start at its [README](explo_planner/README.md). |
| [`explo_planner_msgs/`](explo_planner_msgs/) | `TreeTarget` and `RobotIntent` message definitions. |
| [`docs/`](docs/) | Repository-level docs, indexed below. |

## Documentation

| I want to… | Read |
|------------|------|
| Get an overview of how the planner works and how the code is laid out | [explo_planner/README.md](explo_planner/README.md) |
| Run a two-robot field trial: bring-up, what to watch, panic stop, data offload | [docs/user_manual.md](docs/user_manual.md) |
| Set up and run the planner on one robot's hardware: ROI, planning and traversability maps | [docs/field_setup.md](docs/field_setup.md) |
| See how the real-robot parameter values were derived, and what to set before a real run | [docs/real_robot_tuning.md](docs/real_robot_tuning.md) |
| Know what the planner needs from a robot's nav2, and how to change its topic names | [docs/nav2_config.md](docs/nav2_config.md) |
| Integrate or drive the planner from ROS: nodes, topics, parameters, tf, launch files, CSV | [docs/ros_api.md](docs/ros_api.md) |
| Read the method in full, with every number (written for a paper) | [docs/planner_method.md](docs/planner_method.md) |
| Dry-run exploration on a bag first | [explo_planner/doc/dscovox_exploration_run.md](explo_planner/doc/dscovox_exploration_run.md) |
| Dry-run the vantage ring and target queue on a bag | [explo_planner/doc/dscovox_exploitation_run.md](explo_planner/doc/dscovox_exploitation_run.md) |
| Look up a parameter's default and the reasoning behind it | [explo_planner/config/shared_params.yaml](explo_planner/config/shared_params.yaml) |
| Understand why something behaves oddly | [explo_planner/doc/limitations.md](explo_planner/doc/limitations.md) |

The field-trial **campaign design** (area of operations, run matrix, target
list, metrics) is `doc/experiment_script_forest_inspection.md` in the parent
[hmr_explo](https://github.com/kalhansb/hmr_explo) workspace. The user manual
is the operating procedure for it.

## Build

An `ament_cmake` package. It depends on SCovox (`scovox_core`, `scovox_msgs`)
and `nav2_msgs`, so build it in a workspace that overlays SCovox:

```bash
cd <ws>
colcon build --packages-up-to explo_planner
source install/setup.bash
```

Use `--packages-up-to`, not `--packages-select`: `explo_planner_msgs` is a
sibling package that has to build first.

## License

BSD-3-Clause.
