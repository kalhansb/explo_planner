# calibrate_txpower.sh — design notes and history

The long comments of `sim/calibrate_txpower.sh`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 1
- [sweep_point()](#sweep_point) — 1

## Top level

### calib-offline-replay

**Why the sweep replays recorded poses** — attached to `set -u` (line 5)

```text
Offline and cheap by construction. The emulator's fading is a pure function of
(seed, tick) and its link model reads only poses + the world SDF, so the whole
sweep runs against a recorded bag with no Gazebo, no planners and no mapping —
minutes per point instead of the ~an hour a live run costs. What it CANNOT do
is anticipate behaviour: once comms actually degrade, trajectories change and
the replayed duty cycle stops being the realised one. That is why §4 calls this
"aiming" and why the analysis re-measures realised severity per run and reports
it without ever conditioning on it.
```

## sweep_point()

### calib-play-only-clock-and-poses

**Replay only the clock and poses** — attached to `local play_topics="/clock"` (line 82)

```text
Play ONLY the clock and the poses. This is not an optimisation, it is
correctness: the bag also contains /hmr_comms_sim/link_states recorded
during the control run, and replaying that publishes the ORIGINAL run's
link rows onto the same topic the logger is subscribed to. The trace then
interleaves rows computed at the control's tx_power_dbm with rows computed
at the swept one, and since the control was deliberately run at a power
where the link never drops, every sweep point reads far more connected than
it is. Caught by the implied tx: a row's snr_db + path_loss_db - 101 gives
the transmit power it was computed at, and the first row of every trace
read 160.0 instead of the swept value.
```
