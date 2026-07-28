"""Named exploration-ROI presets, shared by all three launch files.

The planner ROI is an axis-aligned XY box and there is no roi_yaw yet (field
script §11 open item 2), so a rotated AO has to be given as its axis-aligned
bounding box. The forest-inspection AO's plantation rows run at 24.82° to map x,
which makes that bounding box substantially larger than the AO itself — and the
coverage-done measure (done_unknown_fraction) shares the same box, so every
column in the box that the robot can never reach puts a permanent floor under
the unknown fraction.

That matters on a day when only part of the AO is worked. Phase 1 is SA-1 + SA-2
(u in [-40, 60]) and has a much tighter bounding box than the full two-phase AO;
running phase 1 against the full box wastes candidates on SA-3 and leaves the
coverage floor high enough that done_unknown_fraction: 0.05 may never be
satisfiable, so the run ends on battery rather than on coverage.

Boxes live here, once, rather than in each launch file — the numbers are derived
from doc/experiment_script_forest_inspection.md §2 and must not drift apart.

Usage from a launch file in this directory:

    import os, sys
    sys.path.insert(0, os.path.dirname(__file__))
    from roi_presets import roi_overrides, ROI_PRESET_HELP

`full` returns {} so the values in config/exploration_params.yaml are used
unchanged — the yaml stays the single source of truth for the default box, and
this table only carries the alternatives.
"""

# name -> {} (use the yaml) or the four planner ROI bounds, metres, map frame.
ROI_PRESETS = {
    # The full two-phase AO bounding box, as shipped in
    # config/exploration_params.yaml. Empty on purpose: no override.
    "full": {},
    # Phase 1 only (SA-1 + SA-2). doc/experiment_script_forest_inspection.md §2.
    "phase1": {
        "roi_min_x": -51.4,
        "roi_max_x": 64.6,
        "roi_min_y": -38.7,
        "roi_max_y": 57.7,
    },
}

ROI_PRESET_HELP = (
    "Exploration ROI preset: 'full' = the two-phase AO bounding box from "
    "exploration_params.yaml; 'phase1' = the tighter SA-1+SA-2 box (use it when "
    "phase 2 is not run the same day — fewer wasted candidates and a lower "
    "coverage-done floor)"
)


def roi_overrides(name):
    """Resolve a preset name to a parameter-override dict.

    Raises RuntimeError on an unknown name rather than silently falling back to
    the full box: a typo'd roi:= on the launch line would otherwise look like it
    worked and quietly run the wrong area for the whole battery.
    """
    key = (name or "full").strip().lower()
    if key not in ROI_PRESETS:
        raise RuntimeError(
            "explo_planner: unknown roi preset {!r}; expected one of {}".format(
                name, ", ".join(sorted(ROI_PRESETS))))
    return dict(ROI_PRESETS[key])
