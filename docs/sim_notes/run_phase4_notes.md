# run_phase4.sh — design notes and history

The long comments of `sim/run_phase4.sh`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 1

## Top level

### phase4-severity-strata

**Phase 4 severity strata** — attached to `set -u` (line 4)

```text
§3.17 measured realised outage duty from 0.426 to 0.867 across cells that all
ran at the single calibrated power -- nearly the whole range that 20 dB of
transmit power was used to control. Fixing the power therefore does not fix
the treatment. This block spans power deliberately so that completeness can
be fitted against realised pre-treatment duty per arm, which is the only way
to read policy out of a covariate the robots partly choose for themselves.

Three strata: -6 dBm (calibration duty ~0.47), -14 (~0.61), -22 (~0.80).
The -14 stratum ALREADY EXISTS as the p3b pilot -- 4 arms x 2 seeds, all
CLEAN -- so only the outer two are run here. Same seeds throughout so the
fade realisations pair across strata.

T=5800 s follows §2.5's 3x rule against the solo makespan of 3880 s measured
in p1solo2; the 3600 s used earlier is what censored B0b.
```
