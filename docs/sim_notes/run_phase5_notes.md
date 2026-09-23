# run_phase5.sh — design notes and history

The long comments of `sim/run_phase5.sh`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 2

## Top level

### phase5-perfect-vs-real-arms

**Phase 5 perfect and realistic arms** — attached to `set -uo pipefail` (line 6)

```text
Both conditions run the SHIPPED radio (tx_power_dbm = 30.0) on both robots.
Transmit power is fixed hardware, identical on every robot, and no field
experiment can turn it down; every earlier "severity level" in this plan was
produced by moving it, which is why section 3.19 withdrew that whole ladder.
The only difference between the two arms here is whether the message-level
radio emulator is in the path at all:

  p5perfect  --comms 0   no emulator. One broadcast domain, every delta
                         reaches both robots. This is the honest way to say
                         "assume comms never fails" — not a magic 160 dBm
                         radio, which describes nothing a robot could carry.
  p5real     --comms 1   the emulator at 30 dBm: relay hop, delay, airtime
                         budget, fading, and disconnection whenever the link
                         budget actually fails.

--expect-outage 0 on the realistic arm. Whether the honest radio ever drops
in THIS world is the question being asked, so the gate must not fail a run
for answering "no". Section 3.19's arithmetic predicts it will not: at 30 dBm
with 88 stems/ha over the +/-50 m ROI the link sits near 31 dB SNR, the top
tier, and reaching the 2 dB cutoff at 50 m would need roughly 3.8 trunks on
the path — about 240 stems/ha. If that prediction holds, the two arms will
be indistinguishable, and the finding is that THE WORLD cannot exercise the
radio: the next experiment is a denser forest, not another metric.

arms=off throughout: the reconnection manoeuvres are disabled, so nothing
here is confounded by policy. Robots still re-merge opportunistically.

--record 0: no bags. This comparison needs the planner CSVs and the link
trace only, and /tmp was at 97% when this was written. A Phase 5 oracle
re-merge needs bags and must be its own run.
```

### phase5-not-set-e

**Why phase 5 does not set -e** — attached to `set -uo pipefail` (line 40)

```text
NOT set -e. run_campaign.sh returns non-zero when ANY cell failed, and on the
first attempt that aborted this script between the two arms — leaving the
perfect arm measured and the realistic arm, the half that carries the
comparison, never run at all. A failed cell is a cell to re-run, not a reason
to abandon the other condition.
```
