# run_phase7.sh — design notes and history

The long comments of `sim/run_phase7.sh`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 2

## Top level

### phase7-dense-forest-pilot

**Phase 7 dense-forest mode pilot** — attached to `set -uo pipefail` (line 4)

```text
This is the experiment sections 3.16 through 3.22 kept failing to run. The
manoeuvre arms were never the problem; the WORLD was. Across the whole
campaign history only 8 firings ever drove a route (section 3.22), because in
the 81 stems/ha stand at the shipped 30 dBm the link almost never breaks:
Phase 5 measured 97-99% connected, 3-9 outages per run, longest 19.4 s. A
manoeuvre that arms on a 180 s stale peer record cannot fire against a link
that reconnects in 20 s, so every mode collapsed to the same behaviour and the
arms were measuring nothing.

The lever is the forest, not the radio. tx_power_dbm stays at 30.0 on both
robots in every cell here -- it is fixed hardware, identical on every robot,
and no field experiment can turn it down. Section 3.19 withdrew the entire
earlier severity ladder for exactly that reason. What changes is the stand:

  flatforest         81 stems/ha    97.4% connected   longest outage    19 s
  flatforest_dense  250 stems/ha    44.5% connected   longest outage   861 s

250 stems/ha is not an arbitrary bump. The link budget puts the 2 dB cutoff at
50 m separation at roughly 247 stems/ha, so this stand is the first one where
a typical inter-robot path is marginal rather than comfortable. Measured over
two full runs it delivers 37-38 outages, several minutes long -- past the
180 s pursuit staleness bound, past the 5 s claim TTL, deep into the region
where the four modes must actually differ.

Both dense cells so far still reached all_done with CLEAN gates, so the
coverage endpoint survives the denser stand; the pilot is not being run into a
world that cannot terminate.

DEFAULTS EVERYWHERE ON THE PLANNER, AS THEY STOOD WHEN p7modes RAN.
pursuit_staleness_max_sec was 180, coord_claim_ttl_sec 5, reconnect_confirm_sec
3, rendezvous_max_wait_sec 600. Dense outages ran past 180 s, so pursuit
declined on staleness in some episodes. At the time that was taken as a result
about the policy at its shipped settings rather than a misconfiguration to
tune away, since tuning it here would have confounded the mode comparison with
a parameter sweep.

That reading did not survive. A 180 s gate does not decline in SOME episodes,
it declines in every mid-run one: the trigger fires at 240 s of silence, so a
mid-run dispatch carries a record age of ~240-251 s BY CONSTRUCTION and is
always past the gate. pursuit and hybrid's chase were unreachable, not merely
selective, so p7modes measured pursuit-as-off and hybrid-as-rendezvous. The
gate is 900 everywhere since 2026-08-18; see shared_params.yaml for the
evidence. Read any p7modes mode contrast with that in mind.
```

### phase7-reproducing-p7modes

**Reproducing the original p7modes cells** — attached to `set -uo pipefail` (line 48)

```text
!! STALE AS OF THE 2026-08-17 REDESIGN -- THIS SCRIPT NO LONGER REPRODUCES
THE p7modes RUNS. The paragraph above was true when the campaign ran, but the
harness defaults moved underneath it: run_explo_sim_rviz.sh now ships
PURSUIT_STALENESS=900, MIDRUN_SILENCE=90 (240 through generation 8, then 90
with the link veto on by default), RECONNECT_RELEASE_CONFIRM=6 and
HOLD_ESCALATE=1 to match the planner's own code defaults. A bare re-run of
this script therefore executes the POST-redesign policy under the p7modes
tag. To reproduce the original cells bit-for-bit, export:

  MIDRUN_SILENCE=0 RECONNECT_RELEASE_CONFIRM=0 PURSUIT_STALENESS=180 \
  HOLD_ESCALATE=0

Every one of these is echoed into the run manifest, so an already-collected
run can be checked rather than assumed -- read it before pooling any new
p7modes-tagged cell with the old ones.
```
