# Reconnection redesign — proposal for adversarial review (2026-08-17)

User mandate: "examine the code and suggest ways forward, I'm okay to change planner
behaviour. check timing issue." Standing method: adversarial agents on every issue,
then justified decisions.

Repo: /home/kalhan/Projects/hmr_explo_ws/hmr_explo/ws/src/explo_planner
Planner: explo_planner/src/explo_planner_node.cpp (~4000 lines)
Util:    explo_planner/src/planner_util.cpp
Harness: sim/run_explo_sim_rviz.sh, sim/run_campaign.sh
Scenario: dense forest, 2 robots, comms emulator, outages measured up to 861 s.

## Established facts (from the completed p7modes campaign + code reading)

F1. The reconnect dispatch `finishOrRendezvous()` (node.cpp:2876) is called ONLY from
    doPlan's step-budget check (:2191) and coverage-saturation check (:2244) [and a
    third exhaustion site ~:3987]. I.e. a robot considers reconnection only AFTER it
    has finished exploring. t_team = t_lead + lag, with t_lead (~89% of the median)
    untreatable by any mode, by construction.

F2. The finish line done_unknown_fraction=0.55 (harness default DONE_UNKNOWN, 
    run_explo_sim_rviz.sh:376) sits on the flat tail of the coverage curve. Three
    identical replicates: t_team 890/1429/2700 s (3.03x). Same replicates re-read at
    threshold 0.60: 1.61x; 0.65: 1.20x; 0.75: 1.11x. The noise is the endpoint.

F3. pursuit_staleness_max_sec defaults to 180 (node.cpp:479, loaded :1293; harness
    does NOT override it). pursuitBudgetSec (planner_util.cpp:55) returns 0 when
    staleness >= max → startPursuit declines (:3198). Because the trigger is terminal
    (F1), staleness at trigger ≈ outage duration (measured 186–688 s at finish, tail
    861 s), so the chase NEVER armed in any dense-world run.

F4. Pure pursuit's fallback is holdForTeam (:2925-2929, :3387) = park at current pose
    behind the RETURN_SYNC barrier. If both robots hold, neither moves; barrier
    expiry (RDV_MAX_WAIT=600 via rendezvousWaitExpired, doReturnSync :3133) gives up
    → State::DONE — mission ends with maps unmerged. Observed: 6 hold events, 5
    failed, 1 recovered only because the peer was still driving. One mission lost
    (censored at 5812 s with the union of maps complete but split).

F5. Reconnect release paths (doPursue :3300, doReturnSync :3120) already transition
    back to PLAN with coverage_done_streak_ = 0, so post-reconnect the robot re-plans
    against the merged map and resumes exploring if unknown is above threshold.

F6. team_last_complete_time_ / team_seen_complete_ are maintained on the heartbeat
    (heartbeatTick :3450-3457): team_last_complete_time_ advances only while the team
    is complete, so (now - team_last_complete_time_) is a live "peer missing for" 
    clock available in any state.

F7. Reliable map deltas queue during outages and drain on reconnect (comms emulator).
    So a mid-run physical reconnection delivers the peer's map, which prunes
    frontiers the peer already covered. Under the current terminal trigger this
    delivery happens only after t_lead — too late to shorten anyone's exploration.

## Proposed changes

### C1 — Mid-exploration reconnect trigger (the timing fix)
New planner param `reconnect_midrun_silence_sec` (double, default 0 = off, preserving
field behaviour; harness sets it per-arm). In doPlan's EXPLORE path (after the
exploit-activation branch :2204, after the coverage-saturation block):

    if (reconnect_midrun_silence_sec_ > 0 && rendezvous_enabled_ && have_anchor_
        && team_seen_complete_ && phase_ == Phase::EXPLORE) {
      const double missing_for = (now - team_last_complete_time_).seconds();
      if (missing_for >= reconnect_midrun_silence_sec_
          && (now - last_midrun_attempt_).seconds() >= reconnect_midrun_silence_sec_
          && dispatchReconnect("peer-lost")) {
        last_midrun_attempt_ = now; return;
      }
    }

Refactor: extract the manoeuvre-dispatch body of finishOrRendezvous (:2906-2932,
minus the DONE fallthrough) into `bool dispatchReconnect(const char* reason)`;
finishOrRendezvous keeps its confirm-window + DONE logic and calls it.

Rationale: this is what makes t_team treatable. Mid-run reconnection merges maps
mid-run (F7), pruning redundant frontiers; a mode that reconnects faster/more
reliably then finishes sooner, and the primary metric finally measures the modes.
Threshold ~300 s: an order of magnitude above the confirm gate (3 s), below the
outage tail (861 s), so it fires only on long separations.

Cooldown = same threshold, so a failed attempt doesn't re-trigger immediately.

### C1b — Mid-run manoeuvre must RESUME, not finish
doReturnSync's expiry path (:3133-3147) currently goes DONE. For a manoeuvre armed
mid-run that is wrong (it would end exploration at ~45% unknown). Change: the
dispatcher records `reconnect_terminal_` (true when dispatched from
finishOrRendezvous, false from the mid-run site). On barrier expiry with
reconnect_terminal_ == false → abandon, reset streak, transitionTo(PLAN) (resume
exploring); terminal case keeps today's DONE. Same guard for pursuitFallback-driven
holds mid-run.

### C2 — Pursuit staleness gate sized to the world
Harness passes `-p pursuit_staleness_max_sec:=900.0` (env PURSUIT_STALENESS,
default 900) — above the measured outage tail (861 s). Not a code-default change:
the 180 default stays for field use; the dense-forest experiment overrides it.
With C1 the typical staleness at a mid-run trigger is ~300 s (the silence
threshold), so freshness = 1 - 300/900 ≈ 0.67 and the budget stays meaningful.

### C3 — Hold escalation (deadlock break)
In doReturnSync expiry (:3133), for the TERMINAL case: instead of giving up straight
to DONE, escalate ONCE to `startReturnTo(last_connected_anchor_, "last-connected
anchor", "hold-escalate")` if not already there; a second expiry finishes as today.
Both robots' anchors are their own poses at the last moment the link worked, so both
converging to their anchors restores (approximately) the link-geometry that existed
then. This removes the mutual-hold fixed point (F4) without deleting pure pursuit's
identity (it still chases first, holds second, and only then falls back to the
anchor). Gate behind a new param `hold_escalate` (bool, default false; harness true)
if reviewers think the pure-pursuit A/B must stay pure — but note the A/B already
returned its verdict: pure hold loses missions.

### C4 — Move the finish line off the flat tail
Harness DONE_UNKNOWN 0.55 → 0.62 for the NEXT campaign (new tag; no comparability
claim against p7modes at 0.55 — modes_compare's ladder can still read any collected
run at any threshold). Replicate spread at 0.60 is 1.61x vs 3.03x at 0.55; 0.65 is
1.20x. 0.62 buys most of the noise reduction while keeping the mission meaningfully
long (outage-rich). Runs also end sooner → more seeds per night.

### C5 — Campaign plan
After the running campaign exits: skip the n=5 extension of the old design (its own
header calls it the least valuable step; the metric it tightens cannot rank the
modes). Rebuild, smoke one cell, then run p8trigger: arms off/rendezvous/pursuit/
hybrid × seeds 1..5, DONE_UNKNOWN=0.62, midrun silence 300 s (off arm: trigger
inert since rendezvous_enabled=false), duration sized from the ladder's t@0.62.

## REVIEW OUTCOMES (agents B and C; agent A on C1/C1b pending)

ADOPTED AMENDMENTS — the design below is superseded where these conflict:

B1. C2 as written ships "always arm, ~240 s budget" (budget ceiling saturates for
    trail heads >= 18-54 m), and chasing a stale GOAL can drive ~190 m away from
    the peer. Amended C2: keep the 900 s harness override BUT make the trail
    staleness-adaptive in startPursuit — when the record is older than
    ~180 s, drop the rec.peer_goal waypoint (dead hypothesis) and chase only
    rec.peer_pose (bounded by former link geometry). A failed/exhausted chase then
    always ends holding at the swapped contact pair — mutually within former link
    range — which is the deadlock break for free. Additionally echo
    pursuit_staleness_max_sec (and budget) into the run manifest and pass it
    explicitly through run_campaign.sh env. State openly that hybrid's chase leg
    changes too (it routes through startPursuit).
B2. C3's "escalate once if not already there" positional guard LOOPS on an
    unreachable anchor (re-escalate every expiry until the duration cap). Amended:
    sticky per-manoeuvre bool hold_escalated_, set at escalation, cleared where
    reconnect_active_ clears; escalated barrier gets a SHORTER wait (~300 s).
B3. R7 wording corrected: the two anchors are same-WINDOW, not same-event (per-
    direction BER losses can displace one side's record), and can be marginal-fade
    artifacts — escalation is sound on mission outcome, but duration must budget a
    worst-case terminal tail of ~1600 s (chase 240 + hold 600 + nav <=180 + hold).
C1r. C4 IS WITHDRAWN. The 0.55->0.62 noise argument was built on perfect-comms
    replicates; realistic-comms replicates give 0.62 CV 0.354 vs 0.55 CV 0.486
    (no measurable gain at n=3) and 0.60 is the WORST threshold (4.45x). Only
    0.65 is low-noise (1.20x/1.69x) and at 0.65 the mechanism window vanishes.
    DECISION: keep termination at DONE_UNKNOWN=0.55; report t@0.65 as the
    low-noise secondary via the existing ladder.
C2r. C1's silence threshold 300 s at an early finish yields ~0.09 firings/run
    (identical-policy comparison). DECISION: silence = 240 s with termination
    0.55 (5/11 historic runs would fire before T62, more before T55); cooldown =
    threshold.
C3r. Duration stays 5800 s (worst observed team crossing 3240 s + manoeuvre tail;
    a lower cap only saves time on censored runs).
C4r. Build fairness: p7modes cells recorded FOUR planner commits (binary
    identical, but post-hoc unprovable). Rule: build once before the campaign,
    assert identical git_explo_planner across cells, and NO repo commits while a
    campaign runs.
C5r. Arms for p8trigger: off, rendezvous, hybrid x seeds 1..5 (15 cells,
    ~13 h risk-adjusted, permutation floor 2/252). Pursuit arm DROPPED: verdict
    established (deadlock, patched), chase mechanism still exercised via hybrid's
    startPursuit path. The pursuit code fixes (B1/B2) ship regardless.

## Known risks (for reviewers to sharpen or refute)
R1. Thrash: dense-forest outages are frequent; a 300 s silence gate may fire many
    times per run, and each manoeuvre costs exploration time. Net effect could be
    NEGATIVE for all modes vs off — that is a legitimate experimental outcome, but
    the design must not let one robot spend the whole run reconnecting (cooldown).
R2. Symmetric mid-run dispatch: both robots trigger simultaneously and chase each
    other's stale goals (pursuit) — do they dance past each other? doPursue releases
    on quarry_heard, and closing distance restores the link before arrival in most
    geometries; but construct the failure case if it exists.
R3. State-machine interactions: mid-run dispatch while EXPLOIT targets pending, or
    while a vantage claim is active (standDownExploitation is called by
    startPursuit/startReturnTo/holdForTeam — verify that is sufficient); 
    PROXIMITY_HOLD interplay; intent semantics while manoeuvring mid-run.
R4. done_action / DONE semantics: C1b changes an exit path — check every consumer of
    State::DONE and the presence-intent contract (:2940-2948, :3138-3145).
R5. Threshold 0.62 reachability: every run must actually cross 0.62 (check ladder
    data); if a run never reaches it, it ends at max_steps/duration cap instead.
R6. The confirm gate inside dispatchReconnect samples team_last_complete_time_ too
    (:2894-2905) — with missing_for >= 300 s it passes trivially; verify no
    double-counting or clock subtlety (sim time, heartbeat starvation note :3447).
R7. Anchor escalation (C3): if the link originally dropped because BOTH robots moved
    away simultaneously, each anchor is stale by the peer's subsequent motion — but
    the peer converging to ITS anchor restores its half of the geometry; verify the
    pair (anchor_A, anchor_B) was actually a connected configuration (it was: both
    recorded at the same link-drop event, when the link still worked).
