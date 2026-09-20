# Rendezvous and reconnection coordination — architecture record

Destination for the historical and design narrative that Addendum 3 of
`CODE_REVIEW_explo_planner_node_2026-09-20.md` proposes moving out of
`explo_planner_node.cpp`.

| | |
|---|---|
| Source | `explo_planner/src/explo_planner_node.cpp` |
| Created | 2026-09-21 |
| Status | **scaffold** — anchoring policy and generation timeline are populated and verified; the per-mechanism narrative is not yet migrated |
| Companion | [`evidence_comments.md`](evidence_comments.md) — the *measurements*; this file holds the *design history* |

## What belongs here, and what does not

Three documents divide the job. Putting a paragraph in the wrong one is how
this material became unmaintainable in the first place.

| Content | Home | Why |
|---|---|---|
| The invariant a reader must not break | **source comment** | It is needed at the point of the edit that would break it |
| A measurement from a banked campaign (`ρ=0.677`, `21 of 64`, `34.18 s`) | `evidence_comments.md` | It is an unversioned claim about versioned data; it needs a line to be re-checked against `~/hmr_campaign` |
| Why generation N differs from generation N−1 | **this file** | It is narrative, it only grows, and no edit to the code can invalidate it |

The distinction that matters for the migration: a comment saying *"clear the
latch here or a stale peer keeps the restoring force"* is an invariant and
**stays**. A comment saying *"generation 19 removed the recurring schedule and
generation 23 put it back"* is history and **moves here**. Most long blocks in
the node contain both, interleaved — which is why this migration is a rewrite
per block, not a cut-and-paste.

## Anchoring policy — symbols, never line numbers

**Every reference from this file into the source cites a symbol.** Not a line
number, not a line range.

This is not a style preference. It is the one lesson the package has already
paid for twice:

- §7.2 of the review is a 147-line diff reconciling a six-band line-offset
  table, produced because an earlier census cited line numbers.
- `evidence_comments.md` anchors all **141** of its blocks on line ranges
  (`### explo_planner_node.cpp:298-301`). Any comment removal shifts every
  anchor below the first deletion. The migration this file exists to support
  would invalidate the companion document wholesale.
- Measured during the 2026-09-20 review session, across ~20 minutes of an
  ordinary edit: `home_trail_` moved `:3199` → `:3249`, and the
  `NO-SHOW LIST IS GONE` block moved `:1681` → `:1719`. The review's own
  Addendum 3 verification table, written that evening, was stale before
  midnight.

`evidence_comments.md` also records its source revision as `133214c`, which
**is not a valid object in this repository** (`git cat-file -t` fails). Its
provenance cannot currently be resolved to a commit. Re-anchoring it is a
prerequisite of the migration, not a follow-up — see the plan in the review.

## Generation timeline

Reconstructed from the 139 generation references in the node's comments.
21 distinct generations are cited. Mention counts indicate how much of the
current design each generation still explains, not how large the change was.

| Gen | Mentions | What the source records |
|---:|---:|---|
| 5 | 3 | Approach-based homing watchdog introduced (`HomeMode`) |
| 7 | 6 | Fixed a mis-stamp; the fix landed after the read it was meant to correct |
| 8 | 6 | Baseline configuration (T = 240) for the fitted silence rate |
| 9 | 13 | Mid-run silence default 240 → **90 s**; changes behaviour, so gen-8 traces do not transfer |
| 10 | 1 | Reconnect armed from `dispatchReconnect`, behind the mid-run gate |
| 12 | 1 | Peer line-up measured at N=2/3/4 |
| 15 | 1 | Row semantics gained a distinction |
| 17 | 1 | Keyed occurrence off the separation anchor |
| 18 | 10 | Removed the separation anchor (occurrences 34.0 s apart) |
| 19 | 14 | **Removed the recurring schedule**; latch-clear semantics differ here |
| 20 | 1 | Countdown form produced zero occurrences |
| 21 | 1 | At-the-rendezvous hold (2026-09-17) |
| 22 | 11 | "Once, early, while mutual" established as *not* a window bound; applies in every arm |
| 23 | 36 | **TeamWorld**: `team_incomplete`, `appointment_inbound`; recurring schedule restored |
| 24 | 3 | `t_now + notice` floor — forked the ts4 N=3 cell |
| 25 | 13 | Interval floor changed; `doReturnNav` conversion introduced |
| 26 | 5 | N=3 smoke: all three robots parked in `RETURN_SYNC` |
| 27 | 21 | Appointment barrier's release door also consults the reconnect path |
| 28 | 1 | Conversion made reversible (2026-09-19) |
| 29 | 11 | `interval_ms >= 0` tightened; timetable spacing |
| 30 | 2 | Current banked generation — ts4 N=2 rendezvous priced the wedged-pair case |

Gens 6, 11, 13, 14, 16 leave no trace in the node's comments. Absence here
means the source does not explain them; it does not mean nothing shipped.

**Generation 31** is the next one, and it will be minted by the appointment-leg
retry ladder currently uncommitted in the working tree
(`appointmentLegWatchdog`, `resumeAppointmentDrive`, `armManoeuvreLegBudget`,
`rendezvous_escape_max_attempts`). Per §0 of the review, every cosmetic change
in that document is gated on riding that rebuild rather than minting its own.

## Mechanism index

Symbols the migrated narrative will be organised under. Populated as each
block is rewritten; an entry with no summary has not been migrated yet.

| Symbol | Role | Narrative migrated? |
|---|---|---|
| `RendezvousProposal` | Proposal record exchanged between peers | not yet |
| `maintainRendezvousProposal` | Per-tick proposal upkeep while mutual | not yet |
| `deriveRendezvousProposal` | Builds a proposal from the current team view | not yet |
| `appointmentLegWatchdog` | Takes over a leg stopped too far from the cell | gen-31, written in the target style |
| `resumeAppointmentDrive` | Ends an escape leg, re-aims at the agreed cell | gen-31, written in the target style |
| `armManoeuvreLegBudget` | Sizes the nav budget / no-progress window for a leg | gen-31, written in the target style |
| `home_trail_` / `HomeMode::RETRACE` | Breadcrumb retrace when direct homing stalls | not yet — **carries a known-stale claim, see review §6** |
| `reconnect_midrun_silence_sec_` | Mid-run silence threshold (90 s since gen 9) | not yet — 84-line block, the largest in the class body |
| `appointment_unplaceable_` | De-`mutable`d in the 2026-09-20 cleanup | n/a |
| `TeamWorld` | Team-completeness / inbound view added gen 23 | not yet |

The gen-31 ladder methods are listed because they are the **worked example of
the target style**: a two-to-four line `///` contract on the declaration, with
the reasoning beside the algorithm in the body. Anything migrated here should
leave the source reading like those three.
