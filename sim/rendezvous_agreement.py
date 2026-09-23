#!/usr/bin/env python3
# Moved comments: docs/sim_notes/rendezvous_agreement_notes.md
"""Per-cell validity gate: a SCHEDULED arm must actually have scheduled something.

WHY THIS EXISTS
---------------
The ts4 generation-15 smoke ran six cells. All six exited 0, all six passed
every comms gate, and three of the six did not run the arm on their own
directory name. The N=4 rendezvous cell is the clearest: 21 `rendezvous_agreed`
rows across four robots, every one of them `cell: -1` with the refusal "no
(cell, interval) pair was agreed by the whole team before contact was lost",
600 s of wall clock, and a `run_gates_verdict=CLEAN` in the manifest. It was an
`off` cell wearing a `rendezvous` label, and nothing in the pipeline could say
so — the campaign index, the manifest, the gate report and the arm suffix all
agreed it was a treatment cell, because all four of them read the REQUEST.

That is the failure mode this file exists for, and it is worse than a crash: a
crashed cell is redone, whereas this one banks and dilutes the arm it was meant
to populate. A campaign of them reads as a null result.

WHAT IS CHECKED, AND WHY IT IS THESE THREE THINGS
-------------------------------------------------
The witness is the node's commit line, `Rendezvous AGREED by all N robots`,
which is printed exactly once per robot per distinct triple, from the one place
that assigns `rendezvous_agreed_` (explo_planner_node.cpp, maintainRendezvous-
Proposal). It is deliberately NOT the `rendezvous_agreed` event row: that row is
written when an appointment is ARMED, which only happens if the link drops, so
keying on it would conflate "the team never agreed" with "the team never
separated". Those want opposite remedies and only the first is a defect.

  G1  every robot committed.       A cell where some robots hold an agreement
      and some hold none is the split the protocol exists to prevent; it is also
      exactly what the N>=3 team-mutual window produces when it is too narrow
      (at N=4 the ts4 smoke measured mutual-and-whole lasting about two seconds
      before the team fragmented).

  G2  they CONVERGED on the same CELL.  The protocol adopts the proposer's
      integers verbatim precisely so this comparison is exact arithmetic and not
      a float that has been through anybody's hands. If it ever fails, half the
      fleet is standing in one cell and half in another.

      THE CELL, NOT THE TRIPLE. This compared all three integers until
      generation 20 made two of them inert, and it stays on the cell alone now
      that generation 23 has made them live again.

      On gen 20-22, `t_meet` was not agreed at all — armAppointment overwrote it
      with each robot's own countdown from the moment it noticed the team was
      incomplete, and gen 20 stopped printing it — and `interval` had not been
      read for timing since the recurrence was removed. Comparing them here
      meant two robots both driving to cell 44 could FAIL as a SPLIT FLEET over
      integers neither of them would ever act on.

      On gen 23 both are live again: the countdown is gone and armAppointment
      attends the agreed occurrence of the agreed recurring schedule. The test
      still keys on the cell, for a different reason — a same-cell disagreement
      about WHEN is self-healing here and a disagreement about WHERE is not.
      rendezvous_appointment_wait_sec is 0.0 on every campaign arm, an UNBOUNDED
      barrier, so a robot that picked the earlier occurrence stands at the agreed
      cell until the rest arrive; the fleet still meets and the cost is waiting.
      Two robots at two cells never meet at all.

      A gate that can invalidate a healthy cell is worse than no gate, because it
      spends the run AND reports a defect that is not there; three such cells
      abort the campaign.

      The other two are still compared, and a disagreement is still REPORTED on
      the verdict line — it would mean the adopt path had stopped copying
      verbatim, or that an upgrade round reached some robots and not others,
      which is worth knowing even though it is not, by itself, a split. On a
      gen-23 log the report carries the waiting cost in seconds.

      Convergence, not first commit: a robot may commit twice, because a triple
      derived before the allocator had any tour is a placeholder (one candidate,
      the centroid) that may be upgraded exactly once. Robots upgrade in echo
      order, so first-commit keying would fail a healthy run as a split. What
      stops that relaxation from covering a real split is the companion check
      that the SHAPE of each robot's commit sequence is one of [R], [P, R], [P]
      — never three commits, never a final downgraded back to a provisional.

  G3  no robot armed WITHOUT an agreement after it had committed one.  Once
      `rendezvous_agreed_` is set it is never cleared, so a post-commit arming
      that refuses for want of an agreement is a contradiction in the node, not
      a property of the run. Scoped to that ONE refusal text: "the agreed pair
      has already been used in this outage" is a legitimate second arming and
      must not be caught here.

REPORT-ONLY QUANTITIES travel on the PASS line (commit times, the agreed cell,
the two inert integers if they disagree, how many armings were real) because a
gate that only says PASS teaches nothing, and the commit time in particular is
the number that says whether the team agreed while whole or scrambled to agree
during the outage.

EXIT STATUS AND THE VERDICT
---------------------------
Lines are `VERDICT\tname\tdetail`, matching comms_gates.py, and the teardown in
run_explo_sim_rviz.sh appends them to comms_gates.txt where FAIL scores the run
INVALID and UNRUN scores it SUSPECT. This script exits 0 always: the caller
reads the file, and a checker that aborts the teardown trap would cost the
evidence it was run to collect.

UNRUN IS NOT A PASS, and this file is written so that every way of failing to
check lands there. The `checks-that-stopped-checking` lesson applies directly:
the gate greps for a log string the node prints, so a reworded log line would
otherwise turn the whole gate into a silent, permanent PASS. Hence the phrase
and the parse are tested SEPARATELY — a planner log that contains "Rendezvous
AGREED by all" but no line this file can parse is UNRUN, loudly, and never
absence-of-evidence-is-evidence-of-absence.

Usage:  rendezvous_agreement.py <outdir>
"""

import glob
import json
import os
import re
import sys

# The commit line, as emitted by explo_planner_node.cpp. Kept in two pieces on
# purpose: PHRASE is the cheap "did the node say anything at all" test and LINE
# is the parse. A node change that keeps the phrase and breaks the parse is the
# case that must not read as "no agreements", so it is UNRUN and not FAIL.
PHRASE = "Rendezvous AGREED by all"

# THREE TAILS, because three binary generations write this line and a banked run
# must stay re-gateable:
#
#   gen <= 19  "... cell 44, every 30s from t+130s (provisional=0)"
#   gen 20-22  "... cell 44, interval 30s (provisional=0, echoes 2/2)"
#   gen >= 23  "... cell 44, interval 30s from t+130s (provisional=0, echoes 2/2)"
#
# The newest wording appends the meeting time to the interval term so it can
# never parse on the every-Xs alternative. tmeet is None, not 0, on lines with
# no meeting time; 0 is a real value. (notes: gate-commit-line-wordings)
LINE = re.compile(
    r"\[(?P<wall>\d+\.\d+)\]\s*\[(?P<node>[^\]]*)\]:\s*"
    r"Rendezvous AGREED by all (?P<n>\d+) robots: cell (?P<cell>-?\d+), "
    r"(?:"
    r"every (?P<interval_old>-?\d+)s from t\+(?P<tmeet>-?\d+)s"
    r"(?: \(provisional=(?P<prov_old>[01])\))?"
    r"|"
    r"interval (?P<interval>-?\d+)s"
    r"(?: from t\+(?P<tmeet_new>-?\d+)s)?"
    r" \(provisional=(?P<prov>[01])"
    r"(?:, echoes (?P<echoes>\d+)/(?P<peers>\d+))?\)"
    r")"
)

# The ONE arming refusal that means "this robot holds no agreement". Matched as
# a substring so the wording can gain or lose the `t_meet` term (it did, on
# 2026-09-17: "(cell, interval) pair" became "(cell, interval, t_meet) triple")
# without the gate going quiet.
NO_AGREEMENT = "was agreed by the whole team"

# Marks a released barrier (a kept meeting); each one authorises one more
# commit. Matched as a substring on the part the settled and no-settle variants
# share. sim/manoeuvre_events.py greps the same line's prefix.
# (notes: gate-release-phrase)
RELEASE = "-> re-planning against merged map"

# G2b's FAIL bound: wall-clock seconds between the first and last robot to
# commit the final cell; 240 is the node's default mid-run maximum wait. A
# ceiling, not a target: override to calibrate, do not edit without data.
# (notes: gate-split-fail-bound)
SPLIT_FAIL_SEC = float(os.environ.get("RZV_SPLIT_FAIL_SEC", "240"))


def emit(verdict, detail):
    print("%s\trendezvous_agreed\t%s" % (verdict, detail))


def manifest(outdir):
    out = {}
    path = os.path.join(outdir, "run_manifest.txt")
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
    except OSError:
        return None
    return out


def main(argv):
    if len(argv) != 2:
        emit("UNRUN", "usage: rendezvous_agreement.py <outdir>")
        return 0
    outdir = argv[1]

    man = manifest(outdir)
    if man is None:
        emit("UNRUN", "no readable run_manifest.txt in %s" % outdir)
        return 0

    # Decide on the effective rendezvous_schedule, never the arm name. A value
    # other than 1 means nothing to check (silent); an absent key means the gate
    # lost its precondition, so UNRUN. (notes: gate-schedule-key-precondition)
    if "rendezvous_schedule" not in man:
        emit("UNRUN", "run_manifest.txt has no rendezvous_schedule key, so this "
                      "gate cannot tell a scheduled arm from an unscheduled "
                      "one. The launcher writes it; if it was renamed, this "
                      "gate needs updating rather than skipping.")
        return 0
    if man["rendezvous_schedule"] != "1":
        return 0

    robots = [r for r in man.get("robots", "").split(",") if r]
    if not robots:
        emit("UNRUN", "rendezvous_schedule=1 but the manifest names no robots")
        return 0

    # THE DENOMINATOR IS NOT WRITTEN DOWN. n_robots in the manifest and the set
    # of planner logs on disk are two independent statements of how big the team
    # was, and a gate that trusted either alone would score a cell that lost a
    # planner as a clean pass over the robots that survived.
    logs = {}
    for r in robots:
        p = os.path.join(outdir, "planner_%s.log" % r)
        if os.path.exists(p):
            logs[r] = p
    stray = sorted(
        os.path.basename(p)[len("planner_"):-len(".log")]
        for p in glob.glob(os.path.join(outdir, "planner_*.log"))
        if os.path.basename(p)[len("planner_"):-len(".log")] not in robots
    )
    missing = [r for r in robots if r not in logs]
    if missing or stray:
        emit("UNRUN",
             "cannot judge: planner log missing for %s%s (manifest robots=%s, "
             "n_robots=%s)"
             % (",".join(missing) or "nobody",
                "; unexpected planner log(s) for %s" % ",".join(stray)
                if stray else "",
                ",".join(robots), man.get("n_robots", "?")))
        return 0

    # --- G1/G2: who committed, and to what ---------------------------------
    # Every commit line is kept: a robot legitimately recommits once on a
    # provisional-to-final upgrade and once per meeting kept. G2 compares final
    # commits; the shape check below bounds the count.
    # (notes: gate-every-commit-kept)
    commits = {}        # robot -> [(cell, interval, t_meet, prov, wall), ...]
    saw_phrase = set()
    unparsed = []
    # echoes E/P: peers heard holding this triple out of all peers; the commit
    # rule requires E == P == fleet-1. Kept apart from the positionally indexed
    # commit tuple; checked only on lines that set tmeet_new.
    # (notes: gate-echo-terms)
    echo_rows = []      # (robot, n_asserted, echoes, peers, wall) — gen-23 only
    echo_missing = []   # (robot, line) — gen-23 shape with no echo term at all
    releases = {}       # robot -> barriers this robot actually released from
    for r, path in sorted(logs.items()):
        try:
            with open(path, errors="replace") as fh:
                for line in fh:
                    if RELEASE in line:
                        releases[r] = releases.get(r, 0) + 1
                    if PHRASE not in line:
                        continue
                    saw_phrase.add(r)
                    m = LINE.search(line)
                    if not m:
                        unparsed.append((r, line.strip()[:160]))
                        continue
                    # prov is None on a pre-generation-17 log, which has no such
                    # group. That is not an error and not a zero: it is "this
                    # binary could not tell you", and the shape check below sits
                    # out rather than inventing an answer.
                    prov = m.group("prov")
                    if prov is None:
                        prov = m.group("prov_old")
                    interval = m.group("interval")
                    if interval is None:
                        interval = m.group("interval_old")
                    # tmeet is None, not 0, on lines that print no meeting time;
                    # 0 is a value the other wordings can really emit, and the
                    # two must not merge. (notes: gate-tmeet-none-not-zero)
                    tmeet = m.group("tmeet_new")
                    if tmeet is None:
                        tmeet = m.group("tmeet")
                    else:
                        # The node prints the meeting time and the echo term in
                        # one literal, so a line with one but not the other is
                        # an unknown shape: recorded in echo_missing (UNRUN
                        # below), never skipped. (notes: gate-echo-term-missing)
                        if m.group("echoes") is None:
                            echo_missing.append((r, line.strip()[:160]))
                        else:
                            echo_rows.append(
                                (r, int(m.group("n")), int(m.group("echoes")),
                                 int(m.group("peers")), float(m.group("wall"))))
                    commits.setdefault(r, []).append(
                        (int(m.group("cell")), int(interval),
                         None if tmeet is None else int(tmeet),
                         None if prov is None else prov == "1",
                         float(m.group("wall"))))
        except OSError as exc:
            emit("UNRUN", "cannot read %s: %s" % (path, exc))
            return 0

    # Any unparsed commit line makes the cell UNRUN, even if other robots
    # parsed: an unparsed robot must not be read as one that never committed.
    # (notes: gate-unparsed-line-unrun)
    if unparsed:
        who = sorted(set(r for r, _ in unparsed))
        emit("UNRUN",
             "the commit line changed shape — %d line(s) on %d robot(s) (%s) "
             "contain %r but do not match this gate's parser (first: %s). "
             "%d robot(s) did parse, so this is a partial break and the "
             "unparsed robots must NOT be read as silent. Update LINE in "
             "sim/rendezvous_agreement.py."
             % (len(unparsed), len(who), ",".join(who), PHRASE,
                unparsed[0][1], len(commits)))
        return 0

    if len(commits) < len(robots):
        never = [r for r in robots if r not in commits]
        # Parse failures already returned UNRUN, so these robots printed
        # nothing. Asserted via saw_phrase: a robot that printed the phrase but
        # has no commit means the gate's bookkeeping drifted, so UNRUN.
        # (notes: gate-silent-robot-assert)
        mis = [r for r in never if r in saw_phrase]
        if mis:
            emit("UNRUN",
                 "%s printed the commit phrase but produced no parsed commit "
                 "while the parser reported no unparsed lines — the two "
                 "bookkeeping paths in this gate have drifted apart and its "
                 "answer cannot be trusted." % ",".join(sorted(mis)))
            return 0
        # WHY it did not commit, in the node's own words, so the failure is
        # actionable from the gate line alone.
        why = []
        for r in never:
            last = ""
            try:
                with open(logs[r], errors="replace") as fh:
                    for line in fh:
                        if "Rendezvous proposal" in line:
                            last = line.strip()
            except OSError:
                pass
            why.append("%s: %s" % (r, last.split("]: ", 1)[-1] if last
                                   else "no proposal line at all"))
        emit("FAIL",
             "SCHEDULED ARM WITH NO AGREEMENT: %d/%d robot(s) committed a "
             "rendezvous cell (%s never did). This cell ran %s with no treatment. "
             "Last proposal line per silent robot — %s"
             % (len(commits), len(robots), ",".join(never),
                man.get("reconnect_mode_param", "?"), " | ".join(why)))
        return 0

    # --- G1b: the commit rule's own invariant, from the numbers on the line ---
    # On lines with an echo term require E == P (every peer echoed), P == N-1
    # (peers are the fleet minus self) and N == roster size. A violation is
    # FAIL, not UNRUN: the line parsed and contradicts its binary.
    # (notes: gate-g1b-echo-invariant)
    if echo_missing:
        emit("UNRUN",
             "the commit line carries a meeting time but no echo term — %d "
             "line(s) on %d robot(s) (first: %s). Generation 23 prints both "
             "from one format string, so this is a wording this gate does not "
             "know and the commit-rule invariant DID NOT RUN. Update LINE in "
             "sim/rendezvous_agreement.py."
             % (len(echo_missing), len(set(r for r, _ in echo_missing)),
                echo_missing[0][1]))
        return 0
    bad_echo = ["%s@%.1f: N=%d echoes %d/%d" % (r, wall, n, e, p)
                for (r, n, e, p, wall) in echo_rows
                if e != p or p != n - 1 or n != len(robots)]
    if bad_echo:
        emit("FAIL",
             "COMMIT RULE REGRESSED: %d of %d gen-23 commit line(s) violate "
             "echoes==peers==fleet-1 or disagree with the %d-robot roster — %s. "
             "A commit printed with fewer echoes than peers is a triple agreed "
             "without hearing the team agree to it, which is the one thing the "
             "echo round exists to prevent."
             % (len(bad_echo), len(echo_rows), len(robots),
                "; ".join(bad_echo[:6])))
        return 0

    # WHERE THE FLEET ENDED UP. The last commit is the agreement the robot is
    # holding when the run ends, and the only one it can still drive to.
    final = {r: v[-1] for r, v in commits.items()}

    # G2 compares only the final cell. With the unbounded appointment barrier a
    # same-cell disagreement about when costs waiting; two cells never meet.
    # interval and t_meet disagreements are reported, not failed.
    # (notes: gate-g2-cell-not-triple)
    cells = set(v[0] for v in final.values())
    behind_txt = ""
    if len(cells) != 1:
        # Before failing, discount commits nobody can still act on: a
        # rendezvous_outcome exists for their t_meet, or t_meet is past the last
        # t_rel_sec (t_meet's clock, not t_sim). Unproven counts as live.
        # (notes: gate-round-skew-live-commits)
        resolved = {}
        run_end = None
        for r in robots:
            seen = resolved.setdefault(r, set())
            try:
                with open(os.path.join(outdir, "%s.events.jsonl" % r),
                          errors="replace") as fh:
                    for line in fh:
                        try:
                            d = json.loads(line)
                        except ValueError:
                            continue
                        t_rel = d.get("t_rel_sec")
                        if t_rel is not None and (run_end is None
                                                  or float(t_rel) > run_end):
                            run_end = float(t_rel)
                        if d.get("event") != "rendezvous_outcome":
                            continue
                        t_meet = d.get("t_meet_sec")
                        if t_meet is not None:
                            seen.add(round(float(t_meet)))
            except OSError:
                pass
        live = {}
        for r, rows in commits.items():
            for c in reversed(rows):
                if c[2] is not None:
                    if run_end is not None and c[2] > run_end:
                        continue        # nobody could have attended it
                    if round(c[2]) in resolved.get(r, ()):
                        continue        # the node says it came and went
                live[r] = c[0]
                break
        if len(set(live.values())) < 2:
            # Not a split, but G2b and the PASS line need one cell: use the cell
            # of the latest t_meet every robot committed to. With no shared
            # t_meet the FAIL below stands. (notes: gate-round-skew-cell)
            shared = set.intersection(*[set(c[2] for c in rows
                                            if c[2] is not None)
                                        for rows in commits.values()])
            agreed = set(c[0] for rows in commits.values() for c in rows
                         if shared and c[2] == max(shared))
            if len(agreed) == 1:
                cells = set(agreed)
                behind_txt = (
                    "; ROUND SKEW: final commits named %d different cells (%s) "
                    "but none of them was still live — %s. Scored at t_meet "
                    "%ds, the last appointment the whole fleet committed to."
                    % (len(set(v[0] for v in final.values())),
                       ", ".join("%s=cell %d" % (r, final[r][0])
                                 for r in sorted(final)),
                       "no robot held an actionable appointment" if not live
                       else "the live ones agree on cell %d" % live[
                           sorted(live)[0]],
                       max(shared)))
    if len(cells) != 1:
        # A split can arise without a bug: the derive-round cut that
        # TeamWorld.msg accepts, or a cut P->R upgrade. The cell FAILs either
        # way; the detail line says which the evidence points to.
        # (notes: gate-split-no-bug-causes)
        by_prov = {}
        for rr, vv in final.items():
            by_prov.setdefault(vv[3], set()).add(vv[0])
        cross_gen = (None not in by_prov and len(by_prov) > 1
                     and all(len(s) == 1 for s in by_prov.values()))
        cause = ("The disagreement lines up EXACTLY with the provisional flag "
                 "(every placeholder-holder agrees with every other, and so "
                 "does every final-holder), which is the signature of a cut "
                 "upgrade round rather than a propose/echo defect — look at the "
                 "commit wall times first."
                 if cross_gen else
                 "The disagreement does NOT line up with the provisional flag, "
                 "so it is not a cut upgrade round: robots holding the same "
                 "generation ended on different integers. The protocol adopts "
                 "the proposer's integers verbatim, so suspect the propose/echo "
                 "path, a second proposer, or two campaigns sharing a bus.")
        emit("FAIL",
             "SPLIT FLEET: the team converged on %d different cells — %s. %s"
             % (len(cells),
                "; ".join("%s=cell %d (interval %ds, after %d commit%s)"
                          % (r, final[r][0], final[r][1],
                             len(commits[r]), "" if len(commits[r]) == 1 else "s")
                          for r in sorted(commits)),
                cause))
        return 0

    cell = cells.pop()

    # A same-cell disagreement on interval or t_meet is not a split, but may
    # mean the propose/echo path drops fields or an upgrade round was cut. It
    # goes on the detail line, with the t_meet spread in seconds.
    # (notes: gate-schedule-integer-note)
    inert_txt = ""
    inert = set((v[1], v[2]) for v in final.values())
    if len(inert) > 1:
        tmeets = [v[2] for v in final.values() if v[2] is not None]
        cost = ""
        if len(tmeets) == len(final) and len(set(tmeets)) > 1:
            cost = (" The meeting times span %ds, which on a generation that "
                    "prints t_meet is time the early arrivals spend waiting at "
                    "the cell." % (max(tmeets) - min(tmeets)))
        inert_txt = (" NOTE: same cell but %d different (interval, t_meet) "
                     "pairs — %s. The fleet still meets (the appointment "
                     "barrier is unbounded, so the early robot waits), so this "
                     "is not a split; but a propose/echo path that drops fields "
                     "would look exactly like it.%s"
                     % (len(inert),
                        "; ".join("%s=%ds/%s" % (r, final[r][1],
                                                 "t+%ds" % final[r][2]
                                                 if final[r][2] is not None
                                                 else "no t_meet")
                                  for r in sorted(commits)),
                        cost))

    # Per robot, over the whole run: at most 1 commit, plus 1 if the first was
    # provisional, plus 1 per barrier released; no provisional after the first.
    # Skipped when the flag is unreported. (notes: gate-commit-shape-budget)
    shape = []
    for r in sorted(commits):
        provs = [c[3] for c in commits[r]]
        if any(p is None for p in provs):
            continue                      # binary predates the flag; unknowable
        met = releases.get(r, 0)
        budget = 1 + (1 if provs[0] else 0) + met
        if len(provs) > budget:
            shape.append("%s committed %d times but kept %d meeting(s), which "
                         "allows %d" % (r, len(provs), met, budget))
        if any(provs[1:]):
            shape.append("%s committed provisional=1 after its first commit "
                         "(sequence %s)"
                         % (r, "".join("P" if p else "R" for p in provs)))
    if shape:
        emit("FAIL",
             "ILLEGAL COMMIT SEQUENCE: %s. A robot may hold one pair, upgrade "
             "it off the centroid fallback once, and re-agree once per meeting "
             "it kept — nothing else. A commit no meeting paid for means the "
             "one-pair-per-meeting rule in maintainRendezvousProposal is not "
             "holding, and because t_meet is minted as now+interval the extra "
             "derive slides the instant the fleet had already agreed to."
             % "; ".join(shape))
        return 0

    # --- G2b: HOW LONG WAS THE FLEET SPLIT BEFORE IT CONVERGED? ---
    # G2 sees only final commits, so a split healed before the run ends passes
    # it. split_sec is the spread of each robot's first commit of the winning
    # cell, reported on every cell; FAIL only above SPLIT_FAIL_SEC.
    # (notes: gate-g2b-split-duration)
    first_on_winner = {}
    for r in sorted(commits):
        for c in commits[r]:
            if c[0] == cell:
                first_on_winner[r] = c[4]
                break
    if len(first_on_winner) == len(commits):
        split_sec = max(first_on_winner.values()) - min(first_on_winner.values())
    else:
        # Cannot happen while G2 above holds (every robot's last commit IS the
        # winner), so this is a guard against a future edit reordering the two,
        # not a live branch. Say so rather than reporting a wrong zero.
        split_sec = None
    if split_sec is not None and split_sec > SPLIT_FAIL_SEC:
        emit("FAIL",
             "SPLIT APPOINTMENT: the fleet agreed in the end, but the first and "
             "last robot to commit the winning cell (%d) are %.1f s "
             "apart, over the %.0f s bound — %s. For that whole window at least "
             "one robot was driving or waiting at a different cell, which G2 "
             "cannot see because it compares only final commits."
             % (cell, split_sec, SPLIT_FAIL_SEC,
                ", ".join("%s@%.1f" % (r, first_on_winner[r])
                          for r in sorted(first_on_winner))))
        return 0

    # --- G3: an arming that refused for want of the agreement it already held ---
    contradictions = []
    armed_total = 0
    armed_real = 0
    for r in robots:
        ev = os.path.join(outdir, "%s.events.jsonl" % r)
        if not os.path.exists(ev):
            emit("UNRUN", "no %s.events.jsonl — cannot check armings" % r)
            return 0
        # The FIRST commit, deliberately. rendezvous_agreed_ is set by the first
        # one and never cleared — an upgrade overwrites it, it does not unset it
        # — so the contradiction window opens at the earliest commit, not the
        # one the robot finished with.
        commit_wall = commits[r][0][4]
        try:
            with open(ev, errors="replace") as fh:
                for line in fh:
                    if '"rendezvous_agreed"' not in line:
                        continue
                    try:
                        d = json.loads(line)
                    except ValueError:
                        continue
                    if d.get("event") != "rendezvous_agreed":
                        continue
                    armed_total += 1
                    if d.get("cell", -1) >= 0:
                        armed_real += 1
                        continue
                    if NO_AGREEMENT not in (d.get("refused") or ""):
                        continue
                    # t_wall_sec is required: a refused row without a usable
                    # value is UNRUN, never skipped, since skipping would let G3
                    # pass forever. (notes: gate-g3-t-wall-required)
                    t_wall_raw = d.get("t_wall_sec")
                    try:
                        t_wall = float(t_wall_raw)
                    except (TypeError, ValueError):
                        emit("UNRUN",
                             "%s: a refused rendezvous_agreed row carries no "
                             "usable t_wall_sec (%r). G3 orders armings against "
                             "the commit's wall clock, so this cell cannot be "
                             "evaluated — the armed-without-the-triple check "
                             "DID NOT RUN." % (r, t_wall_raw))
                        return 0
                    if t_wall > commit_wall:
                        contradictions.append(
                            "%s at t_rel %.1fs" % (r, d.get("t_rel_sec", -1.0)))
        except OSError as exc:
            emit("UNRUN", "cannot read %s: %s" % (ev, exc))
            return 0

    if contradictions:
        emit("FAIL",
             "ARMED WITHOUT THE TRIPLE IT HELD: %d arming(s) refused for "
             "'no agreement' AFTER their own robot logged the commit — %s. "
             "rendezvous_agreed_ is never cleared, so this is a node defect."
             % (len(contradictions), "; ".join(contradictions[:6])))
        return 0

    first = min(v[0][4] for v in commits.values())
    last = max(v[-1][4] for v in commits.values())

    # The PASS line says whether the fleet holds the centroid placeholder or a
    # tour-informed cell: a placeholder cell passes but measures
    # meet-at-the-centroid, and nothing else in the run records that.
    # (notes: gate-pass-line-mechanism)
    final_prov = [v[3] for v in final.values()]
    upgraded = sum(1 for v in commits.values() if len(v) > 1)
    if any(p is None for p in final_prov):
        how = "; provisional=unreported (binary predates the flag)"
    elif all(final_prov):
        how = ("; PLACEHOLDER: every robot is holding the centroid fallback — "
               "the argmin never saw a tour, so this cell measures "
               "meet-at-the-centroid, not the scheduled treatment")
    elif any(final_prov):
        how = ("; MIXED: %d/%d robot(s) still hold the centroid fallback"
               % (sum(1 for p in final_prov if p), len(final_prov)))
    else:
        how = ("; tour-informed%s"
               % (" (%d robot(s) upgraded off the centroid fallback)" % upgraded
                  if upgraded else ""))

    # split (G2b: first to last adoption of the winning cell) is not span
    # (earliest to latest commit of anything); an upgraded fleet shows a large
    # span. Both are printed so neither is inferred from the other.
    # (notes: gate-split-vs-span)
    split_txt = ("; split %.1fs" % split_sec if split_sec is not None
                 else "; split unmeasurable")
    # An empty bad_echo means either every checked line was sound or there were
    # no echo-term lines to check; the PASS line must tell the two apart.
    # (notes: gate-g1b-ran-on-pass-line)
    echo_txt = ("; %d commit line(s) carried a full echo round" % len(echo_rows)
                if echo_rows else
                "; NO gen-23 commit lines — the echoes==peers==fleet-1 "
                "invariant was not exercised on this cell")
    # With no armings G3 passes vacuously (armings only happen when the link
    # drops); say so in words, since a 0/0 ratio reads as a measurement of the
    # appointment machinery. (notes: gate-zero-armings-wording)
    if armed_total == 0:
        armed_txt = ("no appointment was ever armed, so G3 had nothing to "
                     "check — the team never separated on this cell")
    else:
        armed_txt = ("%d/%d arming(s) carried the agreed cell"
                     % (armed_real, armed_total))
    emit("PASS",
         "all %d robot(s) converged on cell %d; commits span %.1fs%s; %s%s%s%s%s"
         % (len(robots), cell, last - first, split_txt,
            armed_txt, echo_txt, how, inert_txt, behind_txt))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
