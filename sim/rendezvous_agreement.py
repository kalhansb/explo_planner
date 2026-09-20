#!/usr/bin/env python3
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
# The gen-20 wording dropped "from t+Ns" because armAppointment had been changed
# to overwrite the agreed meeting time with a private per-robot countdown, so
# there was no agreed meeting time left to print; it added the echo count because
# a FINAL triple committed on first-hand evidence and the old line asserted the
# fleet size as though it had been observed. Gen 23 restored the agreed instant
# — armAppointment attends the committed schedule again — and so restored the
# meeting time to the line.
#
# GEN 23 DOES NOT REUSE THE GEN-19 PHRASING, deliberately. The two forms carry
# the same pair of integers, so a gen-23 line written as "every 30s from t+130s"
# would parse through the FIRST alternative below and be indistinguishable from a
# pre-gen-20 binary in a re-gated bank. The meeting time is appended to the
# interval term instead, which lands in the third alternative and nowhere else.
#
# `tmeet` is therefore unknown-not-zero on a gen-20..22 log, and the code below
# must keep treating it that way: 0 is a value the other two formats can really
# emit.
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

# A BARRIER THAT ACTUALLY RELEASED. Generation 29 lets a run hold a SEQUENCE of
# triples — a meeting that is kept re-sites the next one — so "how many times
# may this robot commit" stopped being a constant and became "once, plus one for
# each meeting it kept". This is the phrase that marks a keeping. Matched as a
# substring and deliberately on the half of the line that both variants share:
# the settled form appends "(held Ns; the exchange applied ...)" and the
# no-settle form "(no settle configured, ...)", and both authorise exactly one
# re-agreement. sim/manoeuvre_events.py greps the same line's prefix.
RELEASE = "-> re-planning against merged map"

# G2b's bound, in seconds of wall clock between the first and last robot to
# commit the cell the fleet ends on. Set to the node's mid-run maximum wait,
# because that is the point at which the harm stops being theoretical: a robot
# holding the other cell for longer than one full wait can arrive, wait out an
# entire appointment and leave, all at a cell nobody else is coming to.
#
# IT IS A CEILING, NOT A TARGET. The design's own accepted residual is one
# TeamWorld period, so a healthy cell should read single-digit seconds and
# anything approaching this bound is already pathological. It is set here rather
# than tight because no campaign has yet produced a distribution of this
# quantity -- writing a tight bound from a guess is how a gate starts failing
# healthy cells, and writing one from the first campaign's own numbers is how it
# stops failing anything. Override to calibrate; do not edit the default until
# there are numbers behind it.
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

    # Inert on the unscheduled arms BY THE EFFECTIVE PARAMETER, never by the arm
    # name. `reconnect_mode_requested` is the request and the off arm's manifest
    # has been observed carrying a treatment name in it; `rendezvous_schedule` is
    # what the launcher actually passed to the node.
    # A MISSING KEY IS NOT AN INERT ARM. Splitting these two cases is the whole
    # point: `rendezvous_schedule=0` is an arm that legitimately has nothing to
    # check, and silence is right. An ABSENT key means this gate no longer knows
    # how to find its own precondition — a launcher rename, a manifest format
    # change — and the old `!= "1"` lumped them together and returned 0, so a
    # rename would have made this gate pass-by-absence on every cell of a
    # campaign while still printing nothing at all. That is the shape in
    # ~/hmr_campaign: a check that stops checking keeps reporting success.
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
    # EVERY commit line is kept, not just the first. A robot commits more than
    # once for two unrelated reasons, and both are legitimate:
    #
    #   THE UPGRADE. The proposer can derive before the allocator has produced a
    #   single tour, in which case the argmin has one candidate (the centroid of
    #   the frozen snapshot) and the triple it yields is a placeholder rather
    #   than a choice. It is published with rendezvous_provisional=1 and may be
    #   replaced exactly once, by a triple that had something to choose between.
    #
    #   THE RE-AGREEMENT (generation 29). A meeting the team KEEPS ends with the
    #   fleet standing on the cell agreeing where and when to meet next, so a
    #   run holds a sequence of triples rather than one. One per meeting kept,
    #   which is what the shape check below counts.
    #
    # Keying G2 on the FIRST commit — which is what this gate did when neither
    # path existed — fails a healthy upgraded run as a SPLIT FLEET, because
    # robots upgrade in whatever order the echoes land. G2 therefore compares
    # where the fleet CONVERGED, and the shape itself is checked separately
    # below so that the relaxation cannot hide a real split.
    commits = {}        # robot -> [(cell, interval, t_meet, prov, wall), ...]
    saw_phrase = set()
    unparsed = []
    # THE ECHO TERMS, WHICH LINE HAS ALWAYS CAPTURED AND THIS FILE USED TO THROW
    # AWAY. `echoes E/P` is the node's own statement of how many peers had been
    # heard holding this exact triple (E) out of how many there are (P), and on
    # generation 23 the commit rule requires E == P == fleet-1 for EVERY commit
    # (the `if (on_pair != fleet_.size() - 1) return;` early return above the log
    # call). The node's comment beside that line says, in as many words, that if
    # the two ever differ in a banked log the commit rule regressed and a reader
    # holding only the text log should be able to see it. Capturing three regex
    # groups and never comparing them is how that reader never does.
    #
    # Kept SEPARATE from the commit tuple rather than appended to it, because
    # four call sites index that tuple positionally and none of them needs these.
    #
    # G23 ONLY. On generations 20-22 a final triple could legitimately commit on
    # first-hand evidence with E < P, so the same numbers mean different things
    # either side of that boundary and the check has to know which side it is on.
    # The discriminator is `tmeet_new`: only the third regex alternative can set
    # it, and that alternative is the gen-23 wording.
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
                    # tmeet is absent on a generation-20..22 log by design — the
                    # line stopped printing a meeting time when there stopped
                    # being one, and gen 23 put it back. None, not 0: 0 is a
                    # value both printing formats could really emit, and the two
                    # must not merge.
                    tmeet = m.group("tmeet_new")
                    if tmeet is None:
                        tmeet = m.group("tmeet")
                    else:
                        # Gen-23 wording. The format string prints the meeting
                        # time and the echo term TOGETHER, in one literal, so a
                        # line carrying one without the other is a shape this
                        # parser does not actually know — the echo group is
                        # optional in LINE only so that gen-19 and gen-23 can
                        # share one alternative's tail. Recorded, not skipped:
                        # skipping is how a reworded line turns the check below
                        # into a permanent silent pass.
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

    # ANY UNPARSED LINE IS A BROKEN GATE, not a quiet robot (2026-09-19).
    #
    # This used to be `if unparsed and not commits`, so a PARTIAL parse failure
    # — some robots on a wording this file knows, some on one it does not — fell
    # straight through to G1 below, where the unparsed robots were reported as
    # never having committed and the cell FAILed as "SCHEDULED ARM WITH NO
    # AGREEMENT". That is the gate's own blind spot misattributed to the node,
    # in the INVALID direction, on a cell that may be perfectly healthy; three of
    # them abort the campaign. The parser is either current or it is not, and
    # when it is not the only honest verdict is that nothing was checked.
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
        # Cannot be a parse failure — that returned UNRUN above — so every robot
        # here printed nothing at all. Asserted rather than assumed, because the
        # difference between the two is the difference between blaming the node
        # and blaming this file, and the whole point of saw_phrase is to keep
        # that distinction observable instead of inferred.
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
    #
    # Generation 23 commits only when every peer has been heard holding the
    # triple, so on a gen-23 line "all N robots" is an OBSERVATION, not an
    # inference from the protocol — and the two numbers that make it one are
    # printed right there. Three things must hold on every such line:
    #
    #   E == P     every peer echoed. E < P means a robot printed "AGREED by
    #              all" while holding fewer echoes than there are peers, i.e.
    #              the early return that guards the commit stopped guarding it.
    #   P == N-1   the peer count is the fleet minus self. A mismatch means the
    #              node's own two statements of team size disagree with each
    #              other, within one format string.
    #   N == roster  the node's fleet is the campaign's roster. G1 above proves
    #              every robot in the manifest committed; this proves the node
    #              was not simultaneously counting somebody else. Without it a
    #              cell where one robot never joined the fleet passes G1 (the
    #              robots that exist all committed) while the line itself says
    #              the team is smaller than the campaign thinks it is.
    #
    # FAIL and not UNRUN: nothing here is a limit of this gate's knowledge. The
    # line parsed, the integers are present, and they contradict the binary that
    # printed them.
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

    # THE CELL, NOT THE TRIPLE. This compared all three integers until
    # generation 20 made two of them inert, and it stays on the cell alone now
    # that generation 23 has made them live again — for a DIFFERENT reason at
    # each end, so both halves of the argument are written down here.
    #
    # On gen 20-22 the other two integers steered nothing: `t_meet` was not
    # printed at all (a robot departed on its own countdown, so there was no
    # fleet-wide meeting instant to agree on) and `interval` was carried for
    # provenance. Failing on them meant two robots both driving to cell 44 could
    # FAIL as a SPLIT FLEET over integers neither of them would ever act on.
    #
    # On gen 23 both are live: armAppointment attends the agreed occurrence of
    # the agreed schedule. But a same-cell disagreement about WHEN is
    # self-healing on this generation and a disagreement about WHERE is not.
    # rendezvous_appointment_wait_sec is 0.0 on every campaign arm, which is an
    # UNBOUNDED barrier: the robot that picked the earlier occurrence stands at
    # the agreed cell until the others arrive, so the fleet still meets and the
    # cost is waiting, bounded by one interval. Two robots at two cells never
    # meet at all. So the cell is still the whole test — not because the other
    # two integers are inert, but because only one of the three can cost the
    # experiment its meeting. Disagreement on the other two is reported below.
    #
    # A gate that can invalidate a healthy cell is worse than no gate, because it
    # spends the run AND reports a defect that is not there; three of them abort
    # the campaign.
    cells = set(v[0] for v in final.values())
    if len(cells) != 1:
        # NAME THE TWO NO-BUG PRODUCERS BEFORE BLAMING THE CODE. This detail
        # used to assert the split "cannot happen without a bug in the
        # propose/echo path", which is not what the design says: TeamWorld.msg
        # documents the derive-round cut as the residual the design accepts
        # rather than pretends away, and the one permitted P->R upgrade reopens
        # a second window of the same kind. Both produce a genuine split that no
        # code change would remove, and at N=4 -- where the mutual window is
        # seconds -- they are the likely explanation, not the unlikely one. The
        # cell is still INVALID either way; what changes is where the next hour
        # goes.
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

    # THE OTHER TWO INTEGERS, REPORTED AND NOT FAILED ON. Whether they steer a
    # robot depends on the generation (see the block above): on gen 20-22 they
    # steer nothing, on gen 23 they are the meeting schedule. Either way a
    # same-cell disagreement is not a split fleet — the unbounded appointment
    # barrier turns a wrong occurrence into waiting — but it IS the fingerprint
    # of a propose/echo path that has started dropping fields, or of an upgrade
    # round that reached some robots and not others, so it must be visible
    # rather than dropped on the floor. Carried into the PASS/FAIL detail line
    # below; never a verdict of its own.
    #
    # ON A GEN-23 LOG THIS NOTE HAS A COST ATTACHED and the reader should price
    # it: the robots that disagree pay up to the difference in meeting times
    # standing at the agreed cell, which is exploration time the arm does not
    # get back. It is reported in seconds for exactly that reason.
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

    # THE SHAPE CHECK. G2 above was relaxed from "every commit line agrees" to
    # "every robot ended in the same place"; this is what keeps that relaxation
    # from covering for a real defect. A commit nothing paid for is a protocol
    # violation even when the fleet happens to agree at the end, and a
    # provisional commit AFTER a final one is a downgrade off a real choice back
    # onto a placeholder. Sits out entirely on a pre-generation-17 log, which
    # cannot report the flag.
    #
    # THE BUDGET IS NOT A CONSTANT ANY MORE (generation 29). It was "max 2: P
    # then R" — one pair per run, optionally upgraded once off the centroid
    # fallback — and gen 29 makes a kept meeting re-site the next one, so a
    # healthy two-meeting run commits three times and the old rule failed it by
    # construction. What replaces it is the same rule with the meetings counted:
    # one pair to start, one more if the first was the placeholder, and ONE PER
    # BARRIER THIS ROBOT ACTUALLY RELEASED FROM. Anything above that is a derive
    # no meeting authorised, which is not cosmetic — deriveRendezvousProposal
    # mints t_meet as now+interval, so every unpaid-for commit slides the
    # instant the fleet had agreed to. The ts4 gen-29 N=2 rendezvous smoke is
    # the case: one release, four commits, cell 23 at t+1055s and then t+1061s.
    #
    # Per robot rather than fleet-wide because the two are not interchangeable:
    # a follower adopts the proposer's next triple as soon as it hears it, which
    # can be a tick before its own release, so a running comparison would race.
    # Counted over the whole run, it cannot.
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
    #
    # G2 compares only each robot's LAST commit, which is the right question for
    # "can they still drive to the same place" and the wrong one for "did this
    # cell contain a split appointment". Those come apart in exactly one way:
    # the fleet cuts an upgrade round, some robots drive the placeholder while
    # others drive the final cell, and then the radio comes back and everyone
    # re-latches before the run ends. Every robot's final commit agrees, G2
    # PASSes, and the cell silently contains up to a full wait-cap of one robot
    # standing at the wrong cell. That is a completion-time bias landing
    # squarely in the two arms the campaign is comparing, with no witness.
    #
    # The measurement is the spread of FIRST commits of the winning cell: from
    # the earliest robot to adopt it to the last, at least one robot was holding
    # something else. Reported on every cell rather than only on the bad ones,
    # because a threshold nobody has calibrated is worse than a number everyone
    # can see -- and this number has never been observed, so the FAIL bound
    # below is deliberately set where the harm is unarguable (a robot could have
    # waited out an entire appointment at the wrong cell) rather than where the
    # protocol is merely untidy. Tighten it once a campaign has produced a
    # distribution; do not tighten it from this comment.
    # Keyed on the cell for the same reason G2 is: a robot that re-committed the
    # same cell with a different interval never moved, so counting that as
    # "adopting the winner late" would manufacture a split window out of a field
    # nothing reads.
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
                    # t_wall_sec IS REQUIRED, NOT OPTIONAL. This read
                    # `float(d.get("t_wall_sec", 0.0))` until 2026-09-18, so a
                    # row without the field compared 0.0 > <epoch seconds> —
                    # False, always — and was silently skipped. Drop the field
                    # from the envelope and G3 stops finding contradictions and
                    # PASSes forever, with nothing in the output distinguishing
                    # "no post-commit no-agreement armings happened" from "the
                    # field this test reads is gone". That is precisely what
                    # this module's docstring forbids (UNRUN IS NOT A PASS) and
                    # what the PHRASE/LINE split was built to prevent on the
                    # log-line half of the same file.
                    #
                    # "The commits parsed, so the field must be there" is NOT an
                    # argument: commit_wall comes from the cell>=0 rows, these
                    # are the refused cell<0 rows, and the two schemas can drift
                    # independently.
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

    # WHICH MECHANISM THIS CELL ACTUALLY EXERCISED, on the PASS line, because a
    # cell that converged on a placeholder is a legitimate rendezvous cell that
    # never ran the scheduler's argmin over the allocator's tours. It passes, it
    # banks, and it measures meet-at-the-centroid — a different treatment from
    # the one the arm name implies. That distinction is invisible in every other
    # artifact the run produces, so it has to be said here or not at all.
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

    # `split` is G2b's measurement and is NOT the same number as `span`. `span`
    # runs from the earliest commit of anything to the latest commit of
    # anything, so a healthy [P, R] fleet shows a large span purely because the
    # upgrade happened late. `split` runs between the first and last adoption of
    # the WINNING CELL, which is the interval during which the fleet was
    # actually holding two different appointments. Reading span as if it were
    # split overstates the harm on every upgraded cell; reading split as if it
    # were span hides the upgrade entirely. Both are here so neither has to be
    # inferred from the other.
    split_txt = ("; split %.1fs" % split_sec if split_sec is not None
                 else "; split unmeasurable")
    # WHETHER G1b ACTUALLY RAN, on the PASS line. `bad_echo` being empty has two
    # very different causes — every gen-23 line checked out, or there were no
    # gen-23 lines to check because the binary is older — and a PASS that does
    # not distinguish them is the shape `checks-that-stopped-checking` warns
    # about: the invariant would go quiet the moment the wording changed and the
    # output would look identical.
    echo_txt = ("; %d commit line(s) carried a full echo round" % len(echo_rows)
                if echo_rows else
                "; NO gen-23 commit lines — the echoes==peers==fleet-1 "
                "invariant was not exercised on this cell")
    # 0/0 IS SPELLED OUT RATHER THAN PRINTED AS A RATIO. G3 looks for armings
    # that refused for want of an agreement their own robot already held, so
    # with no armings at all it has nothing to contradict and passes vacuously
    # — which is correct, because armings only happen when the link drops, and a
    # cell whose team never separated is a legitimate rendezvous cell that never
    # reached the mechanism. But "0/0 arming(s) carried the agreed cell" reads
    # like a measurement, and the one thing a reader must not conclude from it
    # is that the appointment machinery was exercised and found sound.
    if armed_total == 0:
        armed_txt = ("no appointment was ever armed, so G3 had nothing to "
                     "check — the team never separated on this cell")
    else:
        armed_txt = ("%d/%d arming(s) carried the agreed cell"
                     % (armed_real, armed_total))
    emit("PASS",
         "all %d robot(s) converged on cell %d; commits span %.1fs%s; %s%s%s%s"
         % (len(robots), cell, last - first, split_txt,
            armed_txt, echo_txt, how, inert_txt))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
