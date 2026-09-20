#!/usr/bin/env python3
"""Known-answer calibration for rendezvous_agreement.py.

WHY A CALIB AND NOT JUST THE GATE
---------------------------------
The gate greps a log string emitted by C++ in a sibling package. Nothing links
the two: rewording the RCLCPP_INFO in explo_planner_node.cpp compiles, ships,
and turns the gate into a permanent silent PASS on every future cell — the exact
shape of `checks-that-stopped-checking`, where eight guards went inert while
still printing PASSes. The gate defends itself against that in ONE direction
(phrase present, parse failed => UNRUN), but it cannot defend against the phrase
itself changing, because then there is nothing left to notice.

So case 1 reads the node's own format string out of the source and renders it.
If the message is reworded, this file fails, in the workspace, before a campaign
is committed to a gate that no longer gates.

Cases 2-20 are synthetic cells with known verdicts. They pin each branch,
including the two that matter most and are the hardest to produce on demand: a
scheduled cell with zero commits (the ts4 generation-15 N=4 failure, which
passed every other gate in the pipeline) and an unscheduled cell, which must
stay SILENT rather than emit a PASS the off arm was never entitled to.

Cases 10-15 pin the provisional/final upgrade, added in generation 17. They come
in a matched pair on purpose: case 10 is a healthy upgrade that the gate must
NOT fail (it used to, by keying on each robot's first commit), and case 14 is a
half-finished upgrade with the SAME first commits that it must still fail. A
relaxation and the thing that stops the relaxation going too far belong in the
same file, or the second one gets dropped the next time this is edited.

Cases 16-18 pin the generation-18 additions, and 16/17 are a matched pair for
the same reason: G2b fails a fleet that took too long to converge (16) and must
NOT fail the healthy upgrade that looks identical except for its timing (17).
Case 18 pins the one failure mode the gate cannot detect from inside itself — a
renamed manifest key, which used to make it pass by absence on every cell.

Cases 19-20 pin the generation-20 relaxation, and they are a matched pair for
the third time: 20 is a cell the gate must no longer fail (same meeting cell,
different values in the two integers generation 20 stopped acting on) and 19 is
the thing that relaxation must not become — a gate that, when its own parser
goes stale on SOME robots, charges the silence to the node and FAILs a healthy
cell. Both directions cost a campaign: 20 as three spurious INVALIDs and an
abort, 19 as an hour spent in the wrong file.

Usage:  rendezvous_agreement_calib.py       # exits non-zero on any failure
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
GATE = os.path.join(HERE, "rendezvous_agreement.py")
NODE = os.path.normpath(os.path.join(
    HERE, "..", "explo_planner", "src", "explo_planner_node.cpp"))

fails = 0
cases = 0


def ok(msg):
    print("  PASS  %s" % msg)


def bad(msg):
    global fails
    fails += 1
    print("  FAIL  %s" % msg)


def run(outdir):
    p = subprocess.run([sys.executable, GATE, outdir],
                       capture_output=True, text=True)
    return p.stdout.strip()


def verdict_of(out):
    return out.split("\t")[0] if out else ""


def cell(tmp, name, *, scheduled=True, robots=("atlas", "bestla"),
         commits=None, armings=None, proposal_lines=None, extra_logs=()):
    """Build a synthetic outdir.

    commits[r] is one commit or a list of them, each

        (wall, n, cell, interval, tmeet)          provisional=0, the common case
        (wall, n, cell, interval, tmeet, prov)    prov in {0, 1, None}
        (wall, n, cell, interval, tmeet, prov, echo)

    where `echo` overrides the `echoes E/P` term on the two wordings that carry
    one. It is either a pair (E, P), or the string "omit" to write a gen-23 line
    with the meeting time and NO echo term — a shape the node's single format
    string cannot produce, which is exactly why the gate has to notice it rather
    than parse around it. Left out, it defaults to the healthy (n-1, n-1).

    where prov=None writes the PRE-GENERATION-17 line, with no `(provisional=N)`
    suffix at all. That is not the same as prov=0 and the difference is the whole
    point of one of the cases below: a binary that cannot report the flag must
    make the gate's shape check sit out, not read as "everything was final".

    WHICH WORDING IS WRITTEN. prov=None writes the gen<=19 wording ("every Xs
    from t+Ys"), because that is the only wording a binary too old to report the
    flag ever emitted; everything else writes the gen>=20 wording ("interval
    Xs (provisional=D, echoes E/P)"), which is what the node emits today. The
    tmeet field is therefore consumed on the old tail and DROPPED on the new
    one — it is kept in the tuple so the old cases still read the same, and so
    that a future reader can see at a glance which tail a case exercises. Both
    tails are live in the gate's regex because banked runs must stay re-gateable,
    so both have to be exercised here or half the parser is untested.
    """
    d = os.path.join(tmp, name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "run_manifest.txt"), "w") as fh:
        fh.write("# synthetic\n")
        fh.write("rendezvous_schedule=%s\n" % ("1" if scheduled else "0"))
        fh.write("reconnect_mode_param=%s\n" % ("rendezvous" if scheduled
                                                else "pursuit"))
        fh.write("robots=%s\n" % ",".join(robots))
        fh.write("n_robots=%d\n" % len(robots))
    for r in list(robots) + list(extra_logs):
        with open(os.path.join(d, "planner_%s.log" % r), "w") as fh:
            for ln in (proposal_lines or {}).get(r, []):
                fh.write("[INFO] [1789600000.000000000] [%s.explo_planner]: %s\n"
                         % (r, ln))
            c = (commits or {}).get(r)
            if c and not isinstance(c, list):
                c = [c]
            for one in (c or []):
                wall, n, cid, interval, tmeet = one[:5]
                prov = one[5] if len(one) > 5 else 0
                echo = one[6] if len(one) > 6 else None
                # The DEFAULT is the invariant the current binary holds, so a
                # case that does not mention echoes writes a healthy line and
                # the planted defect in every other case is unambiguous.
                e_n, e_d = (n - 1, n - 1) if echo in (None, "omit") else echo
                # WHICH OF THE THREE PRINTING FORMATS THIS FIXTURE IS ON, chosen
                # by what the caller left out rather than by a flag, so a case
                # cannot claim one generation and write another:
                #
                #   prov  is None -> gen <= 19  "every Xs from t+Ys."
                #   tmeet is None -> gen 20-22  "interval Xs (provisional=P, echoes a/b)."
                #   otherwise     -> gen >= 23  "interval Xs from t+Ys (provisional=P, echoes a/b)."
                #
                # The default is the CURRENT format on purpose. A calibrator
                # whose fixtures are all on a retired wording proves the gate can
                # read the bank and says nothing about the binary about to run.
                if prov is None:
                    tail = "every %ds from t+%ds." % (interval, tmeet)
                elif tmeet is None:
                    tail = ("interval %ds (provisional=%d, echoes %d/%d)."
                            % (interval, prov, e_n, e_d))
                elif echo == "omit":
                    tail = ("interval %ds from t+%ds (provisional=%d)."
                            % (interval, tmeet, prov))
                else:
                    tail = ("interval %ds from t+%ds (provisional=%d, "
                            "echoes %d/%d)."
                            % (interval, tmeet, prov, e_n, e_d))
                fh.write("[INFO] [%.9f] [%s.explo_planner]: Rendezvous AGREED "
                         "by all %d robots: cell %d, %s\n"
                         % (wall, r, n, cid, tail))
        with open(os.path.join(d, "%s.events.jsonl" % r), "w") as fh:
            for a in (armings or {}).get(r, []):
                fh.write(json.dumps(dict(
                    {"event": "rendezvous_agreed", "robot": r,
                     "t_rel_sec": 100.0, "cell": -1, "refused": ""}, **a)) + "\n")
    return d


tmp = tempfile.mkdtemp(prefix="rzcalib_")
try:
    # --- 1. the gate's parser still matches the node's own format string -----
    cases += 1
    try:
        src = open(NODE, errors="replace").read()
    except OSError as exc:
        src = ""
        bad("cannot read %s: %s" % (NODE, exc))
    # Recover the concatenated string literal the node passes to RCLCPP_INFO,
    # starting at the phrase the gate greps for. Adjacent C string literals are
    # joined the way the compiler joins them.
    #
    # COMMENT OCCURRENCES ARE SKIPPED. The node quotes this phrase in a comment
    # right above the call, to say that the text is load-bearing — and a comment
    # is a one-chunk "literal" that renders to the bare phrase and would fail
    # this case for the wrong reason.
    i = -1
    _at = src.find('"Rendezvous AGREED by all')
    while _at >= 0:
        _bol = src.rfind("\n", 0, _at) + 1
        if not src[_bol:_at].lstrip().startswith("//"):
            i = _at
            break
        _at = src.find('"Rendezvous AGREED by all', _at + 1)
    if i < 0:
        bad("explo_planner_node.cpp no longer contains a literal starting "
            "\"Rendezvous AGREED by all — the gate's PHRASE is dead and every "
            "future cell would pass it silently")
    else:
        lit = ""
        j = i
        while True:
            a = src.find('"', j)
            if a < 0:
                break
            b = a + 1
            while b < len(src) and not (src[b] == '"' and src[b - 1] != "\\"):
                b += 1
            chunk = src[a + 1:b]
            lit += chunk
            # Stop at the end of the argument: the next non-space character
            # after the closing quote is a comma (start of the varargs).
            k = b + 1
            while k < len(src) and src[k] in " \t\r\n":
                k += 1
            if k < len(src) and src[k] != '"':
                break
            j = k
        rendered = (lit.replace("%d robots", "3 robots")
                       .replace("cell %d", "cell 64")
                       .replace("interval %.0fs", "interval 168s")
                       .replace("t+%.0fs", "t+130s")
                       .replace("provisional=%d", "provisional=1")
                       .replace("echoes %d/%d", "echoes 2/2"))
        line = "[INFO] [1789602925.471535390] [atlas.explo_planner]: " + rendered
        sys.path.insert(0, HERE)
        import rendezvous_agreement as ga
        m = ga.LINE.search(line)
        # THE PROVISIONAL GROUP IS ASSERTED, NOT MERELY TOLERATED. LINE has to
        # keep that group optional so pre-generation-17 logs still parse, which
        # means a node that stopped printing the flag would go on matching — the
        # gate would quietly lose its ability to tell a tour-informed cell from
        # a meet-at-the-centroid one and say nothing. Requiring the group HERE,
        # against the node's live format string, is what makes that loud.
        if not m:
            bad("the node's commit message renders as %r, which the gate's "
                "LINE regex does not parse. Rewording it silently disables the "
                "gate; update LINE in rendezvous_agreement.py." % rendered)
        elif "%" in rendered:
            # EVERY CONVERSION MUST HAVE BEEN SUBSTITUTED. The replaces above
            # are literal, so a node that changes `%.0f` to `%d` would leave a
            # raw conversion in the rendered string, the regex could still match
            # the parts it cares about, and this case would PASS while testing a
            # message no node ever emits. Checked here rather than assumed,
            # because that is precisely how a calibrator stops calibrating.
            bad("the node's format string has a conversion this calibrator "
                "does not know how to render (%r). Add it to the replace chain "
                "above, or case 1 is checking a message that does not exist."
                % rendered)
        elif (int(m.group("cell")), int(m.group("interval"))) != (64, 168):
            bad("the gate parses the node's message as cell=%s/interval=%s, "
                "not 64/168: %r"
                % (m.group("cell"), m.group("interval"), rendered))
        elif m.group("tmeet") is not None:
            # THE GEN-23 TAIL MUST NOT MATCH THE GEN-19 ONE. Generation 23 put
            # the meeting time back on this line, and the gen-19 form carried the
            # same two integers — so if the node had restored the OLD phrasing
            # ("every Xs from t+Ys") the line would parse on the FIRST
            # alternative, `interval`/`prov`/`echoes` would all come back None,
            # and the parse loop coalesces those, so nothing would look broken
            # while a gen-23 bank silently read as pre-gen-20 binaries.
            #
            # This is why the node appends `from t+Ys` to the interval term
            # instead of reusing the old wording. Pin it: the tail the node is on
            # is the third alternative and no other.
            bad("the node's commit message parsed on the PRE-GENERATION-20 "
                "tail (t_meet=%s). Generation 23 prints the meeting time again, "
                "but as `interval Xs from t+Ys` precisely so that it CANNOT be "
                "confused with the gen-19 `every Xs from t+Ys`. A node on the "
                "old wording makes gen-23 logs indistinguishable from gen-19 "
                "ones; restore the gen-23 phrasing or split the two tails."
                % m.group("tmeet"))
        elif m.group("tmeet_new") != "130":
            # THE MEETING TIME IS ASSERTED, NOT MERELY TOLERATED, for the same
            # reason the provisional flag is below: the group is optional so that
            # banked gen-20..22 logs still parse, which means a node that stopped
            # printing the agreed instant would go on matching and the gate would
            # quietly lose the only evidence in the text logs that two robots
            # armed the same MEETING rather than merely the same cell. On gen 23
            # that instant is what armAppointment attends, so losing it is losing
            # the arm's defining property.
            bad("the node's commit message does not carry the agreed meeting "
                "time on the generation-23 tail (t_meet_new=%r, renders as %r). "
                "Without it the banked logs cannot distinguish a fleet that "
                "agreed on one meeting from N robots that agreed on one place "
                "and went at N different times."
                % (m.group("tmeet_new"), rendered))
        elif m.group("prov") != "1":
            bad("the node's commit message no longer carries the "
                "(provisional=N) flag (renders as %r). The gate's group is "
                "optional for old logs, so it would keep parsing and stop "
                "distinguishing a tour-informed appointment from the centroid "
                "placeholder. Restore the flag or rework the gate." % rendered)
        else:
            ok("the gate's parser matches the node's current commit message, "
               "provisional flag included")

    # --- 2. the happy path --------------------------------------------------
    cases += 1
    d = cell(tmp, "good", commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72),
        "bestla": (1789600012.5, 2, 56, 28, 72)},
        armings={"atlas": [{"cell": 56, "t_wall_sec": 1789600100.0}],
                 "bestla": [{"cell": 56, "t_wall_sec": 1789600100.0}]})
    out = run(d)
    if (verdict_of(out) == "PASS" and "cell 56" in out and "span 2.5s" in out
            and "tour-informed" in out):
        ok("a fully committed cell passes and reports the agreed cell")
    else:
        bad("expected PASS naming the agreed cell, got: %r" % out)

    # --- 3. THE ts4 FAILURE: scheduled, zero commits ------------------------
    cases += 1
    d = cell(tmp, "no_agreement", robots=("atlas", "bestla", "husky", "skadi"),
             proposal_lines={"atlas": [
                 "Rendezvous proposal: nothing derivable from the frozen "
                 "snapshot (no admissible meeting cell). Keeping the standing "
                 "pair."]},
             armings={r: [{"t_wall_sec": 1789600300.0,
                           "refused": "no (cell, interval, t_meet) triple was "
                                      "agreed by the whole team before contact "
                                      "was lost"}]
                      for r in ("atlas", "bestla", "husky", "skadi")})
    out = run(d)
    if verdict_of(out) == "FAIL" and "0/4" in out and "nothing derivable" in out:
        ok("a scheduled cell that never agreed FAILS and says why")
    else:
        bad("expected FAIL 0/4 with the node's reason, got: %r" % out)

    # --- 4. partial commit (the N>=3 narrow-mutual-window shape) ------------
    cases += 1
    d = cell(tmp, "partial", robots=("atlas", "bestla", "husky"),
             commits={"atlas":  (1789600010.0, 3, 64, 168, 201),
                      "bestla": (1789600011.0, 3, 64, 168, 201)})
    out = run(d)
    if verdict_of(out) == "FAIL" and "2/3" in out and "husky" in out:
        ok("a partially committed team FAILS and names the robot that never did")
    else:
        bad("expected FAIL 2/3 naming husky, got: %r" % out)

    # --- 5. split fleet -----------------------------------------------------
    cases += 1
    d = cell(tmp, "split", commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72),
        "bestla": (1789600011.0, 2, 57, 28, 72)})
    out = run(d)
    if verdict_of(out) == "FAIL" and "SPLIT FLEET" in out:
        ok("two different cells FAIL as a split fleet")
    else:
        bad("expected FAIL SPLIT FLEET, got: %r" % out)

    # --- 6. armed without the agreement it already held ---------------------
    cases += 1
    d = cell(tmp, "contradiction", commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72),
        "bestla": (1789600011.0, 2, 56, 28, 72)},
        armings={"atlas": [
            # BEFORE the commit: the accepted residual, must not fire.
            {"t_wall_sec": 1789600005.0,
             "refused": "no (cell, interval, t_meet) triple was agreed by the "
                        "whole team before contact was lost"},
            # A legitimate second arming in one outage, must not fire.
            {"t_wall_sec": 1789600400.0,
             "refused": "the agreed pair has already been used in this outage"},
            # AFTER the commit, for want of the triple: the contradiction.
            {"t_wall_sec": 1789600500.0, "t_rel_sec": 321.0,
             "refused": "no (cell, interval, t_meet) triple was agreed by the "
                        "whole team before contact was lost"}]})
    out = run(d)
    if verdict_of(out) == "FAIL" and "321.0s" in out and "1 arming" in out:
        ok("one post-commit no-agreement arming FAILS; the pre-commit one and the "
           "spent-pair one do not")
    else:
        bad("expected FAIL naming exactly the t_rel 321.0s arming, got: %r" % out)

    # --- 6b. G3's ordering field is missing -> UNRUN, not a silent pass -----
    # THE REGRESSION THIS PINS. G3 read `float(d.get("t_wall_sec", 0.0))` until
    # 2026-09-18, so a row without the field compared 0.0 > <epoch seconds>,
    # which is False for every row that will ever exist. Drop the field from the
    # envelope — one schema edit, one binary generation — and G3 stops finding
    # contradictions and PASSes forever, emitting output IDENTICAL to a
    # genuinely clean cell. Nothing downstream could tell the two apart.
    #
    # The arming below is case 6's contradiction with t_wall_sec deleted and
    # nothing else changed, so the two cases differ by exactly the field under
    # test: case 6 proves the check fires when it can run, this proves it
    # refuses when it cannot. Under the old code THIS CASE PASSED — which is the
    # only reason it is worth its lines.
    cases += 1
    d = cell(tmp, "no_t_wall", commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72),
        "bestla": (1789600011.0, 2, 56, 28, 72)},
        armings={"atlas": [
            {"t_rel_sec": 321.0,
             "refused": "no (cell, interval, t_meet) triple was agreed by the "
                        "whole team before contact was lost"}]})
    out = run(d)
    if verdict_of(out) == "UNRUN" and "t_wall_sec" in out:
        ok("a refused arming with no t_wall_sec is UNRUN naming the field, not "
           "a silent pass over the contradiction check")
    else:
        bad("expected UNRUN naming t_wall_sec, got: %r" % out)

    # --- 7. the unscheduled arms stay silent --------------------------------
    cases += 1
    d = cell(tmp, "pursuit_arm", scheduled=False)
    out = run(d)
    if out == "":
        ok("an unscheduled arm emits nothing and cannot move the verdict")
    else:
        bad("expected no output on rendezvous_schedule=0, got: %r" % out)

    # --- 8. a reworded commit line is UNRUN, never a pass and never a FAIL --
    cases += 1
    d = cell(tmp, "reworded")
    with open(os.path.join(d, "planner_atlas.log"), "w") as fh:
        fh.write("[INFO] [1789600010.000000000] [atlas.explo_planner]: "
                 "Rendezvous AGREED by all 2 robots: somewhere, sometime.\n")
    out = run(d)
    if verdict_of(out) == "UNRUN" and "changed shape" in out:
        ok("a reworded commit line is UNRUN, not a silent pass")
    else:
        bad("expected UNRUN 'changed shape', got: %r" % out)

    # --- 9. a lost planner log is UNRUN, not a pass over the survivors ------
    cases += 1
    d = cell(tmp, "lost_log", robots=("atlas", "bestla", "husky"),
             commits={"atlas":  (1789600010.0, 3, 64, 168, 201),
                      "bestla": (1789600011.0, 3, 64, 168, 201),
                      "husky":  (1789600012.0, 3, 64, 168, 201)})
    os.remove(os.path.join(d, "planner_husky.log"))
    out = run(d)
    if verdict_of(out) == "UNRUN" and "husky" in out:
        ok("a missing planner log is UNRUN rather than a pass over the rest")
    else:
        bad("expected UNRUN naming husky, got: %r" % out)

    # --- 10. THE UPGRADE: provisional first, final second, in ECHO ORDER -----
    # This is the case that made the gate wrong before the flag existed. The
    # proposer can derive its triple before the allocator has produced a tour;
    # the argmin then has exactly one candidate (the centroid of the frozen
    # snapshot) and the result is a placeholder, published provisional=1 and
    # replaceable exactly once. Robots adopt the replacement whenever the echo
    # reaches them, so their FIRST commits legitimately disagree — and keying
    # G2 on first commits, which is what this gate used to do, scores a
    # perfectly healthy run as a SPLIT FLEET and sends the cell back for a REDO
    # that will do the same thing again.
    cases += 1
    d = cell(tmp, "upgrade", robots=("atlas", "bestla", "husky"), commits={
        "atlas":  [(1789600010.0, 3, 9, 240, 60, 1),
                   (1789600031.0, 3, 64, 168, 201, 0)],
        "bestla": [(1789600011.5, 3, 9, 240, 60, 1),
                   (1789600033.0, 3, 64, 168, 201, 0)],
        "husky":  [(1789600012.0, 3, 9, 240, 60, 1),
                   (1789600038.0, 3, 64, 168, 201, 0)]})
    out = run(d)
    if (verdict_of(out) == "PASS" and "cell 64" in out
            and "3 robot(s) upgraded" in out):
        ok("a legitimate provisional->final upgrade PASSES on the converged "
           "cell and reports the upgrade")
    else:
        bad("expected PASS on cell 64 naming 3 upgrades, got: %r" % out)

    # --- 11. converged on the PLACEHOLDER: passes, but says which mechanism --
    # A cell where nobody ever upgraded is a valid rendezvous cell — the team
    # has one place and one time and will drive to it. It is ALSO a cell whose
    # meeting point is the centroid of the spawn snapshot rather than an argmin
    # over the allocator's tours, i.e. a different treatment from the one the
    # arm name implies. It must not FAIL, and it must not pass silently either,
    # because nothing else in the run's artifacts records the difference.
    cases += 1
    d = cell(tmp, "placeholder", commits={
        "atlas":  (1789600010.0, 2, 9, 240, 60, 1),
        "bestla": (1789600011.0, 2, 9, 240, 60, 1)})
    out = run(d)
    if verdict_of(out) == "PASS" and "PLACEHOLDER" in out and "centroid" in out:
        ok("a cell that never left the centroid fallback passes, flagged as a "
           "placeholder rather than the scheduled treatment")
    else:
        bad("expected PASS flagged PLACEHOLDER, got: %r" % out)

    # --- 12. a downgrade, final -> provisional ------------------------------
    cases += 1
    d = cell(tmp, "downgrade", commits={
        "atlas":  [(1789600010.0, 2, 64, 168, 201, 0),
                   (1789600030.0, 2, 64, 168, 201, 1)],
        "bestla": [(1789600011.0, 2, 64, 168, 201, 0),
                   (1789600031.0, 2, 64, 168, 201, 1)]})
    out = run(d)
    if verdict_of(out) == "FAIL" and "ILLEGAL COMMIT SEQUENCE" in out:
        ok("a final commit downgraded back to a provisional one FAILS even "
           "though the fleet agrees")
    else:
        bad("expected FAIL ILLEGAL COMMIT SEQUENCE on the downgrade, got: %r"
            % out)

    # --- 13. three commits: the latch is not a latch -------------------------
    cases += 1
    d = cell(tmp, "thrice", commits={
        "atlas":  [(1789600010.0, 2, 9, 240, 60, 1),
                   (1789600030.0, 2, 64, 168, 201, 0),
                   (1789600050.0, 2, 77, 168, 201, 0)],
        "bestla": [(1789600011.0, 2, 9, 240, 60, 1),
                   (1789600031.0, 2, 64, 168, 201, 0),
                   (1789600051.0, 2, 77, 168, 201, 0)]})
    out = run(d)
    if (verdict_of(out) == "FAIL" and "ILLEGAL COMMIT SEQUENCE" in out
            and "3 times" in out):
        ok("a third commit FAILS: one upgrade is permitted, not a stream")
    else:
        bad("expected FAIL ILLEGAL COMMIT SEQUENCE naming 3 commits, got: %r"
            % out)

    # --- 14. THE RELAXATION MUST STILL CATCH A REAL SPLIT -------------------
    # Case 10 relaxed G2 from "every commit agrees" to "every robot ends in the
    # same place". The thing that relaxation could plausibly hide is a fleet
    # that started together and ended apart — one robot upgraded, another never
    # heard the echo. It is the worst outcome the protocol has (two robots, two
    # different cells, each waiting for the other) and it must still FAIL.
    cases += 1
    d = cell(tmp, "half_upgraded", commits={
        "atlas":  [(1789600010.0, 2, 9, 240, 60, 1),
                   (1789600031.0, 2, 64, 168, 201, 0)],
        "bestla": [(1789600011.0, 2, 9, 240, 60, 1)]})
    out = run(d)
    if verdict_of(out) == "FAIL" and "SPLIT FLEET" in out and "cell 9" in out:
        ok("half a fleet upgrading still FAILS as a split, despite identical "
           "first commits")
    else:
        bad("expected FAIL SPLIT FLEET on the half-upgraded fleet, got: %r"
            % out)

    # --- 15. a binary that cannot report the flag makes the shape check sit out
    # Written with prov=None, i.e. the pre-generation-17 line. Two commits with
    # no flag could be [P, R] or a downgrade and there is no way to tell; the
    # gate must not guess. It reports the convergence it CAN see and says the
    # flag was unreported, rather than treating a missing group as a zero.
    cases += 1
    d = cell(tmp, "pre_gen17", commits={
        "atlas":  [(1789600010.0, 2, 9, 240, 60, None),
                   (1789600030.0, 2, 64, 168, 201, None)],
        "bestla": [(1789600011.0, 2, 9, 240, 60, None),
                   (1789600031.0, 2, 64, 168, 201, None)]})
    out = run(d)
    if (verdict_of(out) == "PASS" and "cell 64" in out
            and "provisional=unreported" in out):
        ok("a pre-generation-17 log passes on convergence and declines to "
           "guess the provisional flag")
    else:
        bad("expected PASS with provisional=unreported, got: %r" % out)

    # --- 16. A HEALED SPLIT IS STILL A SPLIT WHILE IT LASTS -----------------
    # The case G2 structurally cannot see. Both robots end on the SAME triple,
    # both shapes are the legal [P, R], so G1, G2 and the shape check are all
    # satisfied — and between the two adoptions of the winning triple one robot
    # was standing at cell 9 while the other drove to cell 64. 300 s of that is
    # more than a whole appointment interval, i.e. a robot could have waited out
    # an entire rendezvous at the wrong place, and the cell would have scored as
    # a clean treated cell. If this ever goes quiet, the completion-time bias it
    # guards lands in the two arms the campaign exists to compare.
    cases += 1
    d = cell(tmp, "healed_split_wide", commits={
        "atlas":  [(1789600010.0, 2, 9, 240, 60, 1),
                   (1789600030.0, 2, 64, 168, 201, 0)],
        "bestla": [(1789600011.0, 2, 9, 240, 60, 1),
                   (1789600330.0, 2, 64, 168, 201, 0)]})
    out = run(d)
    if (verdict_of(out) == "FAIL" and "SPLIT APPOINTMENT" in out
            and "300.0 s apart" in out):
        ok("a fleet that converged only after 300 s FAILS as a split "
           "appointment, even though every final commit agrees")
    else:
        bad("expected FAIL SPLIT APPOINTMENT at 300.0 s, got: %r" % out)

    # --- 17. ...AND A NARROW ONE IS NOT, BUT IS STILL REPORTED --------------
    # The matched half of case 16, and the one that stops G2b being a gate that
    # fails every healthy upgrade. Same shape, same convergence, 120 s apart:
    # under the bound, so PASS. What it also pins is that `span` and `split` are
    # DIFFERENT numbers on the same cell — span 140.0 s (first commit of
    # anything to last commit of anything) against split 120.0 s (first to last
    # adoption of the winner). If a future edit collapses them into one, this
    # case is what notices; reading span as split overstates the harm on every
    # upgraded cell.
    cases += 1
    d = cell(tmp, "healed_split_narrow", commits={
        "atlas":  [(1789600010.0, 2, 9, 240, 60, 1),
                   (1789600030.0, 2, 64, 168, 201, 0)],
        "bestla": [(1789600011.0, 2, 9, 240, 60, 1),
                   (1789600150.0, 2, 64, 168, 201, 0)]})
    out = run(d)
    if (verdict_of(out) == "PASS" and "split 120.0s" in out
            and "span 140.0s" in out):
        ok("a 120 s split passes under the bound and reports split 120.0s "
           "separately from span 140.0s")
    else:
        bad("expected PASS with span 140.0s and split 120.0s, got: %r" % out)

    # --- 18. A RENAMED MANIFEST KEY MUST NOT READ AS "NOTHING TO CHECK" -----
    # `rendezvous_schedule` absent used to take the same early `return 0` as
    # `rendezvous_schedule=0`, i.e. an unscheduled arm — so renaming the key in
    # the launcher would have turned this gate into a silent pass on every cell
    # of a campaign, printing nothing, failing nothing. Absence is now UNRUN,
    # which the teardown scores SUSPECT. This case is the only thing linking the
    # launcher's spelling of the key to the gate that depends on it.
    cases += 1
    d = cell(tmp, "no_schedule_key", commits={
        "atlas":  (1789600010.0, 2, 9, 240, 60, 0),
        "bestla": (1789600011.0, 2, 9, 240, 60, 0)})
    man_path = os.path.join(d, "run_manifest.txt")
    with open(man_path) as fh:
        kept = [ln for ln in fh if not ln.startswith("rendezvous_schedule=")]
    with open(man_path, "w") as fh:
        fh.writelines(kept)
    out = run(d)
    if verdict_of(out) == "UNRUN" and "no rendezvous_schedule key" in out:
        ok("a manifest missing rendezvous_schedule is UNRUN, not a silent pass")
    else:
        bad("expected UNRUN on the missing manifest key, got: %r" % out)

    # --- 19. A PARTIAL PARSE FAILURE IS UNRUN, NOT A FAIL ON THE NODE -------
    # The gate used to reach UNRUN only when NOTHING parsed. A partial break —
    # this file current for some robots and stale for others, which is exactly
    # what a wording change looks like while a campaign straddles two binaries —
    # fell through to G1, where the unparsed robots were reported as never having
    # committed and the cell FAILed as "SCHEDULED ARM WITH NO AGREEMENT". That is
    # the gate's own blind spot charged to the node, in the INVALID direction, on
    # a cell that may be perfectly healthy; three of them abort the campaign.
    #
    # THE POINT OF THE CASE IS THE WORD "FAIL" NOT APPEARING. A gate that
    # misattributes its own breakage is worse than one that is merely broken,
    # because the FAIL text names a defect in the node and sends the next hour
    # to the wrong file.
    cases += 1
    d = cell(tmp, "partial_parse", robots=("atlas", "bestla"), commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72, 0),
        "bestla": (1789600011.0, 2, 56, 28, 72, 0)})
    with open(os.path.join(d, "planner_bestla.log"), "w") as fh:
        fh.write("[INFO] [1789600011.000000000] [bestla.explo_planner]: "
                 "Rendezvous AGREED by all 2 robots: cell 56, at the usual "
                 "place.\n")
    out = run(d)
    if (verdict_of(out) == "UNRUN" and "changed shape" in out
            and "bestla" in out and "partial break" in out):
        ok("a PARTIAL parse failure is UNRUN naming the unparsed robot, not a "
           "FAIL blaming the node for a silence this gate invented")
    else:
        bad("expected UNRUN 'partial break' naming bestla, got: %r" % out)

    # --- 20. SAME CELL, DIFFERENT SCHEDULE INTEGERS: PASS WITH A NOTE -------
    # The other half of the G2 relaxation, and the reason it is a relaxation
    # rather than a deletion. Two robots holding cell 56 drive to the same place
    # whatever `interval` and `t_meet` say — on gen 20-22 because nothing read
    # those fields, and on gen 23 because the appointment barrier is unbounded,
    # so the robot that picked the earlier occurrence waits at the cell for the
    # others. Failing the cell would spend a healthy run. But a propose/echo path
    # that had started dropping fields would look EXACTLY like this, so the
    # disagreement has to stay visible. PASS, with the numbers on the line.
    cases += 1
    d = cell(tmp, "inert_disagreement", robots=("atlas", "bestla"), commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72, 0),
        "bestla": (1789600011.0, 2, 56, 99, 72, 0)})
    out = run(d)
    if (verdict_of(out) == "PASS" and "cell 56" in out
            and "different (interval, t_meet) pairs" in out
            and "bestla=99s" in out):
        ok("agreement on the cell with disagreement on the schedule integers "
           "PASSES and reports the disagreement instead of failing on it")
    else:
        bad("expected PASS with the schedule-integer NOTE, got: %r" % out)

    # --- 21. A BANKED GENERATION-20..22 CELL STILL PARSES -------------------
    # The gate is re-run over banked campaigns, and the middle of the three
    # printing formats — the one with no meeting time at all — is the one every
    # cell between 2026-09-17 and generation 23 was written in. If the gen-23
    # tail had been added by REPLACING the middle alternative rather than sitting
    # beside it, every one of those cells would re-gate as UNRUN "the commit line
    # changed shape" and a whole bank would stop being checkable. tmeet=None
    # selects that tail in the fixture writer.
    cases += 1
    d = cell(tmp, "gen20_bank", robots=("atlas", "bestla"), commits={
        "atlas":  (1789600010.0, 2, 56, 28, None, 0),
        "bestla": (1789600011.0, 2, 56, 28, None, 0)})
    out = run(d)
    if verdict_of(out) == "PASS" and "cell 56" in out and "UNRUN" not in out:
        ok("a banked generation-20..22 commit line, which carries no meeting "
           "time, still parses and PASSES")
    else:
        bad("expected PASS on the generation-20..22 tail, got: %r" % out)

    # --- 22. GEN 23: SAME CELL, DIFFERENT MEETING TIME, COST REPORTED -------
    # The failure the rendezvous arm is defined against is "N robots agreed on a
    # place and went at N different times". On gen 23 that does not cost the
    # meeting — the barrier is unbounded, so the early robot waits — but it does
    # cost exploration time, and a reader who sees only PASS learns nothing. The
    # spread has to be on the line, in seconds.
    cases += 1
    d = cell(tmp, "gen23_time_split", robots=("atlas", "bestla"), commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72, 0),
        "bestla": (1789600011.0, 2, 56, 28, 156, 0)})
    out = run(d)
    if (verdict_of(out) == "PASS" and "different (interval, t_meet) pairs" in out
            and "t+72s" in out and "t+156s" in out
            and "meeting times span 84s" in out):
        ok("a generation-23 cell where the fleet agreed on the place but not the "
           "occurrence PASSES and prices the disagreement in seconds of waiting")
    else:
        bad("expected PASS naming both meeting times and an 84s span, got: %r"
            % out)

    # --- 23-28. G1b: THE ECHO TERMS, WHICH USED TO BE PARSED AND DROPPED ----
    # LINE has captured `echoes E/P` since generation 20 and nothing compared
    # them until 2026-09-18. On generation 23 the commit rule requires
    # E == P == fleet-1 at EVERY commit, the node says so in a comment beside the
    # log call, and the integers proving it are printed on the line. These six
    # cases are the difference between that being an invariant and it being a
    # decoration.

    # 23. E < P: the early return that guards the commit stopped guarding it.
    cases += 1
    d = cell(tmp, "echo_short", robots=("atlas", "bestla", "curt"), commits={
        "atlas":  (1789600010.0, 3, 56, 28, 72, 0, (1, 2)),
        "bestla": (1789600011.0, 3, 56, 28, 72, 0),
        "curt":   (1789600012.0, 3, 56, 28, 72, 0)})
    out = run(d)
    if (verdict_of(out) == "FAIL" and "COMMIT RULE REGRESSED" in out
            and "echoes 1/2" in out and "atlas" in out):
        ok("a gen-23 commit with fewer echoes than peers FAILS, naming the "
           "robot and the ratio")
    else:
        bad("expected FAIL COMMIT RULE REGRESSED on echoes 1/2, got: %r" % out)

    # 24. P != N-1: the line's own two statements of team size disagree.
    cases += 1
    d = cell(tmp, "echo_peers_wrong", robots=("atlas", "bestla", "curt"),
             commits={
        "atlas":  (1789600010.0, 3, 56, 28, 72, 0, (1, 1)),
        "bestla": (1789600011.0, 3, 56, 28, 72, 0),
        "curt":   (1789600012.0, 3, 56, 28, 72, 0)})
    out = run(d)
    if (verdict_of(out) == "FAIL" and "COMMIT RULE REGRESSED" in out
            and "echoes 1/1" in out):
        ok("a gen-23 commit whose peer count is not fleet-1 FAILS even though "
           "every peer it counted echoed")
    else:
        bad("expected FAIL on peers != fleet-1, got: %r" % out)

    # 25. N != roster: G1 cannot see this. Every robot the MANIFEST names
    # committed, so G1 is satisfied; the node meanwhile says the team is two.
    # Without this check the cell passes while the binary and the campaign
    # disagree about how many robots are in the experiment.
    cases += 1
    d = cell(tmp, "echo_fleet_wrong", robots=("atlas", "bestla", "curt"),
             commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72, 0),
        "bestla": (1789600011.0, 2, 56, 28, 72, 0),
        "curt":   (1789600012.0, 2, 56, 28, 72, 0)})
    out = run(d)
    if (verdict_of(out) == "FAIL" and "COMMIT RULE REGRESSED" in out
            and "3-robot roster" in out):
        ok("a fleet size that disagrees with the manifest roster FAILS, a split "
           "G1 cannot see because every robot it knows about committed")
    else:
        bad("expected FAIL naming the 3-robot roster, got: %r" % out)

    # 26. The gen-23 wording with the echo term removed. The node prints the
    # meeting time and the echoes from ONE format string, so this shape cannot
    # come from the binary — it can only come from a reword, which is precisely
    # the event that would otherwise turn G1b into a permanent silent pass.
    cases += 1
    d = cell(tmp, "echo_omitted", robots=("atlas", "bestla"), commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72, 0, "omit"),
        "bestla": (1789600011.0, 2, 56, 28, 72, 0, "omit")})
    out = run(d)
    if (verdict_of(out) == "UNRUN" and "no echo term" in out
            and "DID NOT RUN" in out):
        ok("a gen-23 line that lost its echo term is UNRUN naming the check "
           "that did not run, not a pass over it")
    else:
        bad("expected UNRUN on the missing echo term, got: %r" % out)

    # 27. The healthy case says so. A PASS that does not distinguish "checked
    # and sound" from "nothing to check" is the shape this whole file exists to
    # prevent, so the count of checked lines is on the line.
    cases += 1
    d = cell(tmp, "echo_ok", robots=("atlas", "bestla"), commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72, 0),
        "bestla": (1789600011.0, 2, 56, 28, 72, 0)})
    out = run(d)
    if (verdict_of(out) == "PASS"
            and "2 commit line(s) carried a full echo round" in out):
        ok("a healthy gen-23 cell PASSES and reports how many commit lines the "
           "echo invariant was actually checked against")
    else:
        bad("expected PASS reporting 2 checked commit lines, got: %r" % out)

    # 28. A gen-20..22 bank has no gen-23 lines, so the invariant is not
    # exercised — and that is NOT the same PASS as case 27. E < P was legal on
    # those binaries (a final triple could commit on first-hand evidence), so
    # the check must sit out rather than fail them, and the PASS must say it sat
    # out or a whole re-gated bank reads as verified when nothing was verified.
    cases += 1
    d = cell(tmp, "echo_pre23", robots=("atlas", "bestla"), commits={
        "atlas":  (1789600010.0, 2, 56, 28, None, 0, (0, 1)),
        "bestla": (1789600011.0, 2, 56, 28, None, 0, (0, 1))})
    out = run(d)
    if (verdict_of(out) == "PASS" and "NO gen-23 commit lines" in out
            and "not exercised" in out):
        ok("a gen-20..22 bank with E<P PASSES — legal on that binary — and the "
           "line says the invariant was not exercised")
    else:
        bad("expected PASS declaring the invariant unexercised, got: %r" % out)

    # --- 29. 0/0 ARMINGS IS SPELLED OUT, NOT PRINTED AS A RATIO -------------
    # G3 passes vacuously when nothing armed, which is correct: armings only
    # happen when the link drops. But "0/0 arming(s) carried the agreed cell"
    # reads like a measurement, and the one thing a reader must not take from it
    # is that the appointment machinery ran and was found sound.
    cases += 1
    d = cell(tmp, "no_armings", robots=("atlas", "bestla"), commits={
        "atlas":  (1789600010.0, 2, 56, 28, 72, 0),
        "bestla": (1789600011.0, 2, 56, 28, 72, 0)})
    out = run(d)
    if (verdict_of(out) == "PASS" and "no appointment was ever armed" in out
            and "0/0" not in out):
        ok("a cell whose team never separated PASSES saying G3 had nothing to "
           "check, rather than printing a 0/0 ratio that reads as evidence")
    else:
        bad("expected PASS saying no appointment was ever armed, got: %r" % out)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

print()
if fails:
    print("%d FAILURE(S) of %d known-answer cases" % (fails, cases))
else:
    print("ALL PASS (%d known-answer cases)" % cases)
sys.exit(1 if fails else 0)
