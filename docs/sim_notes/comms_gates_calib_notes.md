# comms_gates_calib.py — design notes and history

The long comments of `sim/comms_gates_calib.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [run_overflow](#run_overflow) — 1
- [run_watch](#run_watch) — 1

## run_overflow

### gates-calib-case-substring

**Why each case checks a substring** — attached to `def run_overflow(s):` (line 74)

```text
(name, thunk, expected verdict, substring the message must carry)

The substring is not decoration. A case that only asserts "this refused"
passes when the gate refuses for an unrelated reason, which is how a guard
ends up calibrated against a defect it does not actually detect.
```

## run_watch

### gates-calib-watch-token

**The watch token is load-bearing** — attached to `def run_watch(expect, ever, polls, usable, tripped_in=False):` (line 196)

```text
THE `watch` TOKEN IS WHAT THE TEARDOWN LOOKS FOR. run_explo_sim_rviz.sh reads
comms_gates.txt for a line whose second field is `watch`, and a file without
one is SUSPECT — the guard that stops a silently-dead watcher from banking a
clean cell. That makes the exact token on the `watch` line load-bearing in a
way no other gate's is: PASS clears the run, UNRUN carries the token (so the
teardown still finds its line) but counts an unrunnable and lands SUSPECT.
Both halves are asserted below.
```
