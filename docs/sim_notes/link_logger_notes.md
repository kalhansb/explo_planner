# link_logger.py — design notes and history

The long comments of `sim/link_logger.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 1
- [LinkLogger.__init__](#linklogger__init__) — 1
- [LinkLogger.resolve_stride](#linkloggerresolve_stride) — 2
- [LinkLogger.cb](#linkloggercb) — 1
- [LinkLogger.destroy_node](#linkloggerdestroy_node) — 1

## Module scope

### link-logger-mask-bands

**Mask-rate bands for the sidecar verdict** — attached to `MASK_SUSPECT = 0.10` (line 70)

```text
Mask-rate bands for the sidecar's verdict. Not tuning knobs -- they are the
docstring's own sentence ("if MOST of a trace is masked, the trace is not
evidence of anything") turned into a number a reader can branch on, plus a
lower band for the case that is merely worth knowing about.

A nonzero rate is EXPECTED and is not by itself a problem: every pair is
masked until both its endpoints have published a pose, so a run always opens
with a burst of invalid rows. What the bands separate is that startup burst,
which is a fixed cost and shrinks as a fraction of any real trace, from a
pose pipeline that stayed broken.
```

## LinkLogger.__init__

### link-logger-sidecar-at-startup

**Why the sidecar is written at startup** — attached to `self.write_sidecar()` (line 105)

```text
Written once at startup, before a single message has arrived, so that
on a cell where this node ran the sidecar EXISTS no matter how the
cell ends -- including a SIGKILL at teardown, which reaches neither
destroy_node() nor the periodic write in cb(). That makes the absent
sidecar mean exactly one thing to a reader of a new campaign: this
node never started. The initial contents are rows=0 masked=0 ->
NO_ROWS, which is the correct reading of a cell that died before the
emulator published anything, and it is overwritten from then on.
```

## LinkLogger.resolve_stride

### link-logger-unusable-declared-width

**A declared width this script cannot use** — attached to `why = ("is not one of them"` (line 180)

```text
A declaration that this script cannot use is NOT a reason to
fall back to guessing. It means the publisher is a generation
this script does not know (or its layout disagrees with its
own payload), and the fallback would silently impose a width
the publisher just said it was not using.
`declared` is arbitrary publisher-supplied data and 0 is its
default-constructed value, so the divisibility it fails can
be a division by zero. Report the width, not the remainder.
```

### link-logger-undeclared-width-tie

**Width ties without a declared layout** — attached to `fits = [c for c in (FIELDS, FIELDS_LEGACY) if n % c == 0]` (line 199)

```text
NO DECLARATION, SO DIVISIBILITY IS THE ONLY EVIDENCE -- and it is
not always decisive. Both widths divide any multiple of 90, which
is reachable: 10 pairs (N=5) of 9 columns and 9 pairs of 10 are
the same 90 values. The old loop resolved that tie by preferring
FIELDS, i.e. by assuming the workspace was NOT half-built, which
is the one assumption the header comment above says this function
exists to avoid making. A tie is missing information, not a vote.
```

## LinkLogger.cb

### link-logger-periodic-sidecar-trigger

**What triggers the periodic sidecar write** — attached to `published = self.rows + self.masked` (line 256)

```text
PUBLISHED, not self.rows. Keying the periodic write off the rows that
survived the mask means a trace in which everything is masked -- the
single case this sidecar exists to make visible -- increments nothing,
writes no sidecar, and depends entirely on destroy_node() running.

And a threshold on a running total, not a modulus: one callback adds
a whole message's worth of pairs at once, so `total % 500 == 0` is a
condition the counter can step straight over and never satisfy again.
```

## LinkLogger.destroy_node

### link-logger-shutdown-summary

**The shutdown summary line on stderr** — attached to `print(f"[link_logger] rows={self.rows} masked={self.masked} "` (line 275)

```text
Goes to stderr so the caller can separate it from the CSV path on
stdout. No longer "the only place the mask count is reported" -- the
sidecar is, and this line names it so a reader of the console log can
find the machine-readable copy.
Both mask counts, always, with the total: a rate needs a denominator,
and "which discriminator fired" is how a reader learns whether the
emulator was reporting validity at all.
```
