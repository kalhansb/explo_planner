# CMakeLists.txt — design notes and history

The long comments of `CMakeLists.txt`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 1

## Top level

### msgs-new-types-not-fields

**New message types, not RobotIntent fields** — attached to `"msg/CellState.msg"` (line 12)

```text
The shared coarse world model. New TYPES rather than new fields on
RobotIntent, deliberately: see the compatibility note at the top of
TeamWorld.msg — on Humble a field added to RobotIntent makes a mixed-build
producer permanently invisible to an updated consumer, silently, for
deconfliction traffic as well. An unmatched type fails loudly instead.
```
