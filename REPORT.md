# REPORT — Sokoban Solver (sokoban-solver-01-run-20261121)

Entry: push-optimal Sokoban solver, `main.ts` (Node 22, erasable
TypeScript, no dependencies). Built fresh from the kit's RULES and the
practice levels; no code copied from the kit's starter or baseline.

## Design

**Decomposition (the key idea).** Every generated level is a 1-wide
hallway with 1–4 rooms hanging off it (one door each, 3–4 boxes per room),
and a box pushed out of its room can never come back, so the level optimum
P\* is the sum of the rooms' optima. The solver:

1. Splits the level into rooms via articulation points of the floor graph
   (Tarjan). Candidates are grouped by box set, preferring a door that is
   not a hallway cell and then the largest region; candidates whose box
   set is a strict subset of another's are dropped (a carved room's inner
   bottlenecks can split it into 2+2 box/goal sub-parts — those are not
   rooms). The split is verified: disjoint regions covering every box and
   every goal exactly once, else a whole-level fallback A\* runs.
2. Solves each room push-optimally with A\* over macro-push states
   (player position + sorted box positions, cost = pushes):
   - heuristic: minimum-assignment over precomputed single-box
     push-distances (reverse-push BFS from each goal, ignoring other
     boxes) — admissible;
   - pruning: dead squares (cells from which no goal is push-reachable),
     2×2 wall/box blocks without a goal, wall-frozen boxes;
   - deterministic successor order and heap tie-breaks (f, h, insertion
     counter).
   Each room is solved twice over in one search: once requiring the final
   player position to still reach the entry (exitable), once unconstrained
   (trapped). The room that loses the most pushes to the exit requirement
   is solved last and alone may use its unconstrained optimum.
3. Stitches rooms together with plain BFS walks along the hallway
   (0 pushes added).

**Determinism.** No clock, no randomness, no environment-driven variation.
Search budgets are counted in node expansions (800k per room, 4M global),
never in wall time, so two runs on the same set are byte-identical.

## Practice results (kit practice set, P\* from reference/manifest.json)

| level | boxes | rooms (P\*) | pushes | points |
|-------|-------|-------------|--------|--------|
| i001  | 3     | [13]        | 13     | 1000   |
| i002  | 4     | [16]        | 16     | 1000   |
| i003  | 8     | [15, 14]    | 29     | 1000   |
| i004  | 12    | [15, 17, 15]| 47     | 1000   |
| i005  | 16    | [16, 14, 14, 16] | 60 | 1000   |

**Practice score: 1000/1000 (1,000,000).** Whole-set runtime ~0.3 s
(budget 60 s). Largest single-room expansion count observed: 403.

Official check (`grade-local --driver process --worktree`):
`scores {"answered":5,"i001":1000000,"i002":1000000,"i003":1000000,
"i004":1000000,"i005":1000000,"instances":5,"passed":1,"score":1000000}`,
`result_sha256 004eed7636c95dc2d897f4b459c347c7e373743e7a52443dfcb5c374c8d3548b`,
"L1 regrade reproduced byte for byte". (The note that this
result_sha256 cannot equal the server's is only the Node version:
v24.22.1 here vs v26.13.1 on the server; the score is unaffected.)

## Generalisation (unseen sets)

Because the scored set is drawn from a different drand round, the solver
was also tested on 8 freshly drawn random sets from the kit's own
generator (`program/generator/draw.ts`, random seeds): **8/8 sets at
exactly P\*, 1000/1000 each**, ~0.3 s per set. Per-level P\* ranged
12–21 per room (one room at 21 pushes), all solved optimally.

## Determinism check

Two runs on the practice set, `diff -r` over the output directories:
identical (byte-identical). `grade-local` also runs the bot twice and
reports "L1 regrade reproduced byte for byte".

## Bugs found and fixed during development

1. **Frozen-box pruning counted other boxes as permanent blockers.**
   A box frozen only because another box stands in the way can become
   movable again, so pruning such states killed solvable rooms (practice
   i003/i004 returned empty; i005 was 2 pushes over P\*). Fixed to
   wall-only freezing; all practice levels then hit P\* exactly.
2. **Exit requirement vs room optimum.** Requiring every room's solution
   to end exitable can cost pushes; fixed by returning both the exitable
   and the unconstrained optimum per room and solving the room with the
   largest gap last.
3. **Inner-bottleneck sub-regions.** On one random set, a room's carved
   floor split into 2+2 box/goal sub-parts, which the decomposition first
   mistook for rooms (verification failed, level unanswered). Fixed by
   dropping candidates whose box set is a strict subset of another's and
   preferring the largest region per box set.

## How to test

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
# replay with the kit checker, compare pushes to practice/reference/manifest.json
```
