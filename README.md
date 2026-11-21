# Sokoban Solver — push-optimal, per-room

A push-optimal Sokoban solver for the `sokoban-solver-01-run-20261121`
bounty (make.fun). One run answers all 5 instances of a set in ~0.3 s,
well inside the 60 s wall budget, and is fully deterministic.

## How it works

Every generated level is a 1-wide hallway with 1–4 rooms hanging off it
(one door each, 3–4 boxes per room). A box pushed out of its room can never
come back, so the level optimum P\* is the sum of the rooms' optima. The
solver therefore:

1. **Splits the level into rooms** — articulation points of the floor
   graph (Tarjan). Each room is the component cut off by its door, with
   sub-regions from inner bottlenecks discarded.
2. **Solves each room push-optimally** with A\* over macro-push states
   (player position + sorted box positions, cost = pushes), using a
   minimum-assignment heuristic over precomputed single-box push
   distances, plus dead-square / 2×2-block / wall-frozen-box pruning.
   Each room is solved twice over: once requiring the player to still be
   able to walk out to the entry, once unconstrained. The room that loses
   the most pushes to the exit requirement is solved last (it alone may
   use its unconstrained optimum).
3. **Stitches the rooms together** with plain BFS walks along the hallway.

Search budgets are counted in node expansions, never in wall time; there
is no clock, randomness, or environment-driven variation, so two runs on
the same set produce byte-identical output.

## Files

- `main.ts` — the whole solver (Node 22, erasable TypeScript, no deps).
- `REPORT.md` — algorithm design, practice results, determinism check.
- `LICENSE` — MIT.

## Run locally

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Reads `<input>/<id>/level.txt`, writes `<output>/<id>/solution.txt`
(moves in `LURD`, one letter per step).

## Results

Practice set: all 5 levels solved at exactly P\* — **score 1000/1000
(1,000,000)**. Also verified on 8 freshly drawn random sets from the
kit's own generator: 8/8 at P\*, 1000/1000 each. See `REPORT.md`.
