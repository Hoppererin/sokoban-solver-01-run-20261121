// sokoban-solver-01: push-optimal Sokoban solver.
//
// Approach: the generator builds every level as a 1-wide hallway with 1-4
// rooms hanging off it (one door each, 3-4 boxes per room). A box pushed out
// of its room can never come back, so the level optimum P* is the sum of the
// rooms' optima. This solver therefore
//   1. splits the level into rooms (articulation points of the floor graph),
//   2. solves each room push-optimally with A* over macro-push states
//      (player position + sorted box positions, cost = pushes), using a
//      minimum-assignment heuristic over precomputed push-distances plus
//      dead-square / 2x2-block / frozen-box pruning,
//   3. stitches the rooms together with plain BFS walks along the hallway.
//
// Determinism: no clock, no randomness, no environment-driven variation.
// Search budgets are counted in node expansions, never in wall time, so two
// runs on the same set produce byte-identical output.
//
// Node 22, erasable TypeScript only, no dependencies.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const IN_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DEBUG = process.env.MAKEFUN_DEBUG === '1';
const dbg = (...args: unknown[]): void => {
  if (DEBUG) console.error('[dbg]', ...args);
};

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ['u', 'd', 'l', 'r'];
const INF = 1e9;

// Node budgets (expansions). Count-based, never time-based.
const PER_ROOM_BUDGET = 800000;
const GLOBAL_BUDGET = 4000000;
let globalUsed = 0;

// ---------------------------------------------------------------- parsing

type Level = {
  w: number;
  h: number;
  n: number;
  wall: Uint8Array;
  goal: Uint8Array;
  boxes: number[];
  player: number;
};

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const n = w * h;
  const wall = new Uint8Array(n).fill(1);
  const goal = new Uint8Array(n);
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, n, wall, goal, boxes, player };
}

// Neighbour of cell i in direction d (0=u,1=d,2=l,3=r), or -1 off-board.
function step(lv: Level, i: number, d: number): number {
  const x = i % lv.w;
  const y = (i / lv.w) | 0;
  const nx = x + DX[d];
  const ny = y + DY[d];
  if (nx < 0 || nx >= lv.w || ny < 0 || ny >= lv.h) return -1;
  return ny * lv.w + nx;
}

// ------------------------------------------------------- room decomposition

type Room = {
  cells: Set<number>; // room floor, door cell included
  door: number; // the articulation cell that opens onto the hallway
  entry: number; // hallway cell the player stands on to enter
  boxes: number[];
  goals: number[];
};

function articulationPoints(lv: Level): Set<number> {
  const disc = new Int32Array(lv.n).fill(-1);
  const low = new Int32Array(lv.n);
  const isArt = new Uint8Array(lv.n);
  let time = 0;
  const dfs = (u: number, parent: number): void => {
    disc[u] = low[u] = ++time;
    let children = 0;
    for (let d = 0; d < 4; d++) {
      const v = step(lv, u, d);
      if (v < 0 || lv.wall[v]) continue;
      if (disc[v] < 0) {
        children++;
        dfs(v, u);
        low[u] = Math.min(low[u], low[v]);
        if (parent !== -1 && low[v] >= disc[u]) isArt[u] = 1;
      } else if (v !== parent) {
        low[u] = Math.min(low[u], disc[v]);
      }
    }
    if (parent === -1 && children > 1) isArt[u] = 1;
  };
  for (let i = 0; i < lv.n; i++) {
    if (!lv.wall[i] && disc[i] < 0) dfs(i, -1);
  }
  const out = new Set<number>();
  for (let i = 0; i < lv.n; i++) if (isArt[i]) out.add(i);
  return out;
}

// BFS reachability from `from`, never stepping on `banned` (-1 = no ban).
function reachableFrom(lv: Level, from: number, banned: number): Uint8Array {
  const seen = new Uint8Array(lv.n);
  if (from < 0 || lv.wall[from]) return seen;
  const q = [from];
  seen[from] = 1;
  for (let qi = 0; qi < q.length; qi++) {
    const u = q[qi];
    for (let d = 0; d < 4; d++) {
      const v = step(lv, u, d);
      if (v < 0 || v === banned || lv.wall[v] || seen[v]) continue;
      seen[v] = 1;
      q.push(v);
    }
  }
  return seen;
}

function sameSet(a: Set<number>, b: Set<number>): boolean {
  if (a.size !== b.size) return false;
  for (const x of a) if (!b.has(x)) return false;
  return true;
}

function findRooms(lv: Level): Room[] | null {
  const arts = [...articulationPoints(lv)].sort((a, b) => a - b);
  const hallRow = (lv.player / lv.w) | 0;
  const cands: Room[] = [];
  for (const a of arts) {
    const seen = new Uint8Array(lv.n);
    for (let s = 0; s < lv.n; s++) {
      if (lv.wall[s] || s === a || seen[s]) continue;
      const comp: number[] = [];
      const q = [s];
      seen[s] = 1;
      for (let qi = 0; qi < q.length; qi++) {
        const u = q[qi];
        comp.push(u);
        for (let d = 0; d < 4; d++) {
          const v = step(lv, u, d);
          if (v < 0 || v === a || lv.wall[v] || seen[v]) continue;
          seen[v] = 1;
          q.push(v);
        }
      }
      if (comp.includes(lv.player)) continue;
      const region = new Set<number>(comp);
      region.add(a);
      const boxes: number[] = [];
      const goals: number[] = [];
      for (const c of region) {
        if (lv.boxes.includes(c)) boxes.push(c);
        if (lv.goal[c]) goals.push(c);
      }
      if (boxes.length !== goals.length) continue;
      if (boxes.length < 1 || boxes.length > 4) continue;
      if (region.size > 40) continue;
      // Entry: a neighbour of the door reachable from the player without
      // passing through the door cell itself.
      const playerSide = reachableFrom(lv, lv.player, a);
      let entry = -1;
      for (let d = 0; d < 4; d++) {
        const v = step(lv, a, d);
        if (v >= 0 && !lv.wall[v] && playerSide[v]) {
          entry = v;
          break;
        }
      }
      if (entry < 0) continue;
      boxes.sort((x, y) => x - y);
      goals.sort((x, y) => x - y);
      dbg(' cand door', a, 'entry', entry, 'cells', region.size, 'boxes', boxes.join(','), 'goals', goals.join(','));
      cands.push({ cells: region, door: a, entry, boxes, goals });
    }
  }
  // One room per box set: prefer a door that is not a hallway cell
  // (the true door sits one row off the hallway), then the largest region
  // (an inner articulation point yields a smaller sub-region of the room;
  // the full room is the largest non-hallway region), then the lower door.
  const byBoxes = new Map<string, Room>();
  for (const c of cands) {
    const k = c.boxes.join(',');
    const prev = byBoxes.get(k);
    if (!prev) {
      byBoxes.set(k, c);
      continue;
    }
    const prevHall = ((prev.door / lv.w) | 0) === hallRow;
    const curHall = ((c.door / lv.w) | 0) === hallRow;
    if (prevHall && !curHall) byBoxes.set(k, c);
    else if (prevHall === curHall) {
      if (c.cells.size > prev.cells.size) byBoxes.set(k, c);
      else if (c.cells.size === prev.cells.size && c.door < prev.door) byBoxes.set(k, c);
    }
  }
  // Drop sub-regions: a room's carved floor can have inner bottlenecks
  // that split it into parts with matching boxes/goals of their own. Those
  // parts are strict subsets of the full room's box set, so keep only the
  // maximal box sets.
  const grouped = [...byBoxes.values()];
  const rooms = grouped.filter(
    (c) => !grouped.some((o) => o !== c && c.boxes.length < o.boxes.length && c.boxes.every((b) => o.boxes.includes(b))),
  );
  dbg(' grouped rooms:', rooms.map((r) => `door=${r.door} boxes=${r.boxes.join(',')}`).join(' | '));
  // Verify: disjoint regions, every box and every goal covered exactly once.
  const seenBox = new Set<number>();
  const seenGoal = new Set<number>();
  for (const r of rooms) {
    for (const b of r.boxes) {
      if (seenBox.has(b)) {
        dbg(' findRooms FAIL: box', b, 'in two rooms');
        return null;
      }
      seenBox.add(b);
    }
    for (const g of r.goals) {
      if (seenGoal.has(g)) {
        dbg(' findRooms FAIL: goal', g, 'in two rooms');
        return null;
      }
      seenGoal.add(g);
    }
  }
  if (seenBox.size !== lv.boxes.length) {
    dbg(' findRooms FAIL: boxes covered', seenBox.size, 'of', lv.boxes.length);
    return null;
  }
  let goalCount = 0;
  for (let i = 0; i < lv.n; i++) if (lv.goal[i]) goalCount++;
  if (seenGoal.size !== goalCount) {
    dbg(' findRooms FAIL: goals covered', seenGoal.size, 'of', goalCount);
    return null;
  }
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

// ------------------------------------------------------------- BFS utilities

// BFS from `from` over walkFloor cells avoiding walls and boxes.
// Returns parent array (prev cell, -2 = start, -1 = unreached) and reach set.
function bfsReach(
  lv: Level,
  from: number,
  boxAt: Uint8Array,
  walkFloor: Uint8Array,
): { parent: Int32Array; reach: Uint8Array } {
  const parent = new Int32Array(lv.n).fill(-1);
  const reach = new Uint8Array(lv.n);
  if (from < 0 || lv.wall[from] || boxAt[from]) return { parent, reach };
  const q = [from];
  parent[from] = -2;
  reach[from] = 1;
  for (let qi = 0; qi < q.length; qi++) {
    const u = q[qi];
    for (let d = 0; d < 4; d++) {
      const v = step(lv, u, d);
      if (v < 0 || lv.wall[v] || !walkFloor[v] || boxAt[v] || reach[v]) continue;
      reach[v] = 1;
      parent[v] = u;
      q.push(v);
    }
  }
  return { parent, reach };
}

// Walk-only path from `from` to `to` over any floor cell, avoiding boxes.
// Returns the move string, or null when unreachable.
function walkPath(lv: Level, from: number, to: number, boxes: number[]): string | null {
  const boxAt = new Uint8Array(lv.n);
  for (const b of boxes) boxAt[b] = 1;
  const walkFloor = new Uint8Array(lv.n);
  for (let i = 0; i < lv.n; i++) walkFloor[i] = lv.wall[i] ? 0 : 1;
  const { parent, reach } = bfsReach(lv, from, boxAt, walkFloor);
  if (!reach[to]) return null;
  const moves: string[] = [];
  let cur = to;
  while (parent[cur] !== -2) {
    const prev = parent[cur];
    const d = cur - prev;
    if (d === -lv.w) moves.push('u');
    else if (d === lv.w) moves.push('d');
    else if (d === -1) moves.push('l');
    else moves.push('r');
    cur = prev;
  }
  moves.reverse();
  return moves.join('');
}

// Reconstruct the walk from `from` to `to` using a BFS parent array.
function pathFromParent(lv: Level, parent: Int32Array, from: number, to: number): string {
  const moves: string[] = [];
  let cur = to;
  while (cur !== from) {
    const prev = parent[cur];
    const d = cur - prev;
    if (d === -lv.w) moves.push('u');
    else if (d === lv.w) moves.push('d');
    else if (d === -1) moves.push('l');
    else moves.push('r');
    cur = prev;
  }
  moves.reverse();
  return moves.join('');
}

// ----------------------------------------------------------------- room A*

type RoomCtx = {
  lv: Level;
  boxFloor: Uint8Array; // cells a box may occupy
  walkFloor: Uint8Array; // cells the player may walk (boxFloor + entry)
  goals: number[];
  dists: Int32Array[]; // dists[gIdx][cell] = min pushes, single box, no other boxes
  live: Uint8Array; // cell can reach some goal by pushes (ignoring other boxes)
  entry: number;
  needExit: boolean;
  budget: number; // expansions left for this call (shared global counter)
};

// Minimum pushes to move one box from cell c to goal g, ignoring other boxes.
function pushDistances(ctx: RoomCtx): void {
  const { lv, boxFloor, walkFloor, goals } = ctx;
  for (const g of goals) {
    const dist = new Int32Array(lv.n).fill(INF);
    dist[g] = 0;
    const q = [g];
    for (let qi = 0; qi < q.length; qi++) {
      const y = q[qi]; // current box position
      for (let d = 0; d < 4; d++) {
        const od = d ^ 1;
        const prev = step(lv, y, od); // box was here, pushed along d to y
        if (prev < 0 || !boxFloor[prev] || dist[prev] < INF) continue;
        const pc = step(lv, prev, od); // player stood here
        if (pc < 0 || !walkFloor[pc]) continue;
        dist[prev] = dist[y] + 1;
        q.push(prev);
      }
    }
    ctx.dists.push(dist);
  }
  for (let c = 0; c < lv.n; c++) {
    if (!boxFloor[c]) continue;
    for (const dist of ctx.dists) {
      if (dist[c] < INF) {
        ctx.live[c] = 1;
        break;
      }
    }
  }
}

// Minimum-assignment heuristic: min over goal matchings of summed
// single-box push distances. Admissible: every box needs at least its
// distance in pushes, and pushes move one box at a time.
function heuristic(ctx: RoomCtx, boxes: number[]): number {
  const n = boxes.length;
  const used = new Uint8Array(n);
  let best = INF;
  const rec = (i: number, cost: number): void => {
    if (cost >= best) return;
    if (i === n) {
      best = cost;
      return;
    }
    for (let j = 0; j < n; j++) {
      if (used[j]) continue;
      const c = ctx.dists[j][boxes[i]];
      if (c >= INF) continue;
      used[j] = 1;
      rec(i + 1, cost + c);
      used[j] = 0;
    }
  };
  rec(0, 0);
  return best;
}

function keyOf(p: number, boxes: number[]): number {
  let k = p;
  for (let i = boxes.length - 1; i >= 0; i--) k = k * 256 + boxes[i];
  return k;
}

function solved(ctx: RoomCtx, boxes: number[]): boolean {
  for (let i = 0; i < boxes.length; i++) {
    if (boxes[i] !== ctx.goals[i]) return false;
  }
  return true;
}

// 2x2 block deadlock: four cells all wall-or-box, at least one box, no goal.
function has2x2Deadlock(ctx: RoomCtx, boxAt: Uint8Array, cx: number, cy: number): boolean {
  const { lv } = ctx;
  for (let oy = -1; oy <= 0; oy++) {
    for (let ox = -1; ox <= 0; ox++) {
      const x0 = cx + ox;
      const y0 = cy + oy;
      let boxes = 0;
      let allBlocked = true;
      let hasGoal = false;
      for (let dy = 0; dy < 2; dy++) {
        for (let dx = 0; dx < 2; dx++) {
          const x = x0 + dx;
          const y = y0 + dy;
          let cell = -1;
          let isWall = true;
          let isBox = false;
          let isGoal = false;
          if (x >= 0 && x < lv.w && y >= 0 && y < lv.h) {
            cell = y * lv.w + x;
            isWall = lv.wall[cell] === 1;
            isBox = boxAt[cell] === 1;
            isGoal = lv.goal[cell] === 1;
          }
          if (!isWall && !isBox) allBlocked = false;
          if (isBox) boxes++;
          if (isGoal) hasGoal = true;
        }
      }
      if (allBlocked && boxes > 0 && !hasGoal) return true;
    }
  }
  return false;
}

// A box off its goal that cannot be pushed in any direction because of
// WALLS alone is frozen forever (walls never move). Boxes blocking a push
// are temporary: another box can move away first, so they must NOT count
// as blockers here -- treating them as such wrongly prunes solvable states.
function hasFrozenBox(ctx: RoomCtx, boxes: number[]): boolean {
  const { lv } = ctx;
  for (const b of boxes) {
    if (lv.goal[b]) continue;
    let movable = false;
    for (let d = 0; d < 4; d++) {
      const dest = step(lv, b, d);
      const behind = step(lv, b, d ^ 1);
      if (dest < 0 || behind < 0) continue;
      if (!ctx.boxFloor[dest]) continue;
      if (!ctx.walkFloor[behind]) continue;
      movable = true;
      break;
    }
    if (!movable) return true;
  }
  return false;
}

type HeapNode = {
  f: number;
  h: number;
  c: number;
  g: number;
  p: number;
  boxes: number[];
};

function heapPush(heap: HeapNode[], node: HeapNode): void {
  heap.push(node);
  let i = heap.length - 1;
  while (i > 0) {
    const par = (i - 1) >> 1;
    if (cmpNode(heap[i], heap[par]) >= 0) break;
    const t = heap[i];
    heap[i] = heap[par];
    heap[par] = t;
    i = par;
  }
}

function cmpNode(a: HeapNode, b: HeapNode): number {
  if (a.f !== b.f) return a.f - b.f;
  if (a.h !== b.h) return a.h - b.h;
  return a.c - b.c;
}

function heapPop(heap: HeapNode[]): HeapNode {
  const top = heap[0];
  const last = heap.pop()!;
  if (heap.length > 0) {
    heap[0] = last;
    let i = 0;
    for (;;) {
      const l = i * 2 + 1;
      const r = l + 1;
      let m = i;
      if (l < heap.length && cmpNode(heap[l], heap[m]) < 0) m = l;
      if (r < heap.length && cmpNode(heap[r], heap[m]) < 0) m = r;
      if (m === i) break;
      const t = heap[i];
      heap[i] = heap[m];
      heap[m] = t;
      i = m;
    }
  }
  return top;
}

type SolveResult = {
  moves: string;
  pushes: number;
  expansions: number;
};

// Both variants of a region's solve: `exitable` ends with the player able
// to walk back out to the entry (needed for every room but the last), and
// `trapped` is the unconstrained push-optimum (usable only for the last
// room, where the player never has to leave). When the exitable optimum
// pops first it is also the unconstrained optimum, so both point at it.
// A* over macro-push states. Returns null when the budget runs out or the
// region is unsolvable.
type RoomSolutions = { exitable: SolveResult | null; trapped: SolveResult | null };

function solveRegion(ctx: RoomCtx, startBoxes: number[], startPlayer: number): RoomSolutions | null {
  const { lv } = ctx;
  // Dead start: a box that can never reach any goal.
  for (const b of startBoxes) if (!ctx.live[b]) return null;

  const heap: HeapNode[] = [];
  const bestG = new Map<number, number>();
  // key -> {parentKey, box cell before the push, direction}
  const cameFrom = new Map<number, { pk: number; box: number; dir: number }>();
  let counter = 0;
  let expansions = 0;
  let trapped: { key: number; g: number } | null = null;

  const h0 = heuristic(ctx, startBoxes);
  if (h0 >= INF) return null;
  const startKey = keyOf(startPlayer, startBoxes);
  bestG.set(startKey, 0);
  heapPush(heap, { f: h0, h: h0, c: counter++, g: 0, p: startPlayer, boxes: [...startBoxes] });

  while (heap.length > 0) {
    if (expansions >= ctx.budget || globalUsed >= GLOBAL_BUDGET) return null;
    const node = heapPop(heap);
    const k = keyOf(node.p, node.boxes);
    if (bestG.get(k) !== node.g) continue;
    expansions++;
    globalUsed++;

    if (solved(ctx, node.boxes)) {
      // Goal: all boxes on goals. A* pops goal states in increasing push
      // order, so the first goal of each kind is that kind's optimum.
      const boxAt = new Uint8Array(lv.n);
      for (const b of node.boxes) boxAt[b] = 1;
      const { reach } = bfsReach(lv, node.p, boxAt, ctx.walkFloor);
      const exitable = !ctx.needExit || reach[ctx.entry];
      if (exitable) {
        const sol = finishFromKey(ctx, k, cameFrom, startBoxes, startPlayer, expansions);
        if (!sol) return null;
        // If no trapped optimum popped earlier, this exitable optimum IS
        // the unconstrained optimum as well (it popped first).
        const trappedSol = trapped
          ? finishFromKey(ctx, trapped.key, cameFrom, startBoxes, startPlayer, expansions)
          : sol;
        if (!trappedSol) return null;
        return { exitable: sol, trapped: trappedSol };
      }
      if (!trapped) trapped = { key: k, g: node.g };
      continue;
    }

    const boxAt = new Uint8Array(lv.n);
    for (const b of node.boxes) boxAt[b] = 1;
    const { parent, reach } = bfsReach(lv, node.p, boxAt, ctx.walkFloor);

    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      for (let d = 0; d < 4; d++) {
        const dest = step(lv, b, d);
        const behind = step(lv, b, d ^ 1);
        if (dest < 0 || behind < 0) continue;
        if (!ctx.boxFloor[dest] || boxAt[dest]) continue;
        if (!ctx.walkFloor[behind] || boxAt[behind]) continue;
        if (!reach[behind]) continue;
        const nextBoxes = [...node.boxes];
        nextBoxes[bi] = dest;
        nextBoxes.sort((x, y) => x - y);
        const g2 = node.g + 1;
        const nk = keyOf(b, nextBoxes);
        const prevG = bestG.get(nk);
        if (prevG !== undefined && prevG <= g2) continue;
        // Deadlock pruning on the new state.
        let dead = false;
        for (const nb of nextBoxes) {
          if (!ctx.live[nb]) {
            dead = true;
            break;
          }
        }
        if (dead) continue;
        const nBoxAt = new Uint8Array(lv.n);
        for (const nb of nextBoxes) nBoxAt[nb] = 1;
        const dx = dest % lv.w;
        const dy = (dest / lv.w) | 0;
        if (has2x2Deadlock(ctx, nBoxAt, dx, dy)) continue;
        if (hasFrozenBox(ctx, nextBoxes)) continue;
        const h2 = heuristic(ctx, nextBoxes);
        if (h2 >= INF) continue;
        bestG.set(nk, g2);
        cameFrom.set(nk, { pk: k, box: b, dir: d });
        heapPush(heap, { f: g2 + h2, h: h2, c: counter++, g: g2, p: b, boxes: nextBoxes });
      }
    }
  }
  if (trapped) {
    // No exitable solution exists: this room can only ever be solved last.
    const trappedSol = finishFromKey(ctx, trapped.key, cameFrom, startBoxes, startPlayer, expansions);
    if (!trappedSol) return null;
    return { exitable: null, trapped: trappedSol };
  }
  return null;
}

// Replay the parent chain from the start state, rebuilding the move string
// (walk BFS per macro push, then the push itself), plus the exit walk.
function finishFromKey(
  ctx: RoomCtx,
  goalKey: number,
  cameFrom: Map<number, { pk: number; box: number; dir: number }>,
  startBoxes: number[],
  startPlayer: number,
  expansions: number,
): SolveResult | null {
  const { lv } = ctx;
  // Collect macro steps from goal back to start.
  const steps: { box: number; dir: number }[] = [];
  let cur = goalKey;
  while (cameFrom.has(cur)) {
    const s = cameFrom.get(cur)!;
    steps.push({ box: s.box, dir: s.dir });
    cur = s.pk;
  }
  steps.reverse();
  let moves = '';
  let player = startPlayer;
  let boxes = [...startBoxes];
  let pushes = 0;
  for (const s of steps) {
    const boxAt = new Uint8Array(lv.n);
    for (const b of boxes) boxAt[b] = 1;
    const behind = step(lv, s.box, s.dir ^ 1);
    const { parent, reach } = bfsReach(lv, player, boxAt, ctx.walkFloor);
    if (!reach[behind]) return null;
    moves += pathFromParent(lv, parent, player, behind);
    moves += CH[s.dir];
    pushes++;
    boxes = boxes.map((b) => (b === s.box ? step(lv, s.box, s.dir) : b));
    boxes.sort((a, b) => a - b);
    player = s.box;
  }
  if (ctx.needExit) {
    const boxAt = new Uint8Array(lv.n);
    for (const b of boxes) boxAt[b] = 1;
    const { parent, reach } = bfsReach(lv, player, boxAt, ctx.walkFloor);
    if (reach[ctx.entry]) moves += pathFromParent(lv, parent, player, ctx.entry);
  }
  return { moves, pushes, expansions };
}

function makeCtx(
  lv: Level,
  boxFloor: Uint8Array,
  walkFloor: Uint8Array,
  goals: number[],
  entry: number,
  needExit: boolean,
): RoomCtx {
  const ctx: RoomCtx = {
    lv,
    boxFloor,
    walkFloor,
    goals: [...goals].sort((a, b) => a - b),
    dists: [],
    live: new Uint8Array(lv.n),
    entry,
    needExit,
    budget: PER_ROOM_BUDGET,
  };
  pushDistances(ctx);
  return ctx;
}

function solveRoom(lv: Level, room: Room): RoomSolutions | null {
  const boxFloor = new Uint8Array(lv.n);
  for (const c of room.cells) boxFloor[c] = 1;
  const walkFloor = new Uint8Array(lv.n);
  for (const c of room.cells) walkFloor[c] = 1;
  walkFloor[room.entry] = 1;
  const ctx = makeCtx(lv, boxFloor, walkFloor, room.goals, room.entry, true);
  return solveRegion(ctx, room.boxes, room.entry);
}

// ------------------------------------------------------------------- level

function solveLevel(lv: Level): string {
  const rooms = findRooms(lv);
  dbg('rooms:', rooms ? rooms.length : 'null');
  if (rooms) {
    for (const r of rooms) {
      dbg(' room door', r.door, 'entry', r.entry, 'cells', r.cells.size, 'boxes', r.boxes.join(','), 'goals', r.goals.join(','));
    }
  }
  if (!rooms) {
    // Fallback: treat the whole level as one region.
    const boxFloor = new Uint8Array(lv.n);
    const walkFloor = new Uint8Array(lv.n);
    for (let i = 0; i < lv.n; i++) {
      if (!lv.wall[i]) {
        boxFloor[i] = 1;
        walkFloor[i] = 1;
      }
    }
    const goals: number[] = [];
    for (let i = 0; i < lv.n; i++) if (lv.goal[i]) goals.push(i);
    const ctx = makeCtx(lv, boxFloor, walkFloor, goals, lv.player, false);
    const res = solveRegion(ctx, lv.boxes, lv.player);
    return res && res.exitable ? res.exitable.moves : '';
  }

  // Solve every room both ways, then put the room that loses the most
  // pushes to the exit requirement last (it alone may use its trapped,
  // unconstrained optimum; every other room must end exitable).
  const solved = rooms.map((room) => ({ room, sol: solveRoom(lv, room) }));
  for (const s of solved) {
    dbg(
      ' room door', s.room.door, '->',
      s.sol
        ? `exitable=${s.sol.exitable ? s.sol.exitable.pushes : 'none'} trapped=${s.sol.trapped ? s.sol.trapped.pushes : 'none'} exp=${s.sol.exitable ? s.sol.exitable.expansions : s.sol.trapped ? s.sol.trapped.expansions : 0}`
        : 'NULL',
    );
    if (!s.sol || !s.sol.trapped) return '';
  }
  const noExit = solved.filter((s) => !s.sol!.exitable);
  if (noExit.length > 1) return ''; // two rooms the player can never leave: unsolvable
  let lastIdx = 0;
  let lastGap = -1;
  solved.forEach((s, i) => {
    const gap = s.sol!.exitable ? s.sol!.exitable.pushes - s.sol!.trapped!.pushes : Infinity;
    if (gap > lastGap) {
      lastGap = gap;
      lastIdx = i;
    }
  });
  const order = solved.map((_, i) => i).filter((i) => i !== lastIdx);
  order.push(lastIdx);

  let moves = '';
  let player = lv.player;
  let boxes = [...lv.boxes];
  for (const idx of order) {
    const { room, sol } = solved[idx];
    const isLast = idx === lastIdx;
    const walk = walkPath(lv, player, room.entry, boxes);
    if (walk === null) {
      dbg('walk to entry', room.entry, 'FAILED from', player);
      return '';
    }
    moves += walk;
    player = room.entry;
    const use = isLast ? sol!.trapped! : sol!.exitable!;
    moves += use.moves;
    boxes = boxes.filter((b) => !room.boxes.includes(b)).concat(room.goals);
    boxes.sort((a, b) => a - b);
    player = room.entry;
  }
  return moves;
}

// --------------------------------------------------------------------- main

for (const id of readdirSync(IN_DIR).sort()) {
  const level = parseLevel(readFileSync(join(IN_DIR, id, 'level.txt'), 'utf8'));
  const moves = solveLevel(level);
  mkdirSync(join(OUT_DIR, id), { recursive: true });
  writeFileSync(join(OUT_DIR, id, 'solution.txt'), `${moves}\n`);
}
