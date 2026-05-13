function broadphaseBruteforce(bodies) {
  const pairs = [];
  const n = bodies.length;
  for (let i = 0; i < n; i++) {
    const a = bodies[i];
    for (let j = i + 1; j < n; j++) {
      const b = bodies[j];
      if (a.isStatic && b.isStatic) continue;
      if (aabbOverlap(a, b)) pairs.push([i, j]);
    }
  }
  return pairs;
}

function aabbOverlap(a, b) {
  return (
    a.aabbMin[0] <= b.aabbMax[0] && a.aabbMax[0] >= b.aabbMin[0] &&
    a.aabbMin[1] <= b.aabbMax[1] && a.aabbMax[1] >= b.aabbMin[1] &&
    a.aabbMin[2] <= b.aabbMax[2] && a.aabbMax[2] >= b.aabbMin[2]
  );
}

function makeSpatialGrid(cellSize) {
  return {
    cellSize,
    table: new Map(),
  };
}

function _hashKey(i, j, k) {
  return (((i * 73856093) ^ (j * 19349663) ^ (k * 83492791)) >>> 0);
}

function broadphaseSpatialGrid(bodies, cellSize) {
  const cs = cellSize;
  const inv = 1 / cs;
  const table = new Map();

  for (let bi = 0; bi < bodies.length; bi++) {
    const b = bodies[bi];
    const i0 = Math.floor(b.aabbMin[0] * inv);
    const j0 = Math.floor(b.aabbMin[1] * inv);
    const k0 = Math.floor(b.aabbMin[2] * inv);
    const i1 = Math.floor(b.aabbMax[0] * inv);
    const j1 = Math.floor(b.aabbMax[1] * inv);
    const k1 = Math.floor(b.aabbMax[2] * inv);
    for (let k = k0; k <= k1; k++)
      for (let j = j0; j <= j1; j++)
        for (let i = i0; i <= i1; i++) {
          const key = _hashKey(i, j, k);
          let arr = table.get(key);
          if (!arr) { arr = []; table.set(key, arr); }
          arr.push(bi);
        }
  }

  const seen = new Set();
  const pairs = [];
  for (const arr of table.values()) {
    const m = arr.length;
    if (m < 2) continue;
    for (let a = 0; a < m; a++)
      for (let b = a + 1; b < m; b++) {
        const ia = arr[a], ib = arr[b];
        const lo = Math.min(ia, ib), hi = Math.max(ia, ib);
        const k = lo * 1000003 + hi;
        if (seen.has(k)) continue;
        seen.add(k);
        const A = bodies[ia], B = bodies[ib];
        if (A.isStatic && B.isStatic) continue;
        if (aabbOverlap(A, B)) pairs.push([lo, hi]);
      }
  }
  return pairs;
}
