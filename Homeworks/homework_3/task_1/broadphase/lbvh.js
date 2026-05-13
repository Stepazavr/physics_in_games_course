function broadphaseLBVH(bodies) {
  const n = bodies.length;
  if (n < 2) return [];

  let mn = [Infinity, Infinity, Infinity];
  let mx = [-Infinity, -Infinity, -Infinity];
  for (const b of bodies) {
    if (b.aabbMin[0] < mn[0]) mn[0] = b.aabbMin[0];
    if (b.aabbMin[1] < mn[1]) mn[1] = b.aabbMin[1];
    if (b.aabbMin[2] < mn[2]) mn[2] = b.aabbMin[2];
    if (b.aabbMax[0] > mx[0]) mx[0] = b.aabbMax[0];
    if (b.aabbMax[1] > mx[1]) mx[1] = b.aabbMax[1];
    if (b.aabbMax[2] > mx[2]) mx[2] = b.aabbMax[2];
  }
  const ext = [mx[0]-mn[0] || 1, mx[1]-mn[1] || 1, mx[2]-mn[2] || 1];

  const entries = new Array(n);
  for (let i = 0; i < n; i++) {
    const b = bodies[i];
    const cx = ((b.aabbMin[0] + b.aabbMax[0]) * 0.5 - mn[0]) / ext[0];
    const cy = ((b.aabbMin[1] + b.aabbMax[1]) * 0.5 - mn[1]) / ext[1];
    const cz = ((b.aabbMin[2] + b.aabbMax[2]) * 0.5 - mn[2]) / ext[2];
    const xi = Math.max(0, Math.min(1023, Math.floor(cx * 1023)));
    const yi = Math.max(0, Math.min(1023, Math.floor(cy * 1023)));
    const zi = Math.max(0, Math.min(1023, Math.floor(cz * 1023)));
    entries[i] = { code: _morton30(xi, yi, zi), bi: i };
  }
  entries.sort((a, b) => a.code - b.code);

  function buildNode(lo, hi) {
    if (lo === hi) {
      const body = bodies[entries[lo].bi];
      return {
        leaf: true, bi: entries[lo].bi,
        aabbMin: body.aabbMin, aabbMax: body.aabbMax,
      };
    }
    const mid = (lo + hi) >> 1;
    const L = buildNode(lo, mid);
    const R = buildNode(mid + 1, hi);
    return {
      leaf: false, left: L, right: R,
      aabbMin: [
        L.aabbMin[0] < R.aabbMin[0] ? L.aabbMin[0] : R.aabbMin[0],
        L.aabbMin[1] < R.aabbMin[1] ? L.aabbMin[1] : R.aabbMin[1],
        L.aabbMin[2] < R.aabbMin[2] ? L.aabbMin[2] : R.aabbMin[2],
      ],
      aabbMax: [
        L.aabbMax[0] > R.aabbMax[0] ? L.aabbMax[0] : R.aabbMax[0],
        L.aabbMax[1] > R.aabbMax[1] ? L.aabbMax[1] : R.aabbMax[1],
        L.aabbMax[2] > R.aabbMax[2] ? L.aabbMax[2] : R.aabbMax[2],
      ],
    };
  }
  const root = buildNode(0, n - 1);

  const pairs = [];

  function nodesOverlap(A, B) {
    return A.aabbMin[0] <= B.aabbMax[0] && A.aabbMax[0] >= B.aabbMin[0] &&
           A.aabbMin[1] <= B.aabbMax[1] && A.aabbMax[1] >= B.aabbMin[1] &&
           A.aabbMin[2] <= B.aabbMax[2] && A.aabbMax[2] >= B.aabbMin[2];
  }

  function emitPair(i, j) {
    if (i === j) return;
    const a = bodies[i], b = bodies[j];
    if (a.isStatic && b.isStatic) return;
    if (i < j) pairs.push([i, j]);
    else        pairs.push([j, i]);
  }

  function descend(A, B) {
    if (!nodesOverlap(A, B)) return;
    if (A.leaf && B.leaf) { emitPair(A.bi, B.bi); return; }
    if (A.leaf)            { descend(A, B.left); descend(A, B.right); return; }
    if (B.leaf)            { descend(A.left, B); descend(A.right, B); return; }
    descend(A.left,  B.left);
    descend(A.left,  B.right);
    descend(A.right, B.left);
    descend(A.right, B.right);
  }

  function selfPairs(node) {
    if (node.leaf) return;
    selfPairs(node.left);
    selfPairs(node.right);
    descend(node.left, node.right);
  }
  selfPairs(root);

  return pairs;
}

function _morton30(x, y, z) {
  return _part1by2(x) | (_part1by2(y) << 1) | (_part1by2(z) << 2);
}

function _part1by2(v) {
  v &= 0x3ff;
  v = (v | (v << 16)) & 0xff0000ff;
  v = (v | (v << 8))  & 0x0300f00f;
  v = (v | (v << 4))  & 0x030c30c3;
  v = (v | (v << 2))  & 0x09249249;
  return v >>> 0;
}
