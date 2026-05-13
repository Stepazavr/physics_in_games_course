function createSAPStructure() {
  return {
    axes: [[], [], []],
    pairs: new Map(),
  };
}

function testAABBIntersection(a, b) {
  return (
    a.aabbMin[0] <= b.aabbMax[0] && a.aabbMax[0] >= b.aabbMin[0] &&
    a.aabbMin[1] <= b.aabbMax[1] && a.aabbMax[1] >= b.aabbMin[1] &&
    a.aabbMin[2] <= b.aabbMax[2] && a.aabbMax[2] >= b.aabbMin[2]
  );
}

function initializeSAPEvents(sap, bodies) {
  for (let a = 0; a < 3; a++) {
    sap.axes[a].length = 0;
    for (let i = 0; i < bodies.length; i++) {
      const b = bodies[i];
      sap.axes[a].push({ pos: b.aabbMin[a], isMax: false, figure: i });
      sap.axes[a].push({ pos: b.aabbMax[a], isMax: true,  figure: i });
    }
    sap.axes[a].sort((x, y) => x.pos - y.pos);
  }
}

function updateSAPEventsByInsertionSort(arr, bodies, axis) {
  for (const e of arr) {
    e.pos = e.isMax ? bodies[e.figure].aabbMax[axis] : bodies[e.figure].aabbMin[axis];
  }
  for (let i = 1; i < arr.length; i++) {
    const cur = arr[i];
    let j = i - 1;
    while (j >= 0 && arr[j].pos > cur.pos) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = cur;
  }
}

function findPotentialPairsSAP(bodies, sap) {
  if (sap.axes[0].length !== bodies.length * 2) {
    initializeSAPEvents(sap, bodies);
  } else {
    for (let a = 0; a < 3; a++) updateSAPEventsByInsertionSort(sap.axes[a], bodies, a);
  }

  const overlaps = [new Set(), new Set(), new Set()];
  for (let a = 0; a < 3; a++) {
    const active = new Set();
    for (const e of sap.axes[a]) {
      if (e.isMax) {
        active.delete(e.figure);
      } else {
        for (const other of active) {
          const lo = Math.min(other, e.figure), hi = Math.max(other, e.figure);
          overlaps[a].add(lo * 1000003 + hi);
        }
        active.add(e.figure);
      }
    }
  }

  const pairs = [];
  for (const key of overlaps[0]) {
    if (overlaps[1].has(key) && overlaps[2].has(key)) {
      const hi = key % 1000003;
      const lo = (key - hi) / 1000003;
      if (bodies[lo].isStatic && bodies[hi].isStatic) continue;
      pairs.push([lo, hi]);
    }
  }
  return pairs;
}



function findPotentialPairsLBVH(bodies) {
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
    entries[i] = { code: computeMortonCode30(xi, yi, zi), bi: i };
  }
  entries.sort((a, b) => a.code - b.code);

  function buildLBVHNode(lo, hi) {
    if (lo === hi) {
      const figure = bodies[entries[lo].bi];
      return {
        leaf: true, bi: entries[lo].bi,
        aabbMin: figure.aabbMin, aabbMax: figure.aabbMax,
      };
    }
    const mid = (lo + hi) >> 1;
    const L = buildLBVHNode(lo, mid);
    const R = buildLBVHNode(mid + 1, hi);
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
  const root = buildLBVHNode(0, n - 1);

  const pairs = [];

  function doAABBNodesIntersect(A, B) {
    return A.aabbMin[0] <= B.aabbMax[0] && A.aabbMax[0] >= B.aabbMin[0] &&
           A.aabbMin[1] <= B.aabbMax[1] && A.aabbMax[1] >= B.aabbMin[1] &&
           A.aabbMin[2] <= B.aabbMax[2] && A.aabbMax[2] >= B.aabbMin[2];
  }

  function addUniquePair(i, j) {
    if (i === j) return;
    const a = bodies[i], b = bodies[j];
    if (a.isStatic && b.isStatic) return;
    if (i < j) pairs.push([i, j]);
    else        pairs.push([j, i]);
  }

  function traverseNodePair(A, B) {
    if (!doAABBNodesIntersect(A, B)) return;
    if (A.leaf && B.leaf) { addUniquePair(A.bi, B.bi); return; }
    if (A.leaf)            { traverseNodePair(A, B.left); traverseNodePair(A, B.right); return; }
    if (B.leaf)            { traverseNodePair(A.left, B); traverseNodePair(A.right, B); return; }
    traverseNodePair(A.left,  B.left);
    traverseNodePair(A.left,  B.right);
    traverseNodePair(A.right, B.left);
    traverseNodePair(A.right, B.right);
  }

  function collectInternalNodePairs(node) {
    if (node.leaf) return;
    collectInternalNodePairs(node.left);
    collectInternalNodePairs(node.right);
    traverseNodePair(node.left, node.right);
  }
  collectInternalNodePairs(root);

  return pairs;
}

function computeMortonCode30(x, y, z) {
  return expandBitsForMorton(x) | (expandBitsForMorton(y) << 1) | (expandBitsForMorton(z) << 2);
}

function expandBitsForMorton(v) {
  v &= 0x3ff;
  v = (v | (v << 16)) & 0xff0000ff;
  v = (v | (v << 8))  & 0x0300f00f;
  v = (v | (v << 4))  & 0x030c30c3;
  v = (v | (v << 2))  & 0x09249249;
  return v >>> 0;
}
