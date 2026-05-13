function makeSAP() {
  return {
    axes: [[], [], []],
    pairs: new Map(),
  };
}

function aabbOverlap(a, b) {
  return (
    a.aabbMin[0] <= b.aabbMax[0] && a.aabbMax[0] >= b.aabbMin[0] &&
    a.aabbMin[1] <= b.aabbMax[1] && a.aabbMax[1] >= b.aabbMin[1] &&
    a.aabbMin[2] <= b.aabbMax[2] && a.aabbMax[2] >= b.aabbMin[2]
  );
}

function _sapInsertEvents(sap, bodies) {
  for (let a = 0; a < 3; a++) {
    sap.axes[a].length = 0;
    for (let i = 0; i < bodies.length; i++) {
      const b = bodies[i];
      sap.axes[a].push({ pos: b.aabbMin[a], isMax: false, body: i });
      sap.axes[a].push({ pos: b.aabbMax[a], isMax: true,  body: i });
    }
    sap.axes[a].sort((x, y) => x.pos - y.pos);
  }
}

function _sapInsertionSort(arr, bodies, axis) {
  for (const e of arr) {
    e.pos = e.isMax ? bodies[e.body].aabbMax[axis] : bodies[e.body].aabbMin[axis];
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

function broadphaseSAP(bodies, sap) {
  if (sap.axes[0].length !== bodies.length * 2) {
    _sapInsertEvents(sap, bodies);
  } else {
    for (let a = 0; a < 3; a++) _sapInsertionSort(sap.axes[a], bodies, a);
  }

  const overlaps = [new Set(), new Set(), new Set()];
  for (let a = 0; a < 3; a++) {
    const active = new Set();
    for (const e of sap.axes[a]) {
      if (e.isMax) {
        active.delete(e.body);
      } else {
        for (const other of active) {
          const lo = Math.min(other, e.body), hi = Math.max(other, e.body);
          overlaps[a].add(lo * 1000003 + hi);
        }
        active.add(e.body);
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
