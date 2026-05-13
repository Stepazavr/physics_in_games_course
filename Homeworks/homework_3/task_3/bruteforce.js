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
