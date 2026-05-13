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
