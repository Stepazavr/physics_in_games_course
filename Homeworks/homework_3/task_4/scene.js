
function _sceneVaried() {
  const floor = makeFigure({ x: [0, 0, 0], halfExtents: [8, 0.2, 8], static: true, color: [60, 70, 80] });
  sim.bodies.push(floor);
  
  // Invisible walls (marked with invisible flag)
  const wallH = 4;
  const wall1 = makeFigure({ x: [ 8.2, 0.2 + wallH, 0], halfExtents: [0.2, wallH, 8], static: true });
  wall1.invisible = true;
  sim.bodies.push(wall1);
  const wall2 = makeFigure({ x: [-8.2, 0.2 + wallH, 0], halfExtents: [0.2, wallH, 8], static: true });
  wall2.invisible = true;
  sim.bodies.push(wall2);
  const wall3 = makeFigure({ x: [0, 0.2 + wallH,  8.2], halfExtents: [8, wallH, 0.2], static: true });
  wall3.invisible = true;
  sim.bodies.push(wall3);
  const wall4 = makeFigure({ x: [0, 0.2 + wallH, -8.2], halfExtents: [8, wallH, 0.2], static: true });
  wall4.invisible = true;
  sim.bodies.push(wall4);

  const COUNT = 1000;
  for (let i = 0; i < COUNT; i++) {
    const s = 0.15 + Math.random() * 0.7;
    const hy = s*(0.5+Math.random());
    const x = (Math.random() - 0.5) * 14;
    const z = (Math.random() - 0.5) * 14;
    const y = 1.5 + hy + Math.random() * 18;
    const colorIdx = i % BRIGHT_COLORS.length;
    const b = makeFigure({
      x: [x, y, z],
      halfExtents: [s, hy, s*(0.5+Math.random())],
      m: s*s*s*4,
      color: BRIGHT_COLORS[colorIdx].slice(),
    });
    b.q = quatFromAxisAngle(vNorm([Math.random()-0.5, Math.random()-0.5, Math.random()-0.5]),
                            Math.random() * Math.PI);
    sim.bodies.push(b);
  }
}
