
function _sceneStack() {
  const floor = makeBody({
    x: [0, 0, 0],
    halfExtents: [6, 0.2, 6],
    static: true, color: [60, 70, 80],
  });
  sim.bodies.push(floor);
  
  // Invisible walls (marked with invisible flag)
  const wallH = 3;
  const wall1 = makeBody({ x: [ 6.2, 0.2 + wallH, 0], halfExtents: [0.2, wallH, 6], static: true });
  wall1.invisible = true;
  sim.bodies.push(wall1);
  const wall2 = makeBody({ x: [-6.2, 0.2 + wallH, 0], halfExtents: [0.2, wallH, 6], static: true });
  wall2.invisible = true;
  sim.bodies.push(wall2);
  const wall3 = makeBody({ x: [0, 0.2 + wallH,  6.2], halfExtents: [6, wallH, 0.2], static: true });
  wall3.invisible = true;
  sim.bodies.push(wall3);
  const wall4 = makeBody({ x: [0, 0.2 + wallH, -6.2], halfExtents: [6, wallH, 0.2], static: true });
  wall4.invisible = true;
  sim.bodies.push(wall4);
  
  for (let i = 0; i < 10; i++) {
    const jx = (Math.random() - 0.5) * 0.02;
    const jz = (Math.random() - 0.5) * 0.02;
    const colorIdx = i % BRIGHT_COLORS.length;
    const b = makeBody({
      x: [jx, 0.9 + i * 2.22, jz],
      halfExtents: [0.6, 0.6, 0.6],
      m: 1, color: BRIGHT_COLORS[colorIdx].slice(),
    });
    b.v = [jx * 100, 0, jz * 100];
    sim.bodies.push(b);
  }
}

function _scenePile() {
  const floor = makeBody({ x: [0, 0, 0], halfExtents: [8, 0.2, 8], static: true, color: [60, 70, 80] });
  sim.bodies.push(floor);
  const wallH = 4;
  const wall1 = makeBody({ x: [ 8.2, 0.2 + wallH, 0], halfExtents: [0.2, wallH, 8], static: true });
  wall1.invisible = true;
  sim.bodies.push(wall1);
  const wall2 = makeBody({ x: [-8.2, 0.2 + wallH, 0], halfExtents: [0.2, wallH, 8], static: true });
  wall2.invisible = true;
  sim.bodies.push(wall2);
  const wall3 = makeBody({ x: [0, 0.2 + wallH,  8.2], halfExtents: [8, wallH, 0.2], static: true });
  wall3.invisible = true;
  sim.bodies.push(wall3);
  const wall4 = makeBody({ x: [0, 0.2 + wallH, -8.2], halfExtents: [8, wallH, 0.2], static: true });
  wall4.invisible = true;
  sim.bodies.push(wall4);

  const COUNT = 1000;
  for (let i = 0; i < COUNT; i++) {
    const x = (Math.random() - 0.5) * 14;
    const z = (Math.random() - 0.5) * 14;
    const y = 1.5 + Math.random() * 18;
    const colorIdx = i % BRIGHT_COLORS.length;
    const b = makeBody({
      x: [x, y, z],
      halfExtents: [0.45, 0.45, 0.45],
      m: 1,
      color: BRIGHT_COLORS[colorIdx].slice(),
    });
    b.q = quatFromAxisAngle(vNorm([Math.random()-0.5, Math.random()-0.5, Math.random()-0.5]),
                            Math.random() * Math.PI);
    sim.bodies.push(b);
  }
}
