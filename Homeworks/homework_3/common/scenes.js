const SCENES = {
  '1A': { part: 1, label: 'в глобальных координатах', mode: '1A' },
  '1B': { part: 1, label: 'без гироскопического слагаемого', mode: '1B' },
  '1C': { part: 1, label: 'с гироскопическим слагаемым в явном выражении', mode: '1C' },
  '1D': { part: 1, label: 'с гироскопическим слагаемым в неявном выражении', mode: '1D' },

  '2A': { part: 2, label: '2A · spring force', kind: 'springForce' },
  '2B': { part: 2, label: '2B · spring soft (Buddha)', kind: 'springSoft' },
  '2C': { part: 2, label: '2C · distance XPBD', kind: 'distXPBD' },
  '2D': { part: 2, label: '2D · distance SI',   kind: 'distSI' },

  '3A': { part: 3, label: '3A · 10 кубиков (Bruteforce)',     kind: 'stack' },
  '3B': { part: 3, label: '3B · 1000 кубиков (SpatialGrid)',   kind: 'pile' },

  '4A': { part: 4, label: '4A · varied sizes', kind: 'varied' },
};

// Palette of 10 bright colors for cubes
const BRIGHT_COLORS = [
  [255, 100, 100],  // Bright red
  [255, 180, 50],   // Bright orange
  [255, 255, 100],  // Bright yellow
  [150, 255, 100],  // Bright lime
  [100, 255, 150],  // Bright cyan-green
  [100, 200, 255],  // Bright light blue
  [100, 150, 255],  // Bright blue
  [200, 100, 255],  // Bright purple
  [255, 100, 200],  // Bright pink
  [255, 150, 100],  // Bright coral
];

function loadScene(id) {
  const def = SCENES[id];
  if (!def) return;

  sim.sceneId   = id;
  sim.part      = def.part;
  sim.bodies    = [];
  sim.springs   = [];
  sim.constraints = [];
  sim.contacts  = [];
  sim.broadphasePairs = 0;
  sim.energyHistory = [];
  sim.L0 = null;
  sim.E0 = null;
  sim.elapsed = 0;
  sim.maxC = 0;

  if (def.part === 1) {
    sim.freeRotMode = def.mode;
    _scenePart1();
  } else if (def.part === 2) {
    sim.part2Kind = def.kind;
    _scenePart2(def.kind);
  } else if (def.part === 3) {
    sim.part3Kind = def.kind;
    if (def.kind === 'stack') _sceneStack();
    else _scenePile();
  } else if (def.part === 4) {
    sim.part4Kind = def.kind;
    _sceneVaried();
  }

  if (def.part === 1) {
    const b = sim.bodies[0];
    sim.L0 = bodyAngularMomentum(b);
    sim.E0 = bodyKineticEnergy(b);
    b._L = sim.L0.slice();
  }
}

function _scenePart1() {
  const b = makeBody({
    x: [0, 0, 0],
    halfExtents: [1.5, 0.5, 0.25],
    m: 2.0,
    color: [255, 150, 150],
  });
  b.w = [0.05, 2.0, 0.05];
  sim.bodies.push(b);

  const floor = makeBody({
    x: [0, -3, 0],
    halfExtents: [8, 0.2, 8],
    static: true, color: [60, 70, 80],
  });
  sim.bodies.push(floor);
  
}

function _scenePart2(kind) {
  if (kind === 'springForce' || kind === 'springSoft') {
    const b = makeBody({
      x: [0, 1.0, 0],
      halfExtents: [1.5, 0.5, 0.25],
      m: 2.0, color: [255, 150, 150],
    });
    b.q = quatFromAxisAngle([0, 0, 1], 0.3);
    b.w = [0.2, 0.1, 1.5];
    sim.bodies.push(b);
    const rLocal = [1.2, 0.3, 0];
    sim.springs.push({
      bodyIdx: 0,
      rLocal,
      pWorld: [0, 4.5, 0],
      restLen: 3.2,
      k: 200, c: 4,
      lambdaAccum: 0,
    });
  } else {
    const A = makeBody({
      x: [-2.0, 2.5, 0], halfExtents: [1.5, 0.5, 0.25],
      m: 2.0, color: [255, 150, 150],
    });
    const B = makeBody({
      x: [2.0, 2.5, 0], halfExtents: [1.5, 0.5, 0.25],
      m: 2.0, color: [150, 180, 255],
    });
    A.w = [0.1, 0.05, 1.8];
    B.w = [-0.1, -0.05, -1.8];
    sim.bodies.push(A, B);

    const rAloc = [0.8, -0.3, 0.1];
    const rBloc = [-0.8, -0.3, -0.1];
    const pA0 = vAdd(A.x, quatRotate(A.q, rAloc));
    const pB0 = vAdd(B.x, quatRotate(B.q, rBloc));
    const restLen = vLen(vSub(pA0, pB0));

    sim.constraints.push({
      a: 0, b: 1,
      rAloc, rBloc,
      restLen,
      lambdaAccum: 0, lambdaPosAccum: 0,
    });
  }
  const floor = makeBody({
    x: [0, -3, 0],
    halfExtents: [8, 0.2, 8],
    static: true, color: [60, 70, 80],
  });
  sim.bodies.push(floor);
  
}

function _sceneStack() {
  const floor = makeBody({
    x: [0, 0, 0],
    halfExtents: [6, 0.20, 6],
    static: true, color: [60, 70, 80],
  });
  sim.bodies.push(floor);
  
  // Invisible walls (marked with invisible flag)
  const wallH = 3;
  const wall1 = makeBody({ x: [ 6.0, wallH, 0], halfExtents: [0.4, wallH, 6], static: true });
  wall1.invisible = true;
  sim.bodies.push(wall1);
  const wall2 = makeBody({ x: [-6.0, wallH, 0], halfExtents: [0.4, wallH, 6], static: true });
  wall2.invisible = true;
  sim.bodies.push(wall2);
  const wall3 = makeBody({ x: [0, wallH,  6.0], halfExtents: [6, wallH, 0.4], static: true });
  wall3.invisible = true;
  sim.bodies.push(wall3);
  const wall4 = makeBody({ x: [0, wallH, -6.4], halfExtents: [6, wallH, 0.4], static: true });
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
  const floor = makeBody({ x: [0, 0, 0], halfExtents: [8, 0.2, 8], static: true });
  sim.bodies.push(floor);
  const wallH = 4;
  const wall1 = makeBody({ x: [ 8.4, wallH, 0], halfExtents: [0.4, wallH, 8], static: true });
  wall1.invisible = true;
  sim.bodies.push(wall1);
  const wall2 = makeBody({ x: [-8.4, wallH, 0], halfExtents: [0.4, wallH, 8], static: true });
  wall2.invisible = true;
  sim.bodies.push(wall2);
  const wall3 = makeBody({ x: [0, wallH,  8.4], halfExtents: [8, wallH, 0.4], static: true });
  wall3.invisible = true;
  sim.bodies.push(wall3);
  const wall4 = makeBody({ x: [0, wallH, -8.4], halfExtents: [8, wallH, 0.4], static: true });
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
      halfExtents: [0.25, 0.25, 0.25],
      m: 1,
      color: BRIGHT_COLORS[colorIdx].slice(),
    });
    b.q = quatFromAxisAngle(vNorm([Math.random()-0.5, Math.random()-0.5, Math.random()-0.5]),
                            Math.random() * Math.PI);
    sim.bodies.push(b);
  }
}

function _sceneVaried() {
  const floor = makeBody({ x: [0, 0, 0], halfExtents: [10, 0.2, 10], static: true });
  sim.bodies.push(floor);
  sim.bodies.push(makeBody({ x: [ 10, 4, 0], halfExtents: [0.4, 4, 10], static: true }));
  sim.bodies.push(makeBody({ x: [-10, 4, 0], halfExtents: [0.4, 4, 10], static: true }));
  sim.bodies.push(makeBody({ x: [0, 4,  10], halfExtents: [10, 4, 0.4], static: true }));
  sim.bodies.push(makeBody({ x: [0, 4, -10], halfExtents: [10, 4, 0.4], static: true }));

  for (let i = 0; i < 400; i++) {
    const s = 0.15 + Math.random() * 0.7;
    const hy = s*(0.5+Math.random());
    const b = makeBody({
      x: [(Math.random()-0.5)*16,
          1.5 + hy + Math.random()*14,
          (Math.random()-0.5)*16],
      halfExtents: [s, hy, s*(0.5+Math.random())],
      m: s*s*s*4,
      color: [80 + Math.random()*160, 80 + Math.random()*160, 80 + Math.random()*160],
    });
    b.q = quatFromAxisAngle(vNorm([Math.random()-0.5, Math.random()-0.5, Math.random()-0.5]),
                            Math.random() * Math.PI);
    sim.bodies.push(b);
  }
}
