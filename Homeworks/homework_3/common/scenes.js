const SCENES = {
  '1A': { part: 1, label: 'в глобальных координатах', mode: '1A' },
  '1B': { part: 1, label: 'без гироскопического слагаемого', mode: '1B' },
  '1C': { part: 1, label: 'с гироскопическим слагаемым в явном выражении', mode: '1C' },
  '1D': { part: 1, label: 'с гироскопическим слагаемым в неявном выражении', mode: '1D' },

  '2A': { part: 2, label: '2A · spring force', kind: 'springForce' },
  '2B': { part: 2, label: '2B · spring soft (Buddha)', kind: 'springSoft' },
  '2C': { part: 2, label: '2C · distance XPBD', kind: 'distXPBD' },
  '2D': { part: 2, label: '2D · distance SI',   kind: 'distSI' },

  '3A': { part: 3, label: '3A · 10 boxes',     kind: 'stack' },
  '3B': { part: 3, label: '3B · 1000 boxes',   kind: 'pile' },

  '4A': { part: 4, label: '4A · varied sizes', kind: 'varied' },
};

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
    halfExtents: [8, 0.5, 8],
    static: true, color: [60, 70, 80],
  });
  sim.bodies.push(floor);
  
}

function _scenePart2(kind) {
  if (kind === 'springForce' || kind === 'springSoft') {
    const b = makeBody({
      x: [0, 1.5, 0],
      halfExtents: [0.5, 0.2, 0.3],
      m: 1.0, color: [120, 180, 240],
    });
    b.q = quatFromAxisAngle([0, 0, 1], 0.4);
    sim.bodies.push(b);
    const rLocal = [0.5, 0.2, 0.3];
    sim.springs.push({
      bodyIdx: 0,
      rLocal,
      pWorld: [0, 4.0, 0],
      restLen: 2.0,
      k: 200, c: 4,
      lambdaAccum: 0,
    });
  } else {
    const A = makeBody({
      x: [-0.9, 2.4, 0], halfExtents: [0.4, 0.3, 0.4],
      m: 1, color: [220, 140, 100],
    });
    const B = makeBody({
      x: [ 0.9, 2.4, 0], halfExtents: [0.4, 0.3, 0.4],
      m: 1, color: [120, 180, 240],
    });
    A.w = [0, 0, 2.5];
    sim.bodies.push(A, B);

    const rAloc = [ 0.4, 0.3, 0];
    const rBloc = [-0.4, 0.3, 0];
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
    x: [0, -1, 0],
    halfExtents: [8, 0.5, 8],
    static: true, color: [60, 70, 80],
  });
  sim.bodies.push(floor);
  
}

function _sceneStack() {
  const floor = makeBody({
    x: [0, 0, 0],
    halfExtents: [6, 0.5, 6],
    static: true, color: [60, 70, 80],
  });
  sim.bodies.push(floor);
  for (let i = 0; i < 10; i++) {
    const jx = (Math.random() - 0.5) * 0.02;
    const jz = (Math.random() - 0.5) * 0.02;
    const b = makeBody({
      x: [jx, 0.9 + i * 0.62, jz],
      halfExtents: [0.4, 0.3, 0.4],
      m: 1, color: [120 + i*8, 160, 220 - i*6],
    });
    sim.bodies.push(b);
  }
}

function _scenePile() {
  const floor = makeBody({ x: [0, 0, 0], halfExtents: [8, 0.5, 8], static: true });
  sim.bodies.push(floor);
  const wallH = 4;
  sim.bodies.push(makeBody({ x: [ 8.4, wallH, 0], halfExtents: [0.4, wallH, 8], static: true }));
  sim.bodies.push(makeBody({ x: [-8.4, wallH, 0], halfExtents: [0.4, wallH, 8], static: true }));
  sim.bodies.push(makeBody({ x: [0, wallH,  8.4], halfExtents: [8, wallH, 0.4], static: true }));
  sim.bodies.push(makeBody({ x: [0, wallH, -8.4], halfExtents: [8, wallH, 0.4], static: true }));

  const COUNT = 800;
  for (let i = 0; i < COUNT; i++) {
    const x = (Math.random() - 0.5) * 14;
    const z = (Math.random() - 0.5) * 14;
    const y = 1.5 + Math.random() * 18;
    const b = makeBody({
      x: [x, y, z],
      halfExtents: [0.25, 0.25, 0.25],
      m: 1,
      color: [60 + Math.random()*180, 60 + Math.random()*180, 60 + Math.random()*180],
    });
    b.q = quatFromAxisAngle(vNorm([Math.random()-0.5, Math.random()-0.5, Math.random()-0.5]),
                            Math.random() * Math.PI);
    sim.bodies.push(b);
  }
}

function _sceneVaried() {
  const floor = makeBody({ x: [0, 0, 0], halfExtents: [10, 0.5, 10], static: true });
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
