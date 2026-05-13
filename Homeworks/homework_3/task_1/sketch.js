const cam = {
  theta: 0.45,
  phi:   1.15,
  dist:  18,
  tx: 0, ty: 1.5, tz: 0,
  dragging: false, lx: 0, ly: 0,
};

let sim;

function setup() {
  const cont = document.getElementById('canvas-container');
  createCanvas(cont.clientWidth, cont.clientHeight, WEBGL).parent('canvas-container');
  document.addEventListener('contextmenu', e => e.preventDefault());

  sim = {
    part: 1, sceneId: '1A',
    bodies: [], springs: [], constraints: [], contacts: [],
    dt: 1/60, paused: false, fps: 60,

    freeRotMode: '1A',
    part2Kind: 'springForce',

    solver: 'si',
    postStab: 'baumgarte',
    broadphase: 'brute',

    iterations: 8,
    compliance: 0,
    baumgarteBeta: 0.2,
    muStatic: 0.5, muDynamic: 0.3, restitution: 0,
    gravity: 9.8,
    springK: 200, springDamping: 4,

    L0: null, E0: null,
    broadphasePairs: 0, maxC: 0,
    energyHistory: [],
    elapsed: 0,
  };

  initUI();
  loadScene('1A');
}

function draw() {
  const cont = document.getElementById('canvas-container');
  if (abs(width - cont.clientWidth) > 2 || abs(height - cont.clientHeight) > 2)
    resizeCanvas(cont.clientWidth, cont.clientHeight);

  background(13, 13, 26);
  sim.fps = round(frameRate());

  perspective(PI / 3, width / height, 0.1, 500);
  const eye = _camEye();
  camera(eye.x, eye.y, eye.z, cam.tx, cam.ty, cam.tz, 0, -1, 0);

  ambientLight(80, 80, 95);
  directionalLight(220, 220, 230, -0.4, -0.8, -0.3);
  directionalLight(80, 100, 140,  0.5, -0.2,  0.7);

  if (!sim.paused) simStep();

  _drawScene();
  updateMetrics();
  _drawEnergyPlot();
}

function simStep() {
  const dt = sim.dt;
  sim.elapsed += dt;
  sim.stepCount = (sim.stepCount || 0) + 1;

  if (sim.part === 1)      _stepPart1(dt);
  else if (sim.part === 2) _stepPart2(dt);
  else                     _stepPart34(dt);

  if (sim.part === 1 && sim.bodies.length > 0) {
    const E = bodyKineticEnergy(sim.bodies[0]);
    const ratio = sim.E0 > 1e-9 ? E / sim.E0 : 1;
    sim.energyHistory.push(ratio);
    if (sim.energyHistory.length > 600) sim.energyHistory.shift();
  }
}

function _stepPart1(dt) {
  const b = sim.bodies[0];
  freeRotStep(b, dt, sim.freeRotMode);
}

function _stepPart2(dt) {
  const kind = sim.part2Kind;

  const useGravity = !(kind === 'distXPBD' || kind === 'distSI');
  if (useGravity) {
    for (const b of sim.bodies) {
      if (b.invM === 0) continue;
      b.v[1] -= sim.gravity * dt;
    }
  }

  if (kind === 'springForce') {
    for (const s of sim.springs) applySpringForce(sim.bodies[s.bodyIdx], s, dt);
    for (const b of sim.bodies) _angularImplicitStep(b, dt);
    for (const b of sim.bodies) _positionStep(b, dt);
  } else if (kind === 'springSoft') {
    for (const b of sim.bodies) _angularImplicitStep(b, dt);
    for (const s of sim.springs) s.lambdaAccum = 0;
    for (let it = 0; it < sim.iterations; it++)
      for (const s of sim.springs) solveSpringSoft(sim.bodies[s.bodyIdx], s, dt);
    for (const b of sim.bodies) _positionStep(b, dt);
  } else if (kind === 'distXPBD') {
    const sub = 4;
    const subDt = dt / sub;
    for (let s = 0; s < sub; s++) {
      for (const b of sim.bodies) {
        if (b.invM === 0) continue;
        b.prevX = b.x.slice();
        b.prevQ = b.q.slice();
        b.x[0] += b.v[0] * subDt;
        b.x[1] += b.v[1] * subDt;
        b.x[2] += b.v[2] * subDt;
        b.q = quatIntegrate(b.q, b.w, subDt);
      }
      for (const c of sim.constraints) c.lambda = 0;
      for (let it = 0; it < 2; it++) {
        for (const c of sim.constraints) {
          if (!c._state) c._state = { lambda: 0 };
          c._state.lambda = c.lambda;
          const A = sim.bodies[c.a], B = sim.bodies[c.b];
          const err = xpbdDistanceConstraint(A, B, c.rAloc, c.rBloc, c.restLen,
                                             sim.compliance, subDt, c._state);
          c.lambda = c._state.lambda;
          sim.maxC = Math.max(sim.maxC, err);
        }
      }
      for (const b of sim.bodies) {
        if (b.invM === 0) continue;
        b.v = vMul(vSub(b.x, b.prevX), 1 / subDt);
        const dq = quatMul(b.q, quatConj(b.prevQ));
        const wAxis = [dq[1], dq[2], dq[3]];
        const wn = vLen(wAxis);
        if (wn > 1e-9) {
          const ang = 2 * Math.atan2(wn, dq[0]);
          b.w = vMul(vMul(wAxis, 1/wn), ang / subDt);
        }
      }
    }
    sim.maxC = 0;
    for (const c of sim.constraints) {
      const A = sim.bodies[c.a], B = sim.bodies[c.b];
      const pA = vAdd(A.x, quatRotate(A.q, c.rAloc));
      const pB = vAdd(B.x, quatRotate(B.q, c.rBloc));
      sim.maxC = Math.max(sim.maxC, Math.abs(vLen(vSub(pA, pB)) - c.restLen));
    }
  } else if (kind === 'distSI') {
    for (const b of sim.bodies) _angularImplicitStep(b, dt);
    for (const c of sim.constraints) c.lambdaAccum = 0;
    const params = { beta: sim.baumgarteBeta, k: sim.springK, damping: sim.springDamping };
    for (let it = 0; it < sim.iterations; it++) {
      for (const c of sim.constraints) {
        const A = sim.bodies[c.a], B = sim.bodies[c.b];
        siDistanceConstraint(A, B, c, sim.postStab, params, dt);
      }
    }
    for (const b of sim.bodies) _positionStep(b, dt);
    if (sim.postStab === 'nlgs') {
      for (let it = 0; it < sim.iterations; it++) {
        for (const c of sim.constraints) {
          const A = sim.bodies[c.a], B = sim.bodies[c.b];
          siDistancePosPass(A, B, c);
        }
      }
    }
    sim.maxC = 0;
    for (const c of sim.constraints) {
      const A = sim.bodies[c.a], B = sim.bodies[c.b];
      const pA = vAdd(A.x, quatRotate(A.q, c.rAloc));
      const pB = vAdd(B.x, quatRotate(B.q, c.rBloc));
      sim.maxC = Math.max(sim.maxC, Math.abs(vLen(vSub(pA, pB)) - c.restLen));
    }
  }
}

function _angularImplicitStep(b, dt) {
  if (b.invM === 0) return;
  integrateAngularImplicit(b, [0,0,0], dt);
}

function _positionStep(b, dt) {
  if (b.invM === 0) return;
  b.x[0] += b.v[0] * dt;
  b.x[1] += b.v[1] * dt;
  b.x[2] += b.v[2] * dt;
  b.q = quatIntegrate(b.q, b.w, dt);
}

let sapState = null;

function _stepPart34(dt) {
  for (const b of sim.bodies) {
    if (b.invM === 0) continue;
    b.v[1] -= sim.gravity * dt;
  }

  for (const b of sim.bodies) updateAABB(b);

  let pairs;
  if (sim.broadphase === 'grid')      pairs = broadphaseSpatialGrid(sim.bodies, 1.5);
  else if (sim.broadphase === 'sap') { if (!sapState) sapState = makeSAP();
                                        pairs = broadphaseSAP(sim.bodies, sapState); }
  else if (sim.broadphase === 'lbvh') pairs = broadphaseLBVH(sim.bodies);
  else                                 pairs = broadphaseBruteforce(sim.bodies);

  sim.broadphasePairs = pairs.length;

  const contacts = [];
  for (const [i, j] of pairs) {
    const A = sim.bodies[i], B = sim.bodies[j];
    const list = satBoxBox(A, B);
    if (!list) continue;
    for (const c of list) {
      contacts.push({
        ai: i, bi: j,
        rAloc: c.rAloc, rBloc: c.rBloc,
        n: c.n, depth: c.depth,
        rA: quatRotate(A.q, c.rAloc),
        rB: quatRotate(B.q, c.rBloc),
        lambdaN: 0, lambdaT1: 0, lambdaT2: 0,
        iterCount: 0,
      });
    }
  }
  sim.contacts = contacts;

  if (sim.solver === 'xpbd') _solveXPBD(contacts, dt);
  else                        _solveSI(contacts, dt);
}

function _solveSI(contacts, dt) {
  const params = { beta: sim.baumgarteBeta, mu: sim.muDynamic, restitution: sim.restitution };

  for (const b of sim.bodies) {
    if (b.invM === 0) continue;
    integrateAngularImplicit(b, [0,0,0], dt);
  }

  for (let it = 0; it < sim.iterations; it++) {
    for (const ct of contacts) {
      const A = sim.bodies[ct.ai], B = sim.bodies[ct.bi];
      ct.iterCount = it;
      siContactNormal(A, B, ct, dt, params, sim.postStab);
      siContactFriction(A, B, ct, params);
    }
  }

  for (const b of sim.bodies) {
    if (b.invM === 0) continue;
    b.x[0] += b.v[0] * dt;
    b.x[1] += b.v[1] * dt;
    b.x[2] += b.v[2] * dt;
    b.q = quatIntegrate(b.q, b.w, dt);
  }

  if (sim.postStab === 'nlgs') {
    for (let it = 0; it < Math.min(4, sim.iterations); it++) {
      for (const ct of contacts) {
        const A = sim.bodies[ct.ai], B = sim.bodies[ct.bi];
        siContactPosPass(A, B, ct);
      }
    }
  }
}

function _solveXPBD(contacts, dt) {
  const sub = 4;
  const subDt = dt / sub;
  const alpha = sim.compliance;
  const isPart4 = sim.part === 4;

  for (let s = 0; s < sub; s++) {
    for (const b of sim.bodies) {
      if (b.invM === 0) continue;
      b.prevX = b.x.slice();
      b.prevQ = b.q.slice();
      b.x[0] += b.v[0] * subDt;
      b.x[1] += b.v[1] * subDt;
      b.x[2] += b.v[2] * subDt;
      b.q = quatIntegrate(b.q, b.w, subDt);
    }

    for (const ct of contacts) {
      const A = sim.bodies[ct.ai], B = sim.bodies[ct.bi];
      snapshotContactPoint(A, B, ct);
    }

    for (const ct of contacts) ct.lambdaN = 0;
    for (let it = 0; it < 2; it++) {
      for (const ct of contacts) {
        const A = sim.bodies[ct.ai], B = sim.bodies[ct.bi];
        ct.worldB = vAdd(B.x, quatRotate(B.q, ct.rBloc));
        xpbdContactNormal(A, B, ct, subDt, alpha);
        if (isPart4) xpbdContactFriction(A, B, ct, subDt, sim.muStatic, sim.muDynamic);
        else         xpbdContactFriction(A, B, ct, subDt, sim.muDynamic, sim.muDynamic);
      }
    }

    for (const b of sim.bodies) {
      if (b.invM === 0) continue;
      b.v = vMul(vSub(b.x, b.prevX), 1 / subDt);
      const dq = quatMul(b.q, quatConj(b.prevQ));
      const wAxis = [dq[1], dq[2], dq[3]];
      const wn = vLen(wAxis);
      if (wn > 1e-9) {
        const ang = 2 * Math.atan2(wn, dq[0]);
        b.w = vMul(vMul(wAxis, 1/wn), ang / subDt);
      } else {
        b.w = [0, 0, 0];
      }
    }
  }
}

function _camEye() {
  const cp = cos(cam.phi), sp = sin(cam.phi);
  const ct = cos(cam.theta), st = sin(cam.theta);
  return {
    x: cam.tx + cam.dist * sp * ct,
    y: cam.ty + cam.dist * cp,
    z: cam.tz + cam.dist * sp * st,
  };
}

function _drawScene() {
  for (const b of sim.bodies) drawBody(b);

  if (sim.part === 1) _drawAngularMomentumArrows();
  if (sim.part === 2) _drawJoints();
  if (sim.part >= 3) _drawContacts();
}

function _drawAngularMomentumArrows() {
  const b = sim.bodies[0];
  if (!b) return;
  const L = bodyAngularMomentum(b);
  const L0 = sim.L0 || [0,0,0];
  const Lmax = Math.max(vLen(L), vLen(L0), 0.01);
  const sL = 2.5 / Lmax;
  const c = b.x;
  _arrow(c, vAdd(c, vMul(L0, sL)), [180, 180, 180]);
  _arrow(c, vAdd(c, vMul(L,  sL)), [255, 160, 60]);
}

function _arrow(a, b, color) {
  _thinRod(a, b, 0.025, color);
}

function _thinRod(a, b, radius, color) {
  const dir = vSub(b, a);
  const len = vLen(dir);
  if (len < 1e-6) return;

  const mid = [(a[0]+b[0])*0.5, (a[1]+b[1])*0.5, (a[2]+b[2])*0.5];
  const n = vMul(dir, 1 / len);
  const up = [0, 1, 0];
  const axisV = vCross(up, n);
  const an = vLen(axisV);

  push();
    translate(mid[0], mid[1], mid[2]);
    if (an > 1e-6) {
      const ang = Math.atan2(an, vDot(up, n));
      rotate(ang, [axisV[0]/an, axisV[1]/an, axisV[2]/an]);
    } else if (vDot(up, n) < 0) {
      rotate(Math.PI, [1, 0, 0]);
    }
    noStroke();
    fill(color[0], color[1], color[2]);
    cylinder(radius, len, 12, 1, true, true);
  pop();
}

function _drawJoints() {
  for (const s of sim.springs) {
    const A = sim.bodies[s.bodyIdx];
    const pA = vAdd(A.x, quatRotate(A.q, s.rLocal));
    const pB = s.pWorld;
    const dLen = vLen(vSub(pA, pB));
    const stretch = Math.abs(dLen - s.restLen) / s.restLen;
    const col = [
      Math.min(255, 80 + stretch * 800),
      Math.max(40, 200 - stretch * 600),
      80,
    ];
    _thinRod(pA, pB, 0.02, col);
    push(); noStroke(); fill(255, 220, 60);
      translate(pA[0], pA[1], pA[2]); sphere(0.07, 14, 10);
    pop();
    push(); noStroke(); fill(180, 180, 180);
      translate(pB[0], pB[1], pB[2]); sphere(0.1, 14, 10);
    pop();
  }
  for (const c of sim.constraints) {
    const A = sim.bodies[c.a], B = sim.bodies[c.b];
    const pA = vAdd(A.x, quatRotate(A.q, c.rAloc));
    const pB = vAdd(B.x, quatRotate(B.q, c.rBloc));
    const err = Math.abs(vLen(vSub(pA, pB)) - c.restLen);
    const col = [
      Math.min(255, 60 + err * 4000),
      Math.max(40, 220 - err * 3000),
      80,
    ];
    _thinRod(pA, pB, 0.02, col);
    push(); noStroke(); fill(255, 220, 60);
      translate(pA[0], pA[1], pA[2]); sphere(0.07, 14, 10);
    pop();
    push(); noStroke(); fill(255, 220, 60);
      translate(pB[0], pB[1], pB[2]); sphere(0.07, 14, 10);
    pop();
  }
}

function _drawContacts() {
  if (sim.contacts.length > 200) return;
  push();
    noStroke();
    fill(255, 90, 90);
    for (const ct of sim.contacts) {
      const A = sim.bodies[ct.ai];
      const p = vAdd(A.x, quatRotate(A.q, ct.rAloc));
      push(); translate(p[0], p[1], p[2]); sphere(0.05, 10, 7); pop();
    }
  pop();
}

function _drawEnergyPlot() {
  const cv = document.getElementById('energy-plot');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.fillStyle = '#0a0a18';
  ctx.fillRect(0, 0, W, H);

  ctx.strokeStyle = '#222244';
  ctx.beginPath();
  ctx.moveTo(0, H * 0.5);
  ctx.lineTo(W, H * 0.5);
  ctx.stroke();

  const N = sim.energyHistory.length;
  if (N < 2) return;
  ctx.strokeStyle = '#55cc66';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  for (let i = 0; i < N; i++) {
    const r = sim.energyHistory[i];
    const x = (i / (N - 1)) * W;
    const y = H * 0.5 - (r - 1.0) * H * 0.5;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();

  ctx.fillStyle = '#779';
  ctx.font = '10px "Courier New", monospace';
  ctx.fillText(`${sim.freeRotMode}  E/E₀  range [0.5..1.5]`, 4, 12);
}

function mousePressed(event) {
  if (!event || event.target.tagName !== 'CANVAS') return;
  if (event.button === 2) {
    cam.dragging = true; cam.lx = mouseX; cam.ly = mouseY;
    return false;
  }
}

function mouseDragged() {
  if (cam.dragging) {
    cam.theta += (mouseX - cam.lx) * 0.007;
    cam.phi   -= (mouseY - cam.ly) * 0.007;
    cam.phi    = constrain(cam.phi, 0.1, PI - 0.1);
    cam.lx = mouseX; cam.ly = mouseY;
    return false;
  }
}

function mouseReleased(event) {
  if (!event || event.button === 2) cam.dragging = false;
}

function mouseWheel(e) {
  cam.dist = constrain(cam.dist + e.delta * 0.01, 3, 80);
  return false;
}

function keyPressed() {
  if (key === 'r' || key === 'R') loadScene(sim.sceneId);
}

function windowResized() {
  const cont = document.getElementById('canvas-container');
  resizeCanvas(cont.clientWidth, cont.clientHeight);
}
