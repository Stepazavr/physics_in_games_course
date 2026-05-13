const cam = {
  theta: 0.45,
  phi:   1.15,
  dist:  18,
  tx: 0, ty: 1.5, tz: 0,
  dragging: false, lx: 0, ly: 0,
};

let simulation;

function setup() {
  const cont = document.getElementById('canvas-container');
  createCanvas(cont.clientWidth, cont.clientHeight, WEBGL).parent('canvas-container');
  document.addEventListener('contextmenu', e => e.preventDefault());

  const taskPart = typeof TASK_PART !== 'undefined' ? TASK_PART : 1;
  const initialScene = taskPart === 1 ? '1A' : (taskPart === 2 ? '2A' : (taskPart === 3 ? '3A' : '4A'));

  simulation = {
    part: taskPart, sceneId: initialScene,
    bodies: [], springs: [], constraints: [], contacts: [],
    dt: 1/60, paused: false, fps: 60,

    freeRotMode: '1A',
    part2Kind: 'springForce',
    part2SI_PostStab: 'baumgarte',

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

  setupUserInterface();
  initializeScene(initialScene);
  applySceneSpecificSettings(initialScene);
  updateBroadphaseButtonsState(initialScene);
  updateUIVisibility();
}

function draw() {
  const cont = document.getElementById('canvas-container');
  if (abs(width - cont.clientWidth) > 2 || abs(height - cont.clientHeight) > 2)
    resizeCanvas(cont.clientWidth, cont.clientHeight);

  background(232, 232, 236);
  simulation.fps = round(frameRate());

  perspective(PI / 3, width / height, 0.1, 500);
  const eye = getCameraPosition();
  camera(eye.x, eye.y, eye.z, cam.tx, cam.ty, cam.tz, 0, -1, 0);

  ambientLight(200, 200, 210);
  directionalLight(240, 240, 250, -0.4, -0.8, -0.3);
  directionalLight(140, 160, 200,  0.5, -0.2,  0.7);

  if (!simulation.paused) updateSimulation();

  drawScene();
  updateStatisticsPanel();
}

function updateSimulation() {
  const dt = simulation.dt;
  simulation.elapsed += dt;
  simulation.stepCount = (simulation.stepCount || 0) + 1;

  if (simulation.part === 1)      updateFreeRotation(dt);
  else if (simulation.part === 2) updateConstraints(dt);
  else                     updateCollisions(dt);

  if (simulation.part === 1 && simulation.bodies.length > 0) {
    const E = computeKineticEnergy(simulation.bodies[0]);
    const ratio = simulation.E0 > 1e-9 ? E / simulation.E0 : 1;
    simulation.energyHistory.push(ratio);
    if (simulation.energyHistory.length > 600) simulation.energyHistory.shift();
  }
}

function updateFreeRotation(dt) {
  const b = simulation.bodies[0];
  updateFreeRotationStep(b, dt, simulation.freeRotMode);
}

function updateConstraints(dt) {
  const kind = simulation.part2Kind;

  const useGravity = !(kind === 'distXPBD' || kind === 'distSI');
  if (useGravity) {
    for (const b of simulation.bodies) {
      if (b.invM === 0) continue;
      b.v[1] -= simulation.gravity * dt;
    }
  }

  if (kind === 'springForce') {
    for (const s of simulation.springs) applyExplicitSpringForce(simulation.bodies[s.figureIdx], s, dt);
    for (const b of simulation.bodies) integrateAngularVelocity(b, dt);
    for (const b of simulation.bodies) integratePosition(b, dt);
  } else if (kind === 'springSoft') {
    for (const b of simulation.bodies) integrateAngularVelocity(b, dt);
    for (const s of simulation.springs) s.lambdaAccum = 0;
    for (let it = 0; it < simulation.iterations; it++)
      for (const s of simulation.springs) solveSpringConstraintSoft(simulation.bodies[s.figureIdx], s, dt);
    for (const b of simulation.bodies) integratePosition(b, dt);
  } else if (kind === 'distXPBD') {
    const sub = 4;
    const subDt = dt / sub;
    for (let s = 0; s < sub; s++) {
      for (const b of simulation.bodies) {
        if (b.invM === 0) continue;
        b.prevX = b.x.slice();
        b.prevQ = b.q.slice();
        b.x[0] += b.v[0] * subDt;
        b.x[1] += b.v[1] * subDt;
        b.x[2] += b.v[2] * subDt;
        b.q = integrateQuaternion(b.q, b.w, subDt);
      }
      for (const c of simulation.constraints) c.lambda = 0;
      for (let it = 0; it < 2; it++) {
        for (const c of simulation.constraints) {
          if (!c._state) c._state = { lambda: 0 };
          c._state.lambda = c.lambda;
          const A = simulation.bodies[c.a], B = simulation.bodies[c.b];
          const err = applyDistanceConstraintXPBD(A, B, c.rAloc, c.rBloc, c.restLen,
                                             simulation.compliance, subDt, c._state);
          c.lambda = c._state.lambda;
          simulation.maxC = Math.max(simulation.maxC, err);
        }
      }
      for (const b of simulation.bodies) {
        if (b.invM === 0) continue;
        b.v = scaleVector(subtractVectors(b.x, b.prevX), 1 / subDt);
        const dq = multiplyQuaternions(b.q, conjugateQuaternion(b.prevQ));
        const wAxis = [dq[1], dq[2], dq[3]];
        const wn = vectorLength(wAxis);
        if (wn > 1e-9) {
          const ang = 2 * Math.atan2(wn, dq[0]);
          b.w = scaleVector(scaleVector(wAxis, 1/wn), ang / subDt);
        }
      }
    }
    simulation.maxC = 0;
    for (const c of simulation.constraints) {
      const A = simulation.bodies[c.a], B = simulation.bodies[c.b];
      const pA = addVectors(A.x, rotateVectorByQuat(A.q, c.rAloc));
      const pB = addVectors(B.x, rotateVectorByQuat(B.q, c.rBloc));
      simulation.maxC = Math.max(simulation.maxC, Math.abs(vectorLength(subtractVectors(pA, pB)) - c.restLen));
    }
  } else if (kind === 'distSI') {
    for (const b of simulation.bodies) integrateAngularVelocity(b, dt);
    for (const c of simulation.constraints) c.lambdaAccum = 0;
    const postStab = simulation.part2SI_PostStab || 'baumgarte';
    const params = { beta: simulation.baumgarteBeta, k: simulation.springK, damping: simulation.springDamping };
    for (let it = 0; it < simulation.iterations; it++) {
      for (const c of simulation.constraints) {
        const A = simulation.bodies[c.a], B = simulation.bodies[c.b];
        applyDistanceConstraintSI(A, B, c, postStab, params, dt);
      }
    }
    for (const b of simulation.bodies) integratePosition(b, dt);
    if (postStab === 'nlgs') {
      for (let it = 0; it < simulation.iterations; it++) {
        for (const c of simulation.constraints) {
          const A = simulation.bodies[c.a], B = simulation.bodies[c.b];
          correctDistancePosition(A, B, c);
        }
      }
    }
    simulation.maxC = 0;
    for (const c of simulation.constraints) {
      const A = simulation.bodies[c.a], B = simulation.bodies[c.b];
      const pA = addVectors(A.x, rotateVectorByQuat(A.q, c.rAloc));
      const pB = addVectors(B.x, rotateVectorByQuat(B.q, c.rBloc));
      simulation.maxC = Math.max(simulation.maxC, Math.abs(vectorLength(subtractVectors(pA, pB)) - c.restLen));
    }
  }
}

function integrateAngularVelocity(b, dt) {
  if (b.invM === 0) return;
  implicitAngularIntegration(b, [0,0,0], dt);
}

function integratePosition(b, dt) {
  if (b.invM === 0) return;
  b.x[0] += b.v[0] * dt;
  b.x[1] += b.v[1] * dt;
  b.x[2] += b.v[2] * dt;
  b.q = integrateQuaternion(b.q, b.w, dt);
}

let sapState = null;

function storeContactPosition(A, B, ct) {
  ct._prevPA = addVectors(A.x, rotateVectorByQuat(A.q, ct.rAloc));
  ct._prevPB = B ? addVectors(B.x, rotateVectorByQuat(B.q, ct.rBloc)) : ct.worldB.slice();
}

function updateCollisions(dt) {
  for (const b of simulation.bodies) {
    if (b.invM === 0) continue;
    b.v[1] -= simulation.gravity * dt;
  }

  for (const b of simulation.bodies) updateBoundingBox(b);

  let pairs;
  if (simulation.broadphase === 'grid')      pairs = findPotentialPairsSpatialGrid(simulation.bodies, 1.5);
  else if (simulation.broadphase === 'sap') { if (!sapState) sapState = createSAPStructure();
                                        pairs = findPotentialPairsSAP(simulation.bodies, sapState); }
  else if (simulation.broadphase === 'lbvh') pairs = findPotentialPairsLBVH(simulation.bodies);
  else                                 pairs = findPotentialPairsBrute(simulation.bodies);

  simulation.broadphasePairs = pairs.length;

  const contacts = [];
  for (const [i, j] of pairs) {
    const A = simulation.bodies[i], B = simulation.bodies[j];
    const list = detectBoxCollisionSAT(A, B);
    if (!list) continue;
    for (const c of list) {
      contacts.push({
        ai: i, bi: j,
        rAloc: c.rAloc, rBloc: c.rBloc,
        n: c.n, depth: c.depth,
        rA: rotateVectorByQuat(A.q, c.rAloc),
        rB: rotateVectorByQuat(B.q, c.rBloc),
        lambdaN: 0, lambdaT1: 0, lambdaT2: 0,
        iterCount: 0,
      });
    }
  }
  simulation.contacts = contacts;

  if (simulation.solver === 'xpbd') resolveContactsXPBD(contacts, dt);
  else                        resolveContactsSI(contacts, dt);
}

function resolveContactsSI(contacts, dt) {
  const params = { beta: simulation.baumgarteBeta, mu: simulation.muDynamic, restitution: simulation.restitution };

  for (const b of simulation.bodies) {
    if (b.invM === 0) continue;
    implicitAngularIntegration(b, [0,0,0], dt);
  }

  for (let it = 0; it < simulation.iterations; it++) {
    for (const ct of contacts) {
      const A = simulation.bodies[ct.ai], B = simulation.bodies[ct.bi];
      ct.iterCount = it;
      applyContactNormalSI(A, B, ct, dt, params, simulation.postStab);
      applyContactFrictionSI(A, B, ct, params);
    }
  }

  for (const b of simulation.bodies) {
    if (b.invM === 0) continue;
    b.x[0] += b.v[0] * dt;
    b.x[1] += b.v[1] * dt;
    b.x[2] += b.v[2] * dt;
    b.q = integrateQuaternion(b.q, b.w, dt);
  }

  if (simulation.postStab === 'nlgs') {
    for (let it = 0; it < Math.min(4, simulation.iterations); it++) {
      for (const ct of contacts) {
        const A = simulation.bodies[ct.ai], B = simulation.bodies[ct.bi];
        correctContactPosition(A, B, ct);
      }
    }
  }
}

function resolveContactsXPBD(contacts, dt) {
  const sub = 4;
  const subDt = dt / sub;
  const alpha = simulation.compliance;
  const isPart4 = simulation.part === 4;

  for (let s = 0; s < sub; s++) {
    for (const b of simulation.bodies) {
      if (b.invM === 0) continue;
      b.prevX = b.x.slice();
      b.prevQ = b.q.slice();
      b.x[0] += b.v[0] * subDt;
      b.x[1] += b.v[1] * subDt;
      b.x[2] += b.v[2] * subDt;
      b.q = integrateQuaternion(b.q, b.w, subDt);
    }

    for (const ct of contacts) {
      const A = simulation.bodies[ct.ai], B = simulation.bodies[ct.bi];
      ct.worldB = addVectors(B.x, rotateVectorByQuat(B.q, ct.rBloc));
      storeContactPosition(A, B, ct);
    }

    for (const ct of contacts) ct.lambdaN = 0;
    for (let it = 0; it < 2; it++) {
      for (const ct of contacts) {
        const A = simulation.bodies[ct.ai], B = simulation.bodies[ct.bi];
        applyContactNormalXPBD(A, B, ct, subDt, alpha);
        if (simulation.part === 4) applyContactFrictionXPBD(A, B, ct, subDt, simulation.muStatic, simulation.muDynamic);
        else                 applyContactFrictionXPBD(A, B, ct, subDt, simulation.muDynamic, simulation.muDynamic);
      }
    }

    for (const b of simulation.bodies) {
      if (b.invM === 0) continue;
      b.v = scaleVector(subtractVectors(b.x, b.prevX), 1 / subDt);
      const dq = multiplyQuaternions(b.q, conjugateQuaternion(b.prevQ));
      const wAxis = [dq[1], dq[2], dq[3]];
      const wn = vectorLength(wAxis);
      if (wn > 1e-9) {
        const ang = 2 * Math.atan2(wn, dq[0]);
        b.w = scaleVector(scaleVector(wAxis, 1/wn), ang / subDt);
      } else {
        b.w = [0, 0, 0];
      }
    }
  }
}

function getCameraPosition() {
  const cp = cos(cam.phi), sp = sin(cam.phi);
  const ct = cos(cam.theta), st = sin(cam.theta);
  const dist = simulation.cameraDistOverride || cam.dist;
  return {
    x: cam.tx + dist * sp * ct,
    y: cam.ty + dist * cp,
    z: cam.tz + dist * sp * st,
  };
}

function drawScene() {
  for (const b of simulation.bodies) drawFigure(b);

  if (simulation.part === 1) drawAngularMomentumVectors();
  if (simulation.part === 2) drawConstraints();
  // Contact points disabled for large scenes (3B, 4A)
}

function drawAngularMomentumVectors() {
  const b = simulation.bodies[0];
  if (!b) return;
  
  const c = b.x;
  const axisLen = 1.5;
  const axisXLen = 3.0;
  
  const axisX = rotateVectorByQuat(b.q, [1, 0, 0]);
  const axisY = rotateVectorByQuat(b.q, [0, 1, 0]);
  const axisZ = rotateVectorByQuat(b.q, [0, 0, 1]);
  
  drawArrowWithHead(c, addVectors(c, scaleVector(axisX, axisXLen)), [255, 50, 50]);    // Bright red
  drawArrowWithHead(c, addVectors(c, scaleVector(axisY, axisLen)), [50, 255, 50]);     // Bright green
  drawArrowWithHead(c, addVectors(c, scaleVector(axisZ, axisLen)), [50, 150, 255]);    // Bright blue
  
  const L = computeAngularMomentum(b);
  const L0 = simulation.L0 || [0,0,0];
  const Lmax = Math.max(vectorLength(L), vectorLength(L0), 0.01);
  const sL = 2.5 / Lmax;
  
  drawArrowWithHead(c, addVectors(c, scaleVector(L0, sL)), [100, 255, 255], 0.6);
  drawArrowWithHead(c, addVectors(c, scaleVector(L,  sL)), [150, 255, 100]);
}

function drawArrowWithHead(a, b, color, opacity = 1.0) {
  drawCylinderBetweenPoints(a, b, 0.02, color, opacity);
  
  const dir = subtractVectors(b, a);
  const len = vectorLength(dir);
  if (len < 1e-6) return;
  
  const n = scaleVector(dir, 1 / len);
  const arrowSize = 0.45;
  
  let perp1 = crossProduct(n, [0, 1, 0]);
  if (vectorLength(perp1) < 0.1) perp1 = crossProduct(n, [1, 0, 0]);
  perp1 = normalizeVector(perp1);
  const perp2 = crossProduct(n, perp1);
  
  const arrowBase = addVectors(b, scaleVector(n, -arrowSize * 0.5));
  const point1 = addVectors(arrowBase, scaleVector(perp1, arrowSize * 0.3));
  const point2 = addVectors(arrowBase, scaleVector(perp2, arrowSize * 0.3));
  
  push();
    noStroke();
    fill(color[0], color[1], color[2], opacity * 255);
    
    beginShape(TRIANGLES);
    vertex(b[0], b[1], b[2]); vertex(point1[0], point1[1], point1[2]); vertex(point2[0], point2[1], point2[2]);
    vertex(b[0], b[1], b[2]); vertex(point2[0], point2[1], point2[2]); 
    vertex(addVectors(arrowBase, scaleVector(perp1, -arrowSize * 0.3))[0], addVectors(arrowBase, scaleVector(perp1, -arrowSize * 0.3))[1], addVectors(arrowBase, scaleVector(perp1, -arrowSize * 0.3))[2]);
    endShape();
  pop();
}

function drawCylinderBetweenPoints(a, b, radius, color, opacity = 1.0) {
  const dir = subtractVectors(b, a);
  const len = vectorLength(dir);
  if (len < 1e-6) return;

  const mid = [(a[0]+b[0])*0.5, (a[1]+b[1])*0.5, (a[2]+b[2])*0.5];
  const n = scaleVector(dir, 1 / len);
  const up = [0, 1, 0];
  const axisV = crossProduct(up, n);
  const an = vectorLength(axisV);

  push();
    translate(mid[0], mid[1], mid[2]);
    if (an > 1e-6) {
      const ang = Math.atan2(an, dotProduct(up, n));
      rotate(ang, [axisV[0]/an, axisV[1]/an, axisV[2]/an]);
    } else if (dotProduct(up, n) < 0) {
      rotate(Math.PI, [1, 0, 0]);
    }
    noStroke();
    fill(color[0], color[1], color[2], opacity * 255);
    cylinder(radius, len, 12, 1, true, true);
  pop();
}

function drawConstraints() {
  for (const s of simulation.springs) {
    const A = simulation.bodies[s.figureIdx];
    const pA = addVectors(A.x, rotateVectorByQuat(A.q, s.rLocal));
    const pB = s.pWorld;
    const dLen = vectorLength(subtractVectors(pA, pB));
    const stretch = Math.abs(dLen - s.restLen) / s.restLen;
    
    const col = [
      Math.min(255, 150 + stretch * 400),
      Math.max(50, 100 - stretch * 200),
      200
    ];
    
    drawCylinderBetweenPoints(pA, pB, 0.02, col);
    
    push(); noStroke(); fill(220, 100, 200);
      translate(pA[0], pA[1], pA[2]); sphere(0.08, 16, 12);
    pop();
    push(); noStroke(); fill(180, 100, 180);
      translate(pB[0], pB[1], pB[2]); sphere(0.1, 16, 12);
    pop();
  }
  for (const c of simulation.constraints) {
    const A = simulation.bodies[c.a], B = simulation.bodies[c.b];
    const pA = addVectors(A.x, rotateVectorByQuat(A.q, c.rAloc));
    const pB = addVectors(B.x, rotateVectorByQuat(B.q, c.rBloc));
    const err = Math.abs(vectorLength(subtractVectors(pA, pB)) - c.restLen);
    
    const col = [
      100 + Math.min(100, err * 2000),
      150 + Math.min(100, err * 1000),
      255
    ];
    drawCylinderBetweenPoints(pA, pB, 0.035, col);
    
    push(); noStroke(); fill(100, 200, 255);
      translate(pA[0], pA[1], pA[2]); sphere(0.08, 16, 12);
    pop();
    push(); noStroke(); fill(100, 200, 255);
      translate(pB[0], pB[1], pB[2]); sphere(0.08, 16, 12);
    pop();
  }
}

function mousePressed(event) {
  if (!event || event.target.tagName !== 'CANVAS') return;
  if (event.button === 0) {
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
  if (!event || event.button === 0) cam.dragging = false;
}

function mouseWheel(e) {
  cam.dist = constrain(cam.dist + e.delta * 0.01, 3, 80);
  if (simulation.cameraDistOverride !== null && simulation.cameraDistOverride !== undefined) {
    simulation.cameraDistOverride = constrain(simulation.cameraDistOverride + e.delta * 0.01, 3, 80);
  }
  return false;
}

function keyPressed() {
  if (key === 'r' || key === 'R') initializeScene(simulation.sceneId);
}

function windowResized() {
  const cont = document.getElementById('canvas-container');
  resizeCanvas(cont.clientWidth, cont.clientHeight);
}
