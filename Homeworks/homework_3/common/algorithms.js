function applyDistanceConstraintSI(A, B, c, mode, params, dt) {
  const rA = rotateVectorByQuat(A.q, c.rAloc);
  const rB = rotateVectorByQuat(B.q, c.rBloc);
  const pA = addVectors(A.x, rA);
  const pB = addVectors(B.x, rB);
  const dvec = subtractVectors(pA, pB);
  const dLen = vectorLength(dvec);
  if (dLen < 1e-9) return 0;
  const n = scaleVector(dvec, 1 / dLen);
  const C = dLen - c.restLen;

  const IAi = transformToWorldInertiaInv(A);
  const IBi = transformToWorldInertiaInv(B);
  const rAxn = crossProduct(rA, n);
  const rBxn = crossProduct(rB, n);
  const wA = A.invM + dotProduct(rAxn, multiplyMatrix3Vector(IAi, rAxn));
  const wB = B.invM + dotProduct(rBxn, multiplyMatrix3Vector(IBi, rBxn));
  let K = wA + wB;
  if (K < 1e-12) return Math.abs(C);

  const vA_pt = addVectors(A.v, crossProduct(A.w, rA));
  const vB_pt = addVectors(B.v, crossProduct(B.w, rB));
  const Cdot = dotProduct(subtractVectors(vA_pt, vB_pt), n);

  let bias = 0;
  let softGamma = 0;
  if (mode === 'baumgarte') {
    const beta = params.beta || 0.2;
    bias = (beta / dt) * C;
  } else if (mode === 'soft') {
    const k = params.k || 200, cdamp = params.damping || 4;
    softGamma = 1 / (dt * (cdamp + dt * k));
    const beta = dt * k / (cdamp + dt * k);
    bias = (beta / dt) * C;
    K += softGamma;
  }

  if (c.lambdaAccum == null) c.lambdaAccum = 0;
  const lambda = -(Cdot + bias + softGamma * c.lambdaAccum) / K;
  c.lambdaAccum += lambda;

  const P = scaleVector(n, lambda);
  applyVelocityImpulse(A,  P, rA, IAi);
  applyVelocityImpulse(B, scaleVector(P, -1), rB, IBi);

  return Math.abs(C);
}

function correctDistancePosition(A, B, c) {
  const rA = rotateVectorByQuat(A.q, c.rAloc);
  const rB = rotateVectorByQuat(B.q, c.rBloc);
  const pA = addVectors(A.x, rA);
  const pB = addVectors(B.x, rB);
  const dvec = subtractVectors(pA, pB);
  const dLen = vectorLength(dvec);
  if (dLen < 1e-9) return 0;
  const n = scaleVector(dvec, 1 / dLen);
  const C = dLen - c.restLen;

  const IAi = transformToWorldInertiaInv(A);
  const IBi = transformToWorldInertiaInv(B);
  const rAxn = crossProduct(rA, n);
  const rBxn = crossProduct(rB, n);
  const wA = A.invM + dotProduct(rAxn, multiplyMatrix3Vector(IAi, rAxn));
  const wB = B.invM + dotProduct(rBxn, multiplyMatrix3Vector(IBi, rBxn));
  const wEff = wA + wB;
  if (wEff < 1e-12) return Math.abs(C);

  const dLambda = -C / wEff;
  const P = scaleVector(n, dLambda);
  if (A.invM > 0) {
    A.x = addVectors(A.x, scaleVector(P, A.invM));
    const dq = multiplyMatrix3Vector(IAi, crossProduct(rA, P));
    applyRotationDelta(A, dq);
  }
  if (B.invM > 0) {
    B.x = subtractVectors(B.x, scaleVector(P, B.invM));
    const dq = multiplyMatrix3Vector(IBi, crossProduct(rB, scaleVector(P, -1)));
    applyRotationDelta(B, dq);
  }
  return Math.abs(C);
}

function applyContactNormalSI(A, B, ct, dt, params, mode) {
  const IAi = transformToWorldInertiaInv(A);
  const IBi = B ? transformToWorldInertiaInv(B) : createZeroMatrix3();
  const rA = ct.rA, rB = ct.rB || [0,0,0];
  const n = ct.n;
  const rAxn = crossProduct(rA, n);
  const rBxn = crossProduct(rB, n);
  const wA = A.invM + dotProduct(rAxn, multiplyMatrix3Vector(IAi, rAxn));
  const wB = B ? (B.invM + dotProduct(rBxn, multiplyMatrix3Vector(IBi, rBxn))) : 0;
  const K = wA + wB;
  if (K < 1e-12) return;

  const vA_pt = addVectors(A.v, crossProduct(A.w, rA));
  const vB_pt = B ? addVectors(B.v, crossProduct(B.w, rB)) : [0,0,0];
  const vRel = dotProduct(subtractVectors(vB_pt, vA_pt), n);

  let bias = 0;
  if (mode === 'baumgarte') {
    const beta = params.beta || 0.2;
    const slop = 0.005;
    bias = -(beta / dt) * Math.max(0, ct.depth - slop);
  }
  if (ct.iterCount === 0 && vRel < -0.5) {
    bias += params.restitution * vRel;
  }

  let dLambda = -(vRel + bias) / K;
  const oldL = ct.lambdaN || 0;
  let newL = Math.max(0, oldL + dLambda);
  dLambda = newL - oldL;
  ct.lambdaN = newL;

  const P = scaleVector(n, dLambda);
  applyVelocityImpulse(A, scaleVector(P, -1), rA, IAi);
  if (B) applyVelocityImpulse(B, P, rB, IBi);
}

function applyContactFrictionSI(A, B, ct, params) {
  const IAi = transformToWorldInertiaInv(A);
  const IBi = B ? transformToWorldInertiaInv(B) : createZeroMatrix3();
  const rA = ct.rA, rB = ct.rB || [0,0,0];
  const n = ct.n;
  const t1 = getPerpendicularAxis(n);
  const t2 = crossProduct(n, t1);

  const mu = params.mu;
  const maxF = mu * (ct.lambdaN || 0);
  for (const t of [t1, t2]) {
    const rAxt = crossProduct(rA, t);
    const rBxt = crossProduct(rB, t);
    const wAt = A.invM + dotProduct(rAxt, multiplyMatrix3Vector(IAi, rAxt));
    const wBt = B ? (B.invM + dotProduct(rBxt, multiplyMatrix3Vector(IBi, rBxt))) : 0;
    const K = wAt + wBt;
    if (K < 1e-12) continue;

    const vA_pt = addVectors(A.v, crossProduct(A.w, rA));
    const vB_pt = B ? addVectors(B.v, crossProduct(B.w, rB)) : [0,0,0];
    const vRel = dotProduct(subtractVectors(vB_pt, vA_pt), t);

    let dLambda = -vRel / K;
    const key = (t === t1) ? 'lambdaT1' : 'lambdaT2';
    const old = ct[key] || 0;
    let cur = old + dLambda;
    if (cur >  maxF) cur =  maxF;
    if (cur < -maxF) cur = -maxF;
    dLambda = cur - old;
    ct[key] = cur;

    const P = scaleVector(t, dLambda);
    applyVelocityImpulse(A, scaleVector(P, -1), rA, IAi);
    if (B) applyVelocityImpulse(B, P, rB, IBi);
  }
}

function correctContactPosition(A, B, ct) {
  const pA = addVectors(A.x, rotateVectorByQuat(A.q, ct.rAloc));
  const pB = B ? addVectors(B.x, rotateVectorByQuat(B.q, ct.rBloc)) : ct.worldB;
  const sep = dotProduct(subtractVectors(pA, pB), ct.n);
  if (sep <= 0) return;

  const rA = subtractVectors(pA, A.x);
  const rB = B ? subtractVectors(pB, B.x) : [0,0,0];
  const IAi = transformToWorldInertiaInv(A);
  const IBi = B ? transformToWorldInertiaInv(B) : createZeroMatrix3();
  const rAxn = crossProduct(rA, ct.n);
  const rBxn = crossProduct(rB, ct.n);
  const wA = A.invM + dotProduct(rAxn, multiplyMatrix3Vector(IAi, rAxn));
  const wB = B ? (B.invM + dotProduct(rBxn, multiplyMatrix3Vector(IBi, rBxn))) : 0;
  const wEff = wA + wB;
  if (wEff < 1e-12) return;

  const slop = 0.002;
  const corr = Math.max(0, sep - slop);
  const dLambda = corr / wEff;
  const P = scaleVector(ct.n, dLambda);
  if (A.invM > 0) {
    A.x = subtractVectors(A.x, scaleVector(P, A.invM));
    applyRotationDelta(A, multiplyMatrix3Vector(IAi, crossProduct(rA, scaleVector(P, -1))));
  }
  if (B && B.invM > 0) {
    B.x = addVectors(B.x, scaleVector(P, B.invM));
    applyRotationDelta(B, multiplyMatrix3Vector(IBi, crossProduct(rB, P)));
  }
}

function applyVelocityImpulse(b, P, r, Iinv) {
  if (b.invM === 0) return;
  b.v[0] += b.invM * P[0];
  b.v[1] += b.invM * P[1];
  b.v[2] += b.invM * P[2];
  const dw = multiplyMatrix3Vector(Iinv, crossProduct(r, P));
  b.w[0] += dw[0];
  b.w[1] += dw[1];
  b.w[2] += dw[2];
}

function applyRotationDelta(b, dq_vec) {
  const inc = multiplyQuaternions([0, dq_vec[0]*0.5, dq_vec[1]*0.5, dq_vec[2]*0.5], b.q);
  b.q = normalizeQuaternion([b.q[0]+inc[0], b.q[1]+inc[1], b.q[2]+inc[2], b.q[3]+inc[3]]);
}

function getPerpendicularAxis(n) {
  const ax = Math.abs(n[0]), ay = Math.abs(n[1]), az = Math.abs(n[2]);
  let v;
  if (ax <= ay && ax <= az) v = [1, 0, 0];
  else if (ay <= az)        v = [0, 1, 0];
  else                       v = [0, 0, 1];
  const t = normalizeVector(subtractVectors(v, scaleVector(n, dotProduct(v, n))));
  return t;
}


function applyDistanceConstraintXPBD(A, B, rAloc, rBloc, restLen, alpha, dt, state) {
  const alphaT = alpha / (dt * dt);
  const rA = rotateVectorByQuat(A.q, rAloc);
  const rB = rotateVectorByQuat(B.q, rBloc);
  const pA = addVectors(A.x, rA);
  const pB = addVectors(B.x, rB);
  const dvec = subtractVectors(pA, pB);
  const dLen = vectorLength(dvec);
  if (dLen < 1e-9) return 0;
  const n = scaleVector(dvec, 1 / dLen);
  const C = dLen - restLen;

  const IAi = transformToWorldInertiaInv(A);
  const IBi = transformToWorldInertiaInv(B);
  const rAxn = crossProduct(rA, n);
  const rBxn = crossProduct(rB, n);
  const wA = A.invM + dotProduct(rAxn, multiplyMatrix3Vector(IAi, rAxn));
  const wB = B.invM + dotProduct(rBxn, multiplyMatrix3Vector(IBi, rBxn));
  const wEff = wA + wB;
  if (wEff < 1e-12) return Math.abs(C);

  if (state.lambda == null) state.lambda = 0;
  const dLambda = (-C - alphaT * state.lambda) / (wEff + alphaT);
  state.lambda += dLambda;

  const P = scaleVector(n, dLambda);
  if (A.invM > 0) {
    A.x = addVectors(A.x, scaleVector(P, A.invM));
    const dqAvec = multiplyMatrix3Vector(IAi, crossProduct(rA, P));
    A.q = normalizeQuaternion(multiplyQuaternions([0, dqAvec[0]*0.5, dqAvec[1]*0.5, dqAvec[2]*0.5], A.q).map((c, i) => c + A.q[i]));
  }
  if (B.invM > 0) {
    B.x = subtractVectors(B.x, scaleVector(P, B.invM));
    const dqBvec = multiplyMatrix3Vector(IBi, crossProduct(rB, P));
    const inc = multiplyQuaternions([0, -dqBvec[0]*0.5, -dqBvec[1]*0.5, -dqBvec[2]*0.5], B.q);
    B.q = normalizeQuaternion([B.q[0]+inc[0], B.q[1]+inc[1], B.q[2]+inc[2], B.q[3]+inc[3]]);
  }

  return Math.abs(C);
}

function correctDistancePositionXPBD(A, B, rA, rB, n, c, alphaT, state) {
  const IAi = A.invM > 0 ? transformToWorldInertiaInv(A) : createZeroMatrix3();
  const IBi = B && B.invM > 0 ? transformToWorldInertiaInv(B) : createZeroMatrix3();
  const rAxn = crossProduct(rA, n);
  const rBxn = B ? crossProduct(rB, n) : [0,0,0];
  const wA = A.invM + (A.invM > 0 ? dotProduct(rAxn, multiplyMatrix3Vector(IAi, rAxn)) : 0);
  const wB = B ? (B.invM + (B.invM > 0 ? dotProduct(rBxn, multiplyMatrix3Vector(IBi, rBxn)) : 0)) : 0;
  const wEff = wA + wB;
  if (wEff < 1e-12) return 0;
  const dLambda = (-c - alphaT * state.lambda) / (wEff + alphaT);
  state.lambda += dLambda;
  const P = scaleVector(n, dLambda);
  if (A.invM > 0) {
    A.x = addVectors(A.x, scaleVector(P, A.invM));
    const dqAvec = multiplyMatrix3Vector(IAi, crossProduct(rA, P));
    const inc = [0, dqAvec[0]*0.5, dqAvec[1]*0.5, dqAvec[2]*0.5];
    const qInc = multiplyQuaternions(inc, A.q);
    A.q = normalizeQuaternion([A.q[0]+qInc[0], A.q[1]+qInc[1], A.q[2]+qInc[2], A.q[3]+qInc[3]]);
  }
  if (B && B.invM > 0) {
    B.x = subtractVectors(B.x, scaleVector(P, B.invM));
    const dqBvec = multiplyMatrix3Vector(IBi, crossProduct(rB, P));
    const inc = [0, -dqBvec[0]*0.5, -dqBvec[1]*0.5, -dqBvec[2]*0.5];
    const qInc = multiplyQuaternions(inc, B.q);
    B.q = normalizeQuaternion([B.q[0]+qInc[0], B.q[1]+qInc[1], B.q[2]+qInc[2], B.q[3]+qInc[3]]);
  }
  return dLambda;
}

function applyContactNormalXPBD(A, B, contact, dt, alpha) {
  const alphaT = alpha / (dt * dt);
  const rA = rotateVectorByQuat(A.q, contact.rAloc);
  const rB = B ? rotateVectorByQuat(B.q, contact.rBloc) : [0,0,0];
  const pA = addVectors(A.x, rA);
  const pB = B ? addVectors(B.x, rB) : contact.worldB;
  const n = contact.n;
  const slop = 0.005;
  const C = dotProduct(subtractVectors(pA, pB), n);
  if (C <= slop) return 0;
  const Ceff = C - slop;

  const IAi = A.invM > 0 ? transformToWorldInertiaInv(A) : createZeroMatrix3();
  const IBi = B && B.invM > 0 ? transformToWorldInertiaInv(B) : createZeroMatrix3();
  const rAxn = crossProduct(rA, n);
  const rBxn = B ? crossProduct(rB, n) : [0,0,0];
  const wA = A.invM + (A.invM > 0 ? dotProduct(rAxn, multiplyMatrix3Vector(IAi, rAxn)) : 0);
  const wB = B ? (B.invM + (B.invM > 0 ? dotProduct(rBxn, multiplyMatrix3Vector(IBi, rBxn)) : 0)) : 0;
  const wEff = wA + wB;
  if (wEff < 1e-12) return 0;

  const lambdaOld = contact.lambdaN || 0;
  let dLambda = (-C - alphaT * lambdaOld) / (wEff + alphaT);
  const lambdaNew = Math.min(0, lambdaOld + dLambda);
  dLambda = lambdaNew - lambdaOld;
  contact.lambdaN = lambdaNew;
  if (dLambda === 0) return C;

  const P = scaleVector(n, dLambda);
  if (A.invM > 0) {
    A.x = addVectors(A.x, scaleVector(P, A.invM));
    const dqA = multiplyMatrix3Vector(IAi, crossProduct(rA, P));
    const inc = multiplyQuaternions([0, dqA[0]*0.5, dqA[1]*0.5, dqA[2]*0.5], A.q);
    A.q = normalizeQuaternion([A.q[0]+inc[0], A.q[1]+inc[1], A.q[2]+inc[2], A.q[3]+inc[3]]);
  }
  if (B && B.invM > 0) {
    B.x = subtractVectors(B.x, scaleVector(P, B.invM));
    const dqB = multiplyMatrix3Vector(IBi, crossProduct(rB, P));
    const inc = multiplyQuaternions([0, -dqB[0]*0.5, -dqB[1]*0.5, -dqB[2]*0.5], B.q);
    B.q = normalizeQuaternion([B.q[0]+inc[0], B.q[1]+inc[1], B.q[2]+inc[2], B.q[3]+inc[3]]);
  }
  return C;
}

function applyContactFrictionXPBD(A, B, ct, dt, muS, muD) {
  if ((ct.lambdaN || 0) <= 0) return;
  if (!ct._prevPA) return;

  const rA = rotateVectorByQuat(A.q, ct.rAloc);
  const rB = B ? rotateVectorByQuat(B.q, ct.rBloc) : [0,0,0];
  const pA = addVectors(A.x, rA);
  const pB = B ? addVectors(B.x, rB) : ct.worldB;
  const n = ct.n;
  const t1 = getPerpendicularAxisXPBD(n);
  const t2 = crossProduct(n, t1);

  const mu = muD;
  const maxF = mu * Math.abs(ct.lambdaN || 0);

  const IAi = A.invM > 0 ? transformToWorldInertiaInv(A) : createZeroMatrix3();
  const IBi = B && B.invM > 0 ? transformToWorldInertiaInv(B) : createZeroMatrix3();

  for (const t of [t1, t2]) {
    const rAxt = crossProduct(rA, t);
    const rBxt = B ? crossProduct(rB, t) : [0,0,0];
    const wAt = A.invM + (A.invM > 0 ? dotProduct(rAxt, multiplyMatrix3Vector(IAi, rAxt)) : 0);
    const wBt = B ? (B.invM + (B.invM > 0 ? dotProduct(rBxt, multiplyMatrix3Vector(IBi, rBxt)) : 0)) : 0;
    const wEff = wAt + wBt;
    if (wEff < 1e-12) continue;

    const vA = addVectors(A.v, crossProduct(A.w, rA));
    const vB = B ? addVectors(B.v, crossProduct(B.w, rB)) : [0,0,0];
    const vRel = dotProduct(subtractVectors(vB, vA), t);

    const key = (t === t1) ? 'lambdaT1' : 'lambdaT2';
    const oldL = ct[key] || 0;
    let dLambda = -vRel / wEff;
    let newL = oldL + dLambda;
    if (newL >  maxF) newL =  maxF;
    if (newL < -maxF) newL = -maxF;
    dLambda = newL - oldL;
    ct[key] = newL;

    const P = scaleVector(t, dLambda);
    if (A.invM > 0) {
      A.x = addVectors(A.x, scaleVector(P, A.invM));
      const dq = multiplyMatrix3Vector(IAi, crossProduct(rA, P));
      const inc = multiplyQuaternions([0, dq[0]*0.5, dq[1]*0.5, dq[2]*0.5], A.q);
      A.q = normalizeQuaternion([A.q[0]+inc[0], A.q[1]+inc[1], A.q[2]+inc[2], A.q[3]+inc[3]]);
    }
    if (B && B.invM > 0) {
      B.x = subtractVectors(B.x, scaleVector(P, B.invM));
      const dq = multiplyMatrix3Vector(IBi, crossProduct(rB, P));
      const inc = multiplyQuaternions([0, -dq[0]*0.5, -dq[1]*0.5, -dq[2]*0.5], B.q);
      B.q = normalizeQuaternion([B.q[0]+inc[0], B.q[1]+inc[1], B.q[2]+inc[2], B.q[3]+inc[3]]);
    }
  }
}

function getPerpendicularAxisXPBD(n) {
  const ax = Math.abs(n[0]), ay = Math.abs(n[1]), az = Math.abs(n[2]);
  let v;
  if (ax <= ay && ax <= az) v = [1, 0, 0];
  else if (ay <= az)        v = [0, 1, 0];
  else                       v = [0, 0, 1];
  const t = normalizeVector(subtractVectors(v, scaleVector(n, dotProduct(v, n))));
  return t;
}
