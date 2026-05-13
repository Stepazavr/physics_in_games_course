function xpbdDistanceConstraint(A, B, rAloc, rBloc, restLen, alpha, dt, state) {
  const alphaT = alpha / (dt * dt);
  const rA = quatRotate(A.q, rAloc);
  const rB = quatRotate(B.q, rBloc);
  const pA = vAdd(A.x, rA);
  const pB = vAdd(B.x, rB);
  const dvec = vSub(pA, pB);
  const dLen = vLen(dvec);
  if (dLen < 1e-9) return 0;
  const n = vMul(dvec, 1 / dLen);
  const C = dLen - restLen;

  const IAi = worldInertiaInv(A);
  const IBi = worldInertiaInv(B);
  const rAxn = vCross(rA, n);
  const rBxn = vCross(rB, n);
  const wA = A.invM + vDot(rAxn, mat3MulVec(IAi, rAxn));
  const wB = B.invM + vDot(rBxn, mat3MulVec(IBi, rBxn));
  const wEff = wA + wB;
  if (wEff < 1e-12) return Math.abs(C);

  if (state.lambda == null) state.lambda = 0;
  const dLambda = (-C - alphaT * state.lambda) / (wEff + alphaT);
  state.lambda += dLambda;

  const P = vMul(n, dLambda);
  if (A.invM > 0) {
    A.x = vAdd(A.x, vMul(P, A.invM));
    const dqAvec = mat3MulVec(IAi, vCross(rA, P));
    A.q = quatNormalize(quatMul([0, dqAvec[0]*0.5, dqAvec[1]*0.5, dqAvec[2]*0.5], A.q).map((c, i) => c + A.q[i]));
  }
  if (B.invM > 0) {
    B.x = vSub(B.x, vMul(P, B.invM));
    const dqBvec = mat3MulVec(IBi, vCross(rB, P));
    const inc = quatMul([0, -dqBvec[0]*0.5, -dqBvec[1]*0.5, -dqBvec[2]*0.5], B.q);
    B.q = quatNormalize([B.q[0]+inc[0], B.q[1]+inc[1], B.q[2]+inc[2], B.q[3]+inc[3]]);
  }

  return Math.abs(C);
}

function xpbdApplyPositionCorrection(A, B, rA, rB, n, c, alphaT, state) {
  const IAi = A.invM > 0 ? worldInertiaInv(A) : mat3Zero();
  const IBi = B && B.invM > 0 ? worldInertiaInv(B) : mat3Zero();
  const rAxn = vCross(rA, n);
  const rBxn = B ? vCross(rB, n) : [0,0,0];
  const wA = A.invM + (A.invM > 0 ? vDot(rAxn, mat3MulVec(IAi, rAxn)) : 0);
  const wB = B ? (B.invM + (B.invM > 0 ? vDot(rBxn, mat3MulVec(IBi, rBxn)) : 0)) : 0;
  const wEff = wA + wB;
  if (wEff < 1e-12) return 0;
  const dLambda = (-c - alphaT * state.lambda) / (wEff + alphaT);
  state.lambda += dLambda;
  const P = vMul(n, dLambda);
  if (A.invM > 0) {
    A.x = vAdd(A.x, vMul(P, A.invM));
    const dqAvec = mat3MulVec(IAi, vCross(rA, P));
    const inc = [0, dqAvec[0]*0.5, dqAvec[1]*0.5, dqAvec[2]*0.5];
    const qInc = quatMul(inc, A.q);
    A.q = quatNormalize([A.q[0]+qInc[0], A.q[1]+qInc[1], A.q[2]+qInc[2], A.q[3]+qInc[3]]);
  }
  if (B && B.invM > 0) {
    B.x = vSub(B.x, vMul(P, B.invM));
    const dqBvec = mat3MulVec(IBi, vCross(rB, P));
    const inc = [0, -dqBvec[0]*0.5, -dqBvec[1]*0.5, -dqBvec[2]*0.5];
    const qInc = quatMul(inc, B.q);
    B.q = quatNormalize([B.q[0]+qInc[0], B.q[1]+qInc[1], B.q[2]+qInc[2], B.q[3]+qInc[3]]);
  }
  return dLambda;
}

function xpbdContactNormal(A, B, contact, dt, alpha) {
  const alphaT = alpha / (dt * dt);
  const rA = quatRotate(A.q, contact.rAloc);
  const rB = B ? quatRotate(B.q, contact.rBloc) : [0,0,0];
  const pA = vAdd(A.x, rA);
  const pB = B ? vAdd(B.x, rB) : contact.worldB;
  const n = contact.n;
  const slop = 0.005;
  const C = vDot(vSub(pA, pB), n);
  if (C <= slop) return 0;
  const Ceff = C - slop;

  const IAi = A.invM > 0 ? worldInertiaInv(A) : mat3Zero();
  const IBi = B && B.invM > 0 ? worldInertiaInv(B) : mat3Zero();
  const rAxn = vCross(rA, n);
  const rBxn = B ? vCross(rB, n) : [0,0,0];
  const wA = A.invM + (A.invM > 0 ? vDot(rAxn, mat3MulVec(IAi, rAxn)) : 0);
  const wB = B ? (B.invM + (B.invM > 0 ? vDot(rBxn, mat3MulVec(IBi, rBxn)) : 0)) : 0;
  const wEff = wA + wB;
  if (wEff < 1e-12) return 0;

  const lambdaOld = contact.lambdaN || 0;
  let dLambda = (-C - alphaT * lambdaOld) / (wEff + alphaT);
  const lambdaNew = Math.min(0, lambdaOld + dLambda);
  dLambda = lambdaNew - lambdaOld;
  contact.lambdaN = lambdaNew;
  if (dLambda === 0) return C;

  const P = vMul(n, dLambda);
  if (A.invM > 0) {
    A.x = vAdd(A.x, vMul(P, A.invM));
    const dqA = mat3MulVec(IAi, vCross(rA, P));
    const inc = quatMul([0, dqA[0]*0.5, dqA[1]*0.5, dqA[2]*0.5], A.q);
    A.q = quatNormalize([A.q[0]+inc[0], A.q[1]+inc[1], A.q[2]+inc[2], A.q[3]+inc[3]]);
  }
  if (B && B.invM > 0) {
    B.x = vSub(B.x, vMul(P, B.invM));
    const dqB = mat3MulVec(IBi, vCross(rB, P));
    const inc = quatMul([0, -dqB[0]*0.5, -dqB[1]*0.5, -dqB[2]*0.5], B.q);
    B.q = quatNormalize([B.q[0]+inc[0], B.q[1]+inc[1], B.q[2]+inc[2], B.q[3]+inc[3]]);
  }
  return C;
}

function xpbdContactFriction(A, B, ct, dt, muS, muD) {
  if ((ct.lambdaN || 0) <= 0) return;
  if (!ct._prevPA) return;

  const rA = quatRotate(A.q, ct.rAloc);
  const rB = B ? quatRotate(B.q, ct.rBloc) : [0,0,0];
  const pA = vAdd(A.x, rA);
  const pB = B ? vAdd(B.x, rB) : ct.worldB;
  const n = ct.n;
  const t1 = _xpbdPerpAxis(n);
  const t2 = vCross(n, t1);

  const mu = muD;
  const maxF = mu * Math.abs(ct.lambdaN || 0);

  const IAi = A.invM > 0 ? worldInertiaInv(A) : mat3Zero();
  const IBi = B && B.invM > 0 ? worldInertiaInv(B) : mat3Zero();

  for (const t of [t1, t2]) {
    const rAxt = vCross(rA, t);
    const rBxt = B ? vCross(rB, t) : [0,0,0];
    const wAt = A.invM + (A.invM > 0 ? vDot(rAxt, mat3MulVec(IAi, rAxt)) : 0);
    const wBt = B ? (B.invM + (B.invM > 0 ? vDot(rBxt, mat3MulVec(IBi, rBxt)) : 0)) : 0;
    const wEff = wAt + wBt;
    if (wEff < 1e-12) continue;

    const vA = vAdd(A.v, vCross(A.w, rA));
    const vB = B ? vAdd(B.v, vCross(B.w, rB)) : [0,0,0];
    const vRel = vDot(vSub(vB, vA), t);

    const key = (t === t1) ? 'lambdaT1' : 'lambdaT2';
    const oldL = ct[key] || 0;
    let dLambda = -vRel / wEff;
    let newL = oldL + dLambda;
    if (newL >  maxF) newL =  maxF;
    if (newL < -maxF) newL = -maxF;
    dLambda = newL - oldL;
    ct[key] = newL;

    const P = vMul(t, dLambda);
    if (A.invM > 0) {
      A.x = vAdd(A.x, vMul(P, A.invM));
      const dq = mat3MulVec(IAi, vCross(rA, P));
      const inc = quatMul([0, dq[0]*0.5, dq[1]*0.5, dq[2]*0.5], A.q);
      A.q = quatNormalize([A.q[0]+inc[0], A.q[1]+inc[1], A.q[2]+inc[2], A.q[3]+inc[3]]);
    }
    if (B && B.invM > 0) {
      B.x = vSub(B.x, vMul(P, B.invM));
      const dq = mat3MulVec(IBi, vCross(rB, P));
      const inc = quatMul([0, -dq[0]*0.5, -dq[1]*0.5, -dq[2]*0.5], B.q);
      B.q = quatNormalize([B.q[0]+inc[0], B.q[1]+inc[1], B.q[2]+inc[2], B.q[3]+inc[3]]);
    }
  }
}

function _xpbdPerpAxis(n) {
  const ax = Math.abs(n[0]), ay = Math.abs(n[1]), az = Math.abs(n[2]);
  let v;
  if (ax <= ay && ax <= az) v = [1, 0, 0];
  else if (ay <= az)        v = [0, 1, 0];
  else                       v = [0, 0, 1];
  const t = vNorm(vSub(v, vMul(n, vDot(v, n))));
  return t;
}
