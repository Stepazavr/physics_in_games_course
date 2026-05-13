function siDistanceConstraint(A, B, c, mode, params, dt) {
  const rA = quatRotate(A.q, c.rAloc);
  const rB = quatRotate(B.q, c.rBloc);
  const pA = vAdd(A.x, rA);
  const pB = vAdd(B.x, rB);
  const dvec = vSub(pA, pB);
  const dLen = vLen(dvec);
  if (dLen < 1e-9) return 0;
  const n = vMul(dvec, 1 / dLen);
  const C = dLen - c.restLen;

  const IAi = worldInertiaInv(A);
  const IBi = worldInertiaInv(B);
  const rAxn = vCross(rA, n);
  const rBxn = vCross(rB, n);
  const wA = A.invM + vDot(rAxn, mat3MulVec(IAi, rAxn));
  const wB = B.invM + vDot(rBxn, mat3MulVec(IBi, rBxn));
  let K = wA + wB;
  if (K < 1e-12) return Math.abs(C);

  const vA_pt = vAdd(A.v, vCross(A.w, rA));
  const vB_pt = vAdd(B.v, vCross(B.w, rB));
  const Cdot = vDot(vSub(vA_pt, vB_pt), n);

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

  const P = vMul(n, lambda);
  _applyImpulse(A,  P, rA, IAi);
  _applyImpulse(B, vMul(P, -1), rB, IBi);

  return Math.abs(C);
}

function siDistancePosPass(A, B, c) {
  const rA = quatRotate(A.q, c.rAloc);
  const rB = quatRotate(B.q, c.rBloc);
  const pA = vAdd(A.x, rA);
  const pB = vAdd(B.x, rB);
  const dvec = vSub(pA, pB);
  const dLen = vLen(dvec);
  if (dLen < 1e-9) return 0;
  const n = vMul(dvec, 1 / dLen);
  const C = dLen - c.restLen;

  const IAi = worldInertiaInv(A);
  const IBi = worldInertiaInv(B);
  const rAxn = vCross(rA, n);
  const rBxn = vCross(rB, n);
  const wA = A.invM + vDot(rAxn, mat3MulVec(IAi, rAxn));
  const wB = B.invM + vDot(rBxn, mat3MulVec(IBi, rBxn));
  const wEff = wA + wB;
  if (wEff < 1e-12) return Math.abs(C);

  const dLambda = -C / wEff;
  const P = vMul(n, dLambda);
  if (A.invM > 0) {
    A.x = vAdd(A.x, vMul(P, A.invM));
    const dq = mat3MulVec(IAi, vCross(rA, P));
    _applyQuatDelta(A, dq);
  }
  if (B.invM > 0) {
    B.x = vSub(B.x, vMul(P, B.invM));
    const dq = mat3MulVec(IBi, vCross(rB, vMul(P, -1)));
    _applyQuatDelta(B, dq);
  }
  return Math.abs(C);
}

function siContactNormal(A, B, ct, dt, params, mode) {
  const IAi = worldInertiaInv(A);
  const IBi = B ? worldInertiaInv(B) : mat3Zero();
  const rA = ct.rA, rB = ct.rB || [0,0,0];
  const n = ct.n;
  const rAxn = vCross(rA, n);
  const rBxn = vCross(rB, n);
  const wA = A.invM + vDot(rAxn, mat3MulVec(IAi, rAxn));
  const wB = B ? (B.invM + vDot(rBxn, mat3MulVec(IBi, rBxn))) : 0;
  const K = wA + wB;
  if (K < 1e-12) return;

  const vA_pt = vAdd(A.v, vCross(A.w, rA));
  const vB_pt = B ? vAdd(B.v, vCross(B.w, rB)) : [0,0,0];
  const vRel = vDot(vSub(vB_pt, vA_pt), n);

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

  const P = vMul(n, dLambda);
  _applyImpulse(A, vMul(P, -1), rA, IAi);
  if (B) _applyImpulse(B, P, rB, IBi);
}

function siContactFriction(A, B, ct, params) {
  const IAi = worldInertiaInv(A);
  const IBi = B ? worldInertiaInv(B) : mat3Zero();
  const rA = ct.rA, rB = ct.rB || [0,0,0];
  const n = ct.n;
  const t1 = _perpAxis(n);
  const t2 = vCross(n, t1);

  const mu = params.mu;
  const maxF = mu * (ct.lambdaN || 0);
  for (const t of [t1, t2]) {
    const rAxt = vCross(rA, t);
    const rBxt = vCross(rB, t);
    const wAt = A.invM + vDot(rAxt, mat3MulVec(IAi, rAxt));
    const wBt = B ? (B.invM + vDot(rBxt, mat3MulVec(IBi, rBxt))) : 0;
    const K = wAt + wBt;
    if (K < 1e-12) continue;

    const vA_pt = vAdd(A.v, vCross(A.w, rA));
    const vB_pt = B ? vAdd(B.v, vCross(B.w, rB)) : [0,0,0];
    const vRel = vDot(vSub(vB_pt, vA_pt), t);

    let dLambda = -vRel / K;
    const key = (t === t1) ? 'lambdaT1' : 'lambdaT2';
    const old = ct[key] || 0;
    let cur = old + dLambda;
    if (cur >  maxF) cur =  maxF;
    if (cur < -maxF) cur = -maxF;
    dLambda = cur - old;
    ct[key] = cur;

    const P = vMul(t, dLambda);
    _applyImpulse(A, vMul(P, -1), rA, IAi);
    if (B) _applyImpulse(B, P, rB, IBi);
  }
}

function siContactPosPass(A, B, ct) {
  const pA = vAdd(A.x, quatRotate(A.q, ct.rAloc));
  const pB = B ? vAdd(B.x, quatRotate(B.q, ct.rBloc)) : ct.worldB;
  const sep = vDot(vSub(pA, pB), ct.n);
  if (sep <= 0) return;

  const rA = vSub(pA, A.x);
  const rB = B ? vSub(pB, B.x) : [0,0,0];
  const IAi = worldInertiaInv(A);
  const IBi = B ? worldInertiaInv(B) : mat3Zero();
  const rAxn = vCross(rA, ct.n);
  const rBxn = vCross(rB, ct.n);
  const wA = A.invM + vDot(rAxn, mat3MulVec(IAi, rAxn));
  const wB = B ? (B.invM + vDot(rBxn, mat3MulVec(IBi, rBxn))) : 0;
  const wEff = wA + wB;
  if (wEff < 1e-12) return;

  const slop = 0.002;
  const corr = Math.max(0, sep - slop);
  const dLambda = corr / wEff;
  const P = vMul(ct.n, dLambda);
  if (A.invM > 0) {
    A.x = vSub(A.x, vMul(P, A.invM));
    _applyQuatDelta(A, mat3MulVec(IAi, vCross(rA, vMul(P, -1))));
  }
  if (B && B.invM > 0) {
    B.x = vAdd(B.x, vMul(P, B.invM));
    _applyQuatDelta(B, mat3MulVec(IBi, vCross(rB, P)));
  }
}

function _applyImpulse(b, P, r, Iinv) {
  if (b.invM === 0) return;
  b.v[0] += b.invM * P[0];
  b.v[1] += b.invM * P[1];
  b.v[2] += b.invM * P[2];
  const dw = mat3MulVec(Iinv, vCross(r, P));
  b.w[0] += dw[0];
  b.w[1] += dw[1];
  b.w[2] += dw[2];
}

function _applyQuatDelta(b, dq_vec) {
  const inc = quatMul([0, dq_vec[0]*0.5, dq_vec[1]*0.5, dq_vec[2]*0.5], b.q);
  b.q = quatNormalize([b.q[0]+inc[0], b.q[1]+inc[1], b.q[2]+inc[2], b.q[3]+inc[3]]);
}

function _perpAxis(n) {
  const ax = Math.abs(n[0]), ay = Math.abs(n[1]), az = Math.abs(n[2]);
  let v;
  if (ax <= ay && ax <= az) v = [1, 0, 0];
  else if (ay <= az)        v = [0, 1, 0];
  else                       v = [0, 0, 1];
  const t = vNorm(vSub(v, vMul(n, vDot(v, n))));
  return t;
}
