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
