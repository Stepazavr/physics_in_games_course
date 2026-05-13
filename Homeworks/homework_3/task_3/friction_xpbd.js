function xpbdContactFriction(A, B, ct, dt, muS, muD) {
  if ((ct.lambdaN || 0) <= 0) return;
  if (!ct._prevPA) return;

  const rA = quatRotate(A.q, ct.rAloc);
  const rB = B ? quatRotate(B.q, ct.rBloc) : [0,0,0];
  const pA = vAdd(A.x, rA);
  const pB = B ? vAdd(B.x, rB) : ct.worldB;

  const dpA = vSub(pA, ct._prevPA);
  const dpB = B ? vSub(pB, ct._prevPB) : [0,0,0];
  const dp  = vSub(dpA, dpB);
  const dpN = vDot(dp, ct.n);
  const dpT = vSub(dp, vMul(ct.n, dpN));
  const dpTlen = vLen(dpT);
  if (dpTlen < 1e-9) return;

  const d = Math.max(0, ct.depth);
  const limitS = muS * d;
  const limitD = muD * d;

  let corr;
  if (dpTlen < limitS) {
    corr = vMul(dpT, -1);
  } else {
    corr = vMul(dpT, -limitD / dpTlen);
  }

  const cLen = vLen(corr);
  if (cLen < 1e-9) return;
  const nDir = vMul(corr, 1 / cLen);

  const IAi = worldInertiaInv(A);
  const IBi = B ? worldInertiaInv(B) : mat3Zero();
  const rAxn = vCross(rA, nDir);
  const rBxn = B ? vCross(rB, nDir) : [0,0,0];
  const wA = A.invM + (A.invM > 0 ? vDot(rAxn, mat3MulVec(IAi, rAxn)) : 0);
  const wB = B ? (B.invM + (B.invM > 0 ? vDot(rBxn, mat3MulVec(IBi, rBxn)) : 0)) : 0;
  const wEff = wA + wB;
  if (wEff < 1e-12) return;

  const dLambda = cLen / wEff;
  const P = vMul(nDir, dLambda);
  if (A.invM > 0) {
    A.x = vAdd(A.x, vMul(P, A.invM));
    const dq = mat3MulVec(IAi, vCross(rA, P));
    A.q = quatNormalize([
      A.q[0] + 0.5*( -dq[0]*A.q[1] - dq[1]*A.q[2] - dq[2]*A.q[3]),
      A.q[1] + 0.5*(  dq[0]*A.q[0] + dq[1]*A.q[3] - dq[2]*A.q[2]),
      A.q[2] + 0.5*( -dq[0]*A.q[3] + dq[1]*A.q[0] + dq[2]*A.q[1]),
      A.q[3] + 0.5*(  dq[0]*A.q[2] - dq[1]*A.q[1] + dq[2]*A.q[0]),
    ]);
  }
  if (B && B.invM > 0) {
    B.x = vSub(B.x, vMul(P, B.invM));
    const dq = mat3MulVec(IBi, vCross(rB, vMul(P, -1)));
    B.q = quatNormalize([
      B.q[0] + 0.5*( -dq[0]*B.q[1] - dq[1]*B.q[2] - dq[2]*B.q[3]),
      B.q[1] + 0.5*(  dq[0]*B.q[0] + dq[1]*B.q[3] - dq[2]*B.q[2]),
      B.q[2] + 0.5*( -dq[0]*B.q[3] + dq[1]*B.q[0] + dq[2]*B.q[1]),
      B.q[3] + 0.5*(  dq[0]*B.q[2] - dq[1]*B.q[1] + dq[2]*B.q[0]),
    ]);
  }
}
