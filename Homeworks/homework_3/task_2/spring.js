function applySpringForce(figure, spring, dt) {
  if (figure.invM === 0) return;
  const r_world_off = quatRotate(figure.q, spring.rLocal);
  const p = vAdd(figure.x, r_world_off);
  const d = vSub(p, spring.pWorld);
  const dLen = vLen(d);
  if (dLen < 1e-9) return;
  const nDir = vMul(d, 1 / dLen);

  const vAttach = vAdd(figure.v, vCross(figure.w, r_world_off));
  const Fspring = vMul(nDir, -spring.k * (dLen - spring.restLen));
  const Fdamp = vMul(vAttach, -spring.c);
  const F = vAdd(Fspring, Fdamp);

  figure.v[0] += dt * figure.invM * F[0];
  figure.v[1] += dt * figure.invM * F[1];
  figure.v[2] += dt * figure.invM * F[2];

  const tau = vCross(r_world_off, F);
  const Iw_inv = worldInertiaInv(figure);
  const dw = mat3MulVec(Iw_inv, vMul(tau, dt));
  figure.w[0] += dw[0];
  figure.w[1] += dw[1];
  figure.w[2] += dw[2];
}

function solveSpringSoft(figure, spring, dt) {
  if (figure.invM === 0) return;
  const r_world_off = quatRotate(figure.q, spring.rLocal);
  const p = vAdd(figure.x, r_world_off);
  const d = vSub(p, spring.pWorld);
  const dLen = vLen(d);
  if (dLen < 1e-9) return;
  const n = vMul(d, 1 / dLen);
  const C = dLen - spring.restLen;

  const k = spring.k, c = spring.c;
  const gamma = 1 / (dt * (c + dt * k));
  const beta = dt * k / (c + dt * k);

  const Iw_inv = worldInertiaInv(figure);
  const rxn = vCross(r_world_off, n);
  const Iinv_rxn = mat3MulVec(Iw_inv, rxn);
  const K = figure.invM + vDot(rxn, Iinv_rxn) + gamma;

  const vAttach = vAdd(figure.v, vCross(figure.w, r_world_off));
  const Cdot = vDot(vAttach, n);

  if (spring.lambdaAccum == null) spring.lambdaAccum = 0;
  const lambda = -(Cdot + (beta / dt) * C + gamma * spring.lambdaAccum) / K;
  spring.lambdaAccum += lambda;

  const P = vMul(n, lambda);
  figure.v[0] += figure.invM * P[0];
  figure.v[1] += figure.invM * P[1];
  figure.v[2] += figure.invM * P[2];
  const dw = mat3MulVec(Iw_inv, vCross(r_world_off, P));
  figure.w[0] += dw[0];
  figure.w[1] += dw[1];
  figure.w[2] += dw[2];
}
