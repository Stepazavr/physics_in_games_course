function applySpringForce(body, spring, dt) {
  if (body.invM === 0) return;
  const r_world_off = quatRotate(body.q, spring.rLocal);
  const p = vAdd(body.x, r_world_off);
  const d = vSub(p, spring.pWorld);
  const dLen = vLen(d);
  if (dLen < 1e-9) return;
  const nDir = vMul(d, 1 / dLen);

  const vAttach = vAdd(body.v, vCross(body.w, r_world_off));
  const Fspring = vMul(nDir, -spring.k * (dLen - spring.restLen));
  const Fdamp = vMul(vAttach, -spring.c);
  const F = vAdd(Fspring, Fdamp);

  body.v[0] += dt * body.invM * F[0];
  body.v[1] += dt * body.invM * F[1];
  body.v[2] += dt * body.invM * F[2];

  const tau = vCross(r_world_off, F);
  const Iw_inv = worldInertiaInv(body);
  const dw = mat3MulVec(Iw_inv, vMul(tau, dt));
  body.w[0] += dw[0];
  body.w[1] += dw[1];
  body.w[2] += dw[2];
}

function solveSpringSoft(body, spring, dt) {
  if (body.invM === 0) return;
  const r_world_off = quatRotate(body.q, spring.rLocal);
  const p = vAdd(body.x, r_world_off);
  const d = vSub(p, spring.pWorld);
  const dLen = vLen(d);
  if (dLen < 1e-9) return;
  const n = vMul(d, 1 / dLen);
  const C = dLen - spring.restLen;

  const k = spring.k, c = spring.c;
  const gamma = 1 / (dt * (c + dt * k));
  const beta = dt * k / (c + dt * k);

  const Iw_inv = worldInertiaInv(body);
  const rxn = vCross(r_world_off, n);
  const Iinv_rxn = mat3MulVec(Iw_inv, rxn);
  const K = body.invM + vDot(rxn, Iinv_rxn) + gamma;

  const vAttach = vAdd(body.v, vCross(body.w, r_world_off));
  const Cdot = vDot(vAttach, n);

  if (spring.lambdaAccum == null) spring.lambdaAccum = 0;
  const lambda = -(Cdot + (beta / dt) * C + gamma * spring.lambdaAccum) / K;
  spring.lambdaAccum += lambda;

  const P = vMul(n, lambda);
  body.v[0] += body.invM * P[0];
  body.v[1] += body.invM * P[1];
  body.v[2] += body.invM * P[2];
  const dw = mat3MulVec(Iw_inv, vCross(r_world_off, P));
  body.w[0] += dw[0];
  body.w[1] += dw[1];
  body.w[2] += dw[2];
}
