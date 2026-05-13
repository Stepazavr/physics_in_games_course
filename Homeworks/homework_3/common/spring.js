function applyExplicitSpringForce(figure, spring, dt) {
  if (figure.invM === 0) return;
  const r_world_off = rotateVectorByQuat(figure.q, spring.rLocal);
  const p = addVectors(figure.x, r_world_off);
  const d = subtractVectors(p, spring.pWorld);
  const dLen = vectorLength(d);
  if (dLen < 1e-9) return;
  const nDir = scaleVector(d, 1 / dLen);

  const vAttach = addVectors(figure.v, crossProduct(figure.w, r_world_off));
  const Fspring = scaleVector(nDir, -spring.k * (dLen - spring.restLen));
  const Fdamp = scaleVector(vAttach, -spring.c);
  const F = addVectors(Fspring, Fdamp);

  figure.v[0] += dt * figure.invM * F[0];
  figure.v[1] += dt * figure.invM * F[1];
  figure.v[2] += dt * figure.invM * F[2];

  const tau = crossProduct(r_world_off, F);
  const Iw_inv = transformToWorldInertiaInv(figure);
  const dw = multiplyMatrix3Vector(Iw_inv, scaleVector(tau, dt));
  figure.w[0] += dw[0];
  figure.w[1] += dw[1];
  figure.w[2] += dw[2];
}

function solveSpringConstraintSoft(figure, spring, dt) {
  if (figure.invM === 0) return;
  const r_world_off = rotateVectorByQuat(figure.q, spring.rLocal);
  const p = addVectors(figure.x, r_world_off);
  const d = subtractVectors(p, spring.pWorld);
  const dLen = vectorLength(d);
  if (dLen < 1e-9) return;
  const n = scaleVector(d, 1 / dLen);
  const C = dLen - spring.restLen;

  const k = spring.k, c = spring.c;
  const gamma = 1 / (dt * (c + dt * k));
  const beta = dt * k / (c + dt * k);

  const Iw_inv = transformToWorldInertiaInv(figure);
  const rxn = crossProduct(r_world_off, n);
  const Iinv_rxn = multiplyMatrix3Vector(Iw_inv, rxn);
  const K = figure.invM + dotProduct(rxn, Iinv_rxn) + gamma;

  const vAttach = addVectors(figure.v, crossProduct(figure.w, r_world_off));
  const Cdot = dotProduct(vAttach, n);

  if (spring.lambdaAccum == null) spring.lambdaAccum = 0;
  const lambda = -(Cdot + (beta / dt) * C + gamma * spring.lambdaAccum) / K;
  spring.lambdaAccum += lambda;

  const P = scaleVector(n, lambda);
  figure.v[0] += figure.invM * P[0];
  figure.v[1] += figure.invM * P[1];
  figure.v[2] += figure.invM * P[2];
  const dw = multiplyMatrix3Vector(Iw_inv, crossProduct(r_world_off, P));
  figure.w[0] += dw[0];
  figure.w[1] += dw[1];
  figure.w[2] += dw[2];
}
