function freeRotStep(body, dt, mode) {
  if (body.invM === 0) return;

  if (mode === '1A') _rot1A(body, dt);
  else if (mode === '1B') _rot1B(body, dt);
  else if (mode === '1C') _rot1C(body, dt);
  else _rot1D(body, dt);

  body.x[0] += body.v[0] * dt;
  body.x[1] += body.v[1] * dt;
  body.x[2] += body.v[2] * dt;
}

function _rot1A(body, dt) {
  if (!body._L) body._L = bodyAngularMomentum(body);
  const Iw_inv = worldInertiaInv(body);
  const w = mat3MulVec(Iw_inv, body._L);
  body.w = w;
  body.q = quatIntegrate(body.q, w, dt);
}

function _ensureWBody(body) {
  if (!body.wBody) {
    const R = quatToMat3(body.q);
    const Rt = mat3Transpose(R);
    body.wBody = mat3MulVec(Rt, body.w);
  }
}

function _rot1B(body, dt) {
  _ensureWBody(body);
  const R = quatToMat3(body.q);
  const w_world = mat3MulVec(R, body.wBody);
  body.w = w_world;
  body.q = quatIntegrate(body.q, w_world, dt);
  const R_new = quatToMat3(body.q);
  body.w = mat3MulVec(R_new, body.wBody);
}

function _rot1C(body, dt) {
  _ensureWBody(body);
  const Ib = body.Ibody, IbInv = body.IbodyInv;
  const w_body = body.wBody;
  const Iw_b = [Ib[0]*w_body[0], Ib[1]*w_body[1], Ib[2]*w_body[2]];
  const gyro = vCross(w_body, Iw_b);
  body.wBody = [
    w_body[0] - dt * IbInv[0] * gyro[0],
    w_body[1] - dt * IbInv[1] * gyro[1],
    w_body[2] - dt * IbInv[2] * gyro[2],
  ];
  const R = quatToMat3(body.q);
  const w_world = mat3MulVec(R, body.wBody);
  body.w = w_world;
  body.q = quatIntegrate(body.q, w_world, dt);
  const R_new = quatToMat3(body.q);
  body.w = mat3MulVec(R_new, body.wBody);
}

function _rot1D(body, dt) {
  _ensureWBody(body);
  const Ib = body.Ibody;
  const I_diag = [Ib[0],0,0, 0,Ib[1],0, 0,0,Ib[2]];
  const w0 = body.wBody;
  const Iw0 = [Ib[0]*w0[0], Ib[1]*w0[1], Ib[2]*w0[2]];

  let wp = w0.slice();
  for (let it = 0; it < 4; it++) {
    const Iwp = [Ib[0]*wp[0], Ib[1]*wp[1], Ib[2]*wp[2]];
    const cross = vCross(wp, Iwp);
    const f = [
      Iwp[0] - Iw0[0] + dt * cross[0],
      Iwp[1] - Iw0[1] + dt * cross[1],
      Iwp[2] - Iw0[2] + dt * cross[2],
    ];
    const W = mat3Cross(wp);
    const IW = mat3Cross(Iwp);
    const WI = mat3Mul(W, I_diag);
    const J = mat3Add(I_diag, mat3Scale(mat3Sub(WI, IW), dt));
    const dWp = mat3MulVec(mat3Inverse(J), [-f[0], -f[1], -f[2]]);
    wp = vAdd(wp, dWp);
    if (vDot(dWp, dWp) < 1e-18) break;
  }
  body.wBody = wp;
  const R = quatToMat3(body.q);
  const w_world = mat3MulVec(R, wp);
  body.w = w_world;
  body.q = quatIntegrate(body.q, w_world, dt);
  const R_new = quatToMat3(body.q);
  body.w = mat3MulVec(R_new, body.wBody);
}

function integrateAngularImplicit(body, torqueWorld, dt) {
  if (body.invM === 0) return;
  const R = quatToMat3(body.q);
  const Rt = mat3Transpose(R);
  const w_body = mat3MulVec(Rt, body.w);
  const tau_body = mat3MulVec(Rt, torqueWorld);
  const Ib = body.Ibody;
  const I_diag = [Ib[0],0,0, 0,Ib[1],0, 0,0,Ib[2]];
  const Iw0 = [Ib[0]*w_body[0], Ib[1]*w_body[1], Ib[2]*w_body[2]];
  const rhs0 = [
    Iw0[0] + dt * tau_body[0],
    Iw0[1] + dt * tau_body[1],
    Iw0[2] + dt * tau_body[2],
  ];

  let wp = w_body.slice();
  for (let it = 0; it < 3; it++) {
    const Iwp = [Ib[0]*wp[0], Ib[1]*wp[1], Ib[2]*wp[2]];
    const cross = vCross(wp, Iwp);
    const f = [
      Iwp[0] + dt * cross[0] - rhs0[0],
      Iwp[1] + dt * cross[1] - rhs0[1],
      Iwp[2] + dt * cross[2] - rhs0[2],
    ];
    const W = mat3Cross(wp);
    const IW = mat3Cross(Iwp);
    const WI = mat3Mul(W, I_diag);
    const J = mat3Add(I_diag, mat3Scale(mat3Sub(WI, IW), dt));
    const dWp = mat3MulVec(mat3Inverse(J), [-f[0], -f[1], -f[2]]);
    wp = vAdd(wp, dWp);
    if (vDot(dWp, dWp) < 1e-18) break;
  }

  body.w = mat3MulVec(R, wp);
}
