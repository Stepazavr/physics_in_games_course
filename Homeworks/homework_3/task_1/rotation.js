function freeRotStep(figure, dt, mode) {
  if (figure.invM === 0) return;

  if (mode === '1A') _rot1A(figure, dt);
  else if (mode === '1B') _rot1B(figure, dt);
  else if (mode === '1C') _rot1C(figure, dt);
  else _rot1D(figure, dt);

  figure.x[0] += figure.v[0] * dt;
  figure.x[1] += figure.v[1] * dt;
  figure.x[2] += figure.v[2] * dt;
}

function _rot1A(figure, dt) {
  if (!figure._L) figure._L = computeAngularMomentum(figure);
  const Iw_inv = worldInertiaInv(figure);
  const w = mat3MulVec(Iw_inv, figure._L);
  figure.w = w;
  figure.q = quatIntegrate(figure.q, w, dt);
}

function _ensureWfigure(figure) {
  if (!figure.wfigure) {
    const R = quatToMat3(figure.q);
    const Rt = mat3Transpose(R);
    figure.wfigure = mat3MulVec(Rt, figure.w);
  }
}

function _rot1B(figure, dt) {
  _ensureWfigure(figure);
  const R = quatToMat3(figure.q);
  const w_world = mat3MulVec(R, figure.wfigure);
  figure.w = w_world;
  figure.q = quatIntegrate(figure.q, w_world, dt);
  const R_new = quatToMat3(figure.q);
  figure.w = mat3MulVec(R_new, figure.wfigure);
}

function _rot1C(figure, dt) {
  _ensureWfigure(figure);
  const Ib = figure.Ifigure, IbInv = figure.IfigureInv;
  const w_figure = figure.wfigure;
  const Iw_b = [Ib[0]*w_figure[0], Ib[1]*w_figure[1], Ib[2]*w_figure[2]];
  const gyro = vCross(w_figure, Iw_b);
  figure.wfigure = [
    w_figure[0] - dt * IbInv[0] * gyro[0],
    w_figure[1] - dt * IbInv[1] * gyro[1],
    w_figure[2] - dt * IbInv[2] * gyro[2],
  ];
  const R = quatToMat3(figure.q);
  const w_world = mat3MulVec(R, figure.wfigure);
  figure.w = w_world;
  figure.q = quatIntegrate(figure.q, w_world, dt);
  const R_new = quatToMat3(figure.q);
  figure.w = mat3MulVec(R_new, figure.wfigure);
}

function _rot1D(figure, dt) {
  _ensureWfigure(figure);
  const Ib = figure.Ifigure;
  const I_diag = [Ib[0],0,0, 0,Ib[1],0, 0,0,Ib[2]];
  const w0 = figure.wfigure;
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
  figure.wfigure = wp;
  const R = quatToMat3(figure.q);
  const w_world = mat3MulVec(R, wp);
  figure.w = w_world;
  figure.q = quatIntegrate(figure.q, w_world, dt);
  const R_new = quatToMat3(figure.q);
  figure.w = mat3MulVec(R_new, figure.wfigure);
}

function integrateAngularImplicit(figure, torqueWorld, dt) {
  if (figure.invM === 0) return;
  const R = quatToMat3(figure.q);
  const Rt = mat3Transpose(R);
  const w_figure = mat3MulVec(Rt, figure.w);
  const tau_figure = mat3MulVec(Rt, torqueWorld);
  const Ib = figure.Ifigure;
  const I_diag = [Ib[0],0,0, 0,Ib[1],0, 0,0,Ib[2]];
  const Iw0 = [Ib[0]*w_figure[0], Ib[1]*w_figure[1], Ib[2]*w_figure[2]];
  const rhs0 = [
    Iw0[0] + dt * tau_figure[0],
    Iw0[1] + dt * tau_figure[1],
    Iw0[2] + dt * tau_figure[2],
  ];

  let wp = w_figure.slice();
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

  figure.w = mat3MulVec(R, wp);
}
