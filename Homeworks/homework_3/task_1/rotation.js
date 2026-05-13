function updateFreeRotationStep(figure, dt, mode) {
  if (figure.invM === 0) return;

  if (mode === '1A') integrateRotationGlobalMomentum(figure, dt);
  else if (mode === '1B') integrateRotationLocalVelocity(figure, dt);
  else if (mode === '1C') integrateRotationExplicitGyro(figure, dt);
  else integrateRotationImplicitGyro(figure, dt);

  figure.x[0] += figure.v[0] * dt;
  figure.x[1] += figure.v[1] * dt;
  figure.x[2] += figure.v[2] * dt;
}

function integrateRotationGlobalMomentum(figure, dt) {
  if (!figure._L) figure._L = computeAngularMomentum(figure);
  const Iw_inv = transformToWorldInertiaInv(figure);
  const w = multiplyMatrix3Vector(Iw_inv, figure._L);
  figure.w = w;
  figure.q = integrateQuaternion(figure.q, w, dt);
}

function ensureLocalAngularVelocity(figure) {
  if (!figure.wfigure) {
    const R = quaternionToMatrix3(figure.q);
    const Rt = transposeMatrix3(R);
    figure.wfigure = multiplyMatrix3Vector(Rt, figure.w);
  }
}

function integrateRotationLocalVelocity(figure, dt) {
  ensureLocalAngularVelocity(figure);
  const R = quaternionToMatrix3(figure.q);
  const w_world = multiplyMatrix3Vector(R, figure.wfigure);
  figure.w = w_world;
  figure.q = integrateQuaternion(figure.q, w_world, dt);
  const R_new = quaternionToMatrix3(figure.q);
  figure.w = multiplyMatrix3Vector(R_new, figure.wfigure);
}

function integrateRotationExplicitGyro(figure, dt) {
  ensureLocalAngularVelocity(figure);
  const Ib = figure.Ifigure, IbInv = figure.IfigureInv;
  const w_figure = figure.wfigure;
  const Iw_b = [Ib[0]*w_figure[0], Ib[1]*w_figure[1], Ib[2]*w_figure[2]];
  const gyro = crossProduct(w_figure, Iw_b);
  figure.wfigure = [
    w_figure[0] - dt * IbInv[0] * gyro[0],
    w_figure[1] - dt * IbInv[1] * gyro[1],
    w_figure[2] - dt * IbInv[2] * gyro[2],
  ];
  const R = quaternionToMatrix3(figure.q);
  const w_world = multiplyMatrix3Vector(R, figure.wfigure);
  figure.w = w_world;
  figure.q = integrateQuaternion(figure.q, w_world, dt);
  const R_new = quaternionToMatrix3(figure.q);
  figure.w = multiplyMatrix3Vector(R_new, figure.wfigure);
}

function integrateRotationImplicitGyro(figure, dt) {
  ensureLocalAngularVelocity(figure);
  const Ib = figure.Ifigure;
  const I_diag = [Ib[0],0,0, 0,Ib[1],0, 0,0,Ib[2]];
  const w0 = figure.wfigure;
  const Iw0 = [Ib[0]*w0[0], Ib[1]*w0[1], Ib[2]*w0[2]];

  let wp = w0.slice();
  for (let it = 0; it < 4; it++) {
    const Iwp = [Ib[0]*wp[0], Ib[1]*wp[1], Ib[2]*wp[2]];
    const cross = crossProduct(wp, Iwp);
    const f = [
      Iwp[0] - Iw0[0] + dt * cross[0],
      Iwp[1] - Iw0[1] + dt * cross[1],
      Iwp[2] - Iw0[2] + dt * cross[2],
    ];
    const W = crossProductMatrix3(wp);
    const IW = crossProductMatrix3(Iwp);
    const WI = multiplyMatrix3Matrix3(W, I_diag);
    const J = addMatrices3(I_diag, scaleMatrix3(subtractMatrices3(WI, IW), dt));
    const dWp = multiplyMatrix3Vector(invertMatrix3(J), [-f[0], -f[1], -f[2]]);
    wp = addVectors(wp, dWp);
    if (dotProduct(dWp, dWp) < 1e-18) break;
  }
  figure.wfigure = wp;
  const R = quaternionToMatrix3(figure.q);
  const w_world = multiplyMatrix3Vector(R, wp);
  figure.w = w_world;
  figure.q = integrateQuaternion(figure.q, w_world, dt);
  const R_new = quaternionToMatrix3(figure.q);
  figure.w = multiplyMatrix3Vector(R_new, figure.wfigure);
}

function implicitAngularIntegration(figure, torqueWorld, dt) {
  if (figure.invM === 0) return;
  const R = quaternionToMatrix3(figure.q);
  const Rt = transposeMatrix3(R);
  const w_figure = multiplyMatrix3Vector(Rt, figure.w);
  const tau_figure = multiplyMatrix3Vector(Rt, torqueWorld);
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
    const cross = crossProduct(wp, Iwp);
    const f = [
      Iwp[0] + dt * cross[0] - rhs0[0],
      Iwp[1] + dt * cross[1] - rhs0[1],
      Iwp[2] + dt * cross[2] - rhs0[2],
    ];
    const W = crossProductMatrix3(wp);
    const IW = crossProductMatrix3(Iwp);
    const WI = multiplyMatrix3Matrix3(W, I_diag);
    const J = addMatrices3(I_diag, scaleMatrix3(subtractMatrices3(WI, IW), dt));
    const dWp = multiplyMatrix3Vector(invertMatrix3(J), [-f[0], -f[1], -f[2]]);
    wp = addVectors(wp, dWp);
    if (dotProduct(dWp, dWp) < 1e-18) break;
  }

  figure.w = multiplyMatrix3Vector(R, wp);
}
