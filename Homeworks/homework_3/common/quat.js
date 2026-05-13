function quatIdentity() { return [1, 0, 0, 0]; }

function quatMul(a, b) {
  return [
    a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
  ];
}

function quatNormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n < 1e-12) return [1, 0, 0, 0];
  return [q[0]/n, q[1]/n, q[2]/n, q[3]/n];
}

function quatFromAxisAngle(axis, angle) {
  const an = Math.hypot(axis[0], axis[1], axis[2]);
  if (an < 1e-12) return [1, 0, 0, 0];
  const s = Math.sin(angle * 0.5) / an;
  return [Math.cos(angle * 0.5), axis[0]*s, axis[1]*s, axis[2]*s];
}

function quatIntegrate(q, w, dt) {
  const wn = Math.hypot(w[0], w[1], w[2]);
  if (wn < 1e-12) return q.slice();
  const dq = quatFromAxisAngle(w, wn * dt);
  return quatNormalize(quatMul(dq, q));
}

function quatToMat3(q) {
  const w = q[0], x = q[1], y = q[2], z = q[3];
  const xx = x*x, yy = y*y, zz = z*z;
  const xy = x*y, xz = x*z, yz = y*z;
  const wx = w*x, wy = w*y, wz = w*z;
  return [
    1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy),
    2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx),
    2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy),
  ];
}

function quatConj(q) { return [q[0], -q[1], -q[2], -q[3]]; }

function quatRotate(q, v) {
  const R = quatToMat3(q);
  return [
    R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
    R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
    R[6]*v[0] + R[7]*v[1] + R[8]*v[2],
  ];
}

function quatRotateInv(q, v) {
  return quatRotate(quatConj(q), v);
}
