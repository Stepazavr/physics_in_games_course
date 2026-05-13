function mat3Identity() { return [1,0,0, 0,1,0, 0,0,1]; }
function mat3Zero()     { return [0,0,0, 0,0,0, 0,0,0]; }

function mat3MulVec(M, v) {
  return [
    M[0]*v[0] + M[1]*v[1] + M[2]*v[2],
    M[3]*v[0] + M[4]*v[1] + M[5]*v[2],
    M[6]*v[0] + M[7]*v[1] + M[8]*v[2],
  ];
}

function mat3Mul(A, B) {
  const C = new Array(9);
  for (let r = 0; r < 3; r++)
    for (let c = 0; c < 3; c++)
      C[r*3+c] = A[r*3]*B[c] + A[r*3+1]*B[c+3] + A[r*3+2]*B[c+6];
  return C;
}

function mat3Transpose(M) {
  return [M[0],M[3],M[6], M[1],M[4],M[7], M[2],M[5],M[8]];
}

function mat3Add(A, B) {
  const C = new Array(9);
  for (let i = 0; i < 9; i++) C[i] = A[i] + B[i];
  return C;
}

function mat3Sub(A, B) {
  const C = new Array(9);
  for (let i = 0; i < 9; i++) C[i] = A[i] - B[i];
  return C;
}

function mat3Scale(M, s) {
  const C = new Array(9);
  for (let i = 0; i < 9; i++) C[i] = M[i] * s;
  return C;
}

function mat3Inverse(M) {
  const a = M[0], b = M[1], c = M[2];
  const d = M[3], e = M[4], f = M[5];
  const g = M[6], h = M[7], i = M[8];
  const A =  (e*i - f*h);
  const B = -(d*i - f*g);
  const C =  (d*h - e*g);
  const det = a*A + b*B + c*C;
  if (Math.abs(det) < 1e-20) return mat3Identity();
  const inv = 1 / det;
  return [
    A*inv,            -(b*i - c*h)*inv,  (b*f - c*e)*inv,
    B*inv,             (a*i - c*g)*inv, -(a*f - c*d)*inv,
    C*inv,            -(a*h - b*g)*inv,  (a*e - b*d)*inv,
  ];
}

function mat3Cross(v) {
  return [
       0, -v[2],  v[1],
     v[2],    0, -v[0],
    -v[1], v[0],    0,
  ];
}

function boxInertia(m, h) {
  const hx2 = h[0]*h[0], hy2 = h[1]*h[1], hz2 = h[2]*h[2];
  return [
    (m / 3) * (hy2 + hz2),
    (m / 3) * (hx2 + hz2),
    (m / 3) * (hx2 + hy2),
  ];
}

function worldInertia(body) {
  if (body.invM === 0) return mat3Zero();
  const R = quatToMat3(body.q);
  const Ib = body.Ibody;
  const RD = [
    R[0]*Ib[0], R[1]*Ib[1], R[2]*Ib[2],
    R[3]*Ib[0], R[4]*Ib[1], R[5]*Ib[2],
    R[6]*Ib[0], R[7]*Ib[1], R[8]*Ib[2],
  ];
  return mat3Mul(RD, mat3Transpose(R));
}

function worldInertiaInv(body) {
  if (body.invM === 0) return mat3Zero();
  const R = quatToMat3(body.q);
  const Ii = body.IbodyInv;
  const RD = [
    R[0]*Ii[0], R[1]*Ii[1], R[2]*Ii[2],
    R[3]*Ii[0], R[4]*Ii[1], R[5]*Ii[2],
    R[6]*Ii[0], R[7]*Ii[1], R[8]*Ii[2],
  ];
  return mat3Mul(RD, mat3Transpose(R));
}

function v3(x,y,z){ return [x,y,z]; }
function vAdd(a,b){ return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
function vSub(a,b){ return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function vMul(a,s){ return [a[0]*s, a[1]*s, a[2]*s]; }
function vDot(a,b){ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
function vCross(a,b){ return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]; }
function vLen(a){ return Math.hypot(a[0],a[1],a[2]); }
function vNorm(a){ const n = vLen(a); return n < 1e-12 ? [0,0,0] : [a[0]/n,a[1]/n,a[2]/n]; }

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
