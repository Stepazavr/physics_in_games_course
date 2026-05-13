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
