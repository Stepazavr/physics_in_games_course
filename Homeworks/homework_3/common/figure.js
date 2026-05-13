function makeBody(opts) {
  const x = opts.x || [0, 0, 0];
  const q = quatNormalize(opts.q || [1, 0, 0, 0]);
  const v = opts.v || [0, 0, 0];
  const w = opts.w || [0, 0, 0];
  const halfExtents = opts.halfExtents || [0.5, 0.5, 0.5];

  const isStatic = !!opts.static;
  const m = isStatic ? 0 : (opts.m != null ? opts.m : 1);
  const invM = isStatic ? 0 : 1 / m;

  let Ibody, IbodyInv;
  if (isStatic) {
    Ibody = [0, 0, 0];
    IbodyInv = [0, 0, 0];
  } else {
    Ibody = boxInertia(m, halfExtents);
    IbodyInv = [1/Ibody[0], 1/Ibody[1], 1/Ibody[2]];
  }

  return {
    x: x.slice(), q: q.slice(),
    v: v.slice(), w: w.slice(),
    m, invM, halfExtents: halfExtents.slice(),
    Ibody, IbodyInv,
    color: opts.color || [120, 170, 230],
    isStatic,
    prevX: x.slice(), prevQ: q.slice(),
    aabbMin: [0,0,0], aabbMax: [0,0,0],
  };
}

function bodyPointWorld(body, rLocal) {
  return vAdd(body.x, quatRotate(body.q, rLocal));
}

function bodyPointVel(body, rWorld) {
  const r = vSub(rWorld, body.x);
  return vAdd(body.v, vCross(body.w, r));
}

function boxVertices(body) {
  const h = body.halfExtents;
  const verts = [];
  for (let sx = -1; sx <= 1; sx += 2)
  for (let sy = -1; sy <= 1; sy += 2)
  for (let sz = -1; sz <= 1; sz += 2)
    verts.push(vAdd(body.x, quatRotate(body.q, [sx*h[0], sy*h[1], sz*h[2]])));
  return verts;
}

function updateAABB(body) {
  const verts = boxVertices(body);
  let mn = [Infinity,Infinity,Infinity], mx = [-Infinity,-Infinity,-Infinity];
  for (const p of verts) {
    if (p[0] < mn[0]) mn[0] = p[0];
    if (p[1] < mn[1]) mn[1] = p[1];
    if (p[2] < mn[2]) mn[2] = p[2];
    if (p[0] > mx[0]) mx[0] = p[0];
    if (p[1] > mx[1]) mx[1] = p[1];
    if (p[2] > mx[2]) mx[2] = p[2];
  }
  body.aabbMin = mn; body.aabbMax = mx;
}

function drawBody(body) {
  if (body.invisible) return;
  push();
    translate(body.x[0], body.x[1], body.x[2]);
    const q = body.q;
    const cw = Math.max(-1, Math.min(1, q[0]));
    const s  = Math.sqrt(1 - cw * cw);
    if (s > 1e-6) {
      const ang = 2 * Math.acos(cw);
      rotate(ang, [q[1] / s, q[2] / s, q[3] / s]);
    }
    const c = body.color;
    noStroke();
    if (body.isStatic) fill(70, 80, 95);
    else                fill(c[0], c[1], c[2]);
    box(body.halfExtents[0]*2, body.halfExtents[1]*2, body.halfExtents[2]*2);
  pop();
}

function bodyAngularMomentum(body) {
  const Iw = worldInertia(body);
  return mat3MulVec(Iw, body.w);
}

function bodyKineticEnergy(body) {
  if (body.invM === 0) return 0;
  const lin = 0.5 * body.m * vDot(body.v, body.v);
  const Iw  = worldInertia(body);
  const Iww = mat3MulVec(Iw, body.w);
  const rot = 0.5 * vDot(body.w, Iww);
  return lin + rot;
}

function _boxAxes(body) {
  const R = quatToMat3(body.q);
  return [
    [R[0], R[3], R[6]],
    [R[1], R[4], R[7]],
    [R[2], R[5], R[8]],
  ];
}

function _projectBox(body, axis, axes) {
  const h = body.halfExtents;
  return Math.abs(vDot(axes[0], axis)) * h[0] +
         Math.abs(vDot(axes[1], axis)) * h[1] +
         Math.abs(vDot(axes[2], axis)) * h[2];
}

function satBoxBox(A, B) {
  const axesA = _boxAxes(A);
  const axesB = _boxAxes(B);
  const t = vSub(B.x, A.x);

  let bestDepth = Infinity;
  let bestAxis = null;
  let bestIsFromA = true;
  let bestFaceIdx = -1;

  const tryAxis = (axis, faceIdx, isFromA) => {
    const len2 = vDot(axis, axis);
    if (len2 < 1e-9) return true;
    const inv = 1 / Math.sqrt(len2);
    const ax = vMul(axis, inv);
    const rA = _projectBox(A, ax, axesA);
    const rB = _projectBox(B, ax, axesB);
    const dist = Math.abs(vDot(t, ax));
    const overlap = rA + rB - dist;
    if (overlap < 0) return false;
    if (overlap < bestDepth) {
      bestDepth = overlap;
      const dir = vDot(t, ax) >= 0 ? 1 : -1;
      bestAxis = vMul(ax, dir);
      bestIsFromA = isFromA;
      bestFaceIdx = faceIdx;
    }
    return true;
  };

  for (let i = 0; i < 3; i++) if (!tryAxis(axesA[i], i, true)) return null;
  for (let i = 0; i < 3; i++) if (!tryAxis(axesB[i], i, false)) return null;
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      const c = vCross(axesA[i], axesB[j]);
      if (!tryAxis(c, -1, true)) return null;
    }

  const n = bestAxis;
  const contacts = [];

  const vertsA = _boxVerticesLocalWorld(A);
  const vertsB = _boxVerticesLocalWorld(B);

  for (const v of vertsA) {
    if (_pointInsideBox(v.world, B)) {
      contacts.push({
        rAloc: v.local,
        rBloc: quatRotateInv(B.q, vSub(v.world, B.x)),
        n,
        depth: bestDepth,
      });
    }
  }
  for (const v of vertsB) {
    if (_pointInsideBox(v.world, A)) {
      contacts.push({
        rAloc: quatRotateInv(A.q, vSub(v.world, A.x)),
        rBloc: v.local,
        n,
        depth: bestDepth,
      });
    }
  }

  if (contacts.length === 0) {
    contacts.push({
      rAloc: quatRotateInv(A.q, vMul(n,  A.halfExtents[0])),
      rBloc: quatRotateInv(B.q, vMul(n, -B.halfExtents[0])),
      n,
      depth: bestDepth,
    });
  }

  const final = [];
  for (const c of contacts) {
    let dup = false;
    for (const e of final) {
      if (vLen(vSub(quatRotate(A.q, c.rAloc), quatRotate(A.q, e.rAloc))) < 1e-3) { dup = true; break; }
    }
    if (!dup) final.push(c);
    if (final.length >= 4) break;
  }
  return final;
}

function _boxVerticesLocalWorld(body) {
  const h = body.halfExtents;
  const out = [];
  for (let sx = -1; sx <= 1; sx += 2)
  for (let sy = -1; sy <= 1; sy += 2)
  for (let sz = -1; sz <= 1; sz += 2) {
    const local = [sx*h[0], sy*h[1], sz*h[2]];
    out.push({ local, world: vAdd(body.x, quatRotate(body.q, local)) });
  }
  return out;
}

function _pointInsideBox(p, body) {
  const local = quatRotateInv(body.q, vSub(p, body.x));
  return Math.abs(local[0]) <= body.halfExtents[0] + 1e-4 &&
         Math.abs(local[1]) <= body.halfExtents[1] + 1e-4 &&
         Math.abs(local[2]) <= body.halfExtents[2] + 1e-4;
}

function _depthBelowPlane(p, body, n, _) {
  const local = quatRotateInv(body.q, vSub(p, body.x));
  const h = body.halfExtents;
  const dx = h[0] - Math.abs(local[0]);
  const dy = h[1] - Math.abs(local[1]);
  const dz = h[2] - Math.abs(local[2]);
  return Math.min(dx, dy, dz);
}
