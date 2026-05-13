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
