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
