function _scenePart2(kind) {
  if (kind === 'springForce' || kind === 'springSoft') {
    const b = makeFigure({
      x: [0, 1.0, 0],
      halfExtents: [1.5, 0.5, 0.25],
      m: 2.0, color: BRIGHT_COLORS[0].slice(),
    });
    b.q = quatFromAxisAngle([0, 0, 1], 0.3);
    b.w = [0.2, 0.1, 1.5];
    sim.bodies.push(b);
    const rLocal = [1.2, 0.3, 0];
    sim.springs.push({
      figureIdx: 0,
      rLocal,
      pWorld: [0, 4.5, 0],
      restLen: 3.2,
      k: 200, c: 4,
      lambdaAccum: 0,
    });
  } else {
    const A = makeFigure({
      x: [-2.0, 2.5, 0], halfExtents: [1.5, 0.5, 0.25],
      m: 2.0, color: BRIGHT_COLORS[0].slice(),
    });
    const B = makeFigure({
      x: [2.0, 2.5, 0], halfExtents: [1.5, 0.5, 0.25],
      m: 2.0, color: BRIGHT_COLORS[1].slice(),
    });
    A.w = [0.1, 0.05, 1.8];
    B.w = [-0.1, -0.05, -1.8];
    sim.bodies.push(A, B);

    const rAloc = [0.8, -0.3, 0.1];
    const rBloc = [-0.8, -0.3, -0.1];
    const pA0 = vAdd(A.x, quatRotate(A.q, rAloc));
    const pB0 = vAdd(B.x, quatRotate(B.q, rBloc));
    const restLen = vLen(vSub(pA0, pB0));

    sim.constraints.push({
      a: 0, b: 1,
      rAloc, rBloc,
      restLen,
      lambdaAccum: 0, lambdaPosAccum: 0,
    });
  }
  const floor = makeFigure({
    x: [0, -3, 0],
    halfExtents: [8, 0.2, 8],
    static: true, color: [60, 70, 80],
  });
  sim.bodies.push(floor);
  
}