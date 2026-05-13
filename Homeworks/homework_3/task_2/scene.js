function buildConstraintScene(kind) {
  if (kind === 'springForce' || kind === 'springSoft') {
    const b = makeFigure({
      x: [0, 1.0, 0],
      halfExtents: [1.5, 0.5, 0.25],
      m: 2.0, color: BRIGHT_COLORS[0].slice(),
    });
    b.q = quaternionFromAxisAngle([0, 0, 1], 0.3);
    b.w = [0.2, 0.1, 1.5];
    simulation.bodies.push(b);
    const rLocal = [1.2, 0.3, 0];
    simulation.springs.push({
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
    simulation.bodies.push(A, B);

    const rAloc = [0.8, -0.3, 0.1];
    const rBloc = [-0.8, -0.3, -0.1];
    const pA0 = addVectors(A.x, rotateVectorByQuat(A.q, rAloc));
    const pB0 = addVectors(B.x, rotateVectorByQuat(B.q, rBloc));
    const restLen = vectorLength(subtractVectors(pA0, pB0));

    simulation.constraints.push({
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
  simulation.bodies.push(floor);
  
}