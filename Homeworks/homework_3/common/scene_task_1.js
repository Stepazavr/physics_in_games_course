function _scenePart1() {
  const b = makeFigure({
    x: [0, 0, 0],
    halfExtents: [1.5, 0.5, 0.25],
    m: 2.0,
    color: BRIGHT_COLORS[0].slice(),
  });
  b.w = [0.05, 2.0, 0.05];
  simulation.bodies.push(b);

  const floor = makeFigure({
    x: [0, -3, 0],
    halfExtents: [8, 0.2, 8],
    static: true, color: [60, 70, 80],
  });
  simulation.bodies.push(floor);
  
}
