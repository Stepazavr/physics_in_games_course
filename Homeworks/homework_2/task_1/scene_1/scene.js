// =================================
// Scene 1 - Single Circle
// =================================
// Инициализирует сцену: один круг в центре сцены

function initScene() {
  points = [];
  constraints = [];
  draggedPoint = null;

  // Создаём один круг в центре
  const center = createVector(settings.sceneWidth / 2, settings.sceneHeight / 2 - 100);
  const radius = 80;
  const numPoints = 15;

  // Равномерное распределение точек по кругу
  for (let i = 0; i < numPoints; i++) {
    const angle = TWO_PI * (i / numPoints);
    
    const x = center.x + radius * cos(angle);
    const y = center.y + radius * sin(angle);
    
    const pos = createVector(x, y);
    
    const point = {
      position: pos.copy(),
      velocity: createVector(0, 0),
      acceleration: createVector(settings.gravity.x, settings.gravity.y),
      positionPredicted: pos.copy(),
      mass: 1,
      isFixed: false,
    };
    
    points.push(point);
  }

  // Создаём ограничения между всеми парами точек
  for (let i = 0; i < points.length; i++) {
    for (let j = i + 1; j < points.length; j++) {
      const p1 = points[i];
      const p2 = points[j];
      constraints.push({
        type: 'distance',
        p1: p1,
        p2: p2,
        distance: p5.Vector.dist(p1.position, p2.position),
      });
    }
  }
}
