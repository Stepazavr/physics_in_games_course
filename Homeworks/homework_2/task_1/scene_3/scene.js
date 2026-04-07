// =================================
// Scene 3 - Large Deformable Circle
// =================================
// Инициализирует сцену: большой круг с меньшим числом точек для большей деформируемости

function initScene() {
  points = [];
  constraints = [];
  draggedPoint = null;

  // Большой круг с меньшим числом точек
  const center = createVector(settings.sceneWidth / 2, settings.sceneHeight / 2 - 80);
  const radius = 120;
  const numPoints = 10;

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

  // Создаём ограничения только между соседними точками и через одну для большей гибкости
  for (let i = 0; i < points.length; i++) {
    const next = (i + 1) % points.length;
    const nextNext = (i + 2) % points.length;
    
    // Ребро к соседней точке
    constraints.push({
      type: 'distance',
      p1: points[i],
      p2: points[next],
      distance: p5.Vector.dist(points[i].position, points[next].position),
    });
    
    // Ребро через одну точку
    constraints.push({
      type: 'distance',
      p1: points[i],
      p2: points[nextNext],
      distance: p5.Vector.dist(points[i].position, points[nextNext].position),
    });
  }
}
