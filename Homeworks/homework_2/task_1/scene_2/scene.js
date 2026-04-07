// =================================
// Scene 2 - Two Circles
// =================================
// Инициализирует сцену: два круга слева и справа

function initScene() {
  points = [];
  constraints = [];
  draggedPoint = null;

  // Параметры кругов
  const radius = 60;
  const numPoints = 12;

  // Первый круг (слева)
  const center1 = createVector(settings.sceneWidth / 3, settings.sceneHeight / 2 - 100);
  createCircleInScene(center1, radius, numPoints);

  // Второй круг (справа)
  const center2 = createVector(2 * settings.sceneWidth / 3, settings.sceneHeight / 2 - 100);
  createCircleInScene(center2, radius, numPoints);

  // Создаём ограничения внутри и между кругами
  createConstraintsInScene();
}

function createCircleInScene(center, radius, numPoints) {
  // Добавляет точки круга в массив points
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
}

function createConstraintsInScene() {
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
