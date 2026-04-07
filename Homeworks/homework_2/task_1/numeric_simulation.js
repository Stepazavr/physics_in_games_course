// =================================
// Simulation Settings
// =================================
const settings = {
  // Scene dimensions
  sceneWidth: 800,
  sceneHeight: 600,
  
  // Gravity
  gravity: { x: 0, y: 980 },
  timeStep: 0.01,
  solverIterations: 3,
  constraintStiffness: 1.0, // Жесткость ограничений (0 = нет, 1 = полная)
  flexibility: 1.0, // Упругость при столкновениях (0 = нет, 1 = полная)
  frictionCoefficient: 0.5, // Коэффициент трения (0 = нет, 1 = максимум)
  
  // Rendering
  backgroundColor: 200,
  
  // Constraints rendering
  constraintStrokeColor: { r: 100, g: 100, b: 255, a: 80 },
  constraintStrokeWeight: 2,
  
  // Points rendering
  pointRadius: 8,
  pointColor: { r: 255, g: 255, b: 255 },
  
  // Constraints
  constraintErrorThreshold: 1e-1,
  useApproximateRoot: true,
  
  // Mouse interaction
  dragRadius: 15,
  
  // Walls rendering
  wallStrokeColor: { r: 100, g: 100, b: 100 },
  wallStrokeWeight: 3,
  
  // Solver method
  solverMethod: 'hitman', // 'hitman', 'pbd'
}

// =================================
// Data Structures
// =================================
let figures = [];          // Массив объектов фигур (каждая фигура содержит свои points и constraints)
let points = [];           // Временный массив всех точек со всех фигур (для каждого кадра)
let constraints = [];      // Временный массив всех ограничений со всех фигур (для каждого кадра)

// Mouse and interaction
let draggedPoint = null;
let draggedFigure = null;  // Какой фигуре принадлежит захваченная точка

// Simulation state
let gravityEnabled = true;
let currentSolverMethod = 'hitman';

// =================================
// Setup
// =================================
function setup() {
  createCanvas(settings.sceneWidth, settings.sceneHeight);
  angleMode(RADIANS);
  initScene();
}

// =================================
// Main Loop
// =================================
function draw() {
  // Update simulation
  simulate();

  // Rendering
  background(settings.backgroundColor);

  // Draw elements
  drawWalls();
  drawScene();
}

// =================================
// Simulation Core
// =================================
function simulate() {
  // Собираем все точки и ограничения со всех фигур
  collectPointsAndConstraints();

  // 2. Интегрируем скорость и позицию (БЕЗ обработки коллизий)
  integrateVelocityAndPosition(settings.timeStep);
  
  // 3. Обработка пользовательского ввода (захват мышью)
  handlePointDrag();
  
  // 4. Обнаруживаем коллизии и добавляем их как ограничения
  detectAndAddCollisions();
  //detectObjectCollisions();
  
  // 4.5. Разрешаем самоколизии ткани
  //resolveClothSelfCollisions();
  
  // 5. Разрешаем ВСЕ ограничения (дистанционные + коллизионные)
  const allConstraints = [...constraints, ...frameCollisionConstraints];
  if (currentSolverMethod === 'hitman') {
    solveConstraints_Hitman(allConstraints, settings.solverIterations);
  } else if (currentSolverMethod === 'pbd') {
    solveConstraints_PBD(allConstraints, settings.solverIterations);
  } else if (currentSolverMethod === 'xpbd') {
    solveConstraints_XPBD(allConstraints, settings.solverIterations);
  }
  
  // 6. Обновляем скорости и позиции после разрешения ограничений
  updateAfterConstraints(settings.timeStep);
  
  // 7. Зажимаем позиции в границы сцены (hard constraint)
  constrainAllPointsToWalls();
  
  // 8. Применяем трение и упругость через скорости (post-solve)
  applyCollisionResponseToVelocities();
}

function collectPointsAndConstraints() {
  // Собираем все точки и все ограничения со всех фигур
  points = [];
  constraints = [];
  frameCollisionConstraints = [];

  for (const figure of figures) {
    for (const point of figure.points) {
      points.push(point);
    }
    for (const constraint of figure.constraints) {
      constraints.push(constraint);
    }
  }
}

// =================================
// Physics Systems
// =================================

function approximateDistance(vectorDiff) {
  // Быстрое вычисление длины вектора
  const dSquared = vectorDiff.x * vectorDiff.x + vectorDiff.y * vectorDiff.y;
  return Math.sqrt(dSquared);
}

function integrateVelocityAndPosition(dt) {
  // Интегрируем скорость и позицию (БЕЗ обработки коллизий)
  for (const point of points) {
    if (point.isFixed) continue;
    
    // 1. v_{k+1} = v_k + a_k * dt
    point.velocity.add(p5.Vector.mult(point.acceleration, dt));
    
    // 2. x_{predicted} = x_k + v_{k+1} * dt
    point.positionPredicted.set(point.position);
    point.positionPredicted.add(p5.Vector.mult(point.velocity, dt));
  }
}

function updateAfterConstraints(dt) {
  // Обновляем позиции и скорости после разрешения ограничений:
  // velocity = (x_constrained - x_old) / dt (восстанавливаем скорость из перемещения)
  // x = x_constrained
  // finalVelocity обновится в следующем интегрирования
  for (const point of points) {
    if (point.isFixed) continue;
    const displacement = p5.Vector.sub(point.positionPredicted, point.position);
    point.velocity.set(displacement);
    point.velocity.mult(1 / dt);
    point.position.set(point.positionPredicted);
  }
}

function detectAndAddCollisions() {
  // Обнаруживаем коллизии со стенками и добавляем их как ограничения
  // ВАЖНО: проверяем именно positionPredicted, так как ограничения решаются по ней!

    // 1. Очищаем коллизионные ограничения этого кадра
  frameCollisionConstraints = [];


  for (const point of points) {
    if (point.isFixed) continue;
    
    const padding = 0;
    
    // Левая стенка (x <= 0)
    if (point.positionPredicted.x <= padding) {
      const nx = 1;    // Нормаль указывает вправо (из сцены)
      const ny = 0;
      const qs_x = 0;  // Проекция на поверхность стены
      const qs_y = point.positionPredicted.y;
      createAndAddCollisionConstraint(point, nx, ny, qs_x, qs_y);
    }
    // Правая стенка (x >= width)
    else if (point.positionPredicted.x >= settings.sceneWidth - padding) {
      const nx = -1;
      const ny = 0;
      const qs_x = settings.sceneWidth;
      const qs_y = point.positionPredicted.y;
      createAndAddCollisionConstraint(point, nx, ny, qs_x, qs_y);
    }
    
    // Верхняя стенка (y <= 0)
    if (point.positionPredicted.y <= padding) {
      const nx = 0;
      const ny = 1;
      const qs_x = point.positionPredicted.x;
      const qs_y = 0;
      createAndAddCollisionConstraint(point, nx, ny, qs_x, qs_y);
    }
    // Нижняя стенка (y >= height)
    else if (point.positionPredicted.y >= settings.sceneHeight - padding) {
      const nx = 0;
      const ny = -1;
      const qs_x = point.positionPredicted.x;
      const qs_y = settings.sceneHeight;
      createAndAddCollisionConstraint(point, nx, ny, qs_x, qs_y);
    }
  }
}

function createAndAddCollisionConstraint(point, nx, ny, qs_x, qs_y) {
  // Создаём ограничение коллизии: C(p) = (p - qs) · n = 0
  // Проверяем, нарушено ли ограничение: C(p) < 0 (точка внутри стены)
  // ВАЖНО: используем positionPredicted для проверки!
  const C = (point.positionPredicted.x - qs_x) * nx + (point.positionPredicted.y - qs_y) * ny;
  
  // Добавляем ограничение только если оно нарушено
  if (C < 0) {
    frameCollisionConstraints.push({
      type: 'collision',
      p1: point,
      normalX: nx,
      normalY: ny,
      surface_x: qs_x,  // Проекция точки на поверхность
      surface_y: qs_y,
      // Градиент C = (p - qs) · n по p1:
      gradP1_x: nx,
      gradP1_y: ny,
      lambda: 0,  // Множитель Лагранжа для XPBD
    });
  }
}

function constrainAllPointsToWalls() {
  // Жёсткое зажатие позиции в границы сцены (hard constraint - гарантирует что точка не выйдет)
  for (const point of points) {
    if (point.position.x < 0) point.position.x = 0;
    if (point.position.x > settings.sceneWidth) point.position.x = settings.sceneWidth;
    if (point.position.y < 0) point.position.y = 0;
    if (point.position.y > settings.sceneHeight) point.position.y = settings.sceneHeight;
  }
}

function applyCollisionResponseToVelocities() {
  // Post-solve: применяем трение и упругость через изменение скоростей
  for (const constraint of frameCollisionConstraints) {
    if (constraint.type !== 'collision') continue;
    
    const point = constraint.p1;
    const nx = constraint.normalX;
    const ny = constraint.normalY;
    const e = settings.flexibility;
    const mu = settings.frictionCoefficient;
    
    // Скорость в явном виде
    const vx = point.velocity.x;
    const vy = point.velocity.y;
    
    // Компонента вдоль нормали: v · n
    const vDotN = vx * nx + vy * ny;
    
    // Если скорость направлена в стену (v · n < 0), применяем упругость
    let newVx = vx;
    let newVy = vy;
    
    if (vDotN < 0) {
      // Формула упругого отражения: u = v - (1 + e)(v · n)n
      const factor = (1 + e) * vDotN;
      newVx = vx - factor * nx;
      newVy = vy - factor * ny;
    }
    
    // Применяем трение к касательной компоненте новой скорости
    // Касательная = v - (v · n)n
    const tangent_x = newVx - vDotN * nx;
    const tangent_y = newVy - vDotN * ny;
    
    // После трения: tangent *= (1 - mu)
    // Финальная скорость = нормальная + касательная_после_трения
    const tangent_friction_x = tangent_x * (1 - mu);
    const tangent_friction_y = tangent_y * (1 - mu);
    
    point.velocity.x = vDotN * nx + tangent_friction_x;
    point.velocity.y = vDotN * ny + tangent_friction_y;
  }
}

function solveConstraints_Hitman(allConstraints, iterations) {
  // Метод Hitman (Gauss-Seidel итеративный) решает ВСЕ ограничения
  for (let i = 0; i < iterations; i++) {
    for (const constraint of allConstraints) {
      if (constraint.type === 'distance') {
        // Дистанционное ограничение между двумя точками
        solveDistanceConstraint_Hitman(constraint);
      } else if (constraint.type === 'collision') {
        // Коллизионное ограничение: просто выталкиваем точку из стены
        solveCollisionConstraint_Hitman(constraint);
      }
    }
  }
  
}

function solveDistanceConstraint_Hitman(constraint) {
  const { p1, p2, distance: restLength } = constraint;
  
  const delta = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
  
  let deltaLength = settings.useApproximateRoot
    ? approximateDistance(delta)
    : Math.sqrt(delta.x * delta.x + delta.y * delta.y);
  
  if (deltaLength < 1e-6) return;
  
  const invMass1 = p1.isFixed ? 0 : 1 / p1.mass;
  const invMass2 = p2.isFixed ? 0 : 1 / p2.mass;
  
  if (invMass1 + invMass2 < 1e-6) return;
  
  const diff = (deltaLength - restLength) / (deltaLength * (invMass1 + invMass2));
  
  if (p1.isFixed === false) {
    const correction = p5.Vector.mult(delta, invMass1 * diff * settings.constraintStiffness);
    p1.positionPredicted.add(correction);
  }
  
  if (p2.isFixed === false) {
    const correction = p5.Vector.mult(delta, invMass2 * diff * settings.constraintStiffness);
    p2.positionPredicted.sub(correction);
  }
}

function solveCollisionConstraint_Hitman(constraint) {
  // Коллизионное ограничение: C(p) = (p - qs) · n = 0
  // Если C < 0, нужно выталкнуть точку вдоль нормали
  const { p1, normalX: nx, normalY: ny, surface_x: qs_x, surface_y: qs_y } = constraint;
  
  const px = p1.positionPredicted.x;
  const py = p1.positionPredicted.y;
  
  // C = (p - qs) · n
  const C = (px - qs_x) * nx + (py - qs_y) * ny;
  
  // Если C < 0, выталкиваем вдоль нормали
  if (C < 0) {
    const correction = -C * settings.constraintStiffness;  // Сила выталкивания
    p1.positionPredicted.x += correction * nx;
    p1.positionPredicted.y += correction * ny;
  }
}

function solveConstraints_PBD(allConstraints, iterations) {
  // Метод PBD (Position Based Dynamics) решает ВСЕ ограничения единообразно
  for (let i = 0; i < iterations; i++) {
    for (const constraint of allConstraints) {
      if (constraint.type === 'distance') {
        solveDistanceConstraint_PBD(constraint);
      } else if (constraint.type === 'collision') {
        solveCollisionConstraint_PBD(constraint);
      }
    }
  }
}

function solveDistanceConstraint_PBD(constraint) {
  const { p1, p2, distance: restLength } = constraint;
  
  const delta = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
  const deltaLength = approximateDistance(delta);
  
  if (deltaLength < 1e-6) return;
  
  const C = deltaLength - restLength;
  
  const gradP1 = p5.Vector.mult(delta, -1 / deltaLength);
  const gradP2 = p5.Vector.mult(delta, 1 / deltaLength);
  
  const w1 = p1.isFixed ? 0 : 1 / p1.mass;
  const w2 = p2.isFixed ? 0 : 1 / p2.mass;
  
  const denom = w1 * 1 + w2 * 1;
  
  if (denom < 1e-6) return;
  
  const s = C / denom;
  
  if (p1.isFixed === false) {
    const correction = p5.Vector.mult(gradP1, -s * w1 * settings.constraintStiffness);
    p1.positionPredicted.add(correction);
  }
  
  if (p2.isFixed === false) {
    const correction = p5.Vector.mult(gradP2, -s * w2 * settings.constraintStiffness);
    p2.positionPredicted.add(correction);
  }
}

function solveCollisionConstraint_PBD(constraint) {
  // Коллизионное ограничение: C(p) = (p - qs) · n = 0
  // Градиент: ∇C = n
  const { p1, normalX: nx, normalY: ny, surface_x: qs_x, surface_y: qs_y } = constraint;
  
  const px = p1.positionPredicted.x;
  const py = p1.positionPredicted.y;
  
  // C = (p - qs) · n = (px - qs_x)*nx + (py - qs_y)*ny
  const C = (px - qs_x) * nx + (py - qs_y) * ny;
  
  // Обрабатываем только если нарушено (C < 0)
  if (C < 0) {
    const w1 = p1.isFixed ? 0 : 1 / p1.mass;
    
    if (w1 < 1e-6) return;
    
    // s = C / (∇C · ∇C * w1) = C / (1 * w1)
    const s = C / w1;
    
    // Коррекция: p1 -= s * w1 * stiffness * ∇C = -C * stiffness * n
    const correction = -s * w1 * settings.constraintStiffness;
    p1.positionPredicted.x += correction * nx;
    p1.positionPredicted.y += correction * ny;
  }
}

// =================================
// XPBD Solver
// =================================

function solveConstraints_XPBD(allConstraints, iterations) {
  // Обнуляем ВСЕ лямбды в начале solve (как в XPBD алгоритме)
  for (const constraint of allConstraints) {
    if (constraint.type === 'distance' || constraint.type === 'collision') {
      constraint.lambda = 0;
    }
  }
  
  // Метод XPBD (Extended Position Based Dynamics) решает ВСЕ ограничения
  // Использует множители Лагранжа для физически корректного контроля жесткости
  for (let i = 0; i < iterations; i++) {
    for (const constraint of allConstraints) {
      if (constraint.type === 'distance') {
        solveDistanceConstraint_XPBD(constraint);
      } else if (constraint.type === 'collision') {
        solveCollisionConstraint_XPBD(constraint);
      }
    }
  }
}

function solveDistanceConstraint_XPBD(constraint) {
  // XPBD метод для ограничения расстояния
  // Инициализируем lambda если его нет
  if (constraint.lambda === undefined) {
    constraint.lambda = 0;
  }
  
  const { p1, p2, distance: restLength } = constraint;
  
  const delta = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
  const deltaLength = approximateDistance(delta);
  
  if (deltaLength < 1e-6) return;
  
  // Нормализуем градиент
  const gradP1_x = -delta.x / deltaLength;
  const gradP1_y = -delta.y / deltaLength;
  const gradP2_x = delta.x / deltaLength;
  const gradP2_y = delta.y / deltaLength;
  
  // Вычисляем значение ограничения (ошибка)
  const C = deltaLength - restLength;
  
  // Обратная жесткость (compliance)
  const k = settings.constraintStiffness;
  const compliance = k > 0 ? 1.0 / k : 0;
  
  // Вычисляем весовые коэффициенты
  const w1 = p1.isFixed ? 0 : 1 / p1.mass;
  const w2 = p2.isFixed ? 0 : 1 / p2.mass;
  
  // Вычисляем градиент · градиент
  const gradGrad = 1.0 * (w1 + w2); // Так как |grad| = 1
  
  if (w1 + w2 < 1e-6) return;
  
  // Вычисляем изменение множителя Лагранжа
  const denom = gradGrad + compliance;
  const deltaLambda = -(C + compliance * constraint.lambda) / denom;
  
  // Обновляем множитель Лагранжа
  constraint.lambda += deltaLambda;
  
  // Применяем коррекцию позиций
  if (p1.isFixed === false) {
    const correctionMag = deltaLambda * w1;
    p1.positionPredicted.x += correctionMag * gradP1_x;
    p1.positionPredicted.y += correctionMag * gradP1_y;
  }
  
  if (p2.isFixed === false) {
    const correctionMag = deltaLambda * w2;
    p2.positionPredicted.x += correctionMag * gradP2_x;
    p2.positionPredicted.y += correctionMag * gradP2_y;
  }
}

function solveCollisionConstraint_XPBD(constraint) {
  // XPBD метод для коллизионного ограничения
  // Инициализируем lambda если его нет
  if (constraint.lambda === undefined) {
    constraint.lambda = 0;
  }
  
  const { p1, normalX: nx, normalY: ny, surface_x: qs_x, surface_y: qs_y } = constraint;
  
  const px = p1.positionPredicted.x;
  const py = p1.positionPredicted.y;
  
  // C = (p - qs) · n = (px - qs_x)*nx + (py - qs_y)*ny
  const C = (px - qs_x) * nx + (py - qs_y) * ny;
  
  // Обрабатываем только если нарушено (C < 0)
  if (C < 0) {
    // Обратная жесткость (compliance)
    const k = settings.constraintStiffness;
    const compliance = k > 0 ? 1.0 / k : 0;
    
    // Вычисляем весовой коэффициент
    const w1 = p1.isFixed ? 0 : 1 / p1.mass;
    
    if (w1 < 1e-6) return;
    
    // Вычисляем градиент · градиент (|n| = 1)
    const gradGrad = 1.0 * w1;
    
    // Вычисляем изменение множителя Лагранжа
    const denom = gradGrad + compliance;
    const deltaLambda = -(C + compliance * constraint.lambda) / denom;
    
    // Обновляем множитель Лагранжа
    constraint.lambda += deltaLambda;
    
    // Применяем коррекцию позиции
    const correctionMag = deltaLambda * w1;
    p1.positionPredicted.x += correctionMag * nx;
    p1.positionPredicted.y += correctionMag * ny;
  }
}


// =================================
// Rendering
// =================================

function drawWalls() {
  stroke(settings.wallStrokeColor.r, settings.wallStrokeColor.g, settings.wallStrokeColor.b);
  strokeWeight(settings.wallStrokeWeight);
  
  // Bottom wall
  line(0, settings.sceneHeight, settings.sceneWidth, settings.sceneHeight);
  
  // Top wall
  line(0, 0, settings.sceneWidth, 0);
  
  // Left wall
  line(0, 0, 0, settings.sceneHeight);
  
  // Right wall
  line(settings.sceneWidth, 0, settings.sceneWidth, settings.sceneHeight);
}

function drawScene() {
  // Draw constraints
  stroke(settings.constraintStrokeColor.r, settings.constraintStrokeColor.g, 
         settings.constraintStrokeColor.b, settings.constraintStrokeColor.a);
  strokeWeight(settings.constraintStrokeWeight);
  for (const constraint of constraints) {
    line(
      constraint.p1.position.x, constraint.p1.position.y,
      constraint.p2.position.x, constraint.p2.position.y
    );
  }

  // Draw points
  noStroke();
  for (const point of points) {
    if (point === draggedPoint) {
      fill(255, 200, 0); // Yellow for dragged point
    } else if (point.color) {
      // Используем цвет точки, если он задан (для закрепленных точек)
      fill(point.color.r, point.color.g, point.color.b);
    } else {
      fill(settings.pointColor.r, settings.pointColor.g, settings.pointColor.b);
    }
    circle(point.position.x, point.position.y, settings.pointRadius * 2);
  }
}

// =================================
// Mouse and Interaction
// =================================

function mousePressed() {
  // Ищем ближайшую точку к позиции мышки среди всех фигур
  let closestDistSq = Infinity;
  let closestPoint = null;
  let closestFigure = null;

  for (const figure of figures) {
    for (const point of figure.points) {
      const dx = point.position.x - mouseX;
      const dy = point.position.y - mouseY;
      const distSq = dx * dx + dy * dy;

      if (distSq < closestDistSq) {
        closestDistSq = distSq;
        closestPoint = point;
        closestFigure = figure;
      }
    }
  }

  if (closestPoint && closestDistSq < settings.dragRadius * settings.dragRadius) {
    draggedPoint = closestPoint;
    draggedFigure = closestFigure;
    draggedPoint.isFixed = true;
  }
}

function handlePointDrag() {
  // Перемещаем точку следом за мышкой
  if (!draggedPoint) return;
  
  draggedPoint.position.x = mouseX;
  draggedPoint.position.y = mouseY;
  draggedPoint.positionPredicted.set(draggedPoint.position);
  draggedPoint.velocity.mult(0);  // Зануляем скорость захватанной точки
}

function mouseReleased() {
  if (draggedPoint) {
    draggedPoint.isFixed = false;
    draggedPoint = null;
  }
}

// =================================
// UI Controls
// =================================

function changeSolverMethod(method) {
  currentSolverMethod = method;
  const select = document.getElementById('solverSelect');
  if (select) {
    select.value = method;
  }
}

function toggleGravity() {
  gravityEnabled = !gravityEnabled;
  
  // Зануляем скорость и обновляем ускорение при переключении гравитации
  for (const figure of figures) {
    for (const point of figure.points) {
      point.velocity.mult(0);
      if (!gravityEnabled) {
        point.acceleration.set(0, 0);
      } else {
        point.acceleration.set(settings.gravity.x, settings.gravity.y);
      }
    }
  }
  
  const btn = document.getElementById('gravityBtn');
  btn.textContent = gravityEnabled ? 'Гравитация: ВКЛ' : 'Гравитация: ВЫКЛ';
  draggedPoint = null;
  draggedFigure = null;
}

function resetSimulation() {
  // Reinitialize the scene
  initScene();
  draggedPoint = null;
  draggedFigure = null;
  
  // Применяем состояние гравитации к новым точкам
  if (!gravityEnabled) {
    for (const figure of figures) {
      for (const point of figure.points) {
        point.acceleration.set(0, 0);
      }
    }
  }
}

// Initialize button handlers after the page loads
window.addEventListener('load', () => {
  document.getElementById('gravityBtn').addEventListener('click', toggleGravity);
  document.getElementById('resetBtn').addEventListener('click', resetSimulation);
  const solverSelect = document.getElementById('solverSelect');
  if (solverSelect) {
    solverSelect.addEventListener('change', (e) => {
      changeSolverMethod(e.target.value);
    });
  }
  
  // Параметры симуляции
  const iterationsInput = document.getElementById('iterationsInput');
  const iterationsValue = document.getElementById('iterationsValue');
  if (iterationsInput) {
    iterationsInput.addEventListener('input', (e) => {
      const value = parseInt(e.target.value);
      settings.solverIterations = value;
      iterationsValue.textContent = value;
    });
  }
  
  const stiffnessInput = document.getElementById('stiffnessInput');
  const stiffnessValue = document.getElementById('stiffnessValue');
  if (stiffnessInput) {
    stiffnessInput.addEventListener('input', (e) => {
      const value = parseFloat(e.target.value);
      settings.constraintStiffness = value;
      stiffnessValue.textContent = value.toFixed(2);
    });
  }
  
  const frictionInput = document.getElementById('frictionInput');
  const frictionValue = document.getElementById('frictionValue');
  if (frictionInput) {
    frictionInput.addEventListener('input', (e) => {
      const value = parseFloat(e.target.value);
      settings.frictionCoefficient = value;
      frictionValue.textContent = value.toFixed(2);
    });
  }
});

function updateFigureCenter(figure) {
  if (!figure.points || figure.points.length === 0) {
    figure.center = createVector(0, 0);
    return;
  }

  let centerX = 0;
  let centerY = 0;
  for (const point of figure.points) {
    centerX += point.position.x;
    centerY += point.position.y;
  }
  figure.center = createVector(centerX / figure.points.length, centerY / figure.points.length);

  // Update normals for convex points
  if (figure.convex_points) {
    for (const point of figure.convex_points) {
      point.normal = p5.Vector.sub(point.position, figure.center).normalize();
    }
  }
}

function detectObjectCollisions() {
    // Проверяем столкновения между всеми парами фигур
    for (let i = 0; i < figures.length; i++) {
        for (let j = i + 1; j < figures.length; j++) {
            const fig1 = figures[i];
            const fig2 = figures[j];

            if (!fig1.convex_points || !fig2.convex_points) continue;

            // Обновляем центры обеих фигур перед проверкой
            updateFigureCenter(fig1);
            updateFigureCenter(fig2);

            const pointRadius = settings.pointRadius;  // Радиус точки

            // ============================================================
            // Проверка столкновения fig1 -> fig2 (точки fig1 внутри fig2)
            // ============================================================
            for (const p1 of fig1.convex_points) {
                // Найти ближайшую точку в fig2.convex_points к точке p1
                let minDist = Infinity;
                let closestIdx = 0;
                for (let idx = 0; idx < fig2.convex_points.length; idx++) {
                    const p = fig2.convex_points[idx];
                    const dist = p5.Vector.dist(p1.positionPredicted, p.positionPredicted);
                    if (dist < minDist) {
                        minDist = dist;
                        closestIdx = idx;
                    }
                }
                
                // Проверяем два ребра с ближайшей точкой: (closestIdx-1, closestIdx) и (closestIdx, closestIdx+1)
                const k = closestIdx;
                
                // Ребро 1: (k-1, k)
                {
                    const prevIdx = (k - 1 + fig2.convex_points.length) % fig2.convex_points.length;
                    const p2 = fig2.convex_points[prevIdx];
                    const p3 = fig2.convex_points[k];
                    const edgeNormal = p5.Vector.add(p2.normal, p3.normal).normalize();
                    const vectorToP1 = p5.Vector.sub(p1.positionPredicted, p2.positionPredicted);
                    const distToEdge = vectorToP1.dot(edgeNormal);
                    if (distToEdge < pointRadius) {
                        const pushForce = (pointRadius - distToEdge) * settings.constraintStiffness;
                        p1.positionPredicted.x += pushForce * edgeNormal.x;
                        p1.positionPredicted.y += pushForce * edgeNormal.y;
                    }
                }
                
                // Ребро 2: (k, k+1)
                {
                    const p2 = fig2.convex_points[k];
                    const p3 = fig2.convex_points[(k + 1) % fig2.convex_points.length];
                    const edgeNormal = p5.Vector.add(p2.normal, p3.normal).normalize();
                    const vectorToP1 = p5.Vector.sub(p1.positionPredicted, p2.positionPredicted);
                    const distToEdge = vectorToP1.dot(edgeNormal);
                    if (distToEdge < pointRadius) {
                        const pushForce = (pointRadius - distToEdge) * settings.constraintStiffness;
                        p1.positionPredicted.x += pushForce * edgeNormal.x;
                        p1.positionPredicted.y += pushForce * edgeNormal.y;
                    }
                }
            }

            // ============================================================
            // Проверка столкновения fig2 -> fig1 (точки fig2 внутри fig1)
            // ============================================================
            for (const p2 of fig2.convex_points) {
                // Найти ближайшую точку в fig1.convex_points к точке p2
                let minDist = Infinity;
                let closestIdx = 0;
                for (let idx = 0; idx < fig1.convex_points.length; idx++) {
                    const p = fig1.convex_points[idx];
                    const dist = p5.Vector.dist(p2.positionPredicted, p.positionPredicted);
                    if (dist < minDist) {
                        minDist = dist;
                        closestIdx = idx;
                    }
                }
                
                // Проверяем два ребра с ближайшей точкой: (closestIdx-1, closestIdx) и (closestIdx, closestIdx+1)
                const k = closestIdx;
                
                // Ребро 1: (k-1, k)
                {
                    const prevIdx = (k - 1 + fig1.convex_points.length) % fig1.convex_points.length;
                    const p1 = fig1.convex_points[prevIdx];
                    const p3 = fig1.convex_points[k];
                    const edgeNormal = p5.Vector.add(p1.normal, p3.normal).normalize();
                    const vectorToP2 = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
                    const distToEdge = vectorToP2.dot(edgeNormal);
                    if (distToEdge < pointRadius) {
                        const pushForce = (pointRadius - distToEdge) * settings.constraintStiffness;
                        p2.positionPredicted.x += pushForce * edgeNormal.x;
                        p2.positionPredicted.y += pushForce * edgeNormal.y;
                    }
                }
                
                // Ребро 2: (k, k+1)
                {
                    const p1 = fig1.convex_points[k];
                    const p3 = fig1.convex_points[(k + 1) % fig1.convex_points.length];
                    const edgeNormal = p5.Vector.add(p1.normal, p3.normal).normalize();
                    const vectorToP2 = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
                    const distToEdge = vectorToP2.dot(edgeNormal);
                    if (distToEdge < pointRadius) {
                        const pushForce = (pointRadius - distToEdge) * settings.constraintStiffness;
                        p2.positionPredicted.x += pushForce * edgeNormal.x;
                        p2.positionPredicted.y += pushForce * edgeNormal.y;
                    }
                }
            }
        }
    }
}

// =================================
// Cloth Self-Collision Resolution
// =================================
function resolveClothSelfCollisions() {
  /**
   * Разрешение самоколизий ткани (столкновения между точками одной фигуры)
   * 
   * ПАРАМЕТРЫ:
   * R = settings.pointRadius = радиус частицы
   * S = settings.solverIterations = количество подшагов
   * dt = settings.timeStep = размер временного шага
   * 
   * ФОРМУЛЫ:
   * 1. V_max = (R * S) / dt
   * 2. D_collision = min(2 * R, RestDistance)
   * 3. Если currentDistance < D_collision:
   *    - correction = (D_collision - currentDistance) / 2
   *    - direction = (pos2 - pos1) / currentDistance
   *    - pos1 -= direction * correction
   *    - pos2 += direction * correction
   */
  
  const R = settings.pointRadius;
  const S = settings.solverIterations;
  const dt = settings.timeStep;
  const d = 0.8; // Коэффициент демпфирования (0..1)
  
  // Обраватываем каждую фигуру отдельно
  for (const figure of figures) {
    if (!figure.points || figure.points.length < 2) continue;
    
    const pointsInFigure = figure.points;
    
    // Проверяем все пары точек в фигуре
    for (let i = 0; i < pointsInFigure.length; i++) {
      for (let j = i + 1; j < pointsInFigure.length; j++) {
        const p1 = pointsInFigure[i];
        const p2 = pointsInFigure[j];
        
        // Пропускаем закреплённые точки
        if (p1.isFixed && p2.isFixed) continue;
        
        // Текущее расстояние между точками
        const delta = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
        const currentDistance = delta.mag();
        
        // Избегаем деления на 0
        if (currentDistance < 0.001) continue;
        
        // Ищем ограничение distance между этой парой точек
        let restDistance = currentDistance; // По умолчанию - текущее расстояние
        
        for (const constraint of figure.constraints) {
          if (constraint.type === 'distance') {
            // Проверяем, это ограничение для нашей пары?
            if ((constraint.p1 === p1 && constraint.p2 === p2) ||
                (constraint.p1 === p2 && constraint.p2 === p1)) {
              restDistance = constraint.distance; // Используем restLength из ограничения
              break;
            }
          }
        }
        
        // ФОРМУЛА 2: Дистанция столкновения
        const D_collision = Math.min(2 * R, restDistance);
        
        // Проверяем условие столкновения
        if (currentDistance < D_collision) {
          // ФОРМУЛА 3: Разрешение столкновения
          const correction = (D_collision - currentDistance) / 2;
          const direction = delta.copy().normalize(); // (pos2 - pos1) / currentDistance
          
          // Применяем коррекцию позиций
          if (!p1.isFixed) {
            p1.positionPredicted.x -= direction.x * correction;
            p1.positionPredicted.y -= direction.y * correction;
          }
          
          if (!p2.isFixed) {
            p2.positionPredicted.x += direction.x * correction;
            p2.positionPredicted.y += direction.y * correction;
          }
        }
      }
    }
  }
}


