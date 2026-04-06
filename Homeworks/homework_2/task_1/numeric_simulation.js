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
  dampingFactor: 1.0, // Дэмпинг скорости (0-1, где 1 = нет дэмпинга)
  
  // Circle
  circleCenter: { x: 0, y: 0 },
  circleRadius: 80,
  circleNumPoints: 15,
  
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
let points = [];
let constraints = [];

// Mouse and interaction
let draggedPoint = null;

// Simulation state
let gravityEnabled = true;
let currentSolverMethod = 'hitman';

// =================================
// Setup
// =================================
function setup() {
  createCanvas(settings.sceneWidth, settings.sceneHeight);
  angleMode(RADIANS);
  const center = createVector(settings.sceneWidth / 2, settings.sceneHeight / 2 - 100);
  createCircle(center, settings.circleRadius, settings.circleNumPoints);
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
  drawCircle();
}

// =================================
// Simulation Core
// =================================
function simulate() {
  // 1-3. Интегрируем скорость, применяем дэмпинг и интегрируем позицию
  integrateVelocityAndPosition(settings.timeStep);
  
  // 4. Обработка пользовательского ввода (захват мышю)
  handlePointDrag();
  
  // 5. Обрабатываем коллизии 
  updateCollisions();
  
  // 6. Разрешаем ограничения
  if (currentSolverMethod === 'hitman') {
    solveConstraints_Hitman(settings.solverIterations);
  } else if (currentSolverMethod === 'pbd') {
    solveConstraints_PBD(settings.solverIterations);
  }
  
  // 7. Обновляем скорости после разрешения ограничений
  updateAfterConstraints(settings.timeStep);
  
  // 8. Обновляем скорости при коллизии
  updateVelocities();
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
  // Интегрируем скорость, применяем дэмпинг и интегрируем позицию в один цикл
  for (const point of points) {
    if (point.isFixed) continue;
    
    // v_{k+1} = v_k + a_k * dt
    point.velocity.add(p5.Vector.mult(point.acceleration, dt));
    
    // v *= dampingFactor (вязкое трение)
    point.velocity.mult(settings.dampingFactor);
    
    // x_{predicted} = x_k + v_{k+1} * dt
    point.positionPredicted.set(point.position);
    point.positionPredicted.add(p5.Vector.mult(point.velocity, dt));
  }
}

function updateAfterConstraints(dt) {
  // Обновляем позиции и скорости после разрешения ограничений:
  // v = (x_constrained - x_old) / dt
  // x = x_constrained
  for (const point of points) {
    if (point.isFixed) continue;
    const displacement = p5.Vector.sub(point.positionPredicted, point.position);
    point.velocity.set(displacement);
    point.velocity.mult(1 / dt);
    point.position.set(point.positionPredicted);
  }
}

function constrainPointToWalls(point) {
  // Ограничиваем позицию точки в пределы сцены
  if (point.position.x < 0) {
    point.position.x = 0;
  } else if (point.position.x > settings.sceneWidth) {
    point.position.x = settings.sceneWidth;
  }
  
  if (point.position.y < 0) {
    point.position.y = 0;
  } else if (point.position.y > settings.sceneHeight) {
    point.position.y = settings.sceneHeight;
  }
}

function updateVelocity(point) {
  // Применяем упругость и трение при коллизии со стеной
  const e = settings.flexibility;
  const mu = settings.frictionCoefficient;
  
  // Столкновение с левой/правой стеной
  if (point.position.x <= 0 || point.position.x >= settings.sceneWidth) {
    point.velocity.x *= -e;
    point.velocity.y *= (1 - mu);
  }
  
  // Столкновение с верхней/нижней стеной
  if (point.position.y <= 0 || point.position.y >= settings.sceneHeight) {
    point.velocity.y *= -e;
    point.velocity.x *= (1 - mu);
  }
}

function updateCollisions() {
  // Ограничиваем позиции всех точек в пределы сцены
  for (const point of points) {
    constrainPointToWalls(point);
  }
}

function updateVelocities() {
  // Применяем эффекты коллизии к скоростям всех точек
  for (const point of points) {
    updateVelocity(point);
  }
}

function updateVelocitiesAfterCollisions(dt) {
  // 1. Ограничиваем позиции в пределы сцены
  updateCollisions();
  
  // 2. Применяем эффекты коллизии к скоростям
  updateVelocities();
}

function solveConstraints_Hitman(iterations) {
  // Ограничения теперь применяются к positionPredicted
  for (let i = 0; i < iterations; i++) {
    for (const constraint of constraints) {
      const { p1, p2, distance: restLength } = constraint;
      
      const delta = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
      
      let deltaLength = settings.useApproximateRoot
        ? approximateDistance(delta)
        : Math.sqrt(delta.x * delta.x + delta.y * delta.y);
      
      if (deltaLength < 1e-6) continue;
      
      const invMass1 = p1.isFixed ? 0 : 1 / p1.mass;
      const invMass2 = p2.isFixed ? 0 : 1 / p2.mass;
      
      if (invMass1 + invMass2 < 1e-6) continue;
      
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
  }
}

function solveConstraints_PBD(iterations) {
  // Классический метод Position Based Dynamics:
  // Эсто тработает с positionPredicted
  for (let i = 0; i < iterations; i++) {
    for (const constraint of constraints) {
      const { p1, p2, distance: restLength } = constraint;
      
      const delta = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
      const deltaLength = approximateDistance(delta);
      
      if (deltaLength < 1e-6) continue;
      
      const C = deltaLength - restLength;
      
      const gradP1 = p5.Vector.mult(delta, -1 / deltaLength);
      const gradP2 = p5.Vector.mult(delta, 1 / deltaLength);
      
      const w1 = p1.isFixed ? 0 : 1 / p1.mass;
      const w2 = p2.isFixed ? 0 : 1 / p2.mass;
      
      const denom = w1 * 1 + w2 * 1;
      
      if (denom < 1e-6) continue;
      
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
  }
}



// =================================
// Object Creation
// =================================

function createCircle(center, radius, numPoints) {
  points = [];
  constraints = [];

  // Равномерное распределение точек по кругу
  for (let i = 0; i < numPoints; i++) {
    const angle = TWO_PI * (i / numPoints);
    
    const x = center.x + radius * cos(angle);
    const y = center.y + radius * sin(angle);
    
    const pos = createVector(x, y);
    
    const point = {
      position: pos.copy(),      // x - текущая позиция
      velocity: createVector(0, 0),  // v - текущая скорость
      acceleration: createVector(settings.gravity.x, settings.gravity.y),  // a - эскорение
      positionPredicted: pos.copy(),  // p - временная позиция для ограничений
      mass: 1,
      isFixed: false,
    };
    
    points.push(point);
  }

  // Создаем ограничения между всеми парами точек
  for (let i = 0; i < points.length; i++) {
    for (let j = i + 1; j < points.length; j++) {
      const p1 = points[i];
      const p2 = points[j];
      constraints.push({
        p1: p1,
        p2: p2,
        distance: p5.Vector.dist(p1.position, p2.position),
      });
    }
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

function drawCircle() {
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
  // Ищем ближайшую точку к позиции мышки
  let closestDistSq = Infinity;
  let closestPoint = null;

  for (const point of points) {
    const dx = point.position.x - mouseX;
    const dy = point.position.y - mouseY;
    const distSq = dx * dx + dy * dy;

    if (distSq < closestDistSq) {
      closestDistSq = distSq;
      closestPoint = point;
    }
  }

  if (closestPoint && closestDistSq < settings.dragRadius * settings.dragRadius) {
    draggedPoint = closestPoint;
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
  for (const point of points) {
    point.velocity.mult(0);
    if (!gravityEnabled) {
      point.acceleration.set(0, 0);
    } else {
      point.acceleration.set(settings.gravity.x, settings.gravity.y);
    }
  }
  
  const btn = document.getElementById('gravityBtn');
  btn.textContent = gravityEnabled ? 'Гравитация: ВКЛ' : 'Гравитация: ВЫКЛ';
}

function resetSimulation() {
  // Clear all points and constraints
  points = [];
  constraints = [];
  draggedPoint = null;
  
  // Recreate the circle
  const center = createVector(settings.sceneWidth / 2, settings.sceneHeight / 2 - 100);
  createCircle(center, settings.circleRadius, settings.circleNumPoints);
  
  // Применяем состояние гравитации к новым точкам
  if (!gravityEnabled) {
    for (const point of points) {
      point.acceleration.set(0, 0);
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
  
  const dampingInput = document.getElementById('dampingInput');
  const dampingValue = document.getElementById('dampingValue');
  if (dampingInput) {
    dampingInput.addEventListener('input', (e) => {
      const value = parseFloat(e.target.value);
      settings.dampingFactor = value;
      dampingValue.textContent = value.toFixed(2);
    });
  }
  
  const flexibilityInput = document.getElementById('flexibilityInput');
  const flexibilityValue = document.getElementById('flexibilityValue');
  if (flexibilityInput) {
    flexibilityInput.addEventListener('input', (e) => {
      const value = parseFloat(e.target.value);
      settings.flexibility = value;
      flexibilityValue.textContent = value.toFixed(2);
    });
  }
});


