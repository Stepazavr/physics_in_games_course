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
  restitution: 0.5,
  
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
  applyGravity();
  handlePointDrag();
  updatePoints(settings.timeStep);
  
  // Use selected solver method
  if (currentSolverMethod === 'hitman') {
    solveConstraints_Hitman(settings.solverIterations);
  } else if (currentSolverMethod === 'pbd') {
    solveConstraints_PBD(settings.solverIterations);
  }
  
  handleCollisions();
}

// =================================
// Physics Systems
// =================================

function approximateDistance(vectorDiff) {
  // Быстрое вычисление длины вектора с аппроксимацией sqrt
  const dSquared = vectorDiff.x * vectorDiff.x + vectorDiff.y * vectorDiff.y;
  return Math.sqrt(dSquared);
}

function applyGravity() {
  if (!gravityEnabled) return;
  
  for (const point of points) {
    if (point.isFixed) continue;
    point.acceleration.add(settings.gravity.x, settings.gravity.y);
  }
}

function updatePoints(dt) {
  if (!gravityEnabled) return;
  
  for (const point of points) {
    if (point.isFixed) continue;
    const velocity = p5.Vector.sub(point.position, point.oldPosition);
    point.oldPosition.set(point.position);
    const displacement = p5.Vector.mult(point.acceleration, dt * dt);
    point.position.add(velocity).add(displacement);
    point.acceleration.mult(0);
  }
}

function solveConstraints_Hitman(iterations) {
  // Итеративный метод Gauss-Seidel (исходный алгоритм Hitman)
  for (let i = 0; i < iterations; i++) {
    for (const constraint of constraints) {
      const { p1, p2, distance: restLength } = constraint;
      
      const delta = p5.Vector.sub(p2.position, p1.position);
      
      let deltaLength = settings.useApproximateRoot
        ? approximateDistance(delta)
        : Math.sqrt(delta.x * delta.x + delta.y * delta.y);
      
      if (deltaLength < 1e-6) continue;
      
      const invMass1 = p1.isFixed ? 0 : 1 / p1.mass;
      const invMass2 = p2.isFixed ? 0 : 1 / p2.mass;
      
      if (invMass1 + invMass2 < 1e-6) continue;
      
      const diff = (deltaLength - restLength) / (deltaLength * (invMass1 + invMass2));
      
      if (p1.isFixed === false) {
        const correction = p5.Vector.mult(delta, invMass1 * diff);
        p1.position.add(correction);
      }
      
      if (p2.isFixed === false) {
        const correction = p5.Vector.mult(delta, invMass2 * diff);
        p2.position.sub(correction);
      }
    }
  }
}

function solveConstraints_PBD(iterations) {
  // Классический метод Position Based Dynamics по формутам:
  // Δp_i = -s * w_i * ∇p_i(C)
  // s = C / Σ(w_k * (∇p_k(C))^2)
  
  for (let i = 0; i < iterations; i++) {
    for (const constraint of constraints) {
      const { p1, p2, distance: restLength } = constraint;
      
      // Вектор между точками
      const delta = p5.Vector.sub(p2.position, p1.position);
      const deltaLength = approximateDistance(delta);
      
      if (deltaLength < 1e-6) continue;
      
      // Constraint function: C = ||p2 - p1|| - restLength
      const C = deltaLength - restLength;
      
      // Gradient: ∇p1(C) = (p1 - p2) / ||p2 - p1||
      //           ∇p2(C) = (p2 - p1) / ||p2 - p1||
      const gradP1 = p5.Vector.mult(delta, -1 / deltaLength);
      const gradP2 = p5.Vector.mult(delta, 1 / deltaLength);
      
      // Inverse masses
      const w1 = p1.isFixed ? 0 : 1 / p1.mass;
      const w2 = p2.isFixed ? 0 : 1 / p2.mass;
      
      // Denominator: Σ(w_k * (∇p_k(C))^2)
      // Для двух точек: (∇p1(C))^2 = 1, (∇p2(C))^2 = 1
      const denom = w1 * 1 + w2 * 1;
      
      if (denom < 1e-6) continue;
      
      // Масштабный множитель: s = C / denom
      const s = C / denom;
      
      // Коррекция позиций: Δp_i = -s * w_i * ∇p_i(C)
      if (p1.isFixed === false) {
        const correction = p5.Vector.mult(gradP1, -s * w1);
        p1.position.add(correction);
      }
      
      if (p2.isFixed === false) {
        const correction = p5.Vector.mult(gradP2, -s * w2);
        p2.position.add(correction);
      }
    }
  }
}

function handleCollisions() {
  for (const point of points) {
    // Bottom wall (groundY)
    if (point.position.y > settings.sceneHeight) {
      const velocity = p5.Vector.sub(point.position, point.oldPosition);
      point.position.y = settings.sceneHeight;
      point.oldPosition.y = point.position.y + velocity.y * settings.restitution;
    }
    
    // Top wall
    if (point.position.y < 0) {
      const velocity = p5.Vector.sub(point.position, point.oldPosition);
      point.position.y = 0;
      point.oldPosition.y = point.position.y - velocity.y * settings.restitution;
    }
    
    // Left wall
    if (point.position.x < 0) {
      const velocity = p5.Vector.sub(point.position, point.oldPosition);
      point.position.x = 0;
      point.oldPosition.x = point.position.x - velocity.x * settings.restitution;
    }
    
    // Right wall
    if (point.position.x > settings.sceneWidth) {
      const velocity = p5.Vector.sub(point.position, point.oldPosition);
      point.position.x = settings.sceneWidth;
      point.oldPosition.x = point.position.x + velocity.x * settings.restitution;
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
  // Каждая точка на угле 360/N * i градусов
  for (let i = 0; i < numPoints; i++) {
    const angle = TWO_PI * (i / numPoints);
    
    const x = center.x + radius * cos(angle);
    const y = center.y + radius * sin(angle);
    
    const pos = createVector(x, y);
    
    const point = {
      position: pos,
      oldPosition: pos.copy(),
      acceleration: createVector(0, 0),
      mass: 1,
      isFixed: false,
    };
    
    points.push(point);
  }

  // Create constraints between all pairs of points
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
  
  // Обновляем старую позицию для корректной физики
  draggedPoint.oldPosition.set(draggedPoint.position);
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
  
  // Stop all movement when toggling gravity
  for (const point of points) {
    point.oldPosition.set(point.position);
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
});


