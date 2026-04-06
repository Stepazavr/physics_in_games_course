// =================================
// Simulation Settings
// =================================
const settings = {
  // Canvas
  canvasWidth: 800,
  canvasHeight: 600,
  
  // Gravity
  gravity: { x: 0, y: 980, z: 0 },
  timeStep: 0.01,
  solverIterations: 3,
  groundY: 200,
  restitution: 0.5,
  
  // Sphere
  sphereCenter: { x: 0, y: -100, z: 0 },
  sphereRadius: 60,
  sphereNumPoints: 20,
  
  // Rendering
  backgroundColor: 200,
  ambientLightIntensity: 100,
  pointLightColor: { r: 255, g: 255, b: 255 },
  pointLightPos: { x: -300, y: -400, z: 400 },
  groundColor: 150,
  groundSize: 1000,
  groundOffset: 5,
  
  // Constraints rendering
  constraintStrokeColor: { r: 100, g: 100, b: 255, a: 80 },
  constraintStrokeWeight: 1,
  
  // Points rendering
  pointRadius: 7,
  pointColor: { r: 255, g: 255, b: 255 },
  pointDragColor: { r: 255, g: 255, b: 0 },
  
  // Constraints
  constraintErrorThreshold: 1e-1,
  useApproximateRoot: true,
  
  // Mouse interaction
  dragRadius: 8,
}

// =================================
// Data Structures
// =================================
let points = [];
let constraints = [];

// Mouse and Camera Control
let draggedPoint = null;

// Simulation state
let gravityEnabled = true;

// =================================
// Setup
// =================================
function setup() {
  createCanvas(settings.canvasWidth, settings.canvasHeight, WEBGL);
  angleMode(RADIANS);
  const center = createVector(settings.sphereCenter.x, settings.sphereCenter.y, settings.sphereCenter.z);
  createSphere(center, settings.sphereRadius, settings.sphereNumPoints);
}

// =================================
// Main Loop
// =================================
function draw() {
  // Update simulation
  simulate();

  // Rendering
  background(settings.backgroundColor);
  
  // Standard orbit control, disabled when Shift is held down
  if (!draggedPoint) {
    orbitControl();
  }

  // Lighting
  ambientLight(settings.ambientLightIntensity);
  pointLight(settings.pointLightColor.r, settings.pointLightColor.g, settings.pointLightColor.b,
             settings.pointLightPos.x, settings.pointLightPos.y, settings.pointLightPos.z);

  // Draw elements
  drawGround();
  drawSphere();
  
  // Сохраняем экранные координаты для каждой точки
  for (const point of points) {
    const screenPos = transformToScreenCoordinates(point.position);
    point.sx = screenPos.x;
    point.sy = screenPos.y;
  }
}

// =================================
// Simulation Core
// =================================
function simulate() {
  applyGravity();
  // Drag handling is now in mouseDragged()
  updatePoints(settings.timeStep);
  solveConstraints(settings.solverIterations);
  handleCollisions();
}

// =================================
// Physics Systems
// =================================

function transformToScreenCoordinates(pos3D) {
  // Простое и надежное преобразование 3D в экранные координаты
  // Для WEBGL в p5.js центр экрана (origin) находится в центре canvas
  
  // Используем простой масштаб
  const scale = 2.5;
  
  // Преобразуем с учетом того что Y инвертирован в экранных координатах
  const screenX = settings.canvasWidth / 2 + pos3D.x * scale;
  const screenY = settings.canvasHeight / 2 - pos3D.y * scale;
  
  return { x: screenX, y: screenY };
}

function approximateDistance(vectorDiff) {
  // Быстрое вычисление длины вектора с аппроксимацией sqrt
  // delta *= restlength*restlength/(delta*delta+restlength*restlength)-0.5
  // Это используется для оптимизации в корректировке констрейнтов
  const dSquared = vectorDiff.x * vectorDiff.x + vectorDiff.y * vectorDiff.y + vectorDiff.z * vectorDiff.z;
  
  // Возвращаем sqrt(d²) через встроенную функцию
  // На практике здесь можно применить fast inverse sqrt для ещё большей оптимизации
  return Math.sqrt(dSquared);
}

function applyGravity() {
  if (!gravityEnabled) return; // Skip if gravity is disabled
  
  for (const point of points) {
    if (point.isFixed) continue;
    point.acceleration.add(settings.gravity.x, settings.gravity.y, settings.gravity.z);
  }
}

function updatePoints(dt) {
  if (!gravityEnabled) return; // Don't update positions when gravity is disabled
  
  for (const point of points) {
    if (point.isFixed) continue;
    const velocity = p5.Vector.sub(point.position, point.oldPosition);
    point.oldPosition.set(point.position);
    const displacement = p5.Vector.mult(point.acceleration, dt * dt);
    point.position.add(velocity).add(displacement);
    point.acceleration.mult(0);
  }
}

function solveConstraints(iterations) {
  for (let i = 0; i < iterations; i++) {
    for (const constraint of constraints) {
      const { p1, p2, distance: restLength } = constraint;
      
      // delta = x2 - x1
      const delta = p5.Vector.sub(p2.position, p1.position);
      
      // Вычисляем текущую длину delta
      let deltaLength = settings.useApproximateRoot
        ? approximateDistance(delta)
        : Math.sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
      
      // Пропускаем если точки совпадают
      if (deltaLength < 1e-6) continue;
      
      // Вычисляем inverse mass для каждой точки
      // Если точка зафиксирована, invMass = 0 (бесконечная масса)
      const invMass1 = p1.isFixed ? 0 : 1 / p1.mass;
      const invMass2 = p2.isFixed ? 0 : 1 / p2.mass;
      
      // Если обе точки зафиксированы, ничего не делаем
      if (invMass1 + invMass2 < 1e-6) continue;
      
      // diff = (deltaLength - restLength) / (deltaLength * (invMass1 + invMass2))
      const diff = (deltaLength - restLength) / (deltaLength * (invMass1 + invMass2));
      
      // x1 += invMass1 * delta * diff
      if (p1.isFixed === false) {
        const correction = p5.Vector.mult(delta, invMass1 * diff);
        p1.position.add(correction);
      }
      
      // x2 -= invMass2 * delta * diff
      if (p2.isFixed === false) {
        const correction = p5.Vector.mult(delta, invMass2 * diff);
        p2.position.sub(correction);
      }
    }
  }
}

function handleCollisions() {
  for (const point of points) {
    if (point.position.y > settings.groundY) {
      const velocity = p5.Vector.sub(point.position, point.oldPosition);
      point.position.y = settings.groundY;
      point.oldPosition.y = point.position.y + velocity.y * settings.restitution;
    }
  }
}

// =================================
// Object Creation
// =================================

function createSphere(center, radius, numPoints) {
  points = [];
  constraints = [];

  // Fibonacci sphere algorithm for uniform point distribution
  const phi = PI * (3 - sqrt(5)); // Golden angle in radians

  for (let i = 0; i < numPoints; i++) {
    // y ranges from 1 (north pole) to -1 (south pole)
    const y = 1 - (i / (numPoints - 1)) * 2;
    
    // Radius of the circle at this y-coordinate
    const radiusAtY = sqrt(1 - y * y);
    
    // Angle around the sphere
    const theta = i * phi;
    
    // Position on unit sphere
    const x = cos(theta) * radiusAtY;
    const z = sin(theta) * radiusAtY;
    
    // Scale by radius and add center offset
    const pos = createVector(x * radius, y * radius, z * radius).add(center);
    
    const point = {
      position: pos,
      oldPosition: pos.copy(),
      acceleration: createVector(0, 0, 0),
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

function drawGround() {
  push();
  translate(0, settings.groundY + settings.groundOffset, 0);
  rotateX(PI / 2);
  noStroke();
  fill(settings.groundColor);
  plane(settings.groundSize, settings.groundSize);
  pop();
}

function drawSphere() {
  stroke(settings.constraintStrokeColor.r, settings.constraintStrokeColor.g, 
         settings.constraintStrokeColor.b, settings.constraintStrokeColor.a);
  strokeWeight(settings.constraintStrokeWeight);
  for (const constraint of constraints) {
    line(
      constraint.p1.position.x, constraint.p1.position.y, constraint.p1.position.z,
      constraint.p2.position.x, constraint.p2.position.y, constraint.p2.position.z
    );
  }

  noStroke();
  for (const point of points) {
    push();
    translate(point.position.x, point.position.y, point.position.z);
    if (point === draggedPoint) {
      fill(settings.pointDragColor.r, settings.pointDragColor.g, settings.pointDragColor.b);
    } else {
      fill(settings.pointColor.r, settings.pointColor.g, settings.pointColor.b);
    }
    sphere(settings.pointRadius);
    pop();
  }
}

// =================================
// Mouse and Camera Interaction
// =================================

function mousePressed() {
  // Ищем ближайшую точку к позиции мышки
  let closestDistSq = Infinity;
  let closestPoint = null;

  for (const point of points) {
    const dx = point.sx - mouseX;
    const dy = point.sy - mouseY;
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
  
  // Вычисляем смещение мышки
  const mouseDeltaX = mouseX - pmouseX;
  const mouseDeltaY = mouseY - pmouseY;
  
  // Преобразуем экранное движение в 3D движение
  // Этот простой подход работает хорошо при орбитальной камере
  draggedPoint.position.x += mouseDeltaX * 0.5;
  draggedPoint.position.y += mouseDeltaY * 0.5;
  
  // Обновляем старую позицию для корректной физики
  draggedPoint.oldPosition.set(draggedPoint.position);
}

// p5.js built-in function that is called whenever the mouse is dragged
function mouseDragged() {
  if (draggedPoint) {
    handlePointDrag();
  }
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
  
  // Recreate the sphere
  const center = createVector(settings.sphereCenter.x, settings.sphereCenter.y, settings.sphereCenter.z);
  createSphere(center, settings.sphereRadius, settings.sphereNumPoints);
}

// Initialize button handlers after the page loads
window.addEventListener('load', () => {
  document.getElementById('gravityBtn').addEventListener('click', toggleGravity);
  document.getElementById('resetBtn').addEventListener('click', resetSimulation);
});


