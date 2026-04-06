// =================================
// Simulation Settings
// =================================
const settings = {
  // Canvas
  canvasWidth: 800,
  canvasHeight: 600,
  
  // Gravity
  gravity: { x: 0, y: 980 },
  timeStep: 0.01,
  solverIterations: 3,
  groundY: 600,
  restitution: 0.5,
  
  // Circle
  circleCenter: { x: 0, y: 0 },
  circleRadius: 100,
  circleNumPoints: 20,
  
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
}

// =================================
// Data Structures
// =================================
let points = [];
let constraints = [];

// Simulation state
let gravityEnabled = true;

// =================================
// Setup
// =================================
function setup() {
  createCanvas(settings.canvasWidth, settings.canvasHeight);
  angleMode(RADIANS);
  const center = createVector(settings.canvasWidth / 2, settings.canvasHeight / 2 - 100);
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
  drawGround();
  drawCircle();
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

function solveConstraints(iterations) {
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

function drawGround() {
  stroke(150);
  strokeWeight(2);
  line(0, settings.groundY, settings.canvasWidth, settings.groundY);
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
  fill(settings.pointColor.r, settings.pointColor.g, settings.pointColor.b);
  for (const point of points) {
    circle(point.position.x, point.position.y, settings.pointRadius * 2);
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
  
  // Recreate the circle
  const center = createVector(settings.canvasWidth / 2, settings.canvasHeight / 2 - 100);
  createCircle(center, settings.circleRadius, settings.circleNumPoints);
}

// Initialize button handlers after the page loads
window.addEventListener('load', () => {
  document.getElementById('gravityBtn').addEventListener('click', toggleGravity);
  document.getElementById('resetBtn').addEventListener('click', resetSimulation);
});


