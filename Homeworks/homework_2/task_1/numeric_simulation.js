// =================================
// Simulation Settings
// =================================
const settings = {
  sceneWidth: 800,
  sceneHeight: 600,
  
  gravity: { x: 0, y: 980 },
  timeStep: 0.01,
  solverIterations: 3,
  constraintStiffness: 1.0,
  flexibility: 1.0,
  frictionCoefficient: 0.5,
  
  backgroundColor: 200,
  
  constraintStrokeColor: { r: 100, g: 100, b: 255, a: 80 },
  constraintStrokeWeight: 2,
  
  pointRadius: 4,
  pointColor: { r: 255, g: 255, b: 255 },
  
  constraintErrorThreshold: 1e-1,
  useApproximateRoot: true,
  
  dragRadius: 15,
  
  wallStrokeColor: { r: 100, g: 100, b: 100 },
  wallStrokeWeight: 3,
  
  solverMethod: 'hitman',
}

// =================================
// Data Structures
// =================================
let figures = [];
let points = [];
let constraints = [];

let draggedPoint = null;
let draggedFigure = null;

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
  collectPointsAndConstraints();

  integrateVelocityAndPosition(settings.timeStep);
  
  handlePointDrag();
  
  detectAndAddCollisions();

  detectObjectCollisions();
  
  const allConstraints = [...constraints, ...frameCollisionConstraints];
  if (currentSolverMethod === 'hitman') {
    solveConstraints_Hitman(allConstraints, settings.solverIterations);
  } else if (currentSolverMethod === 'pbd') {
    solveConstraints_PBD(allConstraints, settings.solverIterations);
  } else if (currentSolverMethod === 'xpbd') {
    solveConstraints_XPBD(allConstraints, settings.solverIterations);
  }
  
  updateAfterConstraints(settings.timeStep);
  
  constrainAllPointsToWalls();
  
  applyCollisionResponseToVelocities();
}

function collectPointsAndConstraints() {
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
  const dSquared = vectorDiff.x * vectorDiff.x + vectorDiff.y * vectorDiff.y;
  return Math.sqrt(dSquared);
}

function integrateVelocityAndPosition(dt) {
  for (const point of points) {
    if (point.isFixed) continue;
    
    point.velocity.add(p5.Vector.mult(point.acceleration, dt));
    
    point.positionPredicted.set(point.position);
    point.positionPredicted.add(p5.Vector.mult(point.velocity, dt));
  }
}

function updateAfterConstraints(dt) {
  for (const point of points) {
    if (point.isFixed) continue;
    const displacement = p5.Vector.sub(point.positionPredicted, point.position);
    point.velocity.set(displacement);
    point.velocity.mult(1 / dt);
    point.position.set(point.positionPredicted);
  }
}

function detectAndAddCollisions() {
  frameCollisionConstraints = [];

  for (const point of points) {
    if (point.isFixed) continue;
    
    const padding = 0;
    
    if (point.positionPredicted.x <= padding) {
      const nx = 1;
      const ny = 0;
      const qs_x = 0;
      const qs_y = point.positionPredicted.y;
      createAndAddCollisionConstraint(point, nx, ny, qs_x, qs_y);
    }
    else if (point.positionPredicted.x >= settings.sceneWidth - padding) {
      const nx = -1;
      const ny = 0;
      const qs_x = settings.sceneWidth;
      const qs_y = point.positionPredicted.y;
      createAndAddCollisionConstraint(point, nx, ny, qs_x, qs_y);
    }
    
    if (point.positionPredicted.y <= padding) {
      const nx = 0;
      const ny = 1;
      const qs_x = point.positionPredicted.x;
      const qs_y = 0;
      createAndAddCollisionConstraint(point, nx, ny, qs_x, qs_y);
    }
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
  const C = (point.positionPredicted.x - qs_x) * nx + (point.positionPredicted.y - qs_y) * ny;
  
  if (C < 0) {
    frameCollisionConstraints.push({
      type: 'collision',
      p1: point,
      normalX: nx,
      normalY: ny,
      surface_x: qs_x,
      surface_y: qs_y,
      gradP1_x: nx,
      gradP1_y: ny,
      lambda: 0,
    });
  }
}

function constrainAllPointsToWalls() {
  for (const point of points) {
    if (point.position.x < 0) point.position.x = 0;
    if (point.position.x > settings.sceneWidth) point.position.x = settings.sceneWidth;
    if (point.position.y < 0) point.position.y = 0;
    if (point.position.y > settings.sceneHeight) point.position.y = settings.sceneHeight;
  }
}

function applyCollisionResponseToVelocities() {
  for (const constraint of frameCollisionConstraints) {
    if (constraint.type !== 'collision') continue;
    
    const point = constraint.p1;
    const nx = constraint.normalX;
    const ny = constraint.normalY;
    const e = settings.flexibility;
    const mu = settings.frictionCoefficient;
    
    const vx = point.velocity.x;
    const vy = point.velocity.y;
    
    const vDotN = vx * nx + vy * ny;
    
    let newVx = vx;
    let newVy = vy;
    
    if (vDotN < 0) {
      const factor = (1 + e) * vDotN;
      newVx = vx - factor * nx;
      newVy = vy - factor * ny;
    }
    
    const tangent_x = newVx - vDotN * nx;
    const tangent_y = newVy - vDotN * ny;
    
    const tangent_friction_x = tangent_x * (1 - mu);
    const tangent_friction_y = tangent_y * (1 - mu);
    
    point.velocity.x = vDotN * nx + tangent_friction_x;
    point.velocity.y = vDotN * ny + tangent_friction_y;
  }
}

function solveConstraints_Hitman(allConstraints, iterations) {
  for (let i = 0; i < iterations; i++) {
    for (const constraint of allConstraints) {
      if (constraint.type === 'distance') {
        solveDistanceConstraint_Hitman(constraint);
      } else if (constraint.type === 'collision') {
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
  const { p1, normalX: nx, normalY: ny, surface_x: qs_x, surface_y: qs_y } = constraint;
  
  const px = p1.positionPredicted.x;
  const py = p1.positionPredicted.y;
  
  const C = (px - qs_x) * nx + (py - qs_y) * ny;
  
  if (C < 0) {
    const correction = -C * settings.constraintStiffness;
    p1.positionPredicted.x += correction * nx;
    p1.positionPredicted.y += correction * ny;
  }
}

function solveConstraints_PBD(allConstraints, iterations) {
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
  const { p1, normalX: nx, normalY: ny, surface_x: qs_x, surface_y: qs_y } = constraint;
  
  const px = p1.positionPredicted.x;
  const py = p1.positionPredicted.y;
  
  const C = (px - qs_x) * nx + (py - qs_y) * ny;
  
  if (C < 0) {
    const w1 = p1.isFixed ? 0 : 1 / p1.mass;
    
    if (w1 < 1e-6) return;
    
    const s = C / w1;
    
    const correction = -s * w1 * settings.constraintStiffness;
    p1.positionPredicted.x += correction * nx;
    p1.positionPredicted.y += correction * ny;
  }
}

// =================================
// XPBD Solver
// =================================

function solveConstraints_XPBD(allConstraints, iterations) {
  for (const constraint of allConstraints) {
    if (constraint.type === 'distance' || constraint.type === 'collision') {
      constraint.lambda = 0;
    }
  }
  
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
  if (constraint.lambda === undefined) {
    constraint.lambda = 0;
  }
  
  const { p1, p2, distance: restLength } = constraint;
  
  const delta = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
  const deltaLength = approximateDistance(delta);
  
  if (deltaLength < 1e-6) return;
  
  const gradP1_x = -delta.x / deltaLength;
  const gradP1_y = -delta.y / deltaLength;
  const gradP2_x = delta.x / deltaLength;
  const gradP2_y = delta.y / deltaLength;
  
  const C = deltaLength - restLength;
  
  const k = settings.constraintStiffness;
  const compliance = k > 0 ? 1.0 / k : 0;
  
  const w1 = p1.isFixed ? 0 : 1 / p1.mass;
  const w2 = p2.isFixed ? 0 : 1 / p2.mass;
  
  const gradGrad = 1.0 * (w1 + w2);
  
  if (w1 + w2 < 1e-6) return;
  
  const denom = gradGrad + compliance;
  const deltaLambda = -(C + compliance * constraint.lambda) / denom;
  
  constraint.lambda += deltaLambda;
  
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
  if (constraint.lambda === undefined) {
    constraint.lambda = 0;
  }
  
  const { p1, normalX: nx, normalY: ny, surface_x: qs_x, surface_y: qs_y } = constraint;
  
  const px = p1.positionPredicted.x;
  const py = p1.positionPredicted.y;
  
  const C = (px - qs_x) * nx + (py - qs_y) * ny;
  
  if (C < 0) {
    const k = settings.constraintStiffness;
    const compliance = k > 0 ? 1.0 / k : 0;
    
    const w1 = p1.isFixed ? 0 : 1 / p1.mass;
    
    if (w1 < 1e-6) return;
    
    const gradGrad = 1.0 * w1;
    
    const denom = gradGrad + compliance;
    const deltaLambda = -(C + compliance * constraint.lambda) / denom;
    
    constraint.lambda += deltaLambda;
    
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
  
  line(0, settings.sceneHeight, settings.sceneWidth, settings.sceneHeight);
  line(0, 0, settings.sceneWidth, 0);
  line(0, 0, 0, settings.sceneHeight);
  line(settings.sceneWidth, 0, settings.sceneWidth, settings.sceneHeight);
}

function drawScene() {
  stroke(settings.constraintStrokeColor.r, settings.constraintStrokeColor.g, 
         settings.constraintStrokeColor.b, settings.constraintStrokeColor.a);
  strokeWeight(settings.constraintStrokeWeight);
  for (const constraint of constraints) {
    line(
      constraint.p1.position.x, constraint.p1.position.y,
      constraint.p2.position.x, constraint.p2.position.y
    );
  }

  noStroke();
  for (const point of points) {
    if (point === draggedPoint) {
      fill(255, 200, 0);
    } else if (point.color) {
      fill(point.color.r, point.color.g, point.color.b);
    } else {
      fill(settings.pointColor.r, settings.pointColor.g, settings.pointColor.b);
    }
    const pointRadius = point.figure ? point.figure.pointRadius : settings.pointRadius;
    circle(point.position.x, point.position.y, pointRadius * 2);
  }
}

// =================================
// Mouse and Interaction
// =================================

function mousePressed() {
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
  if (!draggedPoint) return;
  
  draggedPoint.position.x = mouseX;
  draggedPoint.position.y = mouseY;
  draggedPoint.positionPredicted.set(draggedPoint.position);
  draggedPoint.velocity.mult(0);
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
  initScene();
  draggedPoint = null;
  draggedFigure = null;
  
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

  if (figure.convex_points) {
    for (const point of figure.convex_points) {
      point.normal = p5.Vector.sub(point.position, figure.center).normalize();
    }
  }
}

function detectObjectCollisions() {
    for (let i = 0; i < figures.length; i++) {
        for (let j = i + 1; j < figures.length; j++) {
            const fig1 = figures[i];
            const fig2 = figures[j];

            if (!fig1.convex_points || !fig2.convex_points) continue;

            updateFigureCenter(fig1);
            updateFigureCenter(fig2);

            const pointRadius1 = fig1.pointRadius;
            const pointRadius2 = fig2.pointRadius;

            for (const p1 of fig1.convex_points) {
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
                
                const k = closestIdx;
                
                {
                    const prevIdx = (k - 1 + fig2.convex_points.length) % fig2.convex_points.length;
                    const p2 = fig2.convex_points[prevIdx];
                    const p3 = fig2.convex_points[k];
                    const edgeNormal = p5.Vector.add(p2.normal, p3.normal).normalize();
                    const vectorToP1 = p5.Vector.sub(p1.positionPredicted, p2.positionPredicted);
                    const distToEdge = vectorToP1.dot(edgeNormal);
                    if (distToEdge < pointRadius1) {
                        const pushForce = (pointRadius1 - distToEdge) * settings.constraintStiffness;
                        p1.positionPredicted.x += pushForce * edgeNormal.x;
                        p1.positionPredicted.y += pushForce * edgeNormal.y;
                    }
                }
                
                {
                    const p2 = fig2.convex_points[k];
                    const p3 = fig2.convex_points[(k + 1) % fig2.convex_points.length];
                    const edgeNormal = p5.Vector.add(p2.normal, p3.normal).normalize();
                    const vectorToP1 = p5.Vector.sub(p1.positionPredicted, p2.positionPredicted);
                    const distToEdge = vectorToP1.dot(edgeNormal);
                    if (distToEdge < pointRadius1) {
                        const pushForce = (pointRadius1 - distToEdge) * settings.constraintStiffness;
                        p1.positionPredicted.x += pushForce * edgeNormal.x;
                        p1.positionPredicted.y += pushForce * edgeNormal.y;
                    }
                }
            }

            for (const p2 of fig2.convex_points) {
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
                
                const k = closestIdx;
                
                {
                    const prevIdx = (k - 1 + fig1.convex_points.length) % fig1.convex_points.length;
                    const p1 = fig1.convex_points[prevIdx];
                    const p3 = fig1.convex_points[k];
                    const edgeNormal = p5.Vector.add(p1.normal, p3.normal).normalize();
                    const vectorToP2 = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
                    const distToEdge = vectorToP2.dot(edgeNormal);
                    if (distToEdge < pointRadius2) {
                        const pushForce = (pointRadius2 - distToEdge) * settings.constraintStiffness;
                        p2.positionPredicted.x += pushForce * edgeNormal.x;
                        p2.positionPredicted.y += pushForce * edgeNormal.y;
                    }
                }
                
                {
                    const p1 = fig1.convex_points[k];
                    const p3 = fig1.convex_points[(k + 1) % fig1.convex_points.length];
                    const edgeNormal = p5.Vector.add(p1.normal, p3.normal).normalize();
                    const vectorToP2 = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
                    const distToEdge = vectorToP2.dot(edgeNormal);
                    if (distToEdge < pointRadius2) {
                        const pushForce = (pointRadius2 - distToEdge) * settings.constraintStiffness;
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
  const S = settings.solverIterations;
  const dt = settings.timeStep;
  const d = 0.8;
  
  for (const figure of figures) {
    if (!figure.points || figure.points.length < 2) continue;
    
    const pointsInFigure = figure.points;
    const R = figure.pointRadius;
    
    for (let i = 0; i < pointsInFigure.length; i++) {
      for (let j = i + 1; j < pointsInFigure.length; j++) {
        const p1 = pointsInFigure[i];
        const p2 = pointsInFigure[j];
        
        if (p1.isFixed && p2.isFixed) continue;
        
        const delta = p5.Vector.sub(p2.positionPredicted, p1.positionPredicted);
        const currentDistance = delta.mag();
        
        if (currentDistance < 0.001) continue;
        
        let restDistance = currentDistance;
        
        for (const constraint of figure.constraints) {
          if (constraint.type === 'distance') {
            if ((constraint.p1 === p1 && constraint.p2 === p2) ||
                (constraint.p1 === p2 && constraint.p2 === p1)) {
              restDistance = constraint.distance;
              break;
            }
          }
        }
        
        const D_collision = Math.min(2 * R, restDistance);
        
        if (currentDistance < D_collision) {
          const correction = (D_collision - currentDistance) / 2;
          const direction = delta.copy().normalize();
          
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


