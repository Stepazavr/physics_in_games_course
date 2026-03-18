// -----------------------------------------------------------------------------
// BLOCK: STARTUP AND SIMULATION STATE
// -----------------------------------------------------------------------------
// Parameters of the system
let __task12SetupRan = false;

let ballMass = 1.0;
const BALL_RADIUS_PX = 15;
const SIM_DT = 0.01; // s
const PX_PER_M = 120;
const METHOD_IDS = ['explicitEuler', 'implicitEuler', 'semiImplicitEuler', 'verlet'];
const METHOD_INFO = {
  explicitEuler: { label: 'Явный Эйлер', color: [210, 70, 70] },
  implicitEuler: { label: 'Неявный Эйлер', color: [70, 150, 230] },
  semiImplicitEuler: { label: 'Полуявный Эйлер', color: [70, 180, 100] },
  verlet: { label: 'Верле', color: [190, 120, 50] }
};
let methodStates = {};
let selectedMethodId = 'semiImplicitEuler';
let energyMode = 'compare';

// Springs attached perpendicular to their walls
let springH = {
  wallType: 'horizontal',
  lengthM: 0.8,
  k: 20,
  color: [255, 100, 100],
  attachX: 0, attachY: 0,
  attachXM: 0, attachYM: 0
};

let springV = {
  wallType: 'vertical',
  lengthM: 0.8,
  k: 20,
  color: [100, 100, 255],
  attachX: 0, attachY: 0,
  attachXM: 0, attachYM: 0
};

let springG = {
  wallType: 'mu',
  lengthM: 0.8,
  k: 20,
  color: [60, 180, 75],
  attachX: 0, attachY: 0,
  attachXM: 0, attachYM: 0
};

// mu is the fixed angle of the third wall
const MU_FIXED_DEG = 30;
const MU_FIXED_RAD = MU_FIXED_DEG * Math.PI / 180;
let mu = MU_FIXED_RAD;
let dragging = false;

// Sliders and controls
let sliderK1, sliderK2, sliderK3;
let sliderLen1, sliderLen2, sliderLen3;
let sliderMass;
let resetButton;
let methodSelect;
let energyModeSelect;

// For plots
let energyHistory = {};
let trajectoryHistory = {};
let simTime = 0;

// -----------------------------------------------------------------------------
// BLOCK: LAYOUT AND UI CONSTANTS
// -----------------------------------------------------------------------------

// Layout
const UI_X = 170;
const TOP_RIGHT_UI_X = 810;
const MODEL_CORNER_X = 450;
const MODEL_CORNER_Y = 320;
const ENERGY_GRAPH_X = 520;
const ENERGY_GRAPH_Y = 360;
const ENERGY_GRAPH_W = 340;
const ENERGY_GRAPH_H = 220;
const CANVAS_W = 940;
const CANVAS_H = 680;

// UI controls layout (easy to tweak)
const TOP_PANEL_START_Y = 20;
const TOP_PANEL_STEP_Y = 65;
const METHOD_LABEL_Y = TOP_PANEL_START_Y + TOP_PANEL_STEP_Y * 0;
const METHOD_SELECT_Y = METHOD_LABEL_Y + 25;
const ENERGY_LABEL_Y = TOP_PANEL_START_Y + TOP_PANEL_STEP_Y * 1;
const ENERGY_SELECT_Y = ENERGY_LABEL_Y + 25;
const METHOD_SELECT_W = 230;

const CONTROLS_START_Y = 20;
const LABEL_TO_SLIDER_DY = 25;
const CONTROL_GROUP_STEP_Y = 65;
const MASS_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 0;
const MASS_SLIDER_Y = MASS_LABEL_Y + LABEL_TO_SLIDER_DY;
const K1_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 1;
const K1_SLIDER_Y = K1_LABEL_Y + LABEL_TO_SLIDER_DY;
const K2_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 2;
const K2_SLIDER_Y = K2_LABEL_Y + LABEL_TO_SLIDER_DY;
const K3_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 3;
const K3_SLIDER_Y = K3_LABEL_Y + LABEL_TO_SLIDER_DY;
const A_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 4;
const A_SLIDER_Y = A_LABEL_Y + LABEL_TO_SLIDER_DY;
const B_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 5;
const B_SLIDER_Y = B_LABEL_Y + LABEL_TO_SLIDER_DY;
const C_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 6;
const C_SLIDER_Y = C_LABEL_Y + LABEL_TO_SLIDER_DY;

const VAR_SLIDER_W = 210;
const RESET_BUTTON_X_OFFSET = 0;
const RESET_BUTTON_Y = C_SLIDER_Y + 40;

// Mouse interaction bounds for UI panel
const UI_BLOCK_MAX_X_OFFSET = 320;
const UI_BLOCK_MAX_Y = RESET_BUTTON_Y + 103;

// Bottom info text layout
const INFO_TEXT_X1 = 38;
const INFO_TEXT_X2 = 150;
const INFO_TEXT_X3 = 560;
const INFO_TEXT_Y_START = RESET_BUTTON_Y + 32;
const INFO_TEXT_LINE_STEP = 18;

// -----------------------------------------------------------------------------
// BLOCK: GEOMETRY STATE
// -----------------------------------------------------------------------------

// Geometry constants
let cornerX = MODEL_CORNER_X;
let cornerY = MODEL_CORNER_Y;
let cornerXM = 0;
let cornerYM = 0;

let wallLengthH = 0;
let wallLengthV = 0;
let wallLengthHM = 0;
let wallLengthVM = 0;

let muWallLengthM = 2.2;
let muWallX1 = 0;
let muWallY1 = 0;
let muWallX2 = 0;
let muWallY2 = 0;
let eqXM = 0;
let eqYM = 0;

// -----------------------------------------------------------------------------
// BLOCK: UI INITIALIZATION
// -----------------------------------------------------------------------------

function setup() {
  __task12SetupRan = true;
  createCanvas(CANVAS_W, CANVAS_H);
  cornerXM = cornerX / PX_PER_M;
  cornerYM = cornerY / PX_PER_M;

  let labelMethod = createDiv('Численный метод');
  labelMethod.position(TOP_RIGHT_UI_X, METHOD_LABEL_Y);
  methodSelect = createSelect();
  methodSelect.position(TOP_RIGHT_UI_X, METHOD_SELECT_Y);
  methodSelect.style('width', METHOD_SELECT_W + 'px');
  methodSelect.option(METHOD_INFO.explicitEuler.label, 'explicitEuler');
  methodSelect.option(METHOD_INFO.implicitEuler.label, 'implicitEuler');
  methodSelect.option(METHOD_INFO.semiImplicitEuler.label, 'semiImplicitEuler');
  methodSelect.option(METHOD_INFO.verlet.label, 'verlet');
  methodSelect.selected(selectedMethodId);
  methodSelect.changed(function () {
    selectedMethodId = methodSelect.value();
  });

  let labelEnergyMode = createDiv('График E');
  labelEnergyMode.position(TOP_RIGHT_UI_X, ENERGY_LABEL_Y);
  energyModeSelect = createSelect();
  energyModeSelect.position(TOP_RIGHT_UI_X, ENERGY_SELECT_Y);
  energyModeSelect.style('width', METHOD_SELECT_W + 'px');
  energyModeSelect.option('Сравнение', 'compare');
  energyModeSelect.option('Одиночный метод', 'single');
  energyModeSelect.selected(energyMode);
  energyModeSelect.changed(function () {
    energyMode = energyModeSelect.value();
  });

  let labelMass = createDiv('m (kg)');
  labelMass.position(UI_X, MASS_LABEL_Y);
  sliderMass = createSlider(0.1, 10, 1, 0.1);
  sliderMass.position(UI_X, MASS_SLIDER_Y);
  sliderMass.style('width', VAR_SLIDER_W + 'px');

  let labelK1 = createDiv('k1 (N/m), красная');
  labelK1.position(UI_X, K1_LABEL_Y);
  sliderK1 = createSlider(5, 80, 20, 1);
  sliderK1.position(UI_X, K1_SLIDER_Y);
  sliderK1.style('width', VAR_SLIDER_W + 'px');

  let labelK2 = createDiv('k2 (N/m), синяя');
  labelK2.position(UI_X, K2_LABEL_Y);
  sliderK2 = createSlider(5, 80, 20, 1);
  sliderK2.position(UI_X, K2_SLIDER_Y);
  sliderK2.style('width', VAR_SLIDER_W + 'px');

  let labelK3 = createDiv('k3 (N/m), зеленая');
  labelK3.position(UI_X, K3_LABEL_Y);
  sliderK3 = createSlider(5, 80, 20, 1);
  sliderK3.position(UI_X, K3_SLIDER_Y);
  sliderK3.style('width', VAR_SLIDER_W + 'px');

  let labelLen1 = createDiv('a (m), красная');
  labelLen1.position(UI_X, A_LABEL_Y);
  sliderLen1 = createSlider(0.4, 1.8, 0.8, 0.01);
  sliderLen1.position(UI_X, A_SLIDER_Y);
  sliderLen1.style('width', VAR_SLIDER_W + 'px');
  sliderLen1.input(resetToEquilibrium);

  let labelLen2 = createDiv('b (m), синяя');
  labelLen2.position(UI_X, B_LABEL_Y);
  sliderLen2 = createSlider(0.4, 1.8, 0.8, 0.01);
  sliderLen2.position(UI_X, B_SLIDER_Y);
  sliderLen2.style('width', VAR_SLIDER_W + 'px');
  sliderLen2.input(resetToEquilibrium);

  let labelLen3 = createDiv('c (m), зеленая');
  labelLen3.position(UI_X, C_LABEL_Y);
  sliderLen3 = createSlider(0.4, 1.8, 0.8, 0.01);
  sliderLen3.position(UI_X, C_SLIDER_Y);
  sliderLen3.style('width', VAR_SLIDER_W + 'px');
  sliderLen3.input(resetToEquilibrium);

  resetButton = createButton('Сбросить в равновесие');
  resetButton.position(UI_X + RESET_BUTTON_X_OFFSET, RESET_BUTTON_Y);
  resetButton.mousePressed(resetToEquilibrium);

  updateGeometry();
  initializeMethodStates();
  resetToEquilibrium();
}

function initializeMethodStates() {
  methodStates = {};
  energyHistory = {};
  for (let i = 0; i < METHOD_IDS.length; i++) {
    let id = METHOD_IDS[i];
    methodStates[id] = {
      xM: 0,
      yM: 0,
      vx: 0,
      vy: 0,
      radiusPx: BALL_RADIUS_PX,
      trail: []
    };
    energyHistory[id] = [];
  }
}

// -----------------------------------------------------------------------------
// BLOCK: PHYSICS AND NUMERICAL METHODS
// -----------------------------------------------------------------------------

function updateGeometry() {
  let a = sliderLen1.value();
  let b = sliderLen2.value();
  let c = sliderLen3.value();
  mu = MU_FIXED_RAD;

  springH.lengthM = a;
  springV.lengthM = b;
  springG.lengthM = c;

  springH.k = sliderK1.value();
  springV.k = sliderK2.value();
  springG.k = sliderK3.value();

  // For mu = 90° model: horizontal and vertical walls from one corner.
  wallLengthHM = b;
  wallLengthVM = a;
  wallLengthH = wallLengthHM * PX_PER_M;
  wallLengthV = wallLengthVM * PX_PER_M;

  // Attach points of red and blue springs at ends of their walls.
  springH.attachXM = cornerXM + wallLengthHM;
  springH.attachYM = cornerYM;
  springH.attachX = springH.attachXM * PX_PER_M;
  springH.attachY = springH.attachYM * PX_PER_M;

  springV.attachXM = cornerXM;
  springV.attachYM = cornerYM - wallLengthVM;
  springV.attachX = springV.attachXM * PX_PER_M;
  springV.attachY = springV.attachYM * PX_PER_M;

  // Intersection of normals from first two walls.
  let eqXGeom = springH.attachXM;
  let eqYGeom = springV.attachYM;

  // Third wall: direction t3, normal n3.
  let tx3 = Math.cos(mu);
  let ty3 = -Math.sin(mu);
  let nx3 = -Math.sin(mu);
  let ny3 = Math.cos(mu);

  // Place green wall on the opposite side of the ball.
  springG.attachXM = eqXGeom - c * nx3;
  springG.attachYM = eqYGeom - c * ny3;
  springG.attachX = springG.attachXM * PX_PER_M;
  springG.attachY = springG.attachYM * PX_PER_M;

  muWallLengthM = Math.max(2.0, 1.3 + c);
  let halfLenPx = 0.5 * muWallLengthM * PX_PER_M;
  let wallTx = Math.cos(mu);
  let wallTy = Math.sin(mu);
  muWallX1 = springG.attachX - wallTx * halfLenPx;
  muWallY1 = springG.attachY - wallTy * halfLenPx;
  muWallX2 = springG.attachX + wallTx * halfLenPx;
  muWallY2 = springG.attachY + wallTy * halfLenPx;
}

function findEquilibriumPoint() {
  // Analog of task_1: solve intersection of spring-normal lines from walls.
  // For horizontal wall normal: x = const (through red attach point).
  // For vertical wall normal: y = const (through blue attach point).
  let eqX = springH.attachX;
  let eqY = springV.attachY;

  eqX = constrain(eqX, 160, width - 120);
  eqY = constrain(eqY, 80, height - 140);
  return { xM: eqX / PX_PER_M, yM: eqY / PX_PER_M };
}

function getAccelerationAt(xM, yM) {
  let sinMu = sin(mu);
  let cosMu = cos(mu);

  let x = xM - eqXM;
  let y = yM - eqYM;

  let k1 = springH.k;
  let k2 = springV.k;
  let k3 = springG.k;

  let axMath = (-(k2 + sinMu * sinMu * k3) * x - cosMu * sinMu * k3 * y) / ballMass;
  let ayMath = (-cosMu * sinMu * k3 * x - (k1 + cosMu * cosMu * k3) * y) / ballMass;

  return {
    ax: axMath,
    ay: ayMath
  };
}

function stepExplicitEuler(state, dt) {
  let a = getAccelerationAt(state.xM, state.yM);
  state.xM += state.vx * dt;
  state.yM += state.vy * dt;
  state.vx += a.ax * dt;
  state.vy += a.ay * dt;
}

function stepSemiImplicitEuler(state, dt) {
  let a = getAccelerationAt(state.xM, state.yM);
  state.vx += a.ax * dt;
  state.vy += a.ay * dt;
  state.xM += state.vx * dt;
  state.yM += state.vy * dt;
}

function stepImplicitEuler(state, dt) {
  let x0 = state.xM;
  let y0 = state.yM;
  let vx0 = state.vx;
  let vy0 = state.vy;

  let xNext = x0 + vx0 * dt;
  let yNext = y0 + vy0 * dt;
  let vxNext = vx0;
  let vyNext = vy0;

  for (let i = 0; i < 8; i++) {
    let aNext = getAccelerationAt(xNext, yNext);
    vxNext = vx0 + aNext.ax * dt;
    vyNext = vy0 + aNext.ay * dt;

    let xCandidate = x0 + vxNext * dt;
    let yCandidate = y0 + vyNext * dt;

    if (abs(xCandidate - xNext) < 0.0001 && abs(yCandidate - yNext) < 0.0001) {
      xNext = xCandidate;
      yNext = yCandidate;
      break;
    }

    xNext = xCandidate;
    yNext = yCandidate;
  }

  state.xM = xNext;
  state.yM = yNext;
  state.vx = vxNext;
  state.vy = vyNext;
}

function stepVerlet(state, dt) {
  let a0 = getAccelerationAt(state.xM, state.yM);
  let xNext = state.xM + state.vx * dt + 0.5 * a0.ax * dt * dt;
  let yNext = state.yM + state.vy * dt + 0.5 * a0.ay * dt * dt;
  let a1 = getAccelerationAt(xNext, yNext);

  state.vx += 0.5 * (a0.ax + a1.ax) * dt;
  state.vy += 0.5 * (a0.ay + a1.ay) * dt;
  state.xM = xNext;
  state.yM = yNext;
}

function integrateState(state, methodId, dt) {
  if (methodId === 'explicitEuler') {
    stepExplicitEuler(state, dt);
  } else if (methodId === 'implicitEuler') {
    stepImplicitEuler(state, dt);
  } else if (methodId === 'semiImplicitEuler') {
    stepSemiImplicitEuler(state, dt);
  } else {
    stepVerlet(state, dt);
  }
}

function getTotalEnergy(state) {
  let sinMu = sin(mu);
  let cosMu = cos(mu);

  let x = state.xM - eqXM;
  let y = state.yM - eqYM;
  let vx = state.vx;
  let vy = state.vy;

  let k1 = springH.k;
  let k2 = springV.k;
  let k3 = springG.k;

  let kinetic = 0.5 * ballMass * (vx * vx + vy * vy);
  let potential =
    0.5 * (k2 + sinMu * sinMu * k3) * x * x
    + cosMu * sinMu * k3 * x * y
    + 0.5 * (k1 + cosMu * cosMu * k3) * y * y;

  return kinetic + potential;
}

function getActiveState() {
  return methodStates[selectedMethodId];
}

// -----------------------------------------------------------------------------
// BLOCK: RENDER PIPELINE ORCHESTRATION
// -----------------------------------------------------------------------------

function drawTrajectory() {
  let idsToDraw = energyMode === 'single' ? [selectedMethodId] : METHOD_IDS;
  push();
  noFill();
  strokeWeight(2);
  for (let i = 0; i < idsToDraw.length; i++) {
    let id = idsToDraw[i];
    let hist = trajectoryHistory[id];
    if (!hist || hist.length < 2) continue;
    let c = METHOD_INFO[id].color;
    stroke(c[0], c[1], c[2], 160);
    beginShape();
    for (let j = 0; j < hist.length; j++) {
      vertex(hist[j].xM * PX_PER_M, hist[j].yM * PX_PER_M);
    }
    endShape();
  }
  pop();
}

function updateHistory() {
  if (frameCount % 2 !== 0) {
    return;
  }

  for (let i = 0; i < METHOD_IDS.length; i++) {
    let id = METHOD_IDS[i];
    energyHistory[id].push({ t: simTime, val: getTotalEnergy(methodStates[id]) });
  }
}

function applyPhysicsInputs() {
  ballMass = sliderMass.value();
  updateGeometry();
}

function runPhysicsStep(dt) {
  if (dragging) {
    return;
  }

  for (let i = 0; i < METHOD_IDS.length; i++) {
    let id = METHOD_IDS[i];
    integrateState(methodStates[id], id, dt);
    methodStates[id].trail.push({ xM: methodStates[id].xM, yM: methodStates[id].yM });
    trajectoryHistory[id].push({ xM: methodStates[id].xM, yM: methodStates[id].yM });
    while (methodStates[id].trail.length > 220) {
      methodStates[id].trail.shift();
    }
  }
  simTime += dt;
}

function getActiveRenderState() {
  let active = getActiveState();
  return {
    state: active,
    xPx: active.xM * PX_PER_M,
    yPx: active.yM * PX_PER_M,
    color: METHOD_INFO[selectedMethodId].color
  };
}

function drawModel(renderState) {
  drawWalls();
  drawTrajectory();

  drawSpring(springH.attachX, springH.attachY, renderState.xPx, renderState.yPx, springH.color, springH.lengthM * PX_PER_M);
  drawSpring(springV.attachX, springV.attachY, renderState.xPx, renderState.yPx, springV.color, springV.lengthM * PX_PER_M);
  drawSpring(springG.attachX, springG.attachY, renderState.xPx, renderState.yPx, springG.color, springG.lengthM * PX_PER_M);
  drawReferenceAxesAndMu();

  fill(renderState.color[0], renderState.color[1], renderState.color[2]);
  stroke(50);
  strokeWeight(2);
  ellipse(renderState.xPx, renderState.yPx, renderState.state.radiusPx * 2, renderState.state.radiusPx * 2);

  if (dragging) {
    noFill();
    stroke(0, 255, 0);
    strokeWeight(3);
    ellipse(renderState.xPx, renderState.yPx, renderState.state.radiusPx * 2.5, renderState.state.radiusPx * 2.5);
  }
}

function drawInfoOverlay() {
  fill(0);
  noStroke();
  text('method = ' + METHOD_INFO[selectedMethodId].label, INFO_TEXT_X1, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 0);
  text('μ = ' + nf(mu * 180 / PI, 0, 1) + '°, dt = ' + SIM_DT.toFixed(3) + ' s', INFO_TEXT_X1, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 1);
  text('m = ' + ballMass.toFixed(2) + ' kg', INFO_TEXT_X1, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 2);
  text('k1 = ' + springH.k.toFixed(1) + ', a = ' + springH.lengthM.toFixed(2), INFO_TEXT_X1, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 3);
  text('k2 = ' + springV.k.toFixed(1) + ', b = ' + springV.lengthM.toFixed(2), INFO_TEXT_X2, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 3);
  text('k3 = ' + springG.k.toFixed(1) + ', c = ' + springG.lengthM.toFixed(2), INFO_TEXT_X3, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 3);
}

// -----------------------------------------------------------------------------
// BLOCK: FRAME LOOP
// -----------------------------------------------------------------------------

function draw() {
  background(240);
  applyPhysicsInputs();
  runPhysicsStep(SIM_DT);

  let renderState = getActiveRenderState();
  drawModel(renderState);

  updateHistory();
  drawGraphs();
  drawInfoOverlay();
}

// -----------------------------------------------------------------------------
// BLOCK: LOW-LEVEL DRAW HELPERS
// -----------------------------------------------------------------------------

function drawWalls() {
  push();
  stroke(139, 69, 19);
  strokeWeight(4);

  // Lower horizontal wall
  line(cornerX, cornerY, cornerX + wallLengthH, cornerY);

  // Side vertical wall
  line(cornerX, cornerY, cornerX, cornerY - wallLengthV);

  // Third (mu) wall on the opposite side
  line(muWallX1, muWallY1, muWallX2, muWallY2);

  fill(139, 69, 19);
  noStroke();
  ellipse(cornerX, cornerY, 10, 10);

  pop();
}

function drawSpring(x1, y1, x2, y2, col, restLenPx) {
  push();
  stroke(col);
  strokeWeight(2);
  noFill();

  fill(col);
  ellipse(x1, y1, 8, 8);

  let lenPx = dist(x1, y1, x2, y2);
  let baseLenPx = max(restLenPx, 1);
  let targetPitchPx = 14;
  let steps = constrain(floor(baseLenPx / targetPitchPx), 8, 40);
  let coilAmplitude = constrain(0.08 * baseLenPx, 4, 8);

  beginShape();
  for (let i = 0; i <= steps; i++) {
    let t = i / steps;
    let x = lerp(x1, x2, t);
    let y = lerp(y1, y2, t);

    if (i > 0 && i < steps) {
      let ang = atan2(y2 - y1, x2 - x1);
      let perp = (i % 2 === 0) ? coilAmplitude : -coilAmplitude;
      x += perp * cos(ang + HALF_PI);
      y += perp * sin(ang + HALF_PI);
    }
    vertex(x, y);
  }
  endShape();

  pop();
}

function drawReferenceAxesAndMu() {
  let refX = springH.attachX;
  let refY = springV.attachY;
  let upAngle = -HALF_PI;
  let springAngle = atan2(springG.attachY - refY, springG.attachX - refX);

  let axisLen = 95;
  let springAxisLen = 95;

  push();
  stroke(70);
  strokeWeight(1.5);
  noFill();
  drawingContext.setLineDash([6, 6]);

  // Fixed dashed axis upward.
  line(refX, refY, refX, refY - axisLen);

  // Fixed dashed axis along the green spring direction.
  line(
    refX,
    refY,
    refX + springAxisLen * cos(springAngle),
    refY + springAxisLen * sin(springAngle)
  );

  drawingContext.setLineDash([]);

  // Draw the smaller angle between the two dashed axes.
  let delta = springAngle - upAngle;
  while (delta > PI) delta -= TWO_PI;
  while (delta < -PI) delta += TWO_PI;

  let startA = upAngle;
  let endA = upAngle + delta;
  if (delta < 0) {
    startA = upAngle + delta;
    endA = upAngle;
  }

  stroke(0, 120, 0);
  strokeWeight(2);
  let r = 42;
  arc(refX, refY, 2 * r, 2 * r, startA, endA);

  let midA = upAngle + delta * 0.5;
  noStroke();
  fill(0, 120, 0);
  text('μ', refX + (r + 12) * cos(midA), refY + (r + 12) * sin(midA));
  pop();
}

function drawGraphs() {
  drawEnergyGraph(ENERGY_GRAPH_X, ENERGY_GRAPH_Y, ENERGY_GRAPH_W, ENERGY_GRAPH_H);
}

function drawEnergyGraph(x0, y0, w, h) {
  push();
  translate(x0, y0);

  let idsToDraw = energyMode === 'single' ? [selectedMethodId] : METHOD_IDS;
  let minE = Infinity;
  let maxE = -Infinity;
  for (let i = 0; i < idsToDraw.length; i++) {
    let id = idsToDraw[i];
    for (let j = 0; j < energyHistory[id].length; j++) {
      let e = energyHistory[id][j].val;
      if (e < minE) minE = e;
      if (e > maxE) maxE = e;
    }
  }

  if (!isFinite(minE) || !isFinite(maxE)) {
    pop();
    return;
  }

  if (abs(maxE - minE) < 0.000001) {
    maxE = minE + 1;
  }

  drawGraphAxis(w, h, '', 'E, J', minE, maxE, simTime, 4);

  for (let i = 0; i < idsToDraw.length; i++) {
    let id = idsToDraw[i];
    let c = METHOD_INFO[id].color;
    stroke(c[0], c[1], c[2]);
    strokeWeight(2);
    noFill();
    beginShape();
    for (let j = 0; j < energyHistory[id].length; j++) {
      let x = map(j, 0, max(1, energyHistory[id].length - 1), 0, w);
      let y = map(energyHistory[id][j].val, minE, maxE, h, 0);
      vertex(x, y);
    }
    endShape();
  }

  for (let i = 0; i < idsToDraw.length; i++) {
    let id = idsToDraw[i];
    let c = METHOD_INFO[id].color;
    fill(c[0], c[1], c[2]);
    noStroke();
    rect(w - 170, 8 + i * 16, 10, 10);
    fill(0);
    noStroke();
    text(METHOD_INFO[id].label, w - 154, 17 + i * 16);
  }

  pop();
}

function drawGraphAxis(w, h, xLabel, yLabel, yMin, yMax, tMax, ticks) {
  stroke(150);
  strokeWeight(1);
  noFill();
  rect(0, 0, w, h);

  let yTicks = max(2, ticks);
  for (let i = 0; i <= yTicks; i++) {
    let py = map(i, 0, yTicks, h, 0);
    stroke(210);
    line(0, py, w, py);
    stroke(120);
    line(-5, py, 0, py);
    noStroke();
    fill(60);
    let val = map(i, 0, yTicks, yMin, yMax);
    text(val.toFixed(2), -42, py + 4);
  }

  fill(0);
  noStroke();
  text(yLabel, 4, 12);
  if (xLabel) {
    text(xLabel, w - 36, h + 18);
  }
}

// -----------------------------------------------------------------------------
// BLOCK: INTERACTION AND RESET
// -----------------------------------------------------------------------------

function mousePressed() {
  if (mouseX >= UI_X && mouseX <= UI_X + UI_BLOCK_MAX_X_OFFSET && mouseY <= UI_BLOCK_MAX_Y) {
    return;
  }

  let active = getActiveState();
  let d = dist(mouseX, mouseY, active.xM * PX_PER_M, active.yM * PX_PER_M);
  if (d < active.radiusPx + 5) {
    dragging = true;
    for (let i = 0; i < METHOD_IDS.length; i++) {
      methodStates[METHOD_IDS[i]].vx = 0;
      methodStates[METHOD_IDS[i]].vy = 0;
      trajectoryHistory[METHOD_IDS[i]] = [];
    }
  }
}

function mouseDragged() {
  if (dragging) {
    for (let i = 0; i < METHOD_IDS.length; i++) {
      let s = methodStates[METHOD_IDS[i]];
      s.xM = mouseX / PX_PER_M;
      s.yM = mouseY / PX_PER_M;
      s.vx = 0;
      s.vy = 0;
      s.trail.push({ xM: s.xM, yM: s.yM });
      while (s.trail.length > 220) {
        s.trail.shift();
      }
    }
  }
}

function mouseReleased() {
  dragging = false;
}

function resetToEquilibrium() {
  updateGeometry();
  let eq = findEquilibriumPoint();
  eqXM = eq.xM;
  eqYM = eq.yM;
  for (let i = 0; i < METHOD_IDS.length; i++) {
    let s = methodStates[METHOD_IDS[i]];
    s.xM = eq.xM;
    s.yM = eq.yM;
    s.vx = 0;
    s.vy = 0;
    s.trail = [{ xM: eq.xM, yM: eq.yM }];
  }

  energyHistory = {};
  trajectoryHistory = {};
  for (let i = 0; i < METHOD_IDS.length; i++) {
    energyHistory[METHOD_IDS[i]] = [];
    trajectoryHistory[METHOD_IDS[i]] = [];
  }
  simTime = 0;
}

// -----------------------------------------------------------------------------
// BLOCK: BROWSER BOOTSTRAP
// -----------------------------------------------------------------------------

(function bootstrapForBrowser() {
  if (typeof window === 'undefined' || typeof document === 'undefined') {
    return;
  }

  function startIfNeeded() {
    setTimeout(function () {
      if (!__task12SetupRan && typeof window.p5 === 'function') {
        window.__task12Instance = new window.p5();
      }
    }, 60);
  }

  if (typeof window.p5 === 'function') {
    startIfNeeded();
    return;
  }

  const existing = document.querySelector('script[data-task12-p5="1"]');
  if (existing) {
    existing.addEventListener('load', startIfNeeded, { once: true });
    return;
  }

  const script = document.createElement('script');
  script.src = 'https://cdn.jsdelivr.net/npm/p5@1.9.4/lib/p5.min.js';
  script.async = true;
  script.dataset.task12P5 = '1';
  script.onload = startIfNeeded;
  script.onerror = function () {
    console.error('Не удалось загрузить p5.js. Проверьте подключение к сети.');
  };
  document.head.appendChild(script);
})();
