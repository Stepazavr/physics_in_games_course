// -----------------------------------------------------------------------------
// BLOCK: STARTUP AND SIMULATION STATE
// -----------------------------------------------------------------------------
// Параметры системы
let __task11SetupRan = false;

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

// Параметры пружин
let springH = {  // пружина на горизонтальной стенке
  wallType: 'horizontal',
  lengthM: 0.8,
  k: 20,
  color: [255, 100, 100],    // красная
  attachX: 0, attachY: 0,
  attachXM: 0, attachYM: 0,
};

let springA = {  // пружина на наклонной стенке
  wallType: 'angled',
  lengthM: 0.8,
  k: 20,
  color: [100, 100, 255],    // синяя
  attachX: 0, attachY: 0,
  attachXM: 0, attachYM: 0,
};

// Угол наклонной стенки
let alfa;
let dragging = false;

// Ползунки
let sliderK1, sliderK2, sliderLen1, sliderLen2, sliderMass;
let resetButton;
let methodSelect;
let energyModeSelect;

// Для графиков
let energyHistory = {};
let trajectoryHistory = {};
let simTime = 0;

// -----------------------------------------------------------------------------
// BLOCK: LAYOUT AND UI CONSTANTS
// -----------------------------------------------------------------------------

// Компоновка интерфейса
const UI_X = 180;
const MODEL_CORNER_X = 430 - 0.6 * PX_PER_M;
const MODEL_CORNER_Y = 300;
const ENERGY_GRAPH_X = 520;
const ENERGY_GRAPH_Y = 360;
const ENERGY_GRAPH_W = 340;
const ENERGY_GRAPH_H = 220;
const CANVAS_W = 900;
const CANVAS_H = 650;

// Раскладка UI (чтобы удобно править числа в одном месте)
const TOP_PANEL_START_Y = 20;
const TOP_PANEL_STEP_Y = 65;
const METHOD_LABEL_Y = TOP_PANEL_START_Y + TOP_PANEL_STEP_Y * 0;
const METHOD_SELECT_Y = METHOD_LABEL_Y + 25;
const ENERGY_LABEL_Y = TOP_PANEL_START_Y + TOP_PANEL_STEP_Y * 1;
const ENERGY_SELECT_Y = ENERGY_LABEL_Y + 25;
const METHOD_SELECT_W = 220;

const CONTROLS_START_Y = 150;
const LABEL_TO_SLIDER_DY = 25;
const CONTROL_GROUP_STEP_Y = 65;
const MASS_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 0;
const MASS_SLIDER_Y = MASS_LABEL_Y + LABEL_TO_SLIDER_DY;
const K1_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 1;
const K1_SLIDER_Y = K1_LABEL_Y + LABEL_TO_SLIDER_DY;
const K2_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 2;
const K2_SLIDER_Y = K2_LABEL_Y + LABEL_TO_SLIDER_DY;
const A_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 3;
const A_SLIDER_Y = A_LABEL_Y + LABEL_TO_SLIDER_DY;
const B_LABEL_Y = CONTROLS_START_Y + CONTROL_GROUP_STEP_Y * 4;
const B_SLIDER_Y = B_LABEL_Y + LABEL_TO_SLIDER_DY;

const VAR_SLIDER_W = 200;
const RESET_BUTTON_Y = B_SLIDER_Y + 35;

// Mouse interaction bounds for UI panel
const UI_BLOCK_MAX_X_OFFSET = 240;
const UI_BLOCK_MAX_Y = RESET_BUTTON_Y + 50;

// Bottom info text layout
const INFO_TEXT_X = UI_X - 1.1 * PX_PER_M;
const INFO_TEXT_Y_START = RESET_BUTTON_Y + 28;
const INFO_TEXT_LINE_STEP = 18;

// -----------------------------------------------------------------------------
// BLOCK: GEOMETRY STATE
// -----------------------------------------------------------------------------

// Константы для геометрии
let cornerX = MODEL_CORNER_X;
let cornerY = MODEL_CORNER_Y;
let cornerXM = 0;
let cornerYM = 0;
let wallLengthH = 0;
let wallLengthA = 0;
let wallLengthHM = 0;
let wallLengthAM = 0;
let eqXM = 0;
let eqYM = 0;

// -----------------------------------------------------------------------------
// BLOCK: UI INITIALIZATION
// -----------------------------------------------------------------------------

function setup() {
  __task11SetupRan = true;
  createCanvas(CANVAS_W, CANVAS_H);
  cornerXM = cornerX / PX_PER_M;
  cornerYM = cornerY / PX_PER_M;
  
  // Фиксированный угол 30 градусов
  alfa = 30 * PI / 180;
  
  // Выбор численного метода
  let labelMethod = createDiv('Численный метод');
  labelMethod.position(UI_X, METHOD_LABEL_Y);
  methodSelect = createSelect();
  methodSelect.position(UI_X, METHOD_SELECT_Y);
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
  labelEnergyMode.position(UI_X, ENERGY_LABEL_Y);
  energyModeSelect = createSelect();
  energyModeSelect.position(UI_X, ENERGY_SELECT_Y);
  energyModeSelect.style('width', METHOD_SELECT_W + 'px');
  energyModeSelect.option('Сравнение', 'compare');
  energyModeSelect.option('Одиночный метод', 'single');
  energyModeSelect.selected(energyMode);
  energyModeSelect.changed(function () {
    energyMode = energyModeSelect.value();
  });

  // Создаем подписи и ползунки рядом в верхней части
  let labelMass = createDiv('m (kg)');
  labelMass.position(UI_X, MASS_LABEL_Y);
  sliderMass = createSlider(0.1, 10, 1, 0.1);
  sliderMass.position(UI_X, MASS_SLIDER_Y);
  sliderMass.style('width', VAR_SLIDER_W + 'px');

  let labelK1 = createDiv('k1 (N/m)');
  labelK1.position(UI_X, K1_LABEL_Y);
  sliderK1 = createSlider(5, 80, 20, 1);
  sliderK1.position(UI_X, K1_SLIDER_Y);
  sliderK1.style('width', VAR_SLIDER_W + 'px');
  
  let labelK2 = createDiv('k2 (N/m)');
  labelK2.position(UI_X, K2_LABEL_Y);
  sliderK2 = createSlider(5, 80, 20, 1);
  sliderK2.position(UI_X, K2_SLIDER_Y);
  sliderK2.style('width', VAR_SLIDER_W + 'px');
  
  let labelLen1 = createDiv('a (m)');
  labelLen1.position(UI_X, A_LABEL_Y);
  sliderLen1 = createSlider(0.4, 1.5, 0.8, 0.01);
  sliderLen1.position(UI_X, A_SLIDER_Y);
  sliderLen1.style('width', VAR_SLIDER_W + 'px');
  sliderLen1.input(resetToEquilibrium);
  
  let labelLen2 = createDiv('b (m)');
  labelLen2.position(UI_X, B_LABEL_Y);
  sliderLen2 = createSlider(0.4, 1.5, 0.8, 0.01);
  sliderLen2.position(UI_X, B_SLIDER_Y);
  sliderLen2.style('width', VAR_SLIDER_W + 'px');
  sliderLen2.input(resetToEquilibrium);
  
  // Кнопка сброса
  resetButton = createButton('Сбросить в равновесие');
  resetButton.position(UI_X, RESET_BUTTON_Y);
  resetButton.mousePressed(resetToEquilibrium);
  
  // Вычисляем начальную геометрию и положение шарика
  updateGeometry();
  initializeMethodStates();
  
  // Вызываем сброс в равновесие перед симуляцией
  resetToEquilibrium();
}

function initializeMethodStates() {
  methodStates = {};
  energyHistory = {};
  trajectoryHistory = {};
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
    trajectoryHistory[id] = [];
  }
}

// -----------------------------------------------------------------------------
// BLOCK: PHYSICS AND NUMERICAL METHODS
// -----------------------------------------------------------------------------

function updateGeometry() {
  // Получаем текущие значения
  let a = sliderLen1.value();
  let b = sliderLen2.value();
  let alpha = alfa;
  
  // Вычисляем длины стенок по формулам:
  // AB = (a + b * cos(alpha)) / sin(alpha)  - наклонная стенка
  // BC = (b + a * cos(alpha)) / sin(alpha)  - горизонтальная стенка
  let sinAlpha = sin(alpha);
  let cosAlpha = cos(alpha);
  
  wallLengthAM = (a + b * cosAlpha) / sinAlpha;
  wallLengthHM = (b + a * cosAlpha) / sinAlpha;
  wallLengthA = wallLengthAM * PX_PER_M;
  wallLengthH = wallLengthHM * PX_PER_M;
  
  // Точка крепления красной пружины (на горизонтальной стенке)
  springH.attachXM = cornerXM + wallLengthHM;
  springH.attachYM = cornerYM;
  springH.attachX = springH.attachXM * PX_PER_M;
  springH.attachY = springH.attachYM * PX_PER_M;
  springH.lengthM = a;
  springH.k = sliderK1.value();
  
  // Точка крепления синей пружины (на наклонной стенке)
  springA.attachXM = cornerXM + wallLengthAM * cosAlpha;
  springA.attachYM = cornerYM - wallLengthAM * sinAlpha;
  springA.attachX = springA.attachXM * PX_PER_M;
  springA.attachY = springA.attachYM * PX_PER_M;
  springA.lengthM = b;
  springA.k = sliderK2.value();
}





function findEquilibriumPoint() {
  let alpha = alfa;
  let sinAlpha = sin(alpha);
  let cosAlpha = cos(alpha);
  
  // Точка крепления красной пружины (на горизонтальной стенке)
  let x1 = springH.attachX;
  let y1 = springH.attachY;
  
  // Точка крепления синей пружины (на наклонной стенке)
  let x2 = springA.attachX;
  let y2 = springA.attachY;
  
  // Направление красной пружины (перпендикуляр к горизонтальной стенке - вверх)
  let dx1 = 0;
  let dy1 = -1;  // вверх (против направления Y)
  
  // Направление синей пружины (перпендикуляр к наклонной стенке)
  // Вектор стенки: (cosAlpha, -sinAlpha)
  // Перпендикуляр: (-sinAlpha, -cosAlpha) - направлен внутрь угла
  let dx2 = -sinAlpha;
  let dy2 = -cosAlpha;
  
  // Решаем систему параметрических уравнений:
  // x1 + t1*dx1 = x2 + t2*dx2
  // y1 + t1*dy1 = y2 + t2*dy2
  
  // Определитель системы
  let det = dx1 * (-dy2) - (-dx2) * dy1;
  
  if (Math.abs(det) > 0.0001) {
    // Находим параметры
    let t1 = ((x2 - x1) * (-dy2) - (y2 - y1) * (-dx2)) / det;
    
    // Вычисляем точку пересечения
    let eqX = x1 + t1 * dx1;
    let eqY = y1 + t1 * dy1;
    
    // Проверяем, что точка находится в разумных пределах
    eqX = constrain(eqX, cornerX - 50, cornerX + wallLengthH + 150);
    eqY = constrain(eqY, cornerY - wallLengthA - 150, cornerY + 50);
    return { xM: eqX / PX_PER_M, yM: eqY / PX_PER_M };
  } else {
    // Если прямые параллельны, используем запасной вариант
    return {
      xM: springH.attachXM,
      yM: springH.attachYM - springH.lengthM
    };
  }
}

function getAccelerationAt(xM, yM) {
  let sinAlpha = sin(alfa);
  let cosAlpha = cos(alfa);

  // Используем экранную ось Y напрямую для модели по согласованию с пользователем.
  let x = xM - eqXM;
  let y = yM - eqYM;

  let k1 = springH.k;
  let k2 = springA.k;

  let axMath = (-sinAlpha * sinAlpha * k2 * x - cosAlpha * sinAlpha * k2 * y) / ballMass;
  let ayMath = (-cosAlpha * sinAlpha * k2 * x - (k1 + cosAlpha * cosAlpha * k2) * y) / ballMass;

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
  let sinAlpha = sin(alfa);
  let cosAlpha = cos(alfa);

  // Используем ту же ориентацию Y, что и в интеграторе.
  let x = state.xM - eqXM;
  let y = state.yM - eqYM;
  let vx = state.vx;
  let vy = state.vy;

  let k1 = springH.k;
  let k2 = springA.k;

  let kinetic = 0.5 * ballMass * (vx * vx + vy * vy);
  let potential =
    0.5 * sinAlpha * sinAlpha * k2 * x * x
    + cosAlpha * sinAlpha * k2 * x * y
    + 0.5 * (k1 + cosAlpha * cosAlpha * k2) * y * y;

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
    if (hist.length < 2) continue;
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
  drawCorner();
  drawTrajectory();

  drawSpring(springH.attachX, springH.attachY, renderState.xPx, renderState.yPx, springH.color, springH.lengthM * PX_PER_M);
  drawSpring(springA.attachX, springA.attachY, renderState.xPx, renderState.yPx, springA.color, springA.lengthM * PX_PER_M);

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
  text("method = " + METHOD_INFO[selectedMethodId].label, INFO_TEXT_X, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 0);
  text("μ = " + nf(alfa * 180 / PI, 0, 1) + "\u00B0, dt = " + SIM_DT.toFixed(3) + " s", INFO_TEXT_X, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 1);
  text("m = " + ballMass.toFixed(2) + " kg", INFO_TEXT_X, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 2);
  text("k1 = " + springH.k.toFixed(1) + " N/m, a = " + springH.lengthM.toFixed(2) + " m", INFO_TEXT_X, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 3);
  text("k2 = " + springA.k.toFixed(1) + " N/m, b = " + springA.lengthM.toFixed(2) + " m", INFO_TEXT_X, INFO_TEXT_Y_START + INFO_TEXT_LINE_STEP * 4);
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




// Рисуем уголок
function drawCorner() {
  push();
  stroke(139, 69, 19); // коричневый
  strokeWeight(4);
  
  // Горизонтальная стенка
  line(cornerX, cornerY, cornerX + wallLengthH, cornerY);
  
  // Наклонная стенка
  line(cornerX, cornerY, 
       cornerX + wallLengthA * cos(alfa), 
       cornerY - wallLengthA * sin(alfa));
  
  // Вершина уголка
  fill(139, 69, 19);
  noStroke();
  ellipse(cornerX, cornerY, 10, 10);
  
  // Отметка угла (только символ μ)
  noFill();
  stroke(0, 100, 0);
  strokeWeight(1);
  arc(cornerX, cornerY, 60, 60, -alfa, 0);
  text("μ", cornerX + 40, cornerY - 12);
  
  pop();
}

// Рисуем пружину
function drawSpring(x1, y1, x2, y2, col, restLenPx) {
  push();
  stroke(col);
  strokeWeight(2);
  noFill();
  
  // Точка крепления на стенке
  fill(col);
  ellipse(x1, y1, 8, 8);
  
  // Число витков зависит от собственной длины пружины,
  // а расстояние до шарика даёт её реальное растяжение/сжатие.
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
    
    // Добавляем зигзаг
    if (i > 0 && i < steps) {
      let angle = atan2(y2 - y1, x2 - x1);
      let perp = (i % 2 === 0) ? coilAmplitude : -coilAmplitude;
      x += perp * cos(angle + HALF_PI);
      y += perp * sin(angle + HALF_PI);
    }
    vertex(x, y);
  }
  endShape();
  
  pop();
}

// Графики движения и энергии
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

// Отрисовка оси графика
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

// Интерактивность - перетаскивание
function mousePressed() {
  // Проверяем, не нажали ли на ползунок
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

// Сброс шарика в положение равновесия
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

  // Очищаем историю графиков
  energyHistory = {};
  trajectoryHistory = {};
  for (let i = 0; i < METHOD_IDS.length; i++) {
    energyHistory[METHOD_IDS[i]] = [];
    trajectoryHistory[METHOD_IDS[i]] = [{ xM: eq.xM, yM: eq.yM }];
  }
  simTime = 0;
}

// -----------------------------------------------------------------------------
// BLOCK: BROWSER BOOTSTRAP
// -----------------------------------------------------------------------------

// Автозагрузка p5.js и запуск скетча для обычного браузера
(function bootstrapForBrowser() {
  if (typeof window === 'undefined' || typeof document === 'undefined') {
    return;
  }

  function startIfNeeded() {
    // Даем шанс штатному автозапуску p5, и только потом стартуем вручную.
    setTimeout(function () {
      if (!__task11SetupRan && typeof window.p5 === 'function') {
        window.__task11Instance = new window.p5();
      }
    }, 60);
  }

  if (typeof window.p5 === 'function') {
    startIfNeeded();
    return;
  }

  const existing = document.querySelector('script[data-task11-p5="1"]');
  if (existing) {
    existing.addEventListener('load', startIfNeeded, { once: true });
    return;
  }

  const script = document.createElement('script');
  script.src = 'https://cdn.jsdelivr.net/npm/p5@1.9.4/lib/p5.min.js';
  script.async = true;
  script.dataset.task11P5 = '1';
  script.onload = startIfNeeded;
  script.onerror = function () {
    console.error('Не удалось загрузить p5.js. Проверьте подключение к сети.');
  };
  document.head.appendChild(script);
})();
