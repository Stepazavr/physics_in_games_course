// =================================
// All Scenes Definitions
// =================================

let currentSceneNumber = 1;

// =================================
// Scene 1 - Ткань в плоскости XZ, свободно падает
// =================================
function initScene_1() {
  figures = [];

  // Сетка в плоскости XZ с наклоном по Y
  const centerPos = createVector(400, 200, 300);
  const gridWidth = 400;   // размер по X
  const gridDepth = 400;   // размер по Z
  const gridRows = 30;     // количество точек по Z
  const gridCols = 30;     // количество точек по X
  const y1 = 100;          // Y верхней строки
  const y2 = 400;          // Y нижней строки (как гипотенуза)
  
  const grid = createGridFigure_XZ('grid_1', centerPos, gridWidth, gridDepth, gridRows, gridCols, y1, y2);
  
  // Подвесить ткань за одну центральную точку
  const midRow = Math.floor(gridRows / 2);
  const midCol = Math.floor(gridCols / 2);
  grid.grid[midRow][midCol].isFixed = true;
  grid.grid[midRow][midCol].color = { r: 255, g: 0, b: 0 };
  
  figures.push(grid);
}

// =================================
// Scene 2 - Ткань в плоскости XZ, натянута сверху
// =================================
function initScene_2() {
  figures = [];

  // Сетка в плоскости XZ, вытянутая по Y от y1 до y2
  const centerPos = createVector(400, 200, 300);
  const gridWidth = 400;   // размер по X
  const gridDepth = 400;   // размер по Z
  const gridRows = 30;     // количество точек по Z
  const gridCols = 30;     // количество точек по X
  const y1 = 0;            // Y верхней строки (максимально поднята)
  const y2 = 400;          // Y нижней строки
  
  const grid = createGridFigure_XZ('grid_2', centerPos, gridWidth, gridDepth, gridRows, gridCols, y1, y2);
  
  // Зафиксировать только верхнюю строку (y1)
  for (let col = 0; col < gridCols; col++) {
    grid.grid[0][col].isFixed = true;
    grid.grid[0][col].color = { r: 255, g: 0, b: 0 };
  }
  
  figures.push(grid);
}

// =================================
// Scene 3 - Ткань в плоскости XZ с растянутой верхней строкой
// =================================
function initScene_3() {
  figures = [];

  // Сетка в плоскости XZ, вытянутая по Y от y1 до y2
  const centerPos = createVector(400, 200, 300);
  const gridWidth = 400;   // размер по X
  const gridDepth = 400;   // размер по Z
  const gridRows = 30;     // количество точек по Z
  const gridCols = 30;     // количество точек по X
  const y1 = 50;           // Y верхней строки
  const y2 = 400;          // Y нижней строки
  
  const grid = createGridFigure_XZ('grid_3', centerPos, gridWidth, gridDepth, gridRows, gridCols, y1, y2);
  
  // Раздвинуть и зафиксировать верхнюю строку
  if (grid.grid && grid.grid[0]) {
    const stretchedWidth = 800;  // растянутая ширина
    const startX = (800 - stretchedWidth) / 2;  // 800 - ширина canvas
    
    for (let col = 0; col < gridCols; col++) {
      const newX = startX + (col / (gridCols - 1)) * stretchedWidth;
      grid.grid[0][col].position.x = newX;
      grid.grid[0][col].positionPredicted.x = newX;
      grid.grid[0][col].isFixed = true;
      grid.grid[0][col].color = { r: 255, g: 0, b: 0 };
    }
  }
  
  figures.push(grid);
}

// =================================
// Scene 4 - два шара
// =================================
function initScene_4() {
  figures = [];

  const center1 = createVector(settings.sceneWidth / 4, settings.sceneHeight / 2 - 100, 0);
  const radius1 = 60;
  const numPoints1 = 15;
  const circle1 = createCircleFigure('circle_1', center1, radius1, numPoints1);
  figures.push(circle1);

  const center2 = createVector(settings.sceneWidth * 3 / 4, settings.sceneHeight / 2 - 100, 0);
  const radius2 = 60;
  const numPoints2 = 15;
  const circle2 = createCircleFigure('circle_2', center2, radius2, numPoints2);
  figures.push(circle2);
}

// =================================
// Main Scene Initializer
// =================================
function initScene() {
  switch (currentSceneNumber) {
    case 1:
      initScene_1();
      break;
    case 2:
      initScene_2();
      break;
    case 3:
      initScene_3();
      break;
    case 4:
      initScene_4();
      break;
    default:
      initScene_1();
  }
  
  // Reset simulation state
  points = [];
  constraints = [];
  frameCollisionConstraints = [];
  draggedPoint = null;
  draggedFigure = null;
}

// =================================
// Scene Switching Handler
// =================================
function switchScene(sceneNumber) {
  currentSceneNumber = sceneNumber;
  initScene();
}
