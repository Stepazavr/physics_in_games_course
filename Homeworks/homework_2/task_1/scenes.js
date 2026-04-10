// =================================
// All Scenes Definitions
// =================================

let currentSceneNumber = 1;

// =================================
// Scene 1 - Система висит на нескольких точках
// =================================
function initScene_1() {
  figures = [];

  const gridTopLeft = createVector(250, 0);
  const gridWidth = 300;
  const gridHeight = 300;
  const gridRows = 30;
  const gridCols = 30;
  
  const grid = createGridFigure('grid_1', gridTopLeft, gridWidth, gridHeight, gridRows, gridCols);
  
  // Прикрепить ткань в центре верхней строки
  const midCol = Math.floor(gridCols / 2);
  grid.grid[0][midCol].isFixed = true;
  grid.grid[0][midCol].color = { r: 255, g: 0, b: 0 };
  
  figures.push(grid);
}

// =================================
// Scene 2 - Система падает на пол
// =================================
function initScene_2() {
  figures = [];

  const center = createVector(settings.sceneWidth / 2, settings.sceneHeight / 2 + 200);
  const radius = 80;
  const numPoints = 15;

  const circle = createCircleFigure('circle_1', center, radius, numPoints);

  const gridTopLeft = createVector(250, 0);
  const gridWidth = 300;
  const gridHeight = 300;
  const gridRows = 30;
  const gridCols = 30;

  const grid = createGridFigure('hanging_grid_1', gridTopLeft, gridWidth, gridHeight, gridRows, gridCols, gridCols);
  figures.push(grid);
}

// =================================
// Scene 3 - Система с несовместными ограничениями
// =================================
function initScene_3() {
  figures = [];

  const gridTopLeft = createVector(150, 50);
  const gridWidth = 300;
  const gridHeight = 350;
  const gridRows = 30;
  const gridCols = 30;

  const grid = createGridFigure('stretched_grid_1', gridTopLeft, gridWidth, gridHeight, gridRows, gridCols, 0);
  
  if (grid.grid && grid.grid[0]) {
    const stretchedWidth = 700;
    const startX = (settings.sceneWidth - stretchedWidth) / 2;
    const topY = grid.grid[0][0].position.y;
    
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

  const center1 = createVector(settings.sceneWidth / 4, settings.sceneHeight / 2 - 100);
  const radius1 = 60;
  const numPoints1 = 15;
  const circle1 = createCircleFigure('circle_1', center1, radius1, numPoints1);
  figures.push(circle1);

  const center2 = createVector(settings.sceneWidth * 3 / 4, settings.sceneHeight / 2 - 100);
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
