// =================================
// Scene 1 - Single Circle and Grid Fabric
// =================================
// Инициализирует сцену: один круг в центре сцены и сетка-ткань

function initScene() {
  figures = [];

  // Создаём один круг в центре
  const center = createVector(settings.sceneWidth / 2, settings.sceneHeight / 2 - 100);
  const radius = 80;
  const numPoints = 15;

  const circle = createCircleFigure('circle_1', center, radius, numPoints);
  figures.push(circle);

  // Создаём сетку-ткань
  const gridTopLeft = createVector(100, 0);  // y = 0, прикреплена в верхней части сцены
  const gridWidth = 300;
  const gridHeight = 200;
  const gridRows = 5;
  const gridCols = 8;
  
  const grid = createGridFigure('grid_1', gridTopLeft, gridWidth, gridHeight, gridRows, gridCols);
  figures.push(grid);
}
