// =================================
// Scene 1 - Single Circle and Grid Fabric
// =================================

function initScene() {
  figures = [];

  const center = createVector(settings.sceneWidth / 2 + 200, settings.sceneHeight / 2 - 100);
  const radius = 80;
  const numPoints = 15;

  const circle = createCircleFigure('circle_1', center, radius, numPoints);
  figures.push(circle);

  const gridTopLeft = createVector(100, 0);
  const gridWidth = 300;
  const gridHeight = 300;
  const gridRows = 20;
  const gridCols = 20;
  
  const grid = createGridFigure('grid_1', gridTopLeft, gridWidth, gridHeight, gridRows, gridCols);
  figures.push(grid);
}
