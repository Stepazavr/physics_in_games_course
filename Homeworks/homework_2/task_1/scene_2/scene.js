// =================================
// Scene 2 - Hanging Fabric Grid
// =================================

function initScene() {
  figures = [];

  const center = createVector(settings.sceneWidth / 2, settings.sceneHeight / 2 + 200);
  const radius = 80;
  const numPoints = 15;

  const circle = createCircleFigure('circle_1', center, radius, numPoints);

  const gridTopLeft = createVector(150, 0);
  const gridWidth = 500;
  const gridHeight = 500;
  const gridRows = 30;
  const gridCols = 30;

  const grid = createGridFigure('hanging_grid_1', gridTopLeft, gridWidth, gridHeight, gridRows, gridCols, gridCols);
  figures.push(grid);
}
