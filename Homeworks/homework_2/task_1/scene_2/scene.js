// =================================
// Scene 2 - Hanging Fabric Grid
// =================================
// Инициализирует сцену: ткань висит на нескольких закрепленных верхних точках

function initScene() {
  figures = [];

  // Создаём один круг в центре
  const center = createVector(settings.sceneWidth / 2, settings.sceneHeight / 2 - 100);
  const radius = 80;
  const numPoints = 15;

  const circle = createCircleFigure('circle_1', center, radius, numPoints);
  figures.push(circle);

  // Создаём сетку-ткань, которая висит на закрепленных верхних точках
  const gridTopLeft = createVector(150, 0);  // y = 0, прикреплена в верхней части сцены
  const gridWidth = 500;
  const gridHeight = 350;
  const gridRows = 20;
  const gridCols = 20;

  // numFixedPoints = gridCols - закрепляем ВСЕ верхние точки (сцена-специфичный параметр)
  const grid = createGridFigure('hanging_grid_1', gridTopLeft, gridWidth, gridHeight, gridRows, gridCols, gridCols);
  figures.push(grid);
}
