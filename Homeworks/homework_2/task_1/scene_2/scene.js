// =================================
// Scene 2 - Single Circle
// =================================
// Инициализирует сцену: один круг в центре сцены

function initScene() {
  figures = [];

  // Создаём один круг в центре
  const center = createVector(settings.sceneWidth / 2, settings.sceneHeight / 2 - 100);
  const radius = 80;
  const numPoints = 15;

  const circle = createCircleFigure('circle_1', center, radius, numPoints);
  figures.push(circle);
}
