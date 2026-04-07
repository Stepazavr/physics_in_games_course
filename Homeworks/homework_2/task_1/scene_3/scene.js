// =================================
// Scene 3 - Two Circles
// =================================
// Инициализирует сцену: два круга в разных частях сцены

function initScene() {
  figures = [];

  // Создаём первый круг
  const center1 = createVector(settings.sceneWidth / 4, settings.sceneHeight / 2 - 100);
  const radius1 = 60;
  const numPoints1 = 15;
  const circle1 = createCircleFigure('circle_1', center1, radius1, numPoints1);
  figures.push(circle1);

  // Создаём второй круг
  const center2 = createVector(settings.sceneWidth * 3 / 4, settings.sceneHeight / 2 - 100);
  const radius2 = 60;
  const numPoints2 = 15;
  const circle2 = createCircleFigure('circle_2', center2, radius2, numPoints2);
  figures.push(circle2);
}
