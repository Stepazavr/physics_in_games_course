// =================================
// Scene 3 - Grid with Stretched Fixed Points
// =================================
// Сетка-ткань с растянутыми закрепленными точками по оси X
// Демонстрирует несовместные ограничения

function initScene() {
  figures = [];

  // Создаём сетку-ткань
  const gridTopLeft = createVector(150, 50);
  const gridWidth = 500;
  const gridHeight = 350;
  const gridRows = 30;
  const gridCols = 30;

  // Создаём сетку БЕЗ автоматического закрепления верхних точек
  const grid = createGridFigure('stretched_grid_1', gridTopLeft, gridWidth, gridHeight, gridRows, gridCols, 0);
  
  // Вручную закрепляем и растягиваем все верхние точки по оси X
  // Оставляем ограничения без изменений - это создаст несовместные ограничения
  if (grid.grid && grid.grid[0]) {
    const stretchedWidth = 700;
    const startX = (settings.sceneWidth - stretchedWidth) / 2;  // Центрируем растянутую сетку
    const topY = grid.grid[0][0].position.y;  // Y координата остаётся той же
    
    // Для каждой точки верхнего ряда устанавливаем новую позицию X
    for (let col = 0; col < gridCols; col++) {
      const newX = startX + (col / (gridCols - 1)) * stretchedWidth;
      grid.grid[0][col].position.x = newX;
      grid.grid[0][col].positionPredicted.x = newX;
      grid.grid[0][col].isFixed = true;
      grid.grid[0][col].color = { r: 255, g: 0, b: 0 };  // Красный для закреплённой
    }
  }

  figures.push(grid);
}
