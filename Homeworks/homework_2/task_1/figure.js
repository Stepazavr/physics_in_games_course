// =================================
// Figure Creation
// =================================
// Функция для создания фигуры (например, круга с его точками и ограничениями)

function createCircleFigure(name, center, radius, numPoints) {
  // Создаёт объект фигуры с:
  // - name: имя/идентификатор фигуры
  // - points: массив точек
  // - constraints: массив ограничений внутри этой фигуры
  
  const figure = {
    name: name,
    points: [],
    constraints: [],
  };

  // Равномерное распределение точек по кругу
  for (let i = 0; i < numPoints; i++) {
    const angle = TWO_PI * (i / numPoints);
    
    const x = center.x + radius * cos(angle);
    const y = center.y + radius * sin(angle);
    
    const pos = createVector(x, y);
    
    const point = {
      position: pos.copy(),
      velocity: createVector(0, 0),
      acceleration: createVector(settings.gravity.x, settings.gravity.y),
      positionPredicted: pos.copy(),
      mass: 1,
      isFixed: false,
      color: { r: 255, g: 255, b: 255 },  // Белый цвет по умолчанию
    };
    
    figure.points.push(point);
  }

  // Создаём ограничения между всеми парами точек в этой фигуре
  for (let i = 0; i < figure.points.length; i++) {
    for (let j = i + 1; j < figure.points.length; j++) {
      const p1 = figure.points[i];
      const p2 = figure.points[j];
      figure.constraints.push({
        type: 'distance',
        p1: p1,
        p2: p2,
        distance: p5.Vector.dist(p1.position, p2.position),
        lambda: 0,  // Множитель Лагранжа для XPBD
      });
    }
  }

  return figure;
}

// =================================
// Fabric Grid Creation
// =================================
// Функция для создания ткани в виде сетки из точек
// Точки связаны только с соседними (не по диагонали)

function createGridFigure(name, topLeftPos, width, height, numRows, numCols, numFixedPoints = 0) {
  // Создаёт объект ткани-сетки с:
  // - name: имя/идентификатор фигуры
  // - points: массив точек в сетке (2D)
  // - constraints: массив ограничений (связи между соседними точками)
  // - numFixedPoints: количество закрепленных верхних точек (по умолчанию 0 - не закреплены)
  
  const figure = {
    name: name,
    points: [],
    constraints: [],
    grid: [],  // 2D массив для удобного доступа к точкам
  };

  // Шаг между точками в сетке
  const dx = width / (numCols - 1);
  const dy = height / (numRows - 1);

  // Создаём точки сетки
  for (let row = 0; row < numRows; row++) {
    figure.grid[row] = [];
    for (let col = 0; col < numCols; col++) {
      const x = topLeftPos.x + col * dx;
      const y = topLeftPos.y + row * dy;
      
      const pos = createVector(x, y);
      
      const point = {
        position: pos.copy(),
        velocity: createVector(0, 0),
        acceleration: createVector(settings.gravity.x, settings.gravity.y),
        positionPredicted: pos.copy(),
        mass: 1,
        isFixed: false,
        color: { r: 255, g: 255, b: 255 },  // Белый цвет по умолчанию
      };
      
      figure.points.push(point);
      figure.grid[row][col] = point;
    }
  }

  // Закрепляем верхние точки в зависимости от numFixedPoints
  for (let col = 0; col < numFixedPoints; col++) {
    figure.grid[0][col].isFixed = true;
    figure.grid[0][col].color = { r: 255, g: 0, b: 0 };  // Красный цвет для закрепленных
  }

  // Создаём ограничения между соседними точками
  // (только горизонтальные и вертикальные связи, НЕ диагональные)
  
  for (let row = 0; row < numRows; row++) {
    for (let col = 0; col < numCols; col++) {
      const p1 = figure.grid[row][col];
      
      // Связь с правым соседом (col + 1)
      if (col + 1 < numCols) {
        const p2 = figure.grid[row][col + 1];
        figure.constraints.push({
          type: 'distance',
          p1: p1,
          p2: p2,
          distance: p5.Vector.dist(p1.position, p2.position),
          lambda: 0,
        });
      }
      
      // Связь с нижним соседом (row + 1)
      if (row + 1 < numRows) {
        const p2 = figure.grid[row + 1][col];
        figure.constraints.push({
          type: 'distance',
          p1: p1,
          p2: p2,
          distance: p5.Vector.dist(p1.position, p2.position),
          lambda: 0,
        });
      }
    }
  }

  return figure;
}
