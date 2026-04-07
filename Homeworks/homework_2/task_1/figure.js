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
