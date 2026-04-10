// =================================
// Figure Creation
// =================================

function createCircleFigure(name, center, radius, numPoints) {
  const figure = {
    name: name,
    points: [],
    constraints: [],
    center: center.copy(),
    convex_points: [],
    pointRadius: 8,
  };

  for (let i = 0; i < numPoints; i++) {
    const angle = TWO_PI * (i / numPoints);
    const x = center.x + radius * cos(angle);
    const y = center.y + radius * sin(angle);
    const pos = createVector(x, y, 0);
    const normal = p5.Vector.sub(pos, center).normalize();
    
    const point = {
      position: pos.copy(),
      velocity: createVector(0, 0, 0),
      acceleration: createVector(settings.gravity.x, settings.gravity.y, 0),
      positionPredicted: pos.copy(),
      mass: 1,
      isFixed: false,
      isConvex: true,
      normal: normal,
      color: { r: 255, g: 255, b: 255 },
      figure: figure,
    };
    
    figure.points.push(point);
    figure.convex_points.push(point);
  }

  for (let i = 0; i < figure.points.length; i++) {
    for (let j = i + 1; j < figure.points.length; j++) {
      const p1 = figure.points[i];
      const p2 = figure.points[j];
      figure.constraints.push({
        type: 'distance',
        p1: p1,
        p2: p2,
        distance: p5.Vector.dist(p1.position, p2.position),
        lambda: 0,
      });
    }
  }

  return figure;
}

// =================================
// Fabric Grid Creation
// =================================

function createGridFigure(name, topLeftPos, width, height, numRows, numCols, numFixedPoints = 0) {
  const figure = {
    name: name,
    points: [],
    constraints: [],
    grid: [],
    center: createVector(topLeftPos.x + width / 2, topLeftPos.y + height / 2, 0),
    convex_points: [],
    pointRadius: 4,
  };

  const dx = width / (numCols - 1);
  const dy = height / (numRows - 1);

  for (let row = 0; row < numRows; row++) {
    figure.grid[row] = [];
    for (let col = 0; col < numCols; col++) {
      const x = topLeftPos.x + col * dx;
      const y = topLeftPos.y + row * dy;
      const pos = createVector(x, y, 0);
      const isConvex = row === 0 || row === numRows - 1 || col === 0 || col === numCols - 1;

      let normal = createVector(0, 0, 0);
      if (isConvex) {
        normal = p5.Vector.sub(pos, figure.center).normalize();
      }
      
      const point = {
        position: pos.copy(),
        velocity: createVector(0, 0, 0),
        acceleration: createVector(settings.gravity.x, settings.gravity.y, 0),
        positionPredicted: pos.copy(),
        mass: 1,
        isFixed: false,
        isConvex: isConvex,
        normal: normal,
        color: { r: 255, g: 255, b: 255 },
        figure: figure,
      };
      
      figure.points.push(point);
      figure.grid[row][col] = point;
      
      if (isConvex) {
        figure.convex_points.push(point);
      }
    }
  }

  for (let col = 0; col < numFixedPoints; col++) {
    figure.grid[0][col].isFixed = true;
    figure.grid[0][col].color = { r: 255, g: 0, b: 0 };
  }

  for (let row = 0; row < numRows; row++) {
    for (let col = 0; col < numCols; col++) {
      const p1 = figure.grid[row][col];
      
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
