function setup() {
  createCanvas(800, 600, WEBGL);
  angleMode(DEGREES);
}

function draw() {
  background(200);

  // Camera control
  orbitControl();

  // Lighting
  ambientLight(100);
  pointLight(255, 255, 255, 0, -200, 200);

  // Foundation
  push();
  translate(0, 100, 0);
  rotateX(90);
  noStroke();
  fill(150); // Gray color
  plane(400, 400);
  pop();

  // Red Cube
  push();
  translate(0, -50, 0);
  noStroke();
  fill(255, 0, 0); // Red color
  box(50);
  pop();
}
