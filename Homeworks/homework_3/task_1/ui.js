// Task 1 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons
  pausedState = setupUIControls();
  
  // Initialize scene selection UI for task_1 (tree structure)
  const sceneButtonElements = document.querySelectorAll('.scene-btn');
  const localCoordinatesSection = document.getElementById('local-branch');
  const gyroscopicSection = document.getElementById('gyro-branch');
  
  sceneButtonElements.forEach(button => {
    button.addEventListener('click', function() {
      if (this.disabled) return;
      
      const sceneValue = this.dataset.val;
      
      // Remove active from all buttons
      sceneButtonElements.forEach(btn => btn.classList.remove('active'));
      this.classList.add('active');
      
      if (sceneValue === '1A') {
        localCoordinatesSection.style.display = 'none';
        gyroscopicSection.style.display = 'none';
        loadSceneWithDefaultSettings('1A');
      } else if (sceneValue === '1B_parent') {
        localCoordinatesSection.style.display = 'block';
        gyroscopicSection.style.display = 'none';
        document.querySelector('[data-val="1B"]').classList.add('active');
        loadSceneWithDefaultSettings('1B');
      } else if (sceneValue === '1B') {
        gyroscopicSection.style.display = 'none';
        document.querySelector('[data-val="1B_parent"]').classList.add('active');
        this.classList.add('active');
        loadSceneWithDefaultSettings('1B');
      } else if (sceneValue === '1C_parent') {
        gyroscopicSection.style.display = 'block';
        document.querySelector('[data-val="1B_parent"]').classList.add('active');
        this.classList.add('active');
        document.querySelector('[data-val="1C"]').classList.add('active');
        loadSceneWithDefaultSettings('1C');
      } else if (sceneValue === '1C' || sceneValue === '1D') {
        gyroscopicSection.style.display = 'block';
        document.querySelector('[data-val="1B_parent"]').classList.add('active');
        document.querySelector('[data-val="1C_parent"]').classList.add('active');
        this.classList.add('active');
        loadSceneWithDefaultSettings(sceneValue);
      }
    });
  });

  // Initial scene load
  if (simulation && simulation.sceneId) {
    initializeScene(simulation.sceneId);
    simulation.paused = false;
    pausedState = false;
    const pauseBtn = document.getElementById('pause-btn');
    if (pauseBtn) {
      pauseBtn.textContent = '⏸ Pause';
      pauseBtn.classList.remove('active');
    }
  }
});

// Task 1 specific updateMetrics for vector display
function updateStatisticsPanel() {
  if (simulation.bodies.length > 0 && simulation.part === 1) {
    const body = simulation.bodies[0];
    const angularMomentum = computeAngularMomentum(body);
    const kineticEnergy = computeKineticEnergy(body);
    const initialAngularMomentum = simulation.L0 || [0,0,0];
    const angularVelocity = body.w || [0,0,0];
    const angularVelocityMagnitude = vectorLength(angularVelocity);
    
    // Vector display in bracket notation
    setElementTextContent('m-L0-vec', `(${initialAngularMomentum[0].toFixed(3)}, ${initialAngularMomentum[1].toFixed(3)}, ${initialAngularMomentum[2].toFixed(3)})`);
    setElementTextContent('m-L-vec', `(${angularMomentum[0].toFixed(3)}, ${angularMomentum[1].toFixed(3)}, ${angularMomentum[2].toFixed(3)})`);
    
    setElementTextContent('m-E',  kineticEnergy.toFixed(3));
    setElementTextContent('m-E0', simulation.E0 > 1e-9 ? simulation.E0.toFixed(3) : '—');
    setElementTextContent('m-omega', angularVelocityMagnitude.toFixed(3));
  } else {
    setElementTextContent('m-L0-vec', '—');
    setElementTextContent('m-L-vec', '—');
    setElementTextContent('m-E', '—');
    setElementTextContent('m-E0', '—');
    setElementTextContent('m-omega', '—');
  }
}
