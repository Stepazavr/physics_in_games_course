// Task 1 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons
  pausedState = setupUIControls();
  
  // Initialize scene selection UI for task_1 (tree structure)
  const sceneButtons = document.querySelectorAll('.scene-btn');
  const localBranch = document.getElementById('local-branch');
  const gyroBranch = document.getElementById('gyro-branch');
  
  sceneButtons.forEach(btn => {
    btn.addEventListener('click', function() {
      if (this.disabled) return;
      
      const val = this.dataset.val;
      
      // Remove active from all buttons
      sceneButtons.forEach(b => b.classList.remove('active'));
      this.classList.add('active');
      
      if (val === '1A') {
        localBranch.style.display = 'none';
        gyroBranch.style.display = 'none';
        loadSceneWithDefaultSettings('1A');
      } else if (val === '1B_parent') {
        localBranch.style.display = 'block';
        gyroBranch.style.display = 'none';
        document.querySelector('[data-val="1B"]').classList.add('active');
        loadSceneWithDefaultSettings('1B');
      } else if (val === '1B') {
        gyroBranch.style.display = 'none';
        document.querySelector('[data-val="1B_parent"]').classList.add('active');
        this.classList.add('active');
        loadSceneWithDefaultSettings('1B');
      } else if (val === '1C_parent') {
        gyroBranch.style.display = 'block';
        document.querySelector('[data-val="1B_parent"]').classList.add('active');
        this.classList.add('active');
        document.querySelector('[data-val="1C"]').classList.add('active');
        loadSceneWithDefaultSettings('1C');
      } else if (val === '1C' || val === '1D') {
        gyroBranch.style.display = 'block';
        document.querySelector('[data-val="1B_parent"]').classList.add('active');
        document.querySelector('[data-val="1C_parent"]').classList.add('active');
        this.classList.add('active');
        loadSceneWithDefaultSettings(val);
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
    const b = simulation.bodies[0];
    const L = computeAngularMomentum(b);
    const E = computeKineticEnergy(b);
    const L0 = simulation.L0 || [0,0,0];
    const omega = b.w || [0,0,0];
    const omegaLen = vectorLength(omega);
    
    // Vector display in bracket notation
    setElementTextContent('m-L0-vec', `(${L0[0].toFixed(3)}, ${L0[1].toFixed(3)}, ${L0[2].toFixed(3)})`);
    setElementTextContent('m-L-vec', `(${L[0].toFixed(3)}, ${L[1].toFixed(3)}, ${L[2].toFixed(3)})`);
    
    setElementTextContent('m-E',  E.toFixed(3));
    setElementTextContent('m-E0', simulation.E0 > 1e-9 ? simulation.E0.toFixed(3) : '—');
    setElementTextContent('m-omega', omegaLen.toFixed(3));
  } else {
    setElementTextContent('m-L0-vec', '—');
    setElementTextContent('m-L-vec', '—');
    setElementTextContent('m-E', '—');
    setElementTextContent('m-E0', '—');
    setElementTextContent('m-omega', '—');
  }
}
