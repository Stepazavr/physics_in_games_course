// Task 4 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons (no broadphase refresh for task 4)
  const pauseBtn = document.getElementById('pause-btn');
  const resetBtn = document.getElementById('reset-btn');

  if (pauseBtn) {
    pauseBtn.addEventListener('click', function() {
      pausedState = !pausedState;
      simulation.paused = pausedState;
      this.classList.toggle('active', pausedState);
      this.textContent = simulation.paused ? '▶ Resume' : '⏸ Pause';
    });
  }

  if (resetBtn) {
    resetBtn.addEventListener('click', function() {
      initializeScene(simulation.sceneId);
      simulation.paused = pausedState;
      if (pauseBtn) {
        if (pausedState) {
          pauseBtn.classList.add('active');
          pauseBtn.textContent = '▶ Resume';
        } else {
          pauseBtn.classList.remove('active');
          pauseBtn.textContent = '⏸ Pause';
        }
      }
    });
  }

  // Setup sliders for task_4
  bindSliderToParameter('s-iter', 'v-iter', v => simulation.iterations = Math.round(v), v => String(Math.round(v)));
  bindSliderToParameter('s-comp', 'v-comp', v => simulation.compliance = v, v => v.toFixed(4));
  bindSliderToParameter('s-beta', 'v-beta', v => simulation.baumgarteBeta = v, v => v.toFixed(2));
  bindSliderToParameter('s-mus', 'v-mus', v => simulation.muStatic = v, v => v.toFixed(2));
  bindSliderToParameter('s-mud', 'v-mud', v => simulation.muDynamic = v, v => v.toFixed(2));
  bindSliderToParameter('s-rest', 'v-rest', v => simulation.restitution = v, v => v.toFixed(2));
  bindSliderToParameter('s-grav', 'v-grav', v => simulation.gravity = v, v => v.toFixed(2));
  
  // Initial scene load
  if (simulation && simulation.sceneId) {
    initializeScene(simulation.sceneId);
    simulation.paused = false;
    pausedState = false;
    if (pauseBtn) {
      pauseBtn.textContent = '⏸ Pause';
      pauseBtn.classList.remove('active');
    }
  }
});

// Task 4 updateMetrics - show only dynamic bodies count
function refreshStatisticsDisplay() {
  if (simulation.part === 4) {
    const dynamicBodies = simulation.bodies.filter(b => !b.isStatic).length;
    updateUIElement('m-bodies', String(dynamicBodies));
  }
}
