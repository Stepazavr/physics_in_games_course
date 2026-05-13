// Task 4 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons (no broadphase refresh for task 4)
  const pauseBtn = document.getElementById('pause-btn');
  const resetBtn = document.getElementById('reset-btn');

  if (pauseBtn) {
    pauseBtn.addEventListener('click', function() {
      pausedState = !pausedState;
      sim.paused = pausedState;
      this.classList.toggle('active', pausedState);
      this.textContent = sim.paused ? '▶ Resume' : '⏸ Pause';
    });
  }

  if (resetBtn) {
    resetBtn.addEventListener('click', function() {
      loadScene(sim.sceneId);
      sim.paused = pausedState;
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
  setupSlider('s-iter', 'v-iter', v => sim.iterations = Math.round(v), v => String(Math.round(v)));
  setupSlider('s-comp', 'v-comp', v => sim.compliance = v, v => v.toFixed(4));
  setupSlider('s-beta', 'v-beta', v => sim.baumgarteBeta = v, v => v.toFixed(2));
  setupSlider('s-mus', 'v-mus', v => sim.muStatic = v, v => v.toFixed(2));
  setupSlider('s-mud', 'v-mud', v => sim.muDynamic = v, v => v.toFixed(2));
  setupSlider('s-rest', 'v-rest', v => sim.restitution = v, v => v.toFixed(2));
  setupSlider('s-grav', 'v-grav', v => sim.gravity = v, v => v.toFixed(2));
  
  // Initial scene load
  if (sim && sim.sceneId) {
    loadScene(sim.sceneId);
    sim.paused = false;
    pausedState = false;
    if (pauseBtn) {
      pauseBtn.textContent = '⏸ Pause';
      pauseBtn.classList.remove('active');
    }
  }
});

// Task 4 updateMetrics - show only dynamic bodies count
function updateMetrics() {
  if (sim.part === 4) {
    const dynamicBodies = sim.bodies.filter(b => !b.isStatic).length;
    _set('m-bodies', String(dynamicBodies));
  }
}
