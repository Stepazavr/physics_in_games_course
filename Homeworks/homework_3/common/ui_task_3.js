// Task 3 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons
  pausedState = initCommonUIElements();
  
  // Initialize scene selection UI for task_3
  const sceneButtons = document.querySelectorAll('.scene-btn');
  
  sceneButtons.forEach(btn => {
    btn.addEventListener('click', function() {
      if (this.disabled) return;
      
      const val = this.dataset.val;
      
      sceneButtons.forEach(b => b.classList.remove('active'));
      this.classList.add('active');
      
      _loadSceneWithDefaults(val);
    });
  });
  
  // Setup sliders for task_3
  setupSlider('s-iter', 'v-iter', v => simulation.iterations = Math.round(v), v => String(Math.round(v)));
  setupSlider('s-comp', 'v-comp', v => simulation.compliance = v, v => v.toFixed(4));
  setupSlider('s-beta', 'v-beta', v => simulation.baumgarteBeta = v, v => v.toFixed(2));
  setupSlider('s-mud', 'v-mud', v => simulation.muDynamic = v, v => v.toFixed(2));
  setupSlider('s-rest', 'v-rest', v => simulation.restitution = v, v => v.toFixed(2));
  
  // Initial scene load
  if (simulation && simulation.sceneId) {
    loadScene(simulation.sceneId);
    simulation.paused = false;
    pausedState = false;
    const pauseBtn = document.getElementById('pause-btn');
    if (pauseBtn) {
      pauseBtn.textContent = '⏸ Pause';
      pauseBtn.classList.remove('active');
    }
  }
});

// Task 3 updateMetrics - show only dynamic bodies count
function updateMetrics() {
  if (simulation.part === 3) {
    const dynamicBodies = simulation.bodies.filter(b => !b.isStatic).length;
    _set('m-bodies', String(dynamicBodies));
  }
}
