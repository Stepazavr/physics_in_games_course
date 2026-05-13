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
  setupSlider('s-iter', 'v-iter', v => sim.iterations = Math.round(v), v => String(Math.round(v)));
  setupSlider('s-comp', 'v-comp', v => sim.compliance = v, v => v.toFixed(4));
  setupSlider('s-beta', 'v-beta', v => sim.baumgarteBeta = v, v => v.toFixed(2));
  setupSlider('s-mud', 'v-mud', v => sim.muDynamic = v, v => v.toFixed(2));
  setupSlider('s-rest', 'v-rest', v => sim.restitution = v, v => v.toFixed(2));
  
  // Initial scene load
  if (sim && sim.sceneId) {
    loadScene(sim.sceneId);
    sim.paused = false;
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
  if (sim.part === 3) {
    const dynamicBodies = sim.bodies.filter(b => !b.isStatic).length;
    _set('m-bodies', String(dynamicBodies));
  }
}
