// Task 3 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons
  pausedState = setupUIControls();
  
  // Initialize scene selection UI for task_3
  const sceneButtons = document.querySelectorAll('.scene-btn');
  
  sceneButtons.forEach(btn => {
    btn.addEventListener('click', function() {
      if (this.disabled) return;
      
      const val = this.dataset.val;
      
      sceneButtons.forEach(b => b.classList.remove('active'));
      this.classList.add('active');
      
      loadSceneWithDefaultSettings(val);
    });
  });
  
  // Setup sliders for task_3
  bindSliderToParameter('s-iter', 'v-iter', v => simulation.iterations = Math.round(v), v => String(Math.round(v)));
  bindSliderToParameter('s-comp', 'v-comp', v => simulation.compliance = v, v => v.toFixed(4));
  bindSliderToParameter('s-beta', 'v-beta', v => simulation.baumgarteBeta = v, v => v.toFixed(2));
  bindSliderToParameter('s-mud', 'v-mud', v => simulation.muDynamic = v, v => v.toFixed(2));
  bindSliderToParameter('s-rest', 'v-rest', v => simulation.restitution = v, v => v.toFixed(2));
  
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

// Task 3 updateMetrics - show only dynamic bodies count
function updateStatisticsPanel() {
  if (simulation.part === 3) {
    const dynamicBodies = simulation.bodies.filter(b => !b.isStatic).length;
    setElementTextContent('m-bodies', String(dynamicBodies));
  }
}
