// Task 3 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons
  pausedState = setupUIControls();
  
  // Initialize scene selection UI for task_3
  const sceneButtonElements = document.querySelectorAll('.scene-btn');
  
  sceneButtonElements.forEach(button => {
    button.addEventListener('click', function() {
      if (this.disabled) return;
      
      const sceneValue = this.dataset.val;
      
      sceneButtonElements.forEach(btn => btn.classList.remove('active'));
      this.classList.add('active');
      
      loadSceneWithDefaultSettings(sceneValue);
    });
  });
  
  // Setup sliders for task_3
  bindSliderToParameter('s-iter', 'v-iter', value => simulation.iterations = Math.round(value), value => String(Math.round(value)));
  bindSliderToParameter('s-comp', 'v-comp', value => simulation.compliance = value, value => value.toFixed(4));
  bindSliderToParameter('s-beta', 'v-beta', value => simulation.baumgarteBeta = value, value => value.toFixed(2));
  bindSliderToParameter('s-mud', 'v-mud', value => simulation.muDynamic = value, value => value.toFixed(2));
  bindSliderToParameter('s-rest', 'v-rest', value => simulation.restitution = value, value => value.toFixed(2));
  
  // Initial scene load
  if (simulation && simulation.sceneId) {
    initializeScene(simulation.sceneId);
    simulation.paused = false;
    pausedState = false;
    const pauseButton = document.getElementById('pause-btn');
    if (pauseButton) {
      pauseButton.textContent = '⏸ Pause';
      pauseButton.classList.remove('active');
    }
  }
});

// Task 3 updateMetrics - show only dynamic bodies count
function updateStatisticsPanel() {
  if (simulation.part === 3) {
    const dynamicBodiesCount = simulation.bodies.filter(body => !body.isStatic).length;
    setElementTextContent('m-bodies', String(dynamicBodiesCount));
  }
}
