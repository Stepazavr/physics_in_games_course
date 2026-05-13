// Task 4 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons (no broadphase refresh for task 4)
  const pauseButton = document.getElementById('pause-btn');
  const resetButton = document.getElementById('reset-btn');

  if (pauseButton) {
    pauseButton.addEventListener('click', function() {
      pausedState = !pausedState;
      simulation.paused = pausedState;
      this.classList.toggle('active', pausedState);
      this.textContent = simulation.paused ? '▶ Resume' : '⏸ Pause';
    });
  }

  if (resetButton) {
    resetButton.addEventListener('click', function() {
      initializeScene(simulation.sceneId);
      simulation.paused = pausedState;
      if (pauseButton) {
        if (pausedState) {
          pauseButton.classList.add('active');
          pauseButton.textContent = '▶ Resume';
        } else {
          pauseButton.classList.remove('active');
          pauseButton.textContent = '⏸ Pause';
        }
      }
    });
  }

  // Setup sliders for task_4
  bindSliderToParameter('s-iter', 'v-iter', value => simulation.iterations = Math.round(value), value => String(Math.round(value)));
  bindSliderToParameter('s-comp', 'v-comp', value => simulation.compliance = value, value => value.toFixed(4));
  bindSliderToParameter('s-beta', 'v-beta', value => simulation.baumgarteBeta = value, value => value.toFixed(2));
  bindSliderToParameter('s-mus', 'v-mus', value => simulation.muStatic = value, value => value.toFixed(2));
  bindSliderToParameter('s-mud', 'v-mud', value => simulation.muDynamic = value, value => value.toFixed(2));
  bindSliderToParameter('s-rest', 'v-rest', value => simulation.restitution = value, value => value.toFixed(2));
  bindSliderToParameter('s-grav', 'v-grav', value => simulation.gravity = value, value => value.toFixed(2));
  
  // Initial scene load
  if (simulation && simulation.sceneId) {
    initializeScene(simulation.sceneId);
    simulation.paused = false;
    pausedState = false;
    if (pauseButton) {
      pauseButton.textContent = '⏸ Pause';
      pauseButton.classList.remove('active');
    }
  }
});

// Task 4 updateMetrics - show only dynamic bodies count
function updateStatisticsPanel() {
  if (simulation.part === 4) {
    const dynamicBodiesCount = simulation.bodies.filter(body => !body.isStatic).length;
    setElementTextContent('m-bodies', String(dynamicBodiesCount));
  }
}
