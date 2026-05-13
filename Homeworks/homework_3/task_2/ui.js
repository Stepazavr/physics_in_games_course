// Task 2 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons
  pausedState = setupUIControls();
  
  // Initialize scene selection UI for task_2 (tree structure)
  const pauseButton = document.getElementById('pause-btn');
  const sceneButtonElements = document.querySelectorAll('.scene-btn');
  const springSection = document.getElementById('spring-branch');
  const constraintSection = document.getElementById('constraint-branch');
  const siSection = document.getElementById('si-branch');
  
  sceneButtonElements.forEach(button => {
    button.addEventListener('click', function() {
      if (this.disabled) return;
      
      const sceneValue = this.dataset.val;
      
      sceneButtonElements.forEach(btn => btn.classList.remove('active'));
      this.classList.add('active');
      
      if (sceneValue === '2A_parent') {
        springSection.style.display = 'block';
        constraintSection.style.display = 'none';
        siSection.style.display = 'none';
        document.querySelector('[data-val="2A"]').classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseButton.classList.remove('active');
        pauseButton.textContent = '⏸ Pause';
        loadSceneWithDefaultSettings('2A');
        initializeScene('2A');
      } else if (sceneValue === '2A' || sceneValue === '2B') {
        springSection.style.display = 'block';
        constraintSection.style.display = 'none';
        siSection.style.display = 'none';
        document.querySelector('[data-val="2A_parent"]').classList.add('active');
        this.classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseButton.classList.remove('active');
        pauseButton.textContent = '⏸ Pause';
        loadSceneWithDefaultSettings(sceneValue);
        initializeScene(sceneValue);
        initializeScene(sceneValue);
      } else if (sceneValue === '2C_parent') {
        springSection.style.display = 'none';
        constraintSection.style.display = 'block';
        siSection.style.display = 'none';
        document.querySelector('[data-val="2C"]').classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseButton.classList.remove('active');
        pauseButton.textContent = '⏸ Pause';
        loadSceneWithDefaultSettings('2C');
        initializeScene('2C');
      } else if (sceneValue === '2C') {
        springSection.style.display = 'none';
        constraintSection.style.display = 'block';
        siSection.style.display = 'none';
        document.querySelector('[data-val="2C_parent"]').classList.add('active');
        this.classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseButton.classList.remove('active');
        pauseButton.textContent = '⏸ Pause';
        loadSceneWithDefaultSettings('2C');
        initializeScene('2C');
      } else if (sceneValue === '2D_parent') {
        springSection.style.display = 'none';
        constraintSection.style.display = 'block';
        siSection.style.display = 'block';
        document.querySelector('[data-val="2C_parent"]').classList.add('active');
        this.classList.add('active');
        document.querySelector('[data-val="2D_baumgarte"]').classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseButton.classList.remove('active');
        pauseButton.textContent = '⏸ Pause';
        loadSceneWithDefaultSettings('2D');
        initializeScene('2D');
      } else if (sceneValue === '2D_baumgarte' || sceneValue === '2D_nlgs' || sceneValue === '2D_soft') {
        springSection.style.display = 'none';
        constraintSection.style.display = 'block';
        siSection.style.display = 'block';
        document.querySelector('[data-val="2C_parent"]').classList.add('active');
        document.querySelector('[data-val="2D_parent"]').classList.add('active');
        this.classList.add('active');
        
        const postStabModeMap = { '2D_baumgarte': 'baumgarte', '2D_nlgs': 'nlgs', '2D_soft': 'soft' };
        simulation.part2SI_PostStab = postStabModeMap[sceneValue];
        pausedState = false;
        simulation.paused = false;
        pauseButton.classList.remove('active');
        pauseButton.textContent = '⏸ Pause';
        loadSceneWithDefaultSettings('2D');
        initializeScene('2D');
      }
    });
  });
  
  // Setup sliders for task_2
  bindSliderToParameter('s-k', 'v-k', value => { simulation.springK = value; updateSpringStiffness(value); }, value => value.toFixed(0));
  bindSliderToParameter('s-sd', 'v-sd', value => { simulation.springDamping = value; updateSpringDamping(value); }, value => value.toFixed(1));
  bindSliderToParameter('s-iter', 'v-iter', value => simulation.iterations = Math.round(value), value => String(Math.round(value)));
  bindSliderToParameter('s-comp', 'v-comp', value => simulation.compliance = value, value => value.toFixed(4));
  bindSliderToParameter('s-beta', 'v-beta', value => simulation.baumgarteBeta = value, value => value.toFixed(2));
  bindSliderToParameter('s-rest', 'v-rest', value => simulation.restitution = value, value => value.toFixed(2));
  
  // Initial scene load
  loadSceneWithDefaultSettings('2A');
  initializeScene('2A');
});

// Task 2 updateMetrics (empty - can be added if needed)
function updateStatisticsPanel() {
  // Task 2 does not display metrics
}

