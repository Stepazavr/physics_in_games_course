// Task 2 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons
  pausedState = setupUIControls();
  
  // Initialize scene selection UI for task_2 (tree structure)
  const pauseBtn = document.getElementById('pause-btn');
  const sceneButtons = document.querySelectorAll('.scene-btn');
  const springBranch = document.getElementById('spring-branch');
  const constraintBranch = document.getElementById('constraint-branch');
  const siBranch = document.getElementById('si-branch');
  
  sceneButtons.forEach(btn => {
    btn.addEventListener('click', function() {
      if (this.disabled) return;
      
      const val = this.dataset.val;
      
      sceneButtons.forEach(b => b.classList.remove('active'));
      this.classList.add('active');
      
      if (val === '2A_parent') {
        springBranch.style.display = 'block';
        constraintBranch.style.display = 'none';
        siBranch.style.display = 'none';
        document.querySelector('[data-val="2A"]').classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2A');
        initializeScene('2A');
      } else if (val === '2A' || val === '2B') {
        springBranch.style.display = 'block';
        constraintBranch.style.display = 'none';
        siBranch.style.display = 'none';
        document.querySelector('[data-val="2A_parent"]').classList.add('active');
        this.classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults(val);
        initializeScene(val);
        initializeScene(val);
      } else if (val === '2C_parent') {
        springBranch.style.display = 'none';
        constraintBranch.style.display = 'block';
        siBranch.style.display = 'none';
        document.querySelector('[data-val="2C"]').classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2C');
        initializeScene('2C');
      } else if (val === '2C') {
        springBranch.style.display = 'none';
        constraintBranch.style.display = 'block';
        siBranch.style.display = 'none';
        document.querySelector('[data-val="2C_parent"]').classList.add('active');
        this.classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2C');
        initializeScene('2C');
      } else if (val === '2D_parent') {
        springBranch.style.display = 'none';
        constraintBranch.style.display = 'block';
        siBranch.style.display = 'block';
        document.querySelector('[data-val="2C_parent"]').classList.add('active');
        this.classList.add('active');
        document.querySelector('[data-val="2D_baumgarte"]').classList.add('active');
        pausedState = false;
        simulation.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2D');
        initializeScene('2D');
      } else if (val === '2D_baumgarte' || val === '2D_nlgs' || val === '2D_soft') {
        springBranch.style.display = 'none';
        constraintBranch.style.display = 'block';
        siBranch.style.display = 'block';
        document.querySelector('[data-val="2C_parent"]').classList.add('active');
        document.querySelector('[data-val="2D_parent"]').classList.add('active');
        this.classList.add('active');
        
        const modeMap = { '2D_baumgarte': 'baumgarte', '2D_nlgs': 'nlgs', '2D_soft': 'soft' };
        simulation.part2SI_PostStab = modeMap[val];
        pausedState = false;
        simulation.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2D');
        initializeScene('2D');
      }
    });
  });
  
  // Setup sliders for task_2
  bindSliderToParameter('s-k', 'v-k', v => { simulation.springK = v; _setSpringK(v); }, v => v.toFixed(0));
  bindSliderToParameter('s-sd', 'v-sd', v => { simulation.springDamping = v; _setSpringDamping(v); }, v => v.toFixed(1));
  bindSliderToParameter('s-iter', 'v-iter', v => simulation.iterations = Math.round(v), v => String(Math.round(v)));
  bindSliderToParameter('s-comp', 'v-comp', v => simulation.compliance = v, v => v.toFixed(4));
  bindSliderToParameter('s-beta', 'v-beta', v => simulation.baumgarteBeta = v, v => v.toFixed(2));
  bindSliderToParameter('s-rest', 'v-rest', v => simulation.restitution = v, v => v.toFixed(2));
  
  // Initial scene load
  _loadSceneWithDefaults('2A');
  initializeScene('2A');
});

// Task 2 updateMetrics (empty - can be added if needed)
function refreshStatisticsDisplay() {
  // Task 2 does not display metrics
}

