// Task 2 UI initialization and event handling

let pausedState = false;

document.addEventListener('DOMContentLoaded', function() {
  // Initialize pause/reset buttons
  pausedState = initCommonUIElements();
  
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
        sim.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2A');
        loadScene('2A');
      } else if (val === '2A' || val === '2B') {
        springBranch.style.display = 'block';
        constraintBranch.style.display = 'none';
        siBranch.style.display = 'none';
        document.querySelector('[data-val="2A_parent"]').classList.add('active');
        this.classList.add('active');
        pausedState = false;
        sim.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults(val);
        loadScene(val);
        loadScene(val);
      } else if (val === '2C_parent') {
        springBranch.style.display = 'none';
        constraintBranch.style.display = 'block';
        siBranch.style.display = 'none';
        document.querySelector('[data-val="2C"]').classList.add('active');
        pausedState = false;
        sim.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2C');
        loadScene('2C');
      } else if (val === '2C') {
        springBranch.style.display = 'none';
        constraintBranch.style.display = 'block';
        siBranch.style.display = 'none';
        document.querySelector('[data-val="2C_parent"]').classList.add('active');
        this.classList.add('active');
        pausedState = false;
        sim.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2C');
        loadScene('2C');
      } else if (val === '2D_parent') {
        springBranch.style.display = 'none';
        constraintBranch.style.display = 'block';
        siBranch.style.display = 'block';
        document.querySelector('[data-val="2C_parent"]').classList.add('active');
        this.classList.add('active');
        document.querySelector('[data-val="2D_baumgarte"]').classList.add('active');
        pausedState = false;
        sim.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2D');
        loadScene('2D');
      } else if (val === '2D_baumgarte' || val === '2D_nlgs' || val === '2D_soft') {
        springBranch.style.display = 'none';
        constraintBranch.style.display = 'block';
        siBranch.style.display = 'block';
        document.querySelector('[data-val="2C_parent"]').classList.add('active');
        document.querySelector('[data-val="2D_parent"]').classList.add('active');
        this.classList.add('active');
        
        const modeMap = { '2D_baumgarte': 'baumgarte', '2D_nlgs': 'nlgs', '2D_soft': 'soft' };
        sim.part2SI_PostStab = modeMap[val];
        pausedState = false;
        sim.paused = false;
        pauseBtn.classList.remove('active');
        pauseBtn.textContent = '⏸ Pause';
        _loadSceneWithDefaults('2D');
        loadScene('2D');
      }
    });
  });
  
  // Setup sliders for task_2
  setupSlider('s-k', 'v-k', v => { sim.springK = v; _setSpringK(v); }, v => v.toFixed(0));
  setupSlider('s-sd', 'v-sd', v => { sim.springDamping = v; _setSpringDamping(v); }, v => v.toFixed(1));
  setupSlider('s-iter', 'v-iter', v => sim.iterations = Math.round(v), v => String(Math.round(v)));
  setupSlider('s-comp', 'v-comp', v => sim.compliance = v, v => v.toFixed(4));
  setupSlider('s-beta', 'v-beta', v => sim.baumgarteBeta = v, v => v.toFixed(2));
  setupSlider('s-rest', 'v-rest', v => sim.restitution = v, v => v.toFixed(2));
  
  // Initial scene load
  _loadSceneWithDefaults('2A');
  loadScene('2A');
});

// Task 2 updateMetrics (empty - can be added if needed)
function updateMetrics() {
  // Task 2 does not display metrics
}

