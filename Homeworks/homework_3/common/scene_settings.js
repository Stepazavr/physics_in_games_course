const SCENES = {
  '1A': { part: 1, label: 'в глобальных координатах', mode: '1A' },
  '1B': { part: 1, label: 'без гироскопического слагаемого', mode: '1B' },
  '1C': { part: 1, label: 'с гироскопическим слагаемым в явном выражении', mode: '1C' },
  '1D': { part: 1, label: 'с гироскопическим слагаемым в неявном выражении', mode: '1D' },

  '2A': { part: 2, label: '2A · spring force', kind: 'springForce' },
  '2B': { part: 2, label: '2B · spring soft (Buddha)', kind: 'springSoft' },
  '2C': { part: 2, label: '2C · distance XPBD', kind: 'distXPBD' },
  '2D': { part: 2, label: '2D · distance SI',   kind: 'distSI' },

  '3A': { part: 3, label: '3A · 10 кубиков (Bruteforce)',     kind: 'stack' },
  '3B': { part: 3, label: '3B · 1000 кубиков (SpatialGrid)',   kind: 'pile' },

  '4A': { part: 4, label: '4A', kind: 'varied' },
};

// Palette of 10 bright colors for cubes
const BRIGHT_COLORS = [
  [255, 100, 100],  // Bright red
  [255, 180, 50],   // Bright orange
  [255, 255, 100],  // Bright yellow
  [150, 255, 100],  // Bright lime
  [100, 255, 150],  // Bright cyan-green
  [100, 200, 255],  // Bright light blue
  [100, 150, 255],  // Bright blue
  [200, 100, 255],  // Bright purple
  [255, 100, 200],  // Bright pink
  [255, 150, 100],  // Bright coral
];

function loadScene(id) {
  const def = SCENES[id];
  if (!def) return;

  sim.sceneId   = id;
  sim.part      = def.part;
  sim.bodies    = [];
  sim.springs   = [];
  sim.constraints = [];
  sim.contacts  = [];
  sim.broadphasePairs = 0;
  sim.energyHistory = [];
  sim.L0 = null;
  sim.E0 = null;
  sim.elapsed = 0;
  sim.maxC = 0;

  if (def.part === 1) {
    sim.freeRotMode = def.mode;
    _scenePart1();
  } else if (def.part === 2) {
    sim.part2Kind = def.kind;
    _scenePart2(def.kind);
  } else if (def.part === 3) {
    sim.part3Kind = def.kind;
    if (def.kind === 'stack') _sceneStack();
    else _scenePile();
  } else if (def.part === 4) {
    sim.part4Kind = def.kind;
    _sceneVaried();
  }

  if (def.part === 1) {
    const b = sim.bodies[0];
    sim.L0 = bodyAngularMomentum(b);
    sim.E0 = bodyKineticEnergy(b);
    b._L = sim.L0.slice();
  }
}

// ============== Common UI Functions (shared across all tasks) ==============

function setupSlider(slId, lblId, setter, fmt) {
  const sl = document.getElementById(slId);
  if (!sl) return;
  sl.addEventListener('input', () => {
    const v = parseFloat(sl.value);
    const lbl = document.getElementById(lblId);
    if (lbl) lbl.textContent = fmt(v);
    setter(v);
  });
}

function _set(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function initCommonUIElements() {
  let pausedState = false;
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
      if (typeof _refreshBroadphaseEnabled === 'function') {
        _refreshBroadphaseEnabled(SCENES[sim.sceneId]);
      }
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

  return pausedState;
}

// Default empty updateMetrics - override in task-specific scene.js
function updateMetrics() {
  // Override in task-specific files
}
