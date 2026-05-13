const SCENE_CONFIGURATIONS = {
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

function initializeScene(id) {
  const def = SCENE_CONFIGURATIONS[id];
  if (!def) return;

  simulation.sceneId   = id;
  simulation.part      = def.part;
  simulation.bodies    = [];
  simulation.springs   = [];
  simulation.constraints = [];
  simulation.contacts  = [];
  simulation.broadphasePairs = 0;
  simulation.energyHistory = [];
  simulation.L0 = null;
  simulation.E0 = null;
  simulation.elapsed = 0;
  simulation.maxC = 0;

  if (def.part === 1) {
    simulation.freeRotMode = def.mode;
    _scenePart1();
  } else if (def.part === 2) {
    simulation.part2Kind = def.kind;
    _scenePart2(def.kind);
  } else if (def.part === 3) {
    simulation.part3Kind = def.kind;
    if (def.kind === 'stack') _sceneStack();
    else _scenePile();
  } else if (def.part === 4) {
    simulation.part4Kind = def.kind;
    _sceneVaried();
  }

  if (def.part === 1) {
    const b = simulation.bodies[0];
    simulation.L0 = computeAngularMomentum(b);
    simulation.E0 = computeKineticEnergy(b);
    b._L = simulation.L0.slice();
  }
}

// ============== Common UI Functions (shared across all tasks) ==============

function bindSliderToParameter(slId, lblId, setter, fmt) {
  const sl = document.getElementById(slId);
  if (!sl) return;
  sl.addEventListener('input', () => {
    const v = parseFloat(sl.value);
    const lbl = document.getElementById(lblId);
    if (lbl) lbl.textContent = fmt(v);
    setter(v);
  });
}

function updateUIElement(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function setupUIControls() {
  let pausedState = false;
  const pauseBtn = document.getElementById('pause-btn');
  const resetBtn = document.getElementById('reset-btn');

  if (pauseBtn) {
    pauseBtn.addEventListener('click', function() {
      pausedState = !pausedState;
      simulation.paused = pausedState;
      this.classList.toggle('active', pausedState);
      this.textContent = simulation.paused ? '▶ Resume' : '⏸ Pause';
    });
  }

  if (resetBtn) {
    resetBtn.addEventListener('click', function() {
      initializeScene(simulation.sceneId);
      if (typeof _refreshBroadphaseEnabled === 'function') {
        _refreshBroadphaseEnabled(SCENE_CONFIGURATIONS[simulation.sceneId]);
      }
      simulation.paused = pausedState;
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

// Default empty refreshStatisticsDisplay - override in task-specific scene.js
function refreshStatisticsDisplay() {
  // Override in task-specific files
}
