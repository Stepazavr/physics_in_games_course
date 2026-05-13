const SCENES_BY_PART = {
  1: ['1A', '1B', '1C', '1D'],
  2: ['2A', '2B', '2C', '2D'],
  3: ['3A', '3B'],
  4: ['4A'],
};

function setupUserInterface() {
  const partBtns = document.getElementById('part-btns');
  if (partBtns) {
    setupButtonGroup('part-btns', v => {
      simulation.part = parseInt(v);
      updateSceneButtonsPanel();
      loadSceneWithDefaultSettings(SCENES_BY_PART[simulation.part][0]);
    });
  }
  
  updateSceneButtonsPanel();

  const solverBtns = document.getElementById('solver-btns');
  if (solverBtns) {
    setupButtonGroup('solver-btns', v => { simulation.solver = v; updateUIVisibility(); });
  }
  
  const postStabBtns = document.getElementById('poststab-btns');
  if (postStabBtns) {
    setupButtonGroup('poststab-btns', v => { simulation.postStab = v; updateUIVisibility(); });
  }
  
  const broadBtns = document.getElementById('broad-btns');
  if (broadBtns) {
    setupButtonGroup('broad-btns', v => { simulation.broadphase = v; });
  }

  const iterSlider = document.getElementById('s-iter');
  if (iterSlider) setupSliderControl('s-iter', 'v-iter', v => simulation.iterations = Math.round(v), v => String(Math.round(v)));
  
  const compSlider = document.getElementById('s-comp');
  if (compSlider) setupSliderControl('s-comp', 'v-comp', v => simulation.compliance = v, v => v.toFixed(4));
  
  const betaSlider = document.getElementById('s-beta');
  if (betaSlider) setupSliderControl('s-beta', 'v-beta', v => simulation.baumgarteBeta = v, v => v.toFixed(2));
  
  const musSlider = document.getElementById('s-mus');
  if (musSlider) setupSliderControl('s-mus',  'v-mus',  v => simulation.muStatic = v, v => v.toFixed(2));
  
  const mudSlider = document.getElementById('s-mud');
  if (mudSlider) setupSliderControl('s-mud',  'v-mud',  v => simulation.muDynamic = v, v => v.toFixed(2));
  
  const restSlider = document.getElementById('s-rest');
  if (restSlider) setupSliderControl('s-rest', 'v-rest', v => simulation.restitution = v, v => v.toFixed(2));
  
  const gravSlider = document.getElementById('s-grav');
  if (gravSlider) setupSliderControl('s-grav', 'v-grav', v => simulation.gravity = v, v => v.toFixed(2));
  
  const kSlider = document.getElementById('s-k');
  if (kSlider) setupSliderControl('s-k',    'v-k',    v => updateSpringStiffness(v), v => v.toFixed(0));
  
  const sdSlider = document.getElementById('s-sd');
  if (sdSlider) setupSliderControl('s-sd',   'v-sd',   v => updateSpringDamping(v), v => v.toFixed(1));

  const pauseCheckbox = document.getElementById('s-pause');
  if (pauseCheckbox) {
    pauseCheckbox.addEventListener('change', e => simulation.paused = e.target.checked);
  }
  
  const resetBtn = document.getElementById('reset-btn');
  if (resetBtn) {
    resetBtn.addEventListener('click', () => initializeScene(simulation.sceneId));
  }

  updateUIVisibility();
}

function updateSceneButtonsPanel() {
  const cont = document.getElementById('scene-btns');
  if (!cont) return;
  cont.innerHTML = '';
  const list = SCENES_BY_PART[simulation.part];
  list.forEach((id, idx) => {
    const b = document.createElement('button');
    b.className = 'btn' + (idx === 0 ? ' active' : '');
    b.dataset.val = id;
    b.textContent = SCENE_CONFIGURATIONS[id].label;
    b.addEventListener('click', () => {
      cont.querySelectorAll('.btn').forEach(x => x.classList.remove('active'));
      b.classList.add('active');
      loadSceneWithDefaultSettings(id);
    });
    cont.appendChild(b);
  });
}

function loadSceneWithDefaultSettings(id) {
  initializeScene(id);
  applySceneSpecificSettings(id);
  updateBroadphaseButtonsState(id);
  updateUIVisibility();
}

function applySceneSpecificSettings(sc) {
  if (sc === '4A' || sc === '3A' || sc === '3B') {
    setActiveSolverMethod('si');
  }
  const def = { '3A': 'brute', '3B': 'grid', '4A': 'sap' };
  if (def[sc]) setActiveBroadphaseMethod(def[sc]);
  
  // Adjust camera distance for large scenes
  if (sc === '3B' || sc === '4A') {
    simulation.cameraDistOverride = 28;
  } else {
    simulation.cameraDistOverride = null;
  }
  
  if (sc === '2A' || sc === '2B') {
    simulation.part2Kind = sc;
  } else if (sc === '2C') {
    simulation.part2Kind = '2C';
    setActiveSolverMethod('xpbd');
  } else if (sc === '2D') {
    simulation.part2Kind = '2D';
    setActiveSolverMethod('si');
    simulation.part2SI_PostStab = simulation.part2SI_PostStab || 'baumgarte';
  }
}

function updateBroadphaseButtonsState(sc) {
  const brBtns = document.querySelectorAll('#broad-btns .btn');
  if (brBtns.length === 0) return;
  brBtns.forEach(b => b.disabled = false);
  if (sc === '3A' || sc === '3B') {
    brBtns.forEach(b => {
      const ok = b.dataset.val === 'brute' || b.dataset.val === 'grid';
      b.disabled = !ok;
    });
  }
}

function updateUIVisibility() {
  const p = simulation.part;
  const sc = simulation.sceneId;
  const sv = simulation.solver;
  const ps = simulation.postStab;
  const part2SI_PS = simulation.part2SI_PostStab || 'baumgarte';
  const isContactPart = (p === 3 || p === 4);
  const isPart2Spring = (sc === '2A' || sc === '2B');
  const isPart2Dist   = (sc === '2C' || sc === '2D');
  const isXPBDActive  = (sc === '2C') || (isContactPart && sv === 'xpbd');
  const isSIActive    = (sc === '2D') || (isContactPart && sv === 'si');
  const isSI_Baumgarte = sc === '2D' && part2SI_PS === 'baumgarte';
  const isSI_NLGS     = sc === '2D' && part2SI_PS === 'nlgs';
  const isSI_Soft     = sc === '2D' && part2SI_PS === 'soft';
  const softPostStab  = (sc === '2D' && ps === 'soft');

  toggleElementVisibility('sec-solver',   isContactPart);
  toggleElementVisibility('sec-poststab', isSIActive);
  toggleElementVisibility('sec-broad',    isContactPart);

  toggleElementVisibility('row-k',    isPart2Spring || isSI_Soft || softPostStab);
  toggleElementVisibility('row-sd',   isPart2Spring || isSI_Soft || softPostStab);
  
  toggleElementVisibility('row-iter', isPart2Spring || isPart2Dist || isContactPart);
  toggleElementVisibility('row-comp', isXPBDActive);
  toggleElementVisibility('row-beta', isSI_Baumgarte || isSI_Soft || (isContactPart && sv === 'si'));
  toggleElementVisibility('row-rest', isSIActive || isContactPart);
  
  toggleElementVisibility('row-mus',  p === 4 && isXPBDActive);
  toggleElementVisibility('row-mud',  isContactPart);
  toggleElementVisibility('row-grav', isPart2Spring);
}

function toggleElementVisibility(id, visible) {
  const el = document.getElementById(id);
  if (el) el.style.display = visible ? '' : 'none';
}

function setActiveBroadphaseMethod(val) {
  simulation.broadphase = val;
  const btns = document.querySelectorAll('#broad-btns .btn');
  if (btns.length === 0) return;
  btns.forEach(b => {
    if (b.dataset.val === val) b.classList.add('active');
    else b.classList.remove('active');
  });
}

function setActiveSolverMethod(val) {
  simulation.solver = val;
  const btns = document.querySelectorAll('#solver-btns .btn');
  if (btns.length === 0) return;
  btns.forEach(b => {
    if (b.dataset.val === val) b.classList.add('active');
    else b.classList.remove('active');
  });
}

function updateSpringStiffness(v) {
  simulation.springK = v;
  for (const s of simulation.springs) s.k = v;
}
function updateSpringDamping(v) {
  simulation.springDamping = v;
  for (const s of simulation.springs) s.c = v;
}

function updateStatisticsPanel() {
  if (simulation.part === 1) {
    const b = simulation.bodies[0];
    const L = computeAngularMomentum(b);
    const E = computeKineticEnergy(b);
    const L0 = simulation.L0 || [0,0,0];
    const omega = b.w || [0,0,0];
    const omegaLen = vectorLength(omega);
    
    setElementTextContent('m-L0x', L0[0].toFixed(3));
    setElementTextContent('m-L0y', L0[1].toFixed(3));
    setElementTextContent('m-L0z', L0[2].toFixed(3));
    
    setElementTextContent('m-Lx', L[0].toFixed(3));
    setElementTextContent('m-Ly', L[1].toFixed(3));
    setElementTextContent('m-Lz', L[2].toFixed(3));
    
    setElementTextContent('m-E',  E.toFixed(3));
    setElementTextContent('m-E0', simulation.E0 > 1e-9 ? simulation.E0.toFixed(3) : '—');
    setElementTextContent('m-omega', omegaLen.toFixed(3));
  } else if (simulation.part === 2) {
    setElementTextContent('m-k', simulation.springK ? simulation.springK.toFixed(0) : '200');
    setElementTextContent('m-sd', simulation.springDamping ? simulation.springDamping.toFixed(1) : '4.0');
    
    if (simulation.iterations) setElementTextContent('m-iter', String(simulation.iterations));
    if (simulation.compliance !== undefined) setElementTextContent('m-comp', simulation.compliance.toFixed(4));
    if (simulation.baumgarteBeta !== undefined) setElementTextContent('m-beta', simulation.baumgarteBeta.toFixed(2));
  }
}

function setupButtonGroup(id, fn) {
  const g = document.getElementById(id);
  if (!g) return;
  g.querySelectorAll('.btn').forEach(b => {
    b.addEventListener('click', () => {
      if (b.disabled) return;
      g.querySelectorAll('.btn').forEach(x => x.classList.remove('active'));
      b.classList.add('active');
      fn(b.dataset.val);
    });
  });
}

function setupSliderControl(elId, lblId, setter, fmt) {
  const el = document.getElementById(elId);
  if (!el) return;
  el.addEventListener('input', () => {
    const v = parseFloat(el.value);
    const lbl = document.getElementById(lblId);
    if (lbl) lbl.textContent = fmt(v);
    setter(v);
  });
}

function setElementTextContent(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}
