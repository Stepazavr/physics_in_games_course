const PART_SCENES = {
  1: ['1A', '1B', '1C', '1D'],
  2: ['2A', '2B', '2C', '2D'],
  3: ['3A', '3B'],
  4: ['4A'],
};

function initUI() {
  const partBtns = document.getElementById('part-btns');
  if (partBtns) {
    _group('part-btns', v => {
      simulation.part = parseInt(v);
      _rebuildSceneButtons();
      _loadSceneWithDefaults(PART_SCENES[simulation.part][0]);
    });
  }
  
  _rebuildSceneButtons();

  const solverBtns = document.getElementById('solver-btns');
  if (solverBtns) {
    _group('solver-btns', v => { simulation.solver = v; _refreshVisibility(); });
  }
  
  const postStabBtns = document.getElementById('poststab-btns');
  if (postStabBtns) {
    _group('poststab-btns', v => { simulation.postStab = v; _refreshVisibility(); });
  }
  
  const broadBtns = document.getElementById('broad-btns');
  if (broadBtns) {
    _group('broad-btns', v => { simulation.broadphase = v; });
  }

  const iterSlider = document.getElementById('s-iter');
  if (iterSlider) _slider('s-iter', 'v-iter', v => simulation.iterations = Math.round(v), v => String(Math.round(v)));
  
  const compSlider = document.getElementById('s-comp');
  if (compSlider) _slider('s-comp', 'v-comp', v => simulation.compliance = v, v => v.toFixed(4));
  
  const betaSlider = document.getElementById('s-beta');
  if (betaSlider) _slider('s-beta', 'v-beta', v => simulation.baumgarteBeta = v, v => v.toFixed(2));
  
  const musSlider = document.getElementById('s-mus');
  if (musSlider) _slider('s-mus',  'v-mus',  v => simulation.muStatic = v, v => v.toFixed(2));
  
  const mudSlider = document.getElementById('s-mud');
  if (mudSlider) _slider('s-mud',  'v-mud',  v => simulation.muDynamic = v, v => v.toFixed(2));
  
  const restSlider = document.getElementById('s-rest');
  if (restSlider) _slider('s-rest', 'v-rest', v => simulation.restitution = v, v => v.toFixed(2));
  
  const gravSlider = document.getElementById('s-grav');
  if (gravSlider) _slider('s-grav', 'v-grav', v => simulation.gravity = v, v => v.toFixed(2));
  
  const kSlider = document.getElementById('s-k');
  if (kSlider) _slider('s-k',    'v-k',    v => _setSpringK(v), v => v.toFixed(0));
  
  const sdSlider = document.getElementById('s-sd');
  if (sdSlider) _slider('s-sd',   'v-sd',   v => _setSpringDamping(v), v => v.toFixed(1));

  const pauseCheckbox = document.getElementById('s-pause');
  if (pauseCheckbox) {
    pauseCheckbox.addEventListener('change', e => simulation.paused = e.target.checked);
  }
  
  const resetBtn = document.getElementById('reset-btn');
  if (resetBtn) {
    resetBtn.addEventListener('click', () => loadScene(simulation.sceneId));
  }

  _refreshVisibility();
}

function _rebuildSceneButtons() {
  const cont = document.getElementById('scene-btns');
  if (!cont) return;
  cont.innerHTML = '';
  const list = PART_SCENES[simulation.part];
  list.forEach((id, idx) => {
    const b = document.createElement('button');
    b.className = 'btn' + (idx === 0 ? ' active' : '');
    b.dataset.val = id;
    b.textContent = SCENES[id].label;
    b.addEventListener('click', () => {
      cont.querySelectorAll('.btn').forEach(x => x.classList.remove('active'));
      b.classList.add('active');
      _loadSceneWithDefaults(id);
    });
    cont.appendChild(b);
  });
}

function _loadSceneWithDefaults(id) {
  loadScene(id);
  _applySceneDefaults(id);
  _refreshBroadphaseEnabled(id);
  _refreshVisibility();
}

function _applySceneDefaults(sc) {
  if (sc === '4A' || sc === '3A' || sc === '3B') {
    _setSolverButton('si');
  }
  const def = { '3A': 'brute', '3B': 'grid', '4A': 'sap' };
  if (def[sc]) _setBroadButton(def[sc]);
  
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
    _setSolverButton('xpbd');
  } else if (sc === '2D') {
    simulation.part2Kind = '2D';
    _setSolverButton('si');
    simulation.part2SI_PostStab = simulation.part2SI_PostStab || 'baumgarte';
  }
}

function _refreshBroadphaseEnabled(sc) {
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

function _refreshVisibility() {
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

  _show('sec-solver',   isContactPart);
  _show('sec-poststab', isSIActive);
  _show('sec-broad',    isContactPart);

  _show('row-k',    isPart2Spring || isSI_Soft || softPostStab);
  _show('row-sd',   isPart2Spring || isSI_Soft || softPostStab);
  
  _show('row-iter', isPart2Spring || isPart2Dist || isContactPart);
  _show('row-comp', isXPBDActive);
  _show('row-beta', isSI_Baumgarte || isSI_Soft || (isContactPart && sv === 'si'));
  _show('row-rest', isSIActive || isContactPart);
  
  _show('row-mus',  p === 4 && isXPBDActive);
  _show('row-mud',  isContactPart);
  _show('row-grav', isPart2Spring);
}

function _show(id, visible) {
  const el = document.getElementById(id);
  if (el) el.style.display = visible ? '' : 'none';
}

function _setBroadButton(val) {
  simulation.broadphase = val;
  const btns = document.querySelectorAll('#broad-btns .btn');
  if (btns.length === 0) return;
  btns.forEach(b => {
    if (b.dataset.val === val) b.classList.add('active');
    else b.classList.remove('active');
  });
}

function _setSolverButton(val) {
  simulation.solver = val;
  const btns = document.querySelectorAll('#solver-btns .btn');
  if (btns.length === 0) return;
  btns.forEach(b => {
    if (b.dataset.val === val) b.classList.add('active');
    else b.classList.remove('active');
  });
}

function _setSpringK(v) {
  simulation.springK = v;
  for (const s of simulation.springs) s.k = v;
}
function _setSpringDamping(v) {
  simulation.springDamping = v;
  for (const s of simulation.springs) s.c = v;
}

function updateMetrics() {
  if (simulation.part === 1) {
    const b = simulation.bodies[0];
    const L = computeAngularMomentum(b);
    const E = computeKineticEnergy(b);
    const L0 = simulation.L0 || [0,0,0];
    const omega = b.w || [0,0,0];
    const omegaLen = vectorLength(omega);
    
    _set('m-L0x', L0[0].toFixed(3));
    _set('m-L0y', L0[1].toFixed(3));
    _set('m-L0z', L0[2].toFixed(3));
    
    _set('m-Lx', L[0].toFixed(3));
    _set('m-Ly', L[1].toFixed(3));
    _set('m-Lz', L[2].toFixed(3));
    
    _set('m-E',  E.toFixed(3));
    _set('m-E0', simulation.E0 > 1e-9 ? simulation.E0.toFixed(3) : '—');
    _set('m-omega', omegaLen.toFixed(3));
  } else if (simulation.part === 2) {
    _set('m-k', simulation.springK ? simulation.springK.toFixed(0) : '200');
    _set('m-sd', simulation.springDamping ? simulation.springDamping.toFixed(1) : '4.0');
    
    if (simulation.iterations) _set('m-iter', String(simulation.iterations));
    if (simulation.compliance !== undefined) _set('m-comp', simulation.compliance.toFixed(4));
    if (simulation.baumgarteBeta !== undefined) _set('m-beta', simulation.baumgarteBeta.toFixed(2));
  }
}

function _group(id, fn) {
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

function _slider(elId, lblId, setter, fmt) {
  const el = document.getElementById(elId);
  if (!el) return;
  el.addEventListener('input', () => {
    const v = parseFloat(el.value);
    const lbl = document.getElementById(lblId);
    if (lbl) lbl.textContent = fmt(v);
    setter(v);
  });
}

function _set(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}
