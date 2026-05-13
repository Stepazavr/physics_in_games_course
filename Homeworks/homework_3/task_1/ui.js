const PART_SCENES = {
  1: ['1A', '1B', '1C', '1D'],
  2: ['2A', '2B', '2C', '2D'],
  3: ['3A', '3B'],
  4: ['4A', '4B', '4C'],
};

function initUI() {
  _group('part-btns', v => {
    sim.part = parseInt(v);
    _rebuildSceneButtons();
    _loadSceneWithDefaults(PART_SCENES[sim.part][0]);
  });
  _rebuildSceneButtons();

  _group('solver-btns', v => { sim.solver = v; _refreshVisibility(); });
  _group('poststab-btns', v => { sim.postStab = v; _refreshVisibility(); });
  _group('broad-btns', v => {
    sim.broadphase = v;
  });

  _slider('s-iter', 'v-iter', v => sim.iterations = Math.round(v), v => String(Math.round(v)));
  _slider('s-comp', 'v-comp', v => sim.compliance = v, v => v.toFixed(4));
  _slider('s-beta', 'v-beta', v => sim.baumgarteBeta = v, v => v.toFixed(2));
  _slider('s-mus',  'v-mus',  v => sim.muStatic = v, v => v.toFixed(2));
  _slider('s-mud',  'v-mud',  v => sim.muDynamic = v, v => v.toFixed(2));
  _slider('s-rest', 'v-rest', v => sim.restitution = v, v => v.toFixed(2));
  _slider('s-grav', 'v-grav', v => sim.gravity = v, v => v.toFixed(2));
  _slider('s-k',    'v-k',    v => _setSpringK(v), v => v.toFixed(0));
  _slider('s-sd',   'v-sd',   v => _setSpringDamping(v), v => v.toFixed(1));

  document.getElementById('s-pause').addEventListener('change', e => sim.paused = e.target.checked);
  document.getElementById('reset-btn').addEventListener('click', () => loadScene(sim.sceneId));

  _refreshVisibility();
}

function _rebuildSceneButtons() {
  const cont = document.getElementById('scene-btns');
  cont.innerHTML = '';
  const list = PART_SCENES[sim.part];
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
  if (sc === '4C')      _setSolverButton('xpbd');
  else if (sc === '4A' || sc === '4B' || sc === '3A' || sc === '3B') {
    _setSolverButton('si');
  }
  const def = { '3A': 'brute', '3B': 'grid', '4A': 'sap', '4B': 'lbvh', '4C': 'brute' };
  if (def[sc]) _setBroadButton(def[sc]);
}

function _refreshBroadphaseEnabled(sc) {
  const brBtns = document.querySelectorAll('#broad-btns .btn');
  brBtns.forEach(b => b.disabled = false);
  if (sc === '4B') {
    brBtns.forEach(b => b.disabled = b.dataset.val !== 'lbvh');
  } else if (sc === '3A' || sc === '3B') {
    brBtns.forEach(b => {
      const ok = b.dataset.val === 'brute' || b.dataset.val === 'grid';
      b.disabled = !ok;
    });
  }
}

function _refreshVisibility() {
  const p = sim.part;
  const sc = sim.sceneId;
  const sv = sim.solver;
  const ps = sim.postStab;
  const isContactPart = (p === 3 || p === 4);
  const isPart2Spring = (sc === '2A' || sc === '2B');
  const isPart2Dist   = (sc === '2C' || sc === '2D');
  const isXPBDActive  = (sc === '2C') || (isContactPart && sv === 'xpbd');
  const isSIActive    = (sc === '2D') || (isContactPart && sv === 'si');
  const softPostStab  = (sc === '2D' && ps === 'soft');

  _show('sec-solver',   isContactPart);
  _show('sec-poststab', isSIActive);
  _show('sec-broad',    isContactPart);

  _show('row-iter',  sc === '2B' || isSIActive);
  _show('row-comp',  isXPBDActive);
  _show('row-beta',  isSIActive && ps === 'baumgarte');
  _show('row-mus',   p === 4 && isXPBDActive);
  _show('row-mud',   isContactPart);
  _show('row-rest',  isSIActive);
  _show('row-grav',  isPart2Spring || isContactPart);
  _show('row-k',     isPart2Spring || softPostStab);
  _show('row-sd',    isPart2Spring || softPostStab);
}

function _show(id, visible) {
  const el = document.getElementById(id);
  if (el) el.style.display = visible ? '' : 'none';
}

function _setBroadButton(val) {
  sim.broadphase = val;
  document.querySelectorAll('#broad-btns .btn').forEach(b => {
    if (b.dataset.val === val) b.classList.add('active');
    else b.classList.remove('active');
  });
}

function _setSolverButton(val) {
  sim.solver = val;
  document.querySelectorAll('#solver-btns .btn').forEach(b => {
    if (b.dataset.val === val) b.classList.add('active');
    else b.classList.remove('active');
  });
}

function _setSpringK(v) {
  sim.springK = v;
  for (const s of sim.springs) s.k = v;
}
function _setSpringDamping(v) {
  sim.springDamping = v;
  for (const s of sim.springs) s.c = v;
}

function updateMetrics() {
  _set('m-fps', sim.fps);
  _set('m-steps', sim.stepCount || 0);
  _set('m-bodies', sim.bodies.length);
  _set('m-contacts', sim.contacts.length);
  if (sim.bodies.length > 0 && sim.part === 1) {
    const L = bodyAngularMomentum(sim.bodies[0]);
    const E = bodyKineticEnergy(sim.bodies[0]);
    const L0 = sim.L0 || [0,0,0];
    const Ln  = vLen(L);
    const L0n = vLen(L0);
    _set('m-L',  Ln.toFixed(3));
    _set('m-L0', L0n.toFixed(3));
    _set('m-Ldr', L0n > 1e-9 ? (100 * vLen(vSub(L, L0)) / L0n).toFixed(2) + '%' : '—');
    _set('m-E',  E.toFixed(3));
    _set('m-Er', sim.E0 > 1e-9 ? (E / sim.E0).toFixed(3) : '—');
  } else {
    _set('m-L', '—'); _set('m-L0', '—'); _set('m-Ldr', '—');
    _set('m-E', '—'); _set('m-Er', '—');
  }
  _set('m-bp', sim.broadphasePairs);
  _set('m-C', sim.maxC.toFixed(4));
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
