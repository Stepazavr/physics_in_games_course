const SCENES_BY_PART = {
  1: ['1A', '1B', '1C', '1D'],
  2: ['2A', '2B', '2C', '2D'],
  3: ['3A', '3B'],
  4: ['4A'],
};

function setupUserInterface() {
  const partButtonsContainer = document.getElementById('part-btns');
  if (partButtonsContainer) {
    setupButtonGroup('part-btns', part => {
      simulation.part = parseInt(part);
      updateSceneButtonsPanel();
      loadSceneWithDefaultSettings(SCENES_BY_PART[simulation.part][0]);
    });
  }
  
  updateSceneButtonsPanel();

  const solverButtonsContainer = document.getElementById('solver-btns');
  if (solverButtonsContainer) {
    setupButtonGroup('solver-btns', solverMethod => { simulation.solver = solverMethod; updateUIVisibility(); });
  }
  
  const postStabButtonsContainer = document.getElementById('poststab-btns');
  if (postStabButtonsContainer) {
    setupButtonGroup('poststab-btns', postStabMethod => { simulation.postStab = postStabMethod; updateUIVisibility(); });
  }
  
  const broadphaseButtonsContainer = document.getElementById('broad-btns');
  if (broadphaseButtonsContainer) {
    setupButtonGroup('broad-btns', broadphaseMethod => { simulation.broadphase = broadphaseMethod; });
  }

  const iterationsSlider = document.getElementById('s-iter');
  if (iterationsSlider) setupSliderControl('s-iter', 'v-iter', value => simulation.iterations = Math.round(value), value => String(Math.round(value)));
  
  const complianceSlider = document.getElementById('s-comp');
  if (complianceSlider) setupSliderControl('s-comp', 'v-comp', value => simulation.compliance = value, value => value.toFixed(4));
  
  const baumgarteBetaSlider = document.getElementById('s-beta');
  if (baumgarteBetaSlider) setupSliderControl('s-beta', 'v-beta', value => simulation.baumgarteBeta = value, value => value.toFixed(2));
  
  const muStaticSlider = document.getElementById('s-mus');
  if (muStaticSlider) setupSliderControl('s-mus',  'v-mus',  value => simulation.muStatic = value, value => value.toFixed(2));
  
  const muDynamicSlider = document.getElementById('s-mud');
  if (muDynamicSlider) setupSliderControl('s-mud',  'v-mud',  value => simulation.muDynamic = value, value => value.toFixed(2));
  
  const restitutionSlider = document.getElementById('s-rest');
  if (restitutionSlider) setupSliderControl('s-rest', 'v-rest', value => simulation.restitution = value, value => value.toFixed(2));
  
  const gravitySlider = document.getElementById('s-grav');
  if (gravitySlider) setupSliderControl('s-grav', 'v-grav', value => simulation.gravity = value, value => value.toFixed(2));
  
  const springStiffnessSlider = document.getElementById('s-k');
  if (springStiffnessSlider) setupSliderControl('s-k',    'v-k',    value => updateSpringStiffness(value), value => value.toFixed(0));
  
  const springDampingSlider = document.getElementById('s-sd');
  if (springDampingSlider) setupSliderControl('s-sd',   'v-sd',   value => updateSpringDamping(value), value => value.toFixed(1));

  const pauseCheckbox = document.getElementById('s-pause');
  if (pauseCheckbox) {
    pauseCheckbox.addEventListener('change', event => simulation.paused = event.target.checked);
  }
  
  const resetButton = document.getElementById('reset-btn');
  if (resetButton) {
    resetButton.addEventListener('click', () => initializeScene(simulation.sceneId));
  }

  updateUIVisibility();
}

function updateSceneButtonsPanel() {
  const sceneButtonsContainer = document.getElementById('scene-btns');
  if (!sceneButtonsContainer) return;
  sceneButtonsContainer.innerHTML = '';
  const sceneIds = SCENES_BY_PART[simulation.part];
  sceneIds.forEach((sceneId, index) => {
    const buttonElement = document.createElement('button');
    buttonElement.className = 'btn' + (index === 0 ? ' active' : '');
    buttonElement.dataset.val = sceneId;
    buttonElement.textContent = SCENE_CONFIGURATIONS[sceneId].label;
    buttonElement.addEventListener('click', () => {
      sceneButtonsContainer.querySelectorAll('.btn').forEach(btn => btn.classList.remove('active'));
      buttonElement.classList.add('active');
      loadSceneWithDefaultSettings(sceneId);
    });
    sceneButtonsContainer.appendChild(buttonElement);
  });
}

function loadSceneWithDefaultSettings(id) {
  initializeScene(id);
  applySceneSpecificSettings(id);
  updateBroadphaseButtonsState(id);
  updateUIVisibility();
}

function applySceneSpecificSettings(sceneId) {
  if (sceneId === '4A' || sceneId === '3A' || sceneId === '3B') {
    setActiveSolverMethod('si');
  }
  const broadphaseAssignments = { '3A': 'brute', '3B': 'grid', '4A': 'sap' };
  if (broadphaseAssignments[sceneId]) setActiveBroadphaseMethod(broadphaseAssignments[sceneId]);
  
  // Adjust camera distance for large scenes
  if (sceneId === '3B' || sceneId === '4A') {
    simulation.cameraDistOverride = 28;
  } else {
    simulation.cameraDistOverride = null;
  }
  
  if (sceneId === '2A' || sceneId === '2B') {
    simulation.part2Kind = sceneId;
  } else if (sceneId === '2C') {
    simulation.part2Kind = '2C';
    setActiveSolverMethod('xpbd');
  } else if (sceneId === '2D') {
    simulation.part2Kind = '2D';
    setActiveSolverMethod('si');
    simulation.part2SI_PostStab = simulation.part2SI_PostStab || 'baumgarte';
  }
}

function updateBroadphaseButtonsState(sceneId) {
  const broadphaseButtons = document.querySelectorAll('#broad-btns .btn');
  if (broadphaseButtons.length === 0) return;
  broadphaseButtons.forEach(buttonElement => buttonElement.disabled = false);
  if (sceneId === '3A' || sceneId === '3B') {
    broadphaseButtons.forEach(buttonElement => {
      const isValidBroadphaseForScene = buttonElement.dataset.val === 'brute' || buttonElement.dataset.val === 'grid';
      buttonElement.disabled = !isValidBroadphaseForScene;
    });
  }
}

function updateUIVisibility() {
  const part = simulation.part;
  const sceneId = simulation.sceneId;
  const solver = simulation.solver;
  const postStab = simulation.postStab;
  const part2SI_PostStab = simulation.part2SI_PostStab || 'baumgarte';
  const isContactPart = (part === 3 || part === 4);
  const isPart2Spring = (sceneId === '2A' || sceneId === '2B');
  const isPart2Dist   = (sceneId === '2C' || sceneId === '2D');
  const isXPBDActive  = (sceneId === '2C') || (isContactPart && solver === 'xpbd');
  const isSIActive    = (sceneId === '2D') || (isContactPart && solver === 'si');
  const isSI_Baumgarte = sceneId === '2D' && part2SI_PostStab === 'baumgarte';
  const isSI_NLGS     = sceneId === '2D' && part2SI_PostStab === 'nlgs';
  const isSI_Soft     = sceneId === '2D' && part2SI_PostStab === 'soft';
  const softPostStab  = (sceneId === '2D' && postStab === 'soft');

  toggleElementVisibility('sec-solver',   isContactPart);
  toggleElementVisibility('sec-poststab', isSIActive);
  toggleElementVisibility('sec-broad',    isContactPart);

  toggleElementVisibility('row-k',    isPart2Spring || isSI_Soft || softPostStab);
  toggleElementVisibility('row-sd',   isPart2Spring || isSI_Soft || softPostStab);
  
  toggleElementVisibility('row-iter', isPart2Spring || isPart2Dist || isContactPart);
  toggleElementVisibility('row-comp', isXPBDActive);
  toggleElementVisibility('row-beta', isSI_Baumgarte || isSI_Soft || (isContactPart && solver === 'si'));
  toggleElementVisibility('row-rest', isSIActive || isContactPart);
  
  toggleElementVisibility('row-mus',  part === 4 && isXPBDActive);
  toggleElementVisibility('row-mud',  isContactPart);
  toggleElementVisibility('row-grav', isPart2Spring);
}

function toggleElementVisibility(elementId, isVisible) {
  const element = document.getElementById(elementId);
  if (element) element.style.display = isVisible ? '' : 'none';
}

function setActiveBroadphaseMethod(broadphaseMethod) {
  simulation.broadphase = broadphaseMethod;
  const broadphaseButtons = document.querySelectorAll('#broad-btns .btn');
  if (broadphaseButtons.length === 0) return;
  broadphaseButtons.forEach(buttonElement => {
    if (buttonElement.dataset.val === broadphaseMethod) buttonElement.classList.add('active');
    else buttonElement.classList.remove('active');
  });
}

function setActiveSolverMethod(solverMethod) {
  simulation.solver = solverMethod;
  const solverButtons = document.querySelectorAll('#solver-btns .btn');
  if (solverButtons.length === 0) return;
  solverButtons.forEach(buttonElement => {
    if (buttonElement.dataset.val === solverMethod) buttonElement.classList.add('active');
    else buttonElement.classList.remove('active');
  });
}

function updateSpringStiffness(stiffnessValue) {
  simulation.springK = stiffnessValue;
  for (const spring of simulation.springs) spring.k = stiffnessValue;
}
function updateSpringDamping(dampingValue) {
  simulation.springDamping = dampingValue;
  for (const spring of simulation.springs) spring.c = dampingValue;
}

function updateStatisticsPanel() {
  if (simulation.part === 1) {
    const body = simulation.bodies[0];
    const angularMomentum = computeAngularMomentum(body);
    const kineticEnergy = computeKineticEnergy(body);
    const initialAngularMomentum = simulation.L0 || [0,0,0];
    const angularVelocity = body.w || [0,0,0];
    const angularVelocityMagnitude = vectorLength(angularVelocity);
    
    setElementTextContent('m-L0x', initialAngularMomentum[0].toFixed(3));
    setElementTextContent('m-L0y', initialAngularMomentum[1].toFixed(3));
    setElementTextContent('m-L0z', initialAngularMomentum[2].toFixed(3));
    
    setElementTextContent('m-Lx', angularMomentum[0].toFixed(3));
    setElementTextContent('m-Ly', angularMomentum[1].toFixed(3));
    setElementTextContent('m-Lz', angularMomentum[2].toFixed(3));
    
    setElementTextContent('m-E',  kineticEnergy.toFixed(3));
    setElementTextContent('m-E0', simulation.E0 > 1e-9 ? simulation.E0.toFixed(3) : '—');
    setElementTextContent('m-omega', angularVelocityMagnitude.toFixed(3));
  } else if (simulation.part === 2) {
    setElementTextContent('m-k', simulation.springK ? simulation.springK.toFixed(0) : '200');
    setElementTextContent('m-sd', simulation.springDamping ? simulation.springDamping.toFixed(1) : '4.0');
    
    if (simulation.iterations) setElementTextContent('m-iter', String(simulation.iterations));
    if (simulation.compliance !== undefined) setElementTextContent('m-comp', simulation.compliance.toFixed(4));
    if (simulation.baumgarteBeta !== undefined) setElementTextContent('m-beta', simulation.baumgarteBeta.toFixed(2));
  }
}

function setupButtonGroup(groupId, onButtonClick) {
  const buttonGroup = document.getElementById(groupId);
  if (!buttonGroup) return;
  buttonGroup.querySelectorAll('.btn').forEach(button => {
    button.addEventListener('click', () => {
      if (button.disabled) return;
      buttonGroup.querySelectorAll('.btn').forEach(btn => btn.classList.remove('active'));
      button.classList.add('active');
      onButtonClick(button.dataset.val);
    });
  });
}

function setupSliderControl(sliderId, labelId, onValueChange, formatter) {
  const slider = document.getElementById(sliderId);
  if (!slider) return;
  slider.addEventListener('input', () => {
    const value = parseFloat(slider.value);
    const label = document.getElementById(labelId);
    if (label) label.textContent = formatter(value);
    onValueChange(value);
  });
}

function setElementTextContent(elementId, textContent) {
  const element = document.getElementById(elementId);
  if (element) element.textContent = textContent;
}
