import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';

const $ = id => document.getElementById(id);
const esc = text => String(text).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
// The tree's steps and the stage view of each: navigate-only's, and the full mission's drives.
// The reasoner floats over every stage, so choosing a location opens the map it lands on.
const LEAF_STAGE = { 'Home arm': 'home', 'Choose location': 'navigate', 'Go there': 'navigate',
                     'Find target': 'navigate', 'Park': 'navigate' };
const ARM = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'];
const COLOR = { goal: 0xff3d8b, ok: 0x2f9e6e, fail: 0xd64545, chosen: 0x4353d6, other: 0xb9b5ad };

let state = null;       // the run on screen: the live one, or a saved one
let live = null;        // the server's current run
let viewing = null;     // id of the saved run on screen; null for the live one
let offline = true;     // until the server first answers
let runs = [];          // saved runs, newest first
let shown = null;
let follow = true;      // switch to each stage as it goes live, until the user picks one
let lastSwitch = 0;
let replayFrom = 0;     // when the stage on screen started playing its recording

// ---------- stages ----------
function show(stage) {
  shown = stage;
  lastSwitch = performance.now();
  replayFrom = lastSwitch;
  for (const button of $('tabs').children) button.classList.toggle('on', button.dataset.stage === stage);
  for (const id of ['home', 'navigate']) $(id).hidden = id !== stage;
}

function choose(stage) {
  follow = stage === liveStage();
  show(stage);
}

$('tabs').onclick = event => {
  const button = event.target.closest('button');
  if (button) choose(button.dataset.stage);
};
$('tree').onclick = event => {
  const node = event.target.closest('[data-stage]');
  if (node) choose(node.dataset.stage);
};

const runEvents = () => state?.events || [];
const plans = () => runEvents().filter(e => e.kind === 'plan');

function liveStage() {
  return state?.running ? LEAF_STAGE[state.leaf] ?? null : null;
}

// ---------- 3D viewers, both in the ROS frame: x forward, z up ----------
function viewer(element) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(devicePixelRatio);
  element.prepend(renderer.domElement);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 100);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xffffff, 0xd8d4cc, 2.4));
  const sun = new THREE.DirectionalLight(0xffffff, 1.5);
  sun.position.set(3, 6, 4);
  scene.add(sun);
  const grid = new THREE.GridHelper(20, 40, 0xdedbd4, 0xebe8e2);
  scene.add(grid);
  const world = new THREE.Group();
  world.rotation.x = -Math.PI / 2;   // three.js is y-up
  scene.add(world);
  return { element, renderer, scene, camera, controls, grid, world, size: '' };
}

function draw(view) {
  const { clientWidth: width, clientHeight: height } = view.element;
  if (view.size !== `${width}x${height}`) {
    view.size = `${width}x${height}`;
    view.renderer.setSize(width, height);
    view.camera.aspect = width / height;
    view.camera.updateProjectionMatrix();
  }
  view.controls.update();
  view.renderer.render(view.scene, view.camera);
}

// ---------- 1 · Home: the URDF, live while homing, replayed after ----------
const home = viewer($('home'));
home.camera.position.set(1.8, 1.3, 2.0);
home.controls.target.set(0, 0.6, 0);
let robot = null;
const loader = new URDFLoader();
loader.packages = name => '/package/' + name;
loader.load('/robot.urdf', model => { robot = model; home.world.add(model); });
const drawn = {};   // joint values on screen, eased toward their target
let manualHome = 0; // until when the view follows a Home button press live

$('home-button').onclick = async () => {
  const reply = await fetch('/home', { method: 'POST' });
  if (!reply.ok) {
    $('status').textContent = (await reply.json()).error;
    return;
  }
  manualHome = performance.now() + 6000;
};

function replayFrame(frames, now) {
  // Play a recording once from when its stage was shown, then hold the last frame.
  const t = (now - replayFrom) / 1000;
  return frames.find(f => f.t - frames[0].t >= t) || frames.at(-1);
}

function homeTarget(now) {
  const frames = state?.homing || [];
  if (now < manualHome) return [state?.joints || {}, '<b>Homing the arm</b>'];
  if (state?.running && state.leaf === 'Home arm') return [state.joints, '<b>Homing the arm</b> before anything else'];
  if (frames.length < 2) return [state?.joints || {}, 'Live pose'];
  const first = frames[0].joints, last = frames.at(-1).joints;
  let moved = 0;
  for (const name of ARM) moved = Math.max(moved, Math.abs((last[name] ?? 0) - (first[name] ?? 0)));
  if (moved < 0.02) return [last, '<b>Already at home</b>, nothing to move'];
  const span = frames.at(-1).t - frames[0].t;
  return [replayFrame(frames, now).joints, `<b>Arm homed</b> in ${span.toFixed(1)} s · click the tab to replay`];
}

function tickHome(now) {
  const [target, caption] = homeTarget(now);
  if ($('home-caption').innerHTML !== caption) $('home-caption').innerHTML = caption;
  if (!robot) return;
  for (const [name, value] of Object.entries(target)) {
    if (!robot.joints[name]) continue;
    drawn[name] = drawn[name] === undefined ? value : drawn[name] + (value - drawn[name]) * 0.3;
    robot.setJointValue(name, drawn[name]);
  }
}

// ---------- the reasoner, floating over every stage like the camera ----------
function renderReason() {
  const reasons = runEvents().filter(e => e.kind === 'reason');
  $('reasoner').hidden = !reasons.length;
  let html = '';
  if (reasons.length) {
    const all = plans();
    const same = (a, b) => b && a.furniture_id === b.furniture_id && a.object_id === b.object_id;
    const item = location => {
      let tag = '';
      if (same(location, all.at(-1)?.location)) tag = '<span class="tag">visiting now</span>';
      else if (all.slice(0, -1).some(p => same(location, p.location))) tag = '<span class="tag done">checked</span>';
      const where = location.source === 'llm'
        ? `${location.relation} <b>${esc(location.furniture)}</b>`
        : `${esc(location.label)} ${location.relation ?? 'near'} <b>${esc(location.furniture ?? 'where it was last seen')}</b>`;
      const why = location.reason ? `<span class="why">${esc(location.reason)}</span>` : '';
      return `<li>${where}${location.room ? ` · ${esc(location.room)}` : ''}${tag}${why}</li>`;
    };
    const target = `<b>${esc(reasons[0].target)}</b>`;
    html = '<h2>Reasoner</h2>';
    for (const r of reasons) {
      if (r.source === 'scene_graph' && r.locations.length) {
        html += `<p>${target} is already in the scene graph.</p><ol>${r.locations.map(item).join('')}</ol>`;
      } else if (r.source === 'scene_graph') {
        html += `<p>${target} is not in the scene graph yet.</p>`;
      } else if (r.source === 'no_key') {
        html += '<p class="empty">No DEEPSEEK_API_KEY, so only remembered places are searched.</p>';
      } else if (r.source === 'asking' && r === reasons.at(-1)) {
        html += `<p>Asking DeepSeek for the ${r.top_k} likeliest places<span class="dots"><i></i><i></i><i></i></span></p>`;
      } else if (r.source === 'llm') {
        html += `<p>DeepSeek's top ${r.locations.length}, visited in its order:</p><ol>${r.locations.map(item).join('')}</ol>`;
      } else if (r.source === 'cache') {
        html += `<p>DeepSeek's top ${r.locations.length} from an earlier query, visited in its order:</p><ol>${r.locations.map(item).join('')}</ol>`;
      }
    }
  }
  if ($('reasoner').innerHTML !== html) $('reasoner').innerHTML = html;
}

// ---------- 2 · Navigate: the place, its neighbours, where the base parks ----------
const nav = viewer($('navigate'));
const scenery = new THREE.Group();
nav.world.add(scenery);
let marker = null;     // the bobbing arrow over the current goal
let drawnEvents = -1;
let framedPlans = 0;
let framedRobot = false;
let navRobot = null;
loader.load('/robot.urdf', model => { navRobot = model; model.visible = false; nav.world.add(model); });
let drawnBase = null;  // base pose on screen, eased toward the target
const waypoints = new THREE.Group();
nav.world.add(waypoints);
let drawnRoute = '';
const dotGeometry = new THREE.CircleGeometry(0.04, 16);
const dotMaterial = new THREE.MeshBasicMaterial({ color: COLOR.goal, transparent: true, opacity: 0.75 });

function thin(points) {
  // One point every 0.25 m, as the server thins Nav2's plan.
  const kept = [];
  for (const p of points) {
    if (!kept.length || Math.hypot(p[0] - kept.at(-1)[0], p[1] - kept.at(-1)[1]) >= 0.25) kept.push(p);
  }
  return kept;
}

function renderRoute() {
  // Nav2's plan while driving (it replans once a second); the driven track once the run is over.
  const route = (state?.running ? state.path : thin((state?.track || []).map(f => f.pose))) ?? [];
  const key = JSON.stringify(route);
  if (key === drawnRoute) return;
  drawnRoute = key;
  waypoints.clear();
  for (const [x, y] of route) {
    const dot = new THREE.Mesh(dotGeometry, dotMaterial);
    dot.position.set(x, y, 0.01);
    waypoints.add(dot);
  }
}

function frameNav(x, y, span) {
  const distance = Math.max(3, span);
  nav.controls.target.set(x, 0.4, -y);
  nav.camera.position.set(x + 0.9 * distance, distance, -y + 0.9 * distance);
  nav.grid.position.set(Math.round(x), 0, -Math.round(y));
}

function tickNav(now) {
  // Live while a run is going; afterwards the drive it recorded, on a loop.
  const track = state?.track || [];
  const pose = !state?.running && track.length > 1 ? replayFrame(track, now).pose : state?.pose;
  if (!navRobot) return;
  navRobot.visible = !!pose;
  if (!pose) return;
  const [x, y, yaw] = pose;
  // Jump rather than glide across the map, as when the replay starts over.
  if (!drawnBase || Math.hypot(x - drawnBase.x, y - drawnBase.y) > 1) drawnBase = { x, y, yaw };
  drawnBase.x += (x - drawnBase.x) * 0.25;
  drawnBase.y += (y - drawnBase.y) * 0.25;
  drawnBase.yaw += Math.atan2(Math.sin(yaw - drawnBase.yaw), Math.cos(yaw - drawnBase.yaw)) * 0.25;
  navRobot.position.set(drawnBase.x, drawnBase.y, 0);
  navRobot.rotation.z = drawnBase.yaw;
  for (const [name, value] of Object.entries(state.joints)) {
    if (navRobot.joints[name]) navRobot.setJointValue(name, value);
  }
}

function box(centre, size, yaw, color, opacity) {
  const geometry = new THREE.BoxGeometry(...size);
  const group = new THREE.Group();
  group.add(new THREE.Mesh(geometry, new THREE.MeshStandardMaterial(
    { color, transparent: true, opacity, depthWrite: false })));
  group.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry), new THREE.LineBasicMaterial(
    { color, transparent: true, opacity: Math.min(1, opacity * 4) })));
  group.position.set(...centre);
  group.rotation.z = yaw;
  return group;
}

function base([x, y, yaw], radius, color, opacity) {
  // The base footprint on the floor, with a chevron for its heading.
  const material = new THREE.MeshBasicMaterial({ color, transparent: true, opacity, side: THREE.DoubleSide });
  const group = new THREE.Group();
  group.add(new THREE.Mesh(new THREE.RingGeometry(radius - 0.04, radius, 48), material));
  const chevron = new THREE.Shape([[0.2, 0], [-0.08, 0.11], [-0.02, 0], [-0.08, -0.11]].map(p => new THREE.Vector2(...p)));
  group.add(new THREE.Mesh(new THREE.ShapeGeometry(chevron), material));
  group.position.set(x, y, 0.005);
  group.rotation.z = yaw;
  return group;
}

function arrow([x, y], color) {
  const cone = new THREE.Mesh(new THREE.ConeGeometry(0.1, 0.26, 4),
    new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.35 }));
  cone.rotation.x = -Math.PI / 2;   // point down at the floor
  const group = new THREE.Group();
  group.add(cone);
  group.position.set(x, y, 0.95);
  return group;
}

function renderNav() {
  renderRoute();
  const events = runEvents();
  const all = plans();
  const plan = all.at(-1);
  if (!plan) {
    $('nav-caption').innerHTML = 'No place chosen yet';
    scenery.clear();
    marker = null;
    // A new run starts over: its first place is drawn and framed afresh.
    drawnEvents = -1;
    framedPlans = 0;
    if (state?.pose && !framedRobot) {
      framedRobot = true;
      frameNav(state.pose[0], state.pose[1], 3);
    }
    return;
  }
  const drives = events.slice(events.indexOf(plan)).filter(e => e.kind === 'drive');
  const location = plan.location;
  const name = esc(location.furniture ?? location.label);
  const [cx, cy] = location.centroid;
  if (events.length !== drawnEvents) {
    drawnEvents = events.length;
    scenery.clear();
    marker = null;
    for (const piece of plan.furniture) {
      const chosen = piece.id === location.furniture_id;
      scenery.add(box(piece.centre, piece.dimensions, piece.yaw, chosen ? COLOR.chosen : COLOR.other, chosen ? 0.18 : 0.06));
    }
    if (location.object_id !== null) scenery.add(box(location.centroid, [0.08, 0.08, 0.12], 0, COLOR.goal, 0.9));
    for (const pose of plan.poses) scenery.add(base(pose, plan.robot_radius, COLOR.other, 0.35));
    drives.forEach((drive, index) => {
      const color = drive.reached === null ? COLOR.goal : drive.reached ? COLOR.ok : COLOR.fail;
      const current = index === drives.length - 1;
      scenery.add(base(drive.pose, plan.robot_radius, color, current ? 1 : 0.4));
      if (current) scenery.add(marker = arrow(drive.pose, color));
    });
  }
  if (all.length !== framedPlans) {
    // Frame each new place once, robot included; after that the view is the user's to orbit.
    framedPlans = all.length;
    const [rx, ry] = state.pose ?? [cx, cy];
    frameNav((cx + rx) / 2, (cy + ry) / 2, Math.hypot(cx - rx, cy - ry) + 1.5);
  }
  const last = drives.at(-1);
  const count = plan.poses.length;
  let caption = `${count} places to park around <b>${name}</b>`;
  if (last) {
    const number = plan.poses.findIndex(p => Math.hypot(p[0] - last.pose[0], p[1] - last.pose[1]) < 1e-3) + 1;
    if (last.reached === null) caption = `<b>Driving</b> to view ${number} of ${count} at <b>${name}</b>`;
    else if (last.reached) caption = `<b>Parked</b> at view ${number} of ${count}, facing <b>${name}</b>`;
    else caption = `<b>Could not reach</b> view ${number} of ${count}, trying the next`;
  }
  if ($('nav-caption').innerHTML !== caption) $('nav-caption').innerHTML = caption;
}

// ---------- tree, status, tabs ----------
function treeHtml(node) {
  const glyph = { Sequence: '→', Selector: '?', Retry: '↻', Parallel: '⇉' }[node.type] ?? '';
  const stage = LEAF_STAGE[node.name] ? ` data-stage="${LEAF_STAGE[node.name]}"` : '';
  const children = node.children.length ? `<ul>${node.children.map(treeHtml).join('')}</ul>` : '';
  return `<li><div class="node"${stage}><span class="dot ${node.status}"></span>${esc(node.name)}` +
         `<span class="type">${glyph}</span></div>${children}</li>`;
}

function outcome(run) {
  // The step that failed says more than the exit code: startup errors and drives both end nonzero.
  if (run.exit === 0) return 'succeeded';
  if (run.exit === 130) return 'stopped';
  return `failed at ${run.leaf}`;
}

function render() {
  const stage = liveStage();
  if (!shown) show('home');
  if (follow && stage && stage !== shown && performance.now() - lastSwitch > 2500) show(stage);

  const html = state?.tree ? `<ul>${treeHtml(state.tree)}</ul>` : '<p class="empty">Appears when a query runs.</p>';
  if ($('tree').innerHTML !== html) $('tree').innerHTML = html;

  const cached = { home: (state?.homing.length ?? 0) > 1,
                   navigate: plans().length > 0 };
  for (const button of $('tabs').children) {
    const name = button.dataset.stage;
    button.firstChild.className = 'dot' + (name === stage ? ' live' : cached[name] ? ' cached' : '');
  }

  let text = 'idle', dot = '';
  if (state?.running) [text, dot] = [`${state.target} · ${state.leaf ?? 'starting'}`, 'RUNNING'];
  else if (state?.exit != null) [text, dot] = [`${state.target} · ${outcome(state)}`, state.exit === 0 ? 'SUCCESS' : 'FAILURE'];
  // The page keeps what it last had, so a run can still be inspected with the sim gone.
  if (offline && !viewing) [text, dot] = ['server offline', 'FAILURE'];
  $('status').textContent = text;
  $('status-dot').className = 'dot ' + dot;
  $('go').textContent = live?.running ? 'Stop' : 'Fetch';
  $('go').classList.toggle('stop', !!live?.running);
  $('go').disabled = offline;
  $('target').disabled = !!live?.running;
  $('home-button').disabled = offline || !!viewing || live.running;
  const log = (viewing ? `<a href="/runs/${viewing}.log" target="_blank">full log</a> · ` : '') + (state?.log ? esc(state.log) : '&nbsp;');
  if ($('log').innerHTML !== log) $('log').innerHTML = log;

  renderReason();
  renderNav();
}

function when(id) {
  // Run ids are start times, YYYYmmdd-HHMMSS.
  return `${id.slice(4, 6)}/${id.slice(6, 8)} ${id.slice(9, 11)}:${id.slice(11, 13)}`;
}

function renderRuns() {
  let html = `<li><div class="node${viewing ? '' : ' on'}" data-run=""><span class="dot"></span>Live</div></li>`;
  for (const run of runs) {
    html += `<li><div class="node${viewing === run.id ? ' on' : ''}" data-run="${esc(run.id)}">` +
            `<span class="dot ${run.exit === 0 ? 'SUCCESS' : 'FAILURE'}"></span>${esc(run.target)}` +
            `<span class="type">${when(run.id)}</span></div></li>`;
  }
  $('runs').innerHTML = html;
}

async function loadRuns() {
  runs = await (await fetch('/runs')).json();
  renderRuns();
}

async function view(id) {
  // A saved run, or the live one for null; its scene is drawn and framed afresh.
  viewing = id;
  state = id ? await (await fetch(`/runs/${id}.json`)).json() : live;
  drawnEvents = -1;
  framedPlans = 0;
  follow = !id;
  show('home');
  renderRuns();
  render();
}

$('runs').onclick = event => {
  const node = event.target.closest('[data-run]');
  if (node) view(node.dataset.run || null);
};

$('query').onsubmit = async event => {
  event.preventDefault();
  if (live?.running) {
    await fetch('/stop', { method: 'POST' });
    return;
  }
  const target = $('target').value.trim();
  if (!target) return;
  const reply = await fetch('/run', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                      body: JSON.stringify({ target }) });
  if (!reply.ok) {
    $('status').textContent = (await reply.json()).error;
    return;
  }
  view(null);
};

async function poll() {
  try {
    const next = await (await fetch('/state')).json();
    // A restarted server may serve newer page code: load it rather than keep running the old.
    if (live && next.boot !== live.boot) location.reload();
    // The list changes when the server comes back or a run ends and is saved.
    if (offline || (live?.running && !next.running)) loadRuns();
    live = next;
    offline = false;
  } catch {
    offline = true;
  }
  if (!viewing) state = live;
  render();
  setTimeout(poll, 250);
}

function frame(now) {
  requestAnimationFrame(frame);
  tickHome(now);
  tickNav(now);
  if (marker) {
    marker.position.z = 0.95 + 0.07 * Math.sin(now / 330);
    marker.rotation.z = now / 800;
  }
  if (shown === 'home') draw(home);
  if (shown === 'navigate') draw(nav);
}

function pollCamera() {
  // The head camera, frame by frame; hidden while no image arrives.
  const next = new Image();
  next.onload = () => {
    $('camera').src = next.src;
    $('camera').hidden = !!viewing;   // the camera is live; a saved run has none
    setTimeout(pollCamera, 250);
  };
  next.onerror = () => {
    $('camera').hidden = true;
    setTimeout(pollCamera, 1000);
  };
  next.src = '/camera.jpg?' + Date.now();
}

poll();
pollCamera();
requestAnimationFrame(frame);
