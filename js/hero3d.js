// ============================================================
// Hero 3D scene — a field of "concrete columns" with waves
// rolling through it and IoT sensor nodes blinking on top.
// Click the grid to drop a glowing block into the simulation.
// ============================================================
import * as THREE from "three";

const canvas = document.getElementById("heroCanvas");
const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

const PALETTES = {
  dark: {
    bg: 0x0b0e14,
    columnBase: new THREE.Color(0x27304d),
    columnPeak: new THREE.Color(0xf5a623),
    node: 0x48c7f0,
    line: 0x48c7f0,
    pulse: 0xf5a623,
    dropped: 0xf5a623,
    ambient: 0.55,
  },
  light: {
    bg: 0xf5f3ec,
    columnBase: new THREE.Color(0xbfc8dd),
    columnPeak: new THREE.Color(0xd98a10),
    node: 0x0f7bab,
    line: 0x0f7bab,
    pulse: 0xc77800,
    dropped: 0xc77800,
    ambient: 0.9,
  },
};

function currentPalette() {
  return PALETTES[document.documentElement.dataset.theme] || PALETTES.dark;
}

let renderer;
try {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
} catch (e) {
  canvas.style.display = "none";
  throw e;
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(46, 1, 0.1, 100);
camera.position.set(0, 10.5, 17);
camera.lookAt(0, 0.5, 0);

let palette = currentPalette();
scene.fog = new THREE.Fog(palette.bg, 16, 38);

const ambient = new THREE.AmbientLight(0xffffff, palette.ambient);
const keyLight = new THREE.DirectionalLight(0xffffff, 1.1);
keyLight.position.set(6, 14, 8);
const rimLight = new THREE.DirectionalLight(0x88aaff, 0.35);
rimLight.position.set(-8, 6, -6);
scene.add(ambient, keyLight, rimLight);

const world = new THREE.Group();
world.rotation.y = 0.35;
scene.add(world);

// ---------- column field ----------
const GRID = 30;          // GRID x GRID columns
const STEP = 0.82;        // spacing
const HALF = ((GRID - 1) * STEP) / 2;
const COUNT = GRID * GRID;

const boxGeo = new THREE.BoxGeometry(0.58, 1, 0.58);
boxGeo.translate(0, 0.5, 0); // origin at the base so scaling grows upward
const boxMat = new THREE.MeshStandardMaterial({ roughness: 0.85, metalness: 0.1 });
const field = new THREE.InstancedMesh(boxGeo, boxMat, COUNT);
field.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
world.add(field);

const dummy = new THREE.Object3D();
const tmpColor = new THREE.Color();

function columnHeight(x, z, t) {
  return (
    1.35 +
    0.6 * Math.sin(0.5 * x + t * 0.9) * Math.cos(0.38 * z + t * 0.62) +
    0.35 * Math.sin(0.22 * (x + z) + t * 0.45)
  );
}

function updateField(t) {
  let i = 0;
  for (let gx = 0; gx < GRID; gx++) {
    for (let gz = 0; gz < GRID; gz++) {
      const x = gx * STEP - HALF;
      const z = gz * STEP - HALF;
      const h = columnHeight(x, z, t);
      dummy.position.set(x, 0, z);
      dummy.scale.set(1, h, 1);
      dummy.updateMatrix();
      field.setMatrixAt(i, dummy.matrix);
      // tint peaks with the accent colour
      const mix = THREE.MathUtils.clamp((h - 1.0) / 1.3, 0, 1);
      tmpColor.copy(palette.columnBase).lerp(palette.columnPeak, mix * mix);
      field.setColorAt(i, tmpColor);
      i++;
    }
  }
  field.instanceMatrix.needsUpdate = true;
  if (field.instanceColor) field.instanceColor.needsUpdate = true;
}

// ---------- sensor nodes + links ----------
const NODE_CELLS = [
  [4, 5], [9, 22], [7, 13], [14, 8], [15, 24], [20, 15], [24, 4], [25, 21], [21, 26], [3, 17],
];
const nodeGeo = new THREE.SphereGeometry(0.16, 16, 16);
const nodeMat = new THREE.MeshBasicMaterial({ color: palette.node });
const nodes = NODE_CELLS.map(([gx, gz]) => {
  const m = new THREE.Mesh(nodeGeo, nodeMat);
  m.userData = { x: gx * STEP - HALF, z: gz * STEP - HALF, phase: Math.random() * Math.PI * 2 };
  world.add(m);
  return m;
});

const linkPairs = [];
for (let i = 0; i < nodes.length; i++) linkPairs.push([i, (i + 1) % nodes.length]);
linkPairs.push([0, 5], [2, 7], [4, 9]);

const linkPositions = new Float32Array(linkPairs.length * 2 * 3);
const linkGeo = new THREE.BufferGeometry();
linkGeo.setAttribute("position", new THREE.BufferAttribute(linkPositions, 3));
const linkMat = new THREE.LineBasicMaterial({ color: palette.line, transparent: true, opacity: 0.35 });
const links = new THREE.LineSegments(linkGeo, linkMat);
world.add(links);

// a data packet travelling along a random link
const pulseGeo = new THREE.SphereGeometry(0.09, 10, 10);
const pulseMat = new THREE.MeshBasicMaterial({ color: palette.pulse });
const pulse = new THREE.Mesh(pulseGeo, pulseMat);
world.add(pulse);
let pulseLink = 0;
let pulseT = 0;

function updateNodes(t) {
  nodes.forEach((n, idx) => {
    const { x, z, phase } = n.userData;
    n.position.set(x, columnHeight(x, z, t) + 0.55, z);
    const s = 1 + 0.3 * Math.sin(t * 2.2 + phase);
    n.scale.setScalar(s);
  });
  linkPairs.forEach(([a, b], li) => {
    const pa = nodes[a].position, pb = nodes[b].position;
    linkPositions.set([pa.x, pa.y, pa.z, pb.x, pb.y, pb.z], li * 6);
  });
  linkGeo.attributes.position.needsUpdate = true;

  pulseT += 0.012;
  if (pulseT >= 1) {
    pulseT = 0;
    pulseLink = Math.floor(Math.random() * linkPairs.length);
  }
  const [a, b] = linkPairs[pulseLink];
  pulse.position.lerpVectors(nodes[a].position, nodes[b].position, pulseT);
}

// ---------- click-to-drop blocks ----------
const dropped = [];
const MAX_DROPPED = 24;
const raycaster = new THREE.Raycaster();
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const clickPoint = new THREE.Vector2();
const dropGeo = new THREE.BoxGeometry(0.55, 0.55, 0.55);

function spawnBlock(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  clickPoint.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  clickPoint.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(clickPoint, camera);
  const hit = new THREE.Vector3();
  if (!raycaster.ray.intersectPlane(groundPlane, hit)) return;
  hit.applyMatrix4(new THREE.Matrix4().copy(world.matrixWorld).invert());
  if (Math.abs(hit.x) > HALF + 1 || Math.abs(hit.z) > HALF + 1) return;

  const mat = new THREE.MeshStandardMaterial({
    color: palette.dropped, emissive: palette.dropped, emissiveIntensity: 0.35,
    roughness: 0.4, transparent: true,
  });
  const block = new THREE.Mesh(dropGeo, mat);
  block.position.set(hit.x, 9 + Math.random() * 2, hit.z);
  block.rotation.set(Math.random(), Math.random(), Math.random());
  block.userData = { vy: 0, spin: (Math.random() - 0.5) * 0.15, born: performance.now() };
  world.add(block);
  dropped.push(block);
  if (dropped.length > MAX_DROPPED) {
    const old = dropped.shift();
    world.remove(old);
    old.material.dispose();
  }
}

function updateDropped(t, dt) {
  const now = performance.now();
  for (const b of dropped) {
    const floor = columnHeight(b.position.x, b.position.z, t) + 0.28;
    if (b.position.y > floor) {
      b.userData.vy -= 22 * dt;               // gravity
      b.position.y += b.userData.vy * dt;
      b.rotation.x += b.userData.spin;
      b.rotation.z += b.userData.spin * 0.7;
      if (b.position.y <= floor) {
        b.position.y = floor;
        b.userData.vy = Math.abs(b.userData.vy) > 3 ? -b.userData.vy * 0.35 : 0; // bounce
      }
    } else {
      b.position.y = floor;                    // ride the wave
    }
    const age = (now - b.userData.born) / 1000;
    b.material.opacity = age > 20 ? Math.max(0, 1 - (age - 20) / 4) : 1;
  }
}

canvas.addEventListener("pointerdown", (e) => {
  if (!reducedMotion) spawnBlock(e.clientX, e.clientY);
});

// ---------- mouse parallax ----------
const mouse = { x: 0, y: 0 };
window.addEventListener("pointermove", (e) => {
  mouse.x = (e.clientX / window.innerWidth) * 2 - 1;
  mouse.y = (e.clientY / window.innerHeight) * 2 - 1;
});

// ---------- theme switching ----------
new MutationObserver(() => {
  palette = currentPalette();
  scene.fog.color.setHex(palette.bg);
  ambient.intensity = palette.ambient;
  nodeMat.color.setHex(palette.node);
  linkMat.color.setHex(palette.line);
  pulseMat.color.setHex(palette.pulse);
  if (reducedMotion) renderStatic();
}).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });

// ---------- resize ----------
function resize() {
  const w = canvas.clientWidth || window.innerWidth;
  const h = canvas.clientHeight || window.innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);
resize();

// ---------- render loop (paused when hero is off-screen) ----------
let heroVisible = true;
new IntersectionObserver(([entry]) => { heroVisible = entry.isIntersecting; })
  .observe(canvas);

const clock = new THREE.Clock();

function frame() {
  requestAnimationFrame(frame);
  if (!heroVisible || document.hidden) return;
  const dt = Math.min(clock.getDelta(), 0.05);
  const t = clock.elapsedTime;

  world.rotation.y = 0.35 + t * 0.02 + mouse.x * 0.06;
  camera.position.y = 10.5 - mouse.y * 0.7;
  camera.lookAt(0, 0.5, 0);

  updateField(t);
  updateNodes(t);
  updateDropped(t, dt);
  renderer.render(scene, camera);
}

function renderStatic() {
  updateField(1.5);
  updateNodes(1.5);
  renderer.render(scene, camera);
}

if (reducedMotion) {
  renderStatic();
  const hint = document.getElementById("heroHint");
  if (hint) hint.style.display = "none";
} else {
  frame();
}
