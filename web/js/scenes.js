// Escenas 3D (three.js): dojo con el robot y mano del operador.
//
// Marco ROS (x adelante, y izquierda, z arriba) -> three.js (x derecha,
// y arriba, z hacia la camara):  t = (-y, z, -x). Es una rotacion propia,
// asi que la parte vectorial de los quaternions se transforma igual.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
import { RoundedBoxGeometry } from 'three/addons/RoundedBoxGeometry.js';
import { theme } from './ui.js';

let DOJO_R = 2.5;   // m (5 m de diametro); se actualiza con setRadius()
const ROBOT_PINK = 0xff4fa3;
export const ARRIVE_R = 0.15;   // m: radio de la zona de llegada de un destino
const v3 = (x, y, z = 0) => new THREE.Vector3(-y, z, -x);
const quatRos = (x, y, z, w) => new THREE.Quaternion(-y, z, -x, w);

function quatFromRPY(r, p, y) {
  const cr = Math.cos(r / 2), sr = Math.sin(r / 2);
  const cp = Math.cos(p / 2), sp = Math.sin(p / 2);
  const cy = Math.cos(y / 2), sy = Math.sin(y / 2);
  return [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  ];
}

class BaseScene {
  constructor(container) {
    this.el = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.append(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 80);
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    const loop = () => {
      requestAnimationFrame(loop);
      if (this.el.offsetParent !== null) this.render();
    };
    requestAnimationFrame(loop);
  }

  resize() {
    const w = this.el.clientWidth || 1, h = this.el.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  lights() {
    const hemi = new THREE.HemisphereLight(0xdfe6ff, 0x1a1c26, 1.1);
    this.scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(0.6, 1.4, 0.8);
    this.key = key;
    this.scene.add(key.target);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    Object.assign(key.shadow.camera, { left: -0.7, right: 0.7, top: 0.7, bottom: -0.7, near: 0.1, far: 4 });
    key.shadow.bias = -0.0004;
    key.shadow.radius = 4;
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x8b7bff, 0.7);
    rim.position.set(-1, 0.6, -1);
    this.scene.add(rim);
  }
}

// ======================================================================
// Dojo
// ======================================================================
export class DojoScene extends BaseScene {
  constructor(container) {
    super(container);
    this.camMode = 'persp';
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 0, 0);
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.minDistance = 0.25;
    this.controls.maxDistance = 30;
    this.perspView();
    this.lights();
    this.buildWorld();
    this.robot = this.buildRobot();
    this.scene.add(this.robot);

    this.ghosts = {};
    this.trails = {};
    this.shown = { ekf: true, ukf: false, odom: false };   // rastros visibles (los demas se siguen grabando)
    for (const k of ['ekf', 'ukf', 'odom']) {
      this.ghosts[k] = this.buildGhost();
      this.scene.add(this.ghosts[k]);
      this.trails[k] = this.buildTrail();
      this.scene.add(this.trails[k].line);
    }
    this.pins = new THREE.Group();
    this.scene.add(this.pins);
    this.beacons = new Map();
    this.bursts = [];
    this.guide = new THREE.Line(
      new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3)),
      new THREE.LineDashedMaterial({ dashSize: 0.06, gapSize: 0.04, transparent: true, opacity: 0.9 }),
    );
    this.guide.visible = false;
    this.scene.add(this.guide);
    this.cursor = new THREE.Mesh(new THREE.RingGeometry(ARRIVE_R - 0.015, ARRIVE_R, 64),
      new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.8, depthWrite: false, side: THREE.DoubleSide }));
    this.cursor.rotation.x = -Math.PI / 2;
    this.cursor.visible = false;
    this.scene.add(this.cursor);
    this.placing = false;
    this.applyTheme();
  }

  // Vista 3D inicial: cerca del robot (~1.9 m) pero viendo su entorno,
  // no todo el dojo de 5 m (ahi el carro se veia diminuto)
  perspView() {
    const c = this.robot ? this.robot.position : new THREE.Vector3();
    const d = Math.min(Math.max(DOJO_R, 0.385) / 0.385, 2.4);
    this.camera.position.set(c.x + 0.55 * d, 0.62 * d, c.z + 0.75 * d);
    this.controls.target.set(c.x, 0, c.z);
  }

  gridSize() {
    const size = Math.ceil((2 * DOJO_R + 1) / 0.5) * 0.5;
    const step = DOJO_R > 1 ? 0.25 : 0.1;
    return [size, Math.round(size / step)];
  }

  setRadius(r) {
    if (!(r > 0) || Math.abs(r - DOJO_R) < 1e-6 && this.dojo) return;
    DOJO_R = r;
    if (this.dojo) {
      this.scene.remove(this.dojo);
      this.dojo.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
    }
    this.buildDojo();
    this.applyTheme();
    if (this.camMode === 'persp') this.perspView();
    if (this.camMode === 'top') this.setCamMode('top');
  }

  buildWorld() {
    // Piso con rejilla
    this.floorMat = new THREE.ShadowMaterial({ opacity: 0.28 });
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), this.floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.025;
    floor.receiveShadow = true;
    this.scene.add(floor);
    this.grid = new THREE.GridHelper(...this.gridSize());
    this.scene.add(this.grid);   // colores en applyTheme()
    this.buildDojo();
  }

  buildDojo() {
    // Dojo: disco negro con borde blanco (tawara) y lineas de salida
    const dojo = new THREE.Group();
    const edgeW = DOJO_R > 1 ? 0.05 : 0.025;
    const base = new THREE.Mesh(
      new THREE.CylinderGeometry(DOJO_R, DOJO_R + 0.004, 0.025, 160),
      new THREE.MeshStandardMaterial({ color: 0x08080a, roughness: 0.7, metalness: 0.0 }),
    );
    base.position.y = -0.0125;
    base.receiveShadow = true;
    base.castShadow = true;
    dojo.add(base);
    const edge = new THREE.Mesh(
      new THREE.RingGeometry(DOJO_R - edgeW, DOJO_R, 160),
      new THREE.MeshStandardMaterial({ color: 0xf4f4f6, roughness: 0.4 }),
    );
    edge.rotation.x = -Math.PI / 2;
    edge.position.y = 0.0006;
    edge.receiveShadow = true;
    dojo.add(edge);
    for (const x of [0.05, -0.05]) {
      const line = new THREE.Mesh(new THREE.PlaneGeometry(0.10, 0.012), new THREE.MeshStandardMaterial({ color: 0x8a5a33, roughness: 0.6 }));
      line.rotation.x = -Math.PI / 2;
      line.position.copy(v3(x, 0, 0.0007));
      dojo.add(line);
    }
    // Marca del eje x (adelante al arrancar)
    const tick = new THREE.Mesh(new THREE.PlaneGeometry(0.006, 0.05), new THREE.MeshBasicMaterial({ color: 0x7c6cff }));
    tick.rotation.x = -Math.PI / 2;
    tick.position.copy(v3(DOJO_R - edgeW - 0.04, 0, 0.0008));
    dojo.add(tick);
    this.dojo = dojo;
    this.scene.add(dojo);
  }

  buildRobot() {
    const g = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: ROBOT_PINK, roughness: 0.38, metalness: 0.25 });
    // Medidas reales: ruedas de 6.7 cm separadas 15.3 cm, largo total 14.5 cm
    const body = new THREE.Mesh(new RoundedBoxGeometry(0.128, 0.034, 0.118, 4, 0.008), bodyMat);
    body.position.y = 0.03;
    body.castShadow = true;
    g.add(body);

    this.accentMat = new THREE.MeshStandardMaterial({ color: 0x7c6cff, emissive: 0x7c6cff, emissiveIntensity: 0.9, roughness: 0.3 });
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(0.13, 0.004, 0.012), this.accentMat);
    stripe.position.set(0, 0.035, 0.035);
    g.add(stripe);

    // Rampa frontal (marco ROS +x = three -z)
    const scoop = new THREE.Mesh(new THREE.BoxGeometry(0.13, 0.004, 0.034), new THREE.MeshStandardMaterial({ color: 0xc9ccd6, metalness: 0.9, roughness: 0.25 }));
    scoop.position.set(0, 0.012, -0.07);
    scoop.rotation.x = -0.38;
    scoop.castShadow = true;
    g.add(scoop);

    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x111216, roughness: 0.9 });
    const hubMat = new THREE.MeshStandardMaterial({ color: 0x9aa0b4, metalness: 0.8, roughness: 0.3 });
    for (const side of [-1, 1]) {
      const w = new THREE.Mesh(new THREE.CylinderGeometry(0.0335, 0.0335, 0.016, 40), wheelMat);
      w.rotation.z = Math.PI / 2;
      w.position.set(side * 0.0765, 0.0335 - 0.02, 0.012);
      w.castShadow = true;
      g.add(w);
      const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.016, 20), hubMat);
      hub.rotation.z = Math.PI / 2;
      hub.position.copy(w.position);
      g.add(hub);
    }
    // Flecha de frente
    const arrow = new THREE.Mesh(new THREE.ConeGeometry(0.012, 0.026, 3), this.accentMat);
    arrow.rotation.x = -Math.PI / 2;
    arrow.position.set(0, 0.05, -0.018);
    g.add(arrow);
    // Halo rosado en el piso: en un dojo de 5 m el robot (9 cm) se pierde de lejos
    const halo = new THREE.Mesh(
      new THREE.RingGeometry(0.13, 0.155, 64),
      new THREE.MeshBasicMaterial({ color: ROBOT_PINK, transparent: true, opacity: 0.55, side: THREE.DoubleSide, depthWrite: false }),
    );
    halo.rotation.x = -Math.PI / 2;
    halo.position.y = 0.002;
    g.add(halo);
    return g;
  }

  buildGhost() {
    const g = new THREE.Group();
    const mat = new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.9, depthWrite: false });
    const disc = new THREE.Mesh(new THREE.RingGeometry(0.018, 0.024, 40), mat);
    disc.rotation.x = -Math.PI / 2;
    disc.position.y = 0.002;
    g.add(disc);
    const tip = new THREE.Mesh(new THREE.ConeGeometry(0.012, 0.03, 16), mat);
    tip.rotation.x = -Math.PI / 2;
    tip.position.set(0, 0.004, -0.045);
    g.add(tip);
    g.userData.mat = mat;
    return g;
  }

  buildTrail(n = 6000) {
    const geo = new THREE.BufferGeometry();
    const pos = new Float32Array(n * 3);
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setDrawRange(0, 0);
    const mat = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.85 });
    return { line: new THREE.Line(geo, mat), pos, n, count: 0, last: null };
  }

  pushTrail(t, x, y) {
    if (t.last && Math.hypot(x - t.last[0], y - t.last[1]) < 0.004) return;
    // Un salto de mas de 0.5 m entre dos muestras es un reinicio del origen,
    // no movimiento: el rastro empieza de nuevo (si no, quedaba una linea
    // desde la posicion vieja hasta el origen)
    if (t.last && Math.hypot(x - t.last[0], y - t.last[1]) > 0.5) { t.count = 0; t.line.geometry.setDrawRange(0, 0); }
    t.last = [x, y];
    if (t.count >= t.n) { t.pos.copyWithin(0, 3); t.count = t.n - 1; }
    const p = v3(x, y, 0.003);
    t.pos.set([p.x, p.y, p.z], t.count * 3);
    t.count++;
    t.line.geometry.setDrawRange(0, t.count);
    t.line.geometry.attributes.position.needsUpdate = true;
  }

  // Que estimaciones se dibujan en el dojo: {ekf, ukf, odom} -> bool
  setShown(shown) { this.shown = { ...shown }; }

  clearTrails() {
    for (const t of Object.values(this.trails)) { t.count = 0; t.last = null; t.line.geometry.setDrawRange(0, 0); }
  }

  setCamMode(m) {
    this.camMode = m;
    this.controls.enabled = m !== 'follow';
    if (m === 'top') { this.camera.position.set(0, Math.max(1.25, DOJO_R * 3.1), 0.0001); this.controls.target.set(0, 0, 0); }
    if (m === 'persp') this.perspView();
  }

  applyTheme() {
    const th = theme();
    const col = (c) => new THREE.Color(c);
    this.ghosts.ekf.userData.mat.color = col(th.ekf);
    this.ghosts.ukf.userData.mat.color = col(th.ukf);
    this.ghosts.odom.userData.mat.color = col(th.odom);
    this.trails.ekf.line.material.color = col(th.ekf);
    this.trails.ukf.line.material.color = col(th.ukf);
    this.trails.odom.line.material.color = col(th.odom);
    this.accentMat.color = this.accentMat.emissive = col(th.accent);
    this.floorMat.opacity = th.dark ? 0.35 : 0.16;
    // GridHelper usa colores por vertice: se recrea con los del tema
    this.scene.remove(this.grid);
    this.grid.geometry.dispose();
    this.grid = new THREE.GridHelper(...this.gridSize(), th.dark ? 0x3a3c4a : 0xb4b9c9, th.dark ? 0x23252f : 0xd9dce6);
    this.grid.position.y = -0.0249;
    this.scene.add(this.grid);
    this.guide.material.color = col(th.exp);
    this.cursor.material.color = col(th.exp);
    if (this.beacons) for (const [k, b] of this.beacons) this.setBeaconState(k, b.userData.state);
  }

  // ------------------------------------------------------------------
  // Destinos: baliza animada por punto (zona de llegada, ondas, haz de
  // luz, gema flotante y etiqueta), linea guia al destino activo y
  // explosion de particulas al llegar.
  // targets: [{key, x, y, label}]
  // ------------------------------------------------------------------
  setPins(points) { this.setTargets(points.map((p, i) => ({ key: `${p.x},${p.y}`, label: p.label || `P${i + 1}`, ...p }))); }

  setTargets(targets) {
    const keep = new Set(targets.map((t) => t.key));
    for (const [k, b] of this.beacons) {
      if (!keep.has(k)) { this.pins.remove(b); this.disposeTree(b); this.beacons.delete(k); }
    }
    for (const t of targets) {
      let b = this.beacons.get(t.key);
      if (!b || b.userData.label !== t.label) {
        if (b) { this.pins.remove(b); this.disposeTree(b); }
        b = this.buildBeacon(t.label);
        this.beacons.set(t.key, b);
        this.pins.add(b);
      }
      b.position.copy(v3(t.x, t.y));
      b.userData.ros = { x: t.x, y: t.y };
      this.setBeaconState(t.key, b.userData.state || 'pending');
    }
  }

  disposeTree(o) {
    o.traverse((n) => {
      if (n.geometry) n.geometry.dispose();
      if (n.material) { if (n.material.map) n.material.map.dispose(); n.material.dispose(); }
    });
  }

  labelSprite(text) {
    const c = document.createElement('canvas');
    const g = c.getContext('2d');
    const font = '600 64px Inter, system-ui, sans-serif';
    g.font = font;
    const w = Math.ceil(g.measureText(text).width) + 72, h = 104;
    c.width = w; c.height = h;
    g.font = font;
    g.fillStyle = 'rgba(12,13,20,0.78)';
    const r = 30;
    g.beginPath(); g.roundRect(4, 4, w - 8, h - 8, r); g.fill();
    g.lineWidth = 5; g.strokeStyle = 'rgba(255,255,255,0.85)'; g.stroke();
    g.fillStyle = '#ffffff'; g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(text, w / 2, h / 2 + 3);
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = 4;
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthWrite: false, transparent: true }));
    const k = 0.0016;
    sp.scale.set(w * k, h * k, 1);
    sp.renderOrder = 10;
    return sp;
  }

  buildBeacon(label) {
    const g = new THREE.Group();
    const add = (m) => { g.add(m); return m; };
    const basic = (o = {}) => new THREE.MeshBasicMaterial({ transparent: true, depthWrite: false, side: THREE.DoubleSide, ...o });
    // Zona de llegada (radio = ARRIVE_R)
    const zone = add(new THREE.Mesh(new THREE.CircleGeometry(ARRIVE_R, 64), basic({ opacity: 0.18 })));
    zone.rotation.x = -Math.PI / 2; zone.position.y = 0.0016;
    const rim = add(new THREE.Mesh(new THREE.RingGeometry(ARRIVE_R - 0.012, ARRIVE_R, 96), basic({ opacity: 0.95 })));
    rim.rotation.x = -Math.PI / 2; rim.position.y = 0.0018;
    // Mira en cruz
    const cross = new THREE.Group();
    for (const a of [0, Math.PI / 2]) {
      const bar = new THREE.Mesh(new THREE.PlaneGeometry(0.11, 0.008), basic({ opacity: 0.9 }));
      bar.rotation.set(-Math.PI / 2, 0, a);
      cross.add(bar);
    }
    cross.position.y = 0.002;
    add(cross);
    // Ondas que salen del centro
    const waves = [0, 1, 2].map(() => {
      const w = add(new THREE.Mesh(new THREE.RingGeometry(0.93, 1, 96), basic({ opacity: 0.5 })));
      w.rotation.x = -Math.PI / 2; w.position.y = 0.0017;
      return w;
    });
    // Haz de luz
    const beam = add(new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.05, 0.62, 32, 1, true),
      basic({ opacity: 0.22, blending: THREE.AdditiveBlending })));
    beam.position.y = 0.31;
    // Gema flotante
    const gemMat = new THREE.MeshStandardMaterial({ roughness: 0.15, metalness: 0.2, emissiveIntensity: 1.1, flatShading: true });
    const gem = add(new THREE.Mesh(new THREE.OctahedronGeometry(0.045, 0), gemMat));
    gem.scale.set(1, 1.45, 1);
    gem.position.y = 0.5;
    gem.castShadow = true;
    // Etiqueta
    const tag = add(this.labelSprite(label));
    tag.position.y = 0.66;
    g.userData = { label, zone, rim, cross, waves, beam, gem, gemMat, tag, state: 'pending', phase: Math.random() * 6 };
    return g;
  }

  setBeaconState(key, state) {
    const b = this.beacons.get(key);
    if (!b) return;
    const u = b.userData;
    u.state = state;
    const th = theme();
    const col = new THREE.Color(state === 'reached' ? th.good : state === 'active' ? th.exp : th.text3);
    for (const m of [u.zone, u.rim, u.beam, ...u.waves, ...u.cross.children]) m.material.color = col;
    u.gemMat.color = col; u.gemMat.emissive = col;
    u.gemMat.emissiveIntensity = state === 'pending' ? 0.35 : 1.1;
    u.zone.material.opacity = state === 'active' ? 0.22 : 0.12;
    u.beam.visible = state !== 'pending';
  }

  // Linea guia punteada del robot al destino activo (null = ocultar)
  setGuide(t) {
    if (!t) { this.guide.visible = false; return; }
    const a = this.robot.position, b = v3(t.x, t.y);
    const arr = this.guide.geometry.attributes.position.array;
    arr.set([a.x, 0.004, a.z, b.x, 0.004, b.z]);
    this.guide.geometry.attributes.position.needsUpdate = true;
    this.guide.computeLineDistances();
    this.guide.visible = true;
  }

  // Explosion de particulas y onda de choque al llegar
  celebrate(x, y) {
    const th = theme();
    const n = 260;
    const pos = new Float32Array(n * 3), vel = [], col = new Float32Array(n * 3);
    const palette = [th.good, th.exp, th.accent, th.ekf, '#fbbf24'].map((c) => new THREE.Color(c));
    const o = v3(x, y, 0.05);
    for (let i = 0; i < n; i++) {
      pos.set([o.x, o.y, o.z], i * 3);
      const a = Math.random() * Math.PI * 2, up = 0.6 + Math.random() * 1.6, sp = 0.4 + Math.random() * 1.1;
      vel.push(new THREE.Vector3(Math.cos(a) * sp, up, Math.sin(a) * sp));
      const c = palette[i % palette.length];
      col.set([c.r, c.g, c.b], i * 3);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    const pts = new THREE.Points(geo, new THREE.PointsMaterial({ size: 0.035, vertexColors: true, transparent: true, depthWrite: false }));
    const shock = new THREE.Mesh(new THREE.RingGeometry(0.9, 1, 96),
      new THREE.MeshBasicMaterial({ color: new THREE.Color(th.good), transparent: true, depthWrite: false, side: THREE.DoubleSide }));
    shock.rotation.x = -Math.PI / 2;
    shock.position.set(o.x, 0.003, o.z);
    this.scene.add(pts, shock);
    this.bursts.push({ pts, vel, shock, t0: performance.now() / 1000 });
  }

  // Colocar destinos con un clic sobre el piso del dojo
  enablePlacing(cb) {
    const el = this.renderer.domElement;
    const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
    const floor = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const hit = (ev) => {
      const r = el.getBoundingClientRect();
      ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
      ray.setFromCamera(ndc, this.camera);
      const p = new THREE.Vector3();
      return ray.ray.intersectPlane(floor, p) ? p : null;
    };
    let down = null;
    el.addEventListener('pointerdown', (ev) => { down = [ev.clientX, ev.clientY]; });
    el.addEventListener('pointermove', (ev) => {
      if (!this.placing) return;
      const p = hit(ev);
      this.cursor.visible = !!p;
      if (p) this.cursor.position.set(p.x, 0.003, p.z);
    });
    el.addEventListener('pointerup', (ev) => {
      if (!this.placing || !down || Math.hypot(ev.clientX - down[0], ev.clientY - down[1]) > 5) return;
      const p = hit(ev);
      if (p) cb(-p.z, -p.x);   // three -> ROS
    });
  }

  setPlacing(on) {
    this.placing = on;
    this.cursor.visible = false;
    this.renderer.domElement.style.cursor = on ? 'crosshair' : '';
  }

  // poses: {ekf: {x,y,th,cov}, ukf: {...}, odom: {...}}, main: clave del robot solido
  update(poses, main) {
    for (const k of ['ekf', 'ukf', 'odom']) {
      const p = poses[k];
      const gh = this.ghosts[k];
      gh.visible = !!p && !!this.shown[k];
      this.trails[k].line.visible = !!this.shown[k];
      if (!p) continue;
      gh.position.copy(v3(p.x, p.y));
      gh.rotation.y = p.th;
      this.pushTrail(this.trails[k], p.x, p.y);
    }
    const m = poses[main] || poses.ekf || poses.odom;
    if (m) {
      const target = v3(m.x, m.y);
      this.robot.position.lerp(target, 0.5);
      let dth = m.th - this.robot.rotation.y;
      dth = Math.atan2(Math.sin(dth), Math.cos(dth));
      this.robot.rotation.y += dth * 0.5;
    }
  }

  animateTargets() {
    const t = performance.now() / 1000;
    for (const b of this.beacons.values()) {
      const u = b.userData, ph = t + u.phase;
      const active = u.state === 'active';
      u.gem.rotation.y = ph * (active ? 1.6 : 0.6);
      u.gem.position.y = 0.5 + Math.sin(ph * 2.2) * (active ? 0.035 : 0.015);
      u.tag.position.y = u.gem.position.y + 0.16;
      u.cross.rotation.y = ph * 0.4;
      u.waves.forEach((w, i) => {
        const f = ((ph * (active ? 0.7 : 0.35)) + i / 3) % 1;
        const r = ARRIVE_R * (0.25 + f * (active ? 2.6 : 1.4));
        w.scale.set(r, r, 1);
        w.material.opacity = (1 - f) * (active ? 0.6 : 0.25);
        w.visible = u.state !== 'pending' || i === 0;
      });
      u.beam.material.opacity = active ? 0.2 + 0.1 * Math.sin(ph * 3) : 0.12;
    }
    if (this.guide.visible) this.guide.material.dashOffset = -t * 0.25;
    // Explosiones de llegada
    const now = t;
    this.bursts = this.bursts.filter((bu) => {
      const age = now - bu.t0;
      const dt = 1 / 60;
      const arr = bu.pts.geometry.attributes.position.array;
      for (let i = 0; i < bu.vel.length; i++) {
        const v = bu.vel[i];
        v.y -= 2.4 * dt;
        v.multiplyScalar(0.985);
        arr[i * 3] += v.x * dt; arr[i * 3 + 1] = Math.max(0, arr[i * 3 + 1] + v.y * dt); arr[i * 3 + 2] += v.z * dt;
      }
      bu.pts.geometry.attributes.position.needsUpdate = true;
      bu.pts.material.opacity = Math.max(0, 1 - age / 2.2);
      const r = 0.1 + age * 1.6;
      bu.shock.scale.set(r, r, 1);
      bu.shock.material.opacity = Math.max(0, 0.8 - age / 1.2);
      if (age > 2.4) {
        this.scene.remove(bu.pts, bu.shock);
        this.disposeTree(bu.pts); this.disposeTree(bu.shock);
        return false;
      }
      return true;
    });
  }

  render() {
    // La sombra cubre +-0.7 m: la luz sigue al robot por todo el dojo
    if (this.key) {
      const r = this.robot.position;
      this.key.position.set(r.x + 0.6, 1.4, r.z + 0.8);
      this.key.target.position.copy(r);
    }
    this.animateTargets();
    if (this.camMode === 'follow') {
      const r = this.robot;
      const back = new THREE.Vector3(0, 0.22, 0.32).applyAxisAngle(new THREE.Vector3(0, 1, 0), r.rotation.y);
      this.camera.position.lerp(r.position.clone().add(back), 0.12);
      this.camera.lookAt(r.position.x, 0.02, r.position.z);
    } else {
      this.controls.update();
    }
    this.renderer.render(this.scene, this.camera);
  }
}

// ======================================================================
// Mano
// ======================================================================
export class HandScene extends BaseScene {
  constructor(container) {
    super(container);
    this.camera.position.set(0.16, 0.17, 0.26);
    this.camera.lookAt(0, 0.0, 0);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.enablePan = false;
    this.controls.minDistance = 0.18;
    this.controls.maxDistance = 0.8;
    this.lights();
    this.hand = this.buildHand();
    this.scene.add(this.hand);
    this.target = new THREE.Quaternion();

    // Referencia horizontal (postura neutra)
    this.ref = new THREE.Mesh(
      new THREE.RingGeometry(0.105, 0.108, 96),
      new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.35, side: THREE.DoubleSide }),
    );
    this.ref.rotation.x = -Math.PI / 2;
    this.ref.position.y = -0.05;
    this.scene.add(this.ref);
    const shadowPlane = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.ShadowMaterial({ opacity: 0.3 }));
    shadowPlane.rotation.x = -Math.PI / 2;
    shadowPlane.position.y = -0.05;
    shadowPlane.receiveShadow = true;
    this.scene.add(shadowPlane);
    this.applyTheme();
  }

  buildHand() {
    const g = new THREE.Group();
    this.gloveMat = new THREE.MeshStandardMaterial({ color: 0x2b2f42, roughness: 0.62, metalness: 0.08 });
    const palm = new THREE.Mesh(new RoundedBoxGeometry(0.082, 0.024, 0.092, 6, 0.011), this.gloveMat);
    palm.castShadow = true;
    g.add(palm);
    // Dedos hacia adelante (ROS +x = three -z)
    const fingers = [[-0.03, 0.05], [-0.01, 0.058], [0.01, 0.055], [0.029, 0.046]];
    for (const [x, len] of fingers) {
      const f = new THREE.Mesh(new THREE.CapsuleGeometry(0.0085, len, 6, 16), this.gloveMat);
      f.rotation.x = Math.PI / 2;
      f.position.set(x, 0.002, -0.046 - len / 2);
      f.castShadow = true;
      g.add(f);
    }
    const thumb = new THREE.Mesh(new THREE.CapsuleGeometry(0.0095, 0.036, 6, 16), this.gloveMat);
    thumb.rotation.set(Math.PI / 2, 0, -0.9);
    thumb.position.set(-0.05, 0.0, -0.012);
    thumb.castShadow = true;
    g.add(thumb);
    // Modulo MPU6050 en el dorso
    this.chipMat = new THREE.MeshStandardMaterial({ color: 0x7c6cff, emissive: 0x7c6cff, emissiveIntensity: 0.6, roughness: 0.35 });
    const chip = new THREE.Mesh(new RoundedBoxGeometry(0.03, 0.006, 0.026, 3, 0.002), this.chipMat);
    chip.position.set(0, 0.015, 0.008);
    g.add(chip);
    // Ejes del sensor (ROS): x rojo adelante, y verde izquierda, z azul arriba
    const axis = (dir, color) => new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0.02, 0.008), 0.07, color, 0.014, 0.008);
    g.add(axis(new THREE.Vector3(0, 0, -1), 0xff5d6c), axis(new THREE.Vector3(-1, 0, 0), 0x34d399), axis(new THREE.Vector3(0, 1, 0), 0x60a5fa));
    return g;
  }

  applyTheme() {
    const th = theme();
    this.gloveMat.color = new THREE.Color(th.dark ? '#2b2f42' : '#3b4160');
    this.chipMat.color = this.chipMat.emissive = new THREE.Color(th.accent);
    this.ref.material.color = new THREE.Color(th.text3);
  }

  setRPY(roll, pitch, yaw) {
    const q = quatFromRPY(roll, pitch, yaw);
    this.target.copy(quatRos(q[0], q[1], q[2], q[3]));
  }

  render() {
    this.hand.quaternion.slerp(this.target, 0.35);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
