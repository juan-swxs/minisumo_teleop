// Tablero de tele-operacion · punto de entrada.

import {
  $, $$, api, toast, icons, fmt, fmtDeg, DEG, clamp, theme, segmented, fitCanvas,
  planeView, drawPlaneGrid, drawTrail, drawDot, drawArrow, Ring,
} from './ui.js';
import { DojoScene, HandScene } from './scenes.js';
import { rebuildAll } from './charts.js';
import * as signals from './signals.js';
import * as kalman from './kalman.js';
import * as experiments from './experiments.js';
import * as offline from './offline.js';
import * as settings from './settings.js';
import * as targets from './targets.js';
import * as energy from './energy.js';

const ctx = {
  S: null,          // ultimo resumen recibido del servidor
  view: 'teleop',
  setTeleop: async (vals) => {
    try { await api('api/teleop', vals); } catch (e) { toast(`Tele-operación: ${e.message}`, 'err'); }
  },
};
const modules = [targets, signals, kalman, experiments, offline, settings, energy];

icons();

// ---------------------------------------------------------------------
// Navegacion y tema
// ---------------------------------------------------------------------
function showView(name) {
  ctx.view = name;
  $$('.nav-btn[data-view]').forEach((b) => b.classList.toggle('active', b.dataset.view === name));
  $$('.view').forEach((v) => v.classList.toggle('active', v.id === `view-${name}`));
  modules.forEach((m) => m.onShow && m.onShow(name));
  try { localStorage.setItem('view', name); } catch (e) { /* sin almacenamiento */ }
  history.replaceState(null, '', `#${name}`);
}
$$('.nav-btn[data-view]').forEach((b) => b.addEventListener('click', () => showView(b.dataset.view)));
ctx.showView = showView;

// Barra superior: transparente arriba (sin recuadro sobre el fondo con
// degradado) y con fondo translucido solo al hacer scroll
const topbar = $('.topbar');
const onScroll = () => topbar.classList.toggle('scrolled', window.scrollY > 6);
window.addEventListener('scroll', onScroll, { passive: true });
onScroll();

// Menu lateral: compacto (iconos) y se despliega al pasar el mouse; con
// "Fijar menu" queda abierto y el contenido se corre a la derecha.
function setRailPinned(on) {
  document.querySelector('.app').classList.toggle('rail-pinned', on);
  const btn = $('#rail-pin');
  btn.querySelector('b').textContent = on ? 'Ocultar menú' : 'Fijar menú';
  btn.querySelector('small').textContent = on ? 'Volver a solo íconos' : 'Dejarlo siempre abierto';
  btn.title = on ? 'Plegar el menú a solo íconos' : 'Fijar el menú abierto';
  btn.querySelector('.nav-ico').innerHTML = `<i data-lucide="${on ? 'panel-left-close' : 'panel-left-open'}"></i>`;
  icons();
  try { localStorage.setItem('railPinned', on ? '1' : '0'); } catch (e) { /* sin almacenamiento */ }
}
let railPinned = false;
try { railPinned = localStorage.getItem('railPinned') === '1'; } catch (e) { /* sin almacenamiento */ }
setRailPinned(railPinned);
$('#rail-pin').addEventListener('click', () => { railPinned = !railPinned; setRailPinned(railPinned); });

$('#theme-btn').addEventListener('click', () => {
  const r = document.documentElement;
  r.dataset.theme = r.dataset.theme === 'light' ? 'dark' : 'light';
  try { localStorage.setItem('theme', r.dataset.theme); } catch (e) { /* sin almacenamiento */ }
  dojo.applyTheme();
  hand.applyTheme();
  rebuildAll();
});

// ---------------------------------------------------------------------
// Barra superior
// ---------------------------------------------------------------------
function setChip(id, link, label) {
  const chip = $(id);
  const dot = $('.dot', chip);
  const age = link ? link.age : null;
  dot.className = 'dot ' + (age === null ? '' : age < 0.6 ? 'ok' : age < 3 ? 'warn' : 'bad');
  $('.hz', chip).textContent = age === null ? 'sin datos' : age < 3 ? `${fmt(link.hz, 0)} Hz` : label;
  chip.title = `${$('b', chip).textContent}: ${$('.hz', chip).textContent}`;   // visible aunque la barra este compacta
}

let recording = false;
$('#rec-btn').addEventListener('click', async () => {
  try {
    if (!recording) {
      const r = await api('api/record', { on: true, label: $('#rec-label').value });
      toast(`Grabando sesión ${r.name}`);
    } else {
      const r = await api('api/record', { on: false });
      toast(`Sesión guardada: ${r.name}`);
      offline.refresh();
    }
  } catch (e) { toast(e.message, 'err'); }
});

async function estop() {
  try { await api('api/estop', {}); toast('Parada de emergencia: robot detenido'); } catch (e) { toast(e.message, 'err'); }
}
$('#estop-btn').addEventListener('click', estop);
document.addEventListener('keydown', (e) => {
  if (e.target.matches('input, select, textarea')) return;
  if (e.code === 'Space' || e.code === 'Escape') { e.preventDefault(); estop(); }
});

// ---------------------------------------------------------------------
// Vista de tele-operacion
// ---------------------------------------------------------------------
const dojo = new DojoScene($('#dojo-scene'));
const hand = new HandScene($('#hand-scene'));
ctx.dojo = dojo;

const setMode = async (m) => {
  try { await api('api/mode', { mode: m }); } catch (e) { toast(e.message, 'err'); }
};
const segLink = segmented($('#seg-link'), (v) => setMode(v));
const segStrategy = segmented($('#seg-strategy'), (v) => {
  ctx.setTeleop({ strategy: +v });
  // La estrategia 1 necesita los filtros de Kalman del PC
  if (+v === 1 && ctx.S && ctx.S.mode === 'espnow') {
    setMode('pc');
    toast('Estrategia 1 usa la velocidad estimada en el PC: enlace cambiado a PC · ROS 2');
  }
});
const segGlove = segmented($('#seg-glove-input'), (v) => ctx.setTeleop({ glove_input: v }));
const segSource = segmented($('#seg-source'), (v) => ctx.setTeleop({ source: v }));
segmented($('#seg-cam'), (v) => dojo.setCamMode(v));

$('#arm-btn').addEventListener('click', async () => {
  const S = ctx.S;
  if (!S) return;
  if (S.mode === 'espnow') {
    if (S.espnow.link === 0 && S.links.glove.age === null) { toast('El guante no está conectado', 'err'); return; }
    if (S.teleop && S.teleop.params && +S.teleop.params.strategy === 1) {
      toast('Por ESP-NOW el carro usa la estrategia 2 (inclinación)');
    }
    try { await api('api/glove/arm', { on: !S.glove_armed }); } catch (e) { toast(e.message, 'err'); }
    return;
  }
  const tp = S.teleop;
  if (!tp) { toast('El nodo glove_teleop no está corriendo', 'err'); return; }
  ctx.setTeleop({ enabled: !tp.enabled });
});
$('#btn-zero').addEventListener('click', async () => {
  await api('api/glove/zero', {});
  handTrail.EKF.clear(); handTrail.UKF.clear();
  toast('Postura neutra fijada: mantén la mano así para detener el robot');
});
$('#btn-reset-robot').addEventListener('click', async () => {
  await api('api/robot/reset', { car: true });
  dojo.clearTrails();
  kalman.clearTrails();
  targets.resetProgress();
  toast('Robot en el origen: su posición actual es ahora (0, 0) mirando hacia +x');
});
let lastDojoR = null;
$('#sysreset-btn').addEventListener('click', async () => {
  if (!confirm('¿Reiniciar el sistema?\n\nSe detiene y desarma el robot, y su posición actual pasa a ser el nuevo origen (0, 0, rumbo 0). También se reinician los filtros EKF/UKF de la mano y del robot, se borran los rastros y se quitan los destinos del dojo (las tablas de Pruebas se conservan).')) return;
  try {
    await api('api/system/reset', {});
    dojo.clearTrails();
    kalman.clearTrails();
    handTrail.EKF.clear(); handTrail.UKF.clear();
    targets.resetProgress();
    await experiments.clearTargets();
    toast('Sistema reiniciado: el carro está en el nuevo origen y el dojo quedó sin destinos');
  } catch (e) { toast(e.message, 'err'); }
});
$('#btn-clear-trails').addEventListener('click', () => { dojo.clearTrails(); kalman.clearTrails(); });

// Colocar destinos con un clic en el dojo 3D (un clic = un destino)
const placeBtn = $('#btn-place');
function setPlacing(on) {
  dojo.setPlacing(on);
  placeBtn.classList.toggle('on', on);
  $('#dojo-scene').classList.toggle('placing', on);
}
placeBtn.addEventListener('click', () => setPlacing(!dojo.placing));
dojo.enablePlacing(async (x, y) => {
  setPlacing(false);
  try {
    const name = await experiments.addTarget(x, y);
    toast(`Destino ${name} en (${fmt(x, 2)}, ${fmt(y, 2)}) m · llévale el robot`);
  } catch (e) { toast(e.message, 'err'); }
});
$('#btn-hand-reset').addEventListener('click', async () => {
  await api('api/hand/reset', {});
  handTrail.EKF.clear(); handTrail.UKF.clear();
});

// Medidores semicirculares bipolares (centro = 0)
function gauge(svg, value, max, color) {
  const th = theme();
  const cx = 60, cy = 62, r = 48;
  const pt = (a) => [cx + r * Math.cos(a), cy - r * Math.sin(a)];
  const arc = (a0, a1) => {
    const [x0, y0] = pt(a0), [x1, y1] = pt(a1);
    return `M ${x0} ${y0} A ${r} ${r} 0 0 ${a1 < a0 ? 1 : 0} ${x1} ${y1}`;
  };
  const f = clamp(value / max, -1, 1);
  const a = Math.PI / 2 - f * Math.PI / 2;
  const [nx, ny] = pt(a);
  svg.innerHTML = `
    <path d="${arc(Math.PI, 0)}" stroke="${th.grid}" stroke-width="9" fill="none" stroke-linecap="round"/>
    ${Math.abs(f) > 0.005 ? `<path d="${arc(Math.PI / 2, a)}" stroke="${color}" stroke-width="9" fill="none" stroke-linecap="round"/>` : ''}
    <line x1="${cx}" y1="${cy - r - 7}" x2="${cx}" y2="${cy - r + 7}" stroke="${th.text3}" stroke-width="1.5"/>
    <circle cx="${nx}" cy="${ny}" r="5.5" fill="${th.surface}" stroke="${color}" stroke-width="2.5"/>`;
}

// Pad de inclinacion (estrategia 2)
function drawTiltPad(S) {
  const cv = $('#tilt-pad');
  const { ctx: g, w, h } = fitCanvas(cv);
  const th = theme();
  g.clearRect(0, 0, w, h);
  const p = (S.teleop && S.teleop.params) || S.remote.glove_teleop;
  const full = p.full_tilt_deg, dz = p.deadzone_deg;
  const lim = full * 1.25;
  const size = Math.min(w, h) - 40, cx = w / 2, cy = h / 2;
  const s = size / (2 * lim);
  const sq = (d, stroke, fill, dash) => {
    g.save(); g.setLineDash(dash || []);
    g.strokeStyle = stroke; g.fillStyle = fill; g.lineWidth = 1.5;
    g.beginPath(); g.roundRect(cx - d * s, cy - d * s, 2 * d * s, 2 * d * s, 10); if (fill) g.fill(); g.stroke(); g.restore();
  };
  sq(full, th.border, null, [5, 5]);
  sq(dz, th.border, th.dark ? 'rgba(255,255,255,0.04)' : 'rgba(15,23,42,0.04)');
  g.strokeStyle = th.grid; g.lineWidth = 1;
  g.beginPath(); g.moveTo(cx, 18); g.lineTo(cx, h - 18); g.moveTo(18, cy); g.lineTo(w - 18, cy); g.stroke();
  g.fillStyle = th.text3; g.font = '11px Inter, sans-serif'; g.textAlign = 'center';
  g.fillText('adelante', cx, 14); g.fillText('atrás', cx, h - 4);
  g.textAlign = 'left'; g.fillText('izq.', 6, cy - 6); g.textAlign = 'right'; g.fillText('der.', w - 6, cy - 6);

  // roll/pitch del modo de entrada activo (lo calcula el nodo)
  const tp = S.teleop || {};
  let roll = tp.roll, pitch = tp.pitch;
  if (roll === undefined) { roll = S.glove.rpy[0]; pitch = S.glove.rpy[1]; }
  const sp = p.invert_pitch ? -1 : 1, sr = p.invert_roll ? -1 : 1;
  const px = cx + clamp(sr * roll * DEG, -lim, lim) * s;
  const py = cy - clamp(sp * pitch * DEG, -lim, lim) * s;
  const active = tp.enabled && (Math.abs(tp.v_target || 0) > 0 || Math.abs(tp.w_target || 0) > 0);
  const col = active ? th.good : th.accent;
  g.strokeStyle = col; g.globalAlpha = 0.5; g.lineWidth = 2;
  g.beginPath(); g.moveTo(cx, cy); g.lineTo(px, py); g.stroke(); g.globalAlpha = 1;
  g.fillStyle = col; g.globalAlpha = 0.2; g.beginPath(); g.arc(px, py, 16, 0, 7); g.fill();
  g.globalAlpha = 1; g.beginPath(); g.arc(px, py, 7, 0, 7); g.fill();
  g.fillStyle = th.text2; g.font = '11.5px JetBrains Mono, monospace'; g.textAlign = 'left';
  g.fillText(`zona muerta ±${dz.toFixed(0)}°   máx ±${full.toFixed(0)}°`, 10, h - 8);
}

// Mano en el plano (estrategia 1)
const handTrail = { EKF: new Ring(500), UKF: new Ring(500) };
function drawHandPlane(S) {
  const cv = $('#hand-plane');
  const { ctx: g, w, h } = fitCanvas(cv);
  const th = theme();
  g.clearRect(0, 0, w, h);
  // La posicion de la mano se obtiene integrando dos veces la aceleracion
  // y deriva: la vista sigue a la mano en pasos de 20 cm.
  const e = S.hand.EKF;
  const cx = e ? Math.round(e.x / 0.2) * 0.2 : 0, cy = e ? Math.round(e.y / 0.2) * 0.2 : 0;
  const view = planeView(w, h, 0.8, cx, cy);
  drawPlaneGrid(g, w, h, view, 0.1, th);
  for (const f of ['EKF', 'UKF']) {
    const st = S.hand[f];
    if (!st) continue;
    handTrail[f].push([st.x, st.y], 0.002);
    const c = f === 'EKF' ? th.ekf : th.ukf;
    drawTrail(g, view, handTrail[f].a, c, 2, 0.7);
    drawDot(g, view, st.x, st.y, c, 5);
    drawArrow(g, view, st.x, st.y, st.vx * 0.5, st.vy * 0.5, c, 2.5);
  }
  const stt = S.hand.stats;
  g.font = '11.5px Inter, sans-serif';
  g.fillStyle = th.text3;
  g.fillText(`escala: flecha = velocidad × 0.5 s · rejilla 10 cm · centro (${fmt(cx, 1)}, ${fmt(cy, 1)}) m`, 10, h - 8);
  if (stt && stt.zupt) {
    g.fillStyle = th.good;
    g.font = '600 12px Inter, sans-serif';
    g.fillText('● ZUPT · mano quieta', 10, 18);
  }
  if (!S.hand.EKF) {
    g.fillStyle = th.text3; g.textAlign = 'center'; g.font = '13px Inter, sans-serif';
    g.fillText('Sin estimación de la mano (hand_estimator)', w / 2, h / 2);
    g.textAlign = 'left';
  }
}

// Rastros en el dojo: solo el del estimador elegido, salvo que el usuario
// active los demas con un clic en la leyenda. Dos estimaciones distintas
// (p. ej. odometria del ESP32 y EKF) se separan poco a poco y se veian
// como una "linea espejo" al lado del carro.
let extraTracks = new Set();
try { extraTracks = new Set(JSON.parse(localStorage.getItem('extraTracks') || '[]')); } catch (e) { /* sin almacenamiento */ }
$('#dojo-legend').addEventListener('pointerdown', (ev) => {   // la leyenda se redibuja muy seguido: 'click' se perderia
  const li = ev.target.closest('[data-k]');
  if (!li || li.dataset.main === '1') return;
  const k = li.dataset.k;
  if (extraTracks.has(k)) extraTracks.delete(k); else extraTracks.add(k);
  try { localStorage.setItem('extraTracks', JSON.stringify([...extraTracks])); } catch (e) { /* sin almacenamiento */ }
});

function legend(S, main) {
  const th = theme();
  dojo.setShown({ ekf: main === 'ekf' || extraTracks.has('ekf'), ukf: main === 'ukf' || extraTracks.has('ukf'), odom: main === 'odom' || extraTracks.has('odom') });
  const rows = [
    ['ekf', 'EKF', th.ekf, S.robot.EKF],
    ['ukf', 'UKF', th.ukf, S.robot.UKF],
    ['odom', 'Odometría ESP32', th.odom, S.car.odom],
  ];
  $('#dojo-legend').innerHTML = rows.map(([k, n, c, p]) => {
    const isMain = k === main, on = isMain || extraTracks.has(k);
    return `
    <div class="li track ${on ? '' : 'off'}" data-k="${k}" data-main="${isMain ? 1 : 0}" title="${isMain ? 'Estimador del robot (cámbialo en Control → Estimador)' : (on ? 'Clic para ocultar su rastro' : 'Clic para mostrar su rastro en el dojo')}"><span class="sw" style="background:${c}${isMain ? ';box-shadow:0 0 0 2px ' + c + '55' : ''}"></span>${n}${isMain ? ' <span class="muted">· robot</span>' : ''}
    <span class="v">${p ? `${fmt(p.x)} , ${fmt(p.y)} m · ${fmtDeg(p.th, 0)}` : '—'}</span></div>`;
  }).join('') + '<div class="li hint">clic en una fila para mostrar u ocultar su rastro</div>';
}

function updateTeleop(S) {
  const tp = S.teleop;
  const p = (tp && tp.params) || S.remote.glove_teleop;
  const espnow = S.mode === 'espnow';
  const E = S.espnow;
  segLink.set(S.mode);
  segStrategy.set(espnow ? 2 : p.strategy);
  segGlove.set(p.glove_input);
  segSource.set(p.source);

  const linkTxt = ['sin señal del guante', 'enlace activo · guante desarmado', 'guante armado · conduciendo'];
  $('#link-info').innerHTML = espnow
    ? `<span>${linkTxt[E.link] || '—'}</span><span><b>${fmt(E.rate, 0)}</b> paq/s</span><span>RSSI <b>${E.link ? E.rssi : '—'}</b> dBm</span>`
    : '<span>glove_teleop (PC) publica <b>/cmd_vel</b> · necesario para la estrategia 1</span>';

  let on, title, sub, reason, v, w;
  if (espnow) {
    on = !!S.glove_armed;
    title = on ? 'Guante armado' : 'Guante desarmado';
    sub = on ? 'ESP-NOW · el carro sigue la inclinación de la mano' : 'Pulsa aquí o el botón BOOT del guante';
    const guardTxt = ['', 'Cerca del borde: velocidad reducida', 'Borde del dojo: solo giro'];
    reason = E.link === 0 ? 'El carro no recibe al guante por ESP-NOW (mismo canal WiFi y grupo)' : (guardTxt[E.guard] || '');
    v = E.v; w = E.w;
  } else {
    on = !!(tp && tp.enabled);
    title = !tp ? 'Sin nodo de tele-operación' : on ? 'Armado' : 'Desarmado';
    sub = !tp ? 'Lanza glove_teleop' : on ? `Estrategia ${p.strategy} · el robot sigue al guante` : 'El robot ignora el guante';
    reason = tp ? tp.reason : 'glove_teleop no está publicando';
    v = tp ? tp.v : 0; w = tp ? tp.w : 0;
  }
  $('#arm').classList.toggle('on', on);
  $('#arm-title').textContent = title;
  $('#arm-sub').textContent = sub;
  const r = $('#reason');
  r.classList.toggle('alert', !!reason && reason !== 'Desarmado' && !reason.startsWith('Mano'));
  $('span', r).textContent = reason || (on ? 'Enviando comandos al robot' : '—');

  const th = theme();
  gauge($('#g-v'), v, p.v_max, th.ekf);
  gauge($('#g-w'), w, p.w_max, th.ukf);
  $('#g-v-val').textContent = fmt(v, 2);
  $('#g-w-val').textContent = fmt(w, 2);

  const [rr, pp, yy] = S.glove.rpy;
  $('#rpy-r').textContent = fmtDeg(rr);
  $('#rpy-p').textContent = fmtDeg(pp);
  $('#rpy-y').textContent = fmtDeg(yy);
  hand.setRPY(rr, pp, yy);

  const poses = {
    ekf: S.robot.EKF, ukf: S.robot.UKF,
    odom: S.links.car.age !== null ? S.car.odom : null,
  };
  const main = p.source || 'ekf';
  dojo.update(poses, main);
  legend(S, main);
  const m = poses[main] || poses.ekf || poses.odom;
  targets.tick(m || null);
  const R = p.dojo_radius || 2.5;
  const edge = m ? R - Math.hypot(m.x, m.y) : null;
  $('#edge-dist').textContent = edge === null ? '—' : (Math.abs(edge) >= 1 ? `${fmt(edge, 2)} m` : `${fmt(edge * 100, 1)} cm`);
  if (R !== lastDojoR) { lastDojoR = R; dojo.setRadius(R); $('#dojo-sub').textContent = `Posición del robot · Ø ${fmt(2 * R, 2 * R >= 1 ? 1 : 2)} m`; }

  if (ctx.view === 'teleop') { drawTiltPad(S); drawHandPlane(S); }
}

// ---------------------------------------------------------------------
// Conexion en vivo (Server-Sent Events)
// ---------------------------------------------------------------------
// Indicadores del menu lateral: armado (Teleop) y estado del sistema
function updateRail(S) {
  const armed = S.mode === 'pc' ? !!(S.teleop && S.teleop.enabled) : !!S.glove_armed;
  const nb = $('#nb-teleop');
  const txt = armed ? 'ARMADO' : '';
  if (nb.textContent !== txt) { nb.textContent = txt; nb.className = `nav-badge ${armed ? 'ok' : ''}`; }
  const L = S.links;
  const parts = [['guante', L.glove], ['carro', L.car]];
  const down = parts.filter(([, l]) => !l || l.age === null || l.age > 3).map(([n]) => n);
  const rs = $('#rail-status');
  const state = down.length === 0 ? 'ok' : down.length === parts.length ? 'bad' : 'warn';
  rs.className = `rail-status ${state}`;
  $('.dot', rs).className = `dot ${state}`;
  $('#rail-status-txt').textContent = down.length === 0 ? 'guante y carro en línea' : `sin ${down.join(' ni ')}`;
}

let lastMsg = 0;
function connect() {
  const es = new EventSource('events');
  es.onmessage = (ev) => {
    lastMsg = performance.now();
    $('#conn-banner').classList.remove('show');
    const S = JSON.parse(ev.data);
    ctx.S = S;
    setChip('#chip-glove', S.links.glove, 'sin señal');
    setChip('#chip-car', S.links.car, 'sin señal');
    setChip('#chip-est', S.links.robot_est.age !== null ? S.links.robot_est : S.links.hand_est, 'detenido');
    setChip('#chip-teleop', S.links.teleop, 'detenido');
    setChip('#chip-espnow', S.espnow.link ? S.links.espnow : { age: null, hz: 0 }, 'sin señal');
    if (S.espnow.link) $('#chip-espnow .hz').textContent = `${fmt(S.espnow.rate, 0)} paq/s`;
    recording = S.recording.active;
    $('#rec-btn').classList.toggle('on', recording);
    $('#rec-btn .txt').textContent = recording ? `Grabando · ${S.recording.rows}` : 'Grabar';
    updateTeleop(S);
    energy.tick(S);
    updateRail(S);
    modules.forEach((mod) => mod.update && mod.update(S, ctx));
  };
  es.onerror = () => { $('#conn-banner').classList.add('show'); };
}
setInterval(() => { if (performance.now() - lastMsg > 3000) $('#conn-banner').classList.add('show'); }, 1000);

modules.forEach((m) => m.init(ctx));
connect();
let startView = location.hash.slice(1);
if (!startView) {
  try { startView = localStorage.getItem('view') || 'teleop'; } catch (e) { startView = 'teleop'; }
}
showView($(`#view-${startView}`) ? startView : 'teleop');
