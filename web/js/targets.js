// Destinos en el dojo: detecta cuando el robot llega a un punto, muestra
// la distancia en vivo y celebra la llegada (explosion 3D, confeti,
// sonido y tarjeta "Llegaste a tu destino").

import { $, el, api, toast, fmt, icons, theme } from './ui.js';
import { ARRIVE_R } from './scenes.js';

const LEAVE_R = ARRIVE_R * 2.5;   // hay que salir de aqui para volver a celebrar el mismo punto
let ctxRef = null;
let list = [];                    // [{key, id, x, y, label, inside, reached, t0, path, d0}]
let last = null;                  // ultima pose del robot (para el recorrido)
let activeKey = null;

export function init(ctx) {
  ctxRef = ctx;
  buildHud();
  buildOverlay();
  // El navegador solo deja sonar audio despues de un gesto del usuario
  const unlock = () => {
    try {
      audio = audio || new (window.AudioContext || window.webkitAudioContext)();
      if (audio.state === 'suspended') audio.resume();
    } catch (e) { /* sin audio */ }
  };
  window.addEventListener('pointerdown', unlock, { once: true });
  window.addEventListener('keydown', unlock, { once: true });
}

// Lo llama experiments.js con los puntos de la estrategia actual
export function setTargets(pts) {
  const old = new Map(list.map((t) => [t.key, t]));
  const now = performance.now() / 1000;
  list = pts.map((p) => {
    const key = `${p.x},${p.y}`;
    const o = old.get(key);
    return o ? { ...o, label: p.label, id: p.id } : { key, id: p.id, x: p.x, y: p.y, label: p.label, inside: false, reached: false, t0: now, path: 0, d0: null };
  });
  if (ctxRef && ctxRef.dojo) ctxRef.dojo.setTargets(list);
  if (!list.length) { activeKey = null; ctxRef && ctxRef.dojo && ctxRef.dojo.setGuide(null); $('#target-hud').classList.remove('show'); }
}

// Nuevo origen o rastros borrados: todos los destinos vuelven a pendientes
export function resetProgress() {
  const now = performance.now() / 1000;
  for (const t of list) Object.assign(t, { inside: false, reached: false, t0: now, path: 0, d0: null });
  last = null;
}

// m = pose principal del robot {x, y, th}
export function tick(m) {
  const hud = $('#target-hud');
  const dojo = ctxRef.dojo;
  if (!m || !list.length) { dojo.setGuide(null); hud.classList.remove('show'); return; }
  const now = performance.now() / 1000;
  const step = last ? Math.hypot(m.x - last.x, m.y - last.y) : 0;
  last = { x: m.x, y: m.y };

  for (const t of list) {
    const d = Math.hypot(m.x - t.x, m.y - t.y);
    t.dist = d;
    if (step < 0.5) t.path += step;   // un salto grande es un reinicio, no movimiento
    if (!t.inside && d < ARRIVE_R) {
      t.inside = true;
      t.reached = true;
      arrive(t, d, now);
    } else if (t.inside && d > LEAVE_R) {
      t.inside = false;
      t.t0 = now; t.path = 0; t.d0 = null;
    }
  }
  // Destino activo: el pendiente mas cercano; si todos estan alcanzados,
  // el mas cercano del que el robot no este encima
  const pend = list.filter((t) => !t.reached);
  const pool = pend.length ? pend : list.filter((t) => !t.inside);
  const act = pool.reduce((a, b) => (!a || b.dist < a.dist ? b : a), null);
  if (act && act.key !== activeKey) { act.d0 = act.dist; act.t0 = now; act.path = 0; }
  activeKey = act ? act.key : null;
  for (const t of list) dojo.setBeaconState(t.key, t.key === activeKey ? 'active' : t.reached ? 'reached' : 'pending');
  dojo.setGuide(act);

  if (!act) { hud.classList.remove('show'); return; }
  hud.classList.add('show');
  $('#th-label').textContent = act.label;
  $('#th-dist').textContent = act.dist >= 1 ? `${fmt(act.dist, 2)} m` : `${fmt(act.dist * 100, 0)} cm`;
  const prog = act.d0 ? Math.max(0, Math.min(1, 1 - (act.dist - ARRIVE_R) / Math.max(act.d0 - ARRIVE_R, 0.01))) : 0;
  $('#th-bar').style.width = `${(prog * 100).toFixed(1)}%`;
  // Flecha: direccion del destino respecto al frente del robot
  const ang = Math.atan2(act.y - m.y, act.x - m.x) - m.th;
  $('#th-arrow').style.transform = `rotate(${(-ang * 180 / Math.PI).toFixed(1)}deg)`;
  const done = list.filter((t) => t.reached).length;
  $('#th-count').textContent = `${done}/${list.length}`;
}

// ---------------------------------------------------------------------
function buildHud() {
  const hud = el('div', { class: 'overlay tl glass target-hud', id: 'target-hud' },
    el('div', { class: 'th-top' },
      el('div', { class: 'th-arrow-wrap' }, el('i', { 'data-lucide': 'navigation-2', id: 'th-arrow' })),
      el('div', {},
        el('div', { class: 'label' }, 'Destino ', el('b', { id: 'th-label' }, '—'), el('span', { class: 'th-count', id: 'th-count' }, '')),
        el('div', { class: 'num th-dist', id: 'th-dist' }, '—'))),
    el('div', { class: 'th-track' }, el('div', { class: 'th-bar', id: 'th-bar' })));
  $('#dojo-scene').parentElement.append(hud);
  icons();
}

function buildOverlay() {
  const ov = el('div', { class: 'arrive', id: 'arrive' },
    el('canvas', { class: 'confetti', id: 'confetti' }),
    el('div', { class: 'arrive-card' },
      el('div', { class: 'arrive-badge', html: `
        <svg viewBox="0 0 120 120" class="badge-svg">
          <circle class="b-ring r3" cx="60" cy="60" r="56"/>
          <circle class="b-ring r2" cx="60" cy="60" r="46"/>
          <circle class="b-disc" cx="60" cy="60" r="38"/>
          <path class="b-check" d="M42 61 l12 12 l25 -27"/>
        </svg>` }),
      el('div', { class: 'arrive-kicker' }, 'Destino alcanzado'),
      el('h2', { class: 'arrive-title' }, '¡Llegaste a tu destino!'),
      el('div', { class: 'arrive-sub', id: 'arrive-sub' }, ''),
      el('div', { class: 'arrive-stats' },
        stat('crosshair', 'Error final', 'as-err'),
        stat('timer', 'Tiempo', 'as-time'),
        stat('route', 'Recorrido', 'as-path')),
      el('div', { class: 'arrive-actions' },
        el('button', { class: 'btn primary', id: 'arrive-save' }, el('i', { 'data-lucide': 'table-2' }), 'Guardar estimación en la tabla'),
        el('button', { class: 'btn', id: 'arrive-close' }, 'Continuar'))));
  document.body.append(ov);
  icons();
  ov.addEventListener('click', (e) => { if (e.target === ov) closeOverlay(); });
  $('#arrive-close').addEventListener('click', closeOverlay);
}

function stat(icon, name, id) {
  return el('div', { class: 'arrive-stat' }, el('i', { 'data-lucide': icon }), el('div', { class: 'num', id }, '—'), el('div', { class: 'cap' }, name));
}

let closeTimer = null;
function arrive(t, d, now) {
  ctxRef.dojo.celebrate(t.x, t.y);
  chime();
  $('#arrive-sub').textContent = `${t.label} · (${fmt(t.x, 2)}, ${fmt(t.y, 2)}) m`;
  $('#as-err').textContent = `${fmt(d * 100, 1)} cm`;
  const secs = now - t.t0;
  $('#as-time').textContent = secs >= 60 ? `${Math.floor(secs / 60)}:${String(Math.round(secs % 60)).padStart(2, '0')} min` : `${fmt(secs, 1)} s`;
  $('#as-path').textContent = t.path >= 1 ? `${fmt(t.path, 2)} m` : `${fmt(t.path * 100, 0)} cm`;
  const save = $('#arrive-save');
  save.style.display = t.id ? '' : 'none';
  save.disabled = false;
  save.onclick = async () => {
    try {
      await api('api/experiments/capture', { id: t.id });
      toast(`Estimación de ${t.label} guardada en la tabla`);
      save.disabled = true;
    } catch (e) { toast(e.message, 'err'); }
  };
  const ov = $('#arrive');
  ov.classList.remove('show');
  void ov.offsetWidth;   // reinicia las animaciones CSS
  ov.classList.add('show');
  confetti();
  clearTimeout(closeTimer);
  closeTimer = setTimeout(closeOverlay, 7000);
}

function closeOverlay() {
  clearTimeout(closeTimer);
  $('#arrive').classList.remove('show');
}

// Confeti a pantalla completa (canvas 2D, ~3 s)
function confetti() {
  const cv = $('#confetti');
  const g = cv.getContext('2d');
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const W = cv.width = innerWidth * dpr, H = cv.height = innerHeight * dpr;
  const th = theme();
  const cols = [th.good, th.exp, th.accent, th.ekf, '#fbbf24', '#ffffff'];
  const parts = [];
  for (let i = 0; i < 220; i++) {
    const fromLeft = i % 2 === 0;
    parts.push({
      x: fromLeft ? -10 * dpr : W + 10 * dpr, y: H * (0.55 + Math.random() * 0.3),
      vx: (fromLeft ? 1 : -1) * (6 + Math.random() * 9) * dpr, vy: -(10 + Math.random() * 12) * dpr,
      w: (5 + Math.random() * 7) * dpr, h: (8 + Math.random() * 10) * dpr,
      a: Math.random() * 6, va: (Math.random() - 0.5) * 0.35, c: cols[i % cols.length],
    });
  }
  const t0 = performance.now();
  const frame = (tn) => {
    const age = (tn - t0) / 1000;
    g.clearRect(0, 0, W, H);
    for (const p of parts) {
      p.vy += 0.45 * dpr; p.vx *= 0.992; p.vy *= 0.992;
      p.x += p.vx; p.y += p.vy; p.a += p.va;
      g.save();
      g.globalAlpha = Math.max(0, 1 - Math.max(0, age - 2) / 1.2);
      g.translate(p.x, p.y); g.rotate(p.a);
      g.fillStyle = p.c;
      g.fillRect(-p.w / 2, -p.h / 2 * Math.abs(Math.cos(p.a * 2)), p.w, p.h * Math.abs(Math.cos(p.a * 2)) + 1);
      g.restore();
    }
    if (age < 3.3) requestAnimationFrame(frame);
    else g.clearRect(0, 0, W, H);
  };
  requestAnimationFrame(frame);
}

// Arpegio corto (Do-Mi-Sol-Do) con WebAudio
let audio = null;
function chime() {
  try {
    if (!audio || audio.state !== 'running') return;
    const t = audio.currentTime + 0.02;
    [523.25, 659.25, 783.99, 1046.5].forEach((f, i) => {
      const o = audio.createOscillator(), gn = audio.createGain();
      o.type = 'triangle'; o.frequency.value = f;
      gn.gain.setValueAtTime(0, t + i * 0.09);
      gn.gain.linearRampToValueAtTime(0.16, t + i * 0.09 + 0.02);
      gn.gain.exponentialRampToValueAtTime(0.001, t + i * 0.09 + (i === 3 ? 0.9 : 0.35));
      o.connect(gn).connect(audio.destination);
      o.start(t + i * 0.09); o.stop(t + i * 0.09 + 1);
    });
  } catch (e) { /* sin audio */ }
}
