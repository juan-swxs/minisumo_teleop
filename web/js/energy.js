// Vista "Energia": alimentacion del sistema medida con el INA219 del carro.
//  - Icono de bateria tipo celular en la barra superior (siempre visible).
//  - Bateria grande con el nivel, estado, autonomia y carga restante.
//  - Medidas en vivo y tres graficas (tension, corriente, potencia): una
//    magnitud por grafica, sin doble eje.

import { $, el, fmt, icons, segmented, api, toast, theme } from './ui.js';
import { LiveChart } from './charts.js';

let ctxRef = null;
let charts = null;
let windowS = 300;
let lastStatus = null;
let histLoaded = false;
let lastPush = 0;

const STATUS = {
  sin_datos:    { label: 'Sin datos de alimentación', icon: 'circle-dashed', cls: 'none' },
  desconectada: { label: 'Alimentación no conectada al INA219', icon: 'plug-zap', cls: 'none' },
  reposo:       { label: 'En reposo', icon: 'pause', cls: 'ok' },
  descargando:  { label: 'Descargando', icon: 'battery-medium', cls: 'ok' },
  baja:         { label: 'Alimentación baja', icon: 'battery-low', cls: 'warn' },
  critica:      { label: 'Alimentación crítica: carga las baterías', icon: 'battery-warning', cls: 'bad' },
};

// Formato del eje de tiempo de las graficas (segundos relativos)
const xFmt = (x) => {
  const a = Math.abs(x);
  if (a < 0.5) return 'ahora';
  return a >= 60 ? `−${(a / 60).toFixed(a % 60 ? 1 : 0)} min` : `−${a.toFixed(0)} s`;
};

const levelClass = (B) => {
  if (!B || B.soc === null || B.soc === undefined || B.status === 'sin_datos' || B.status === 'desconectada') return 'none';
  if (B.soc <= B.critical_pct) return 'bad';
  if (B.soc <= B.low_pct) return 'warn';
  return 'ok';
};

// ---------------------------------------------------------------------
export function init(ctx) {
  ctxRef = ctx;
  $('#batt-pill').addEventListener('click', () => ctx.showView && ctx.showView('energy'));
  buildKpis();
  buildCharts();
  loadHistory();
  requestAnimationFrame(frame);
  segmented($('#seg-en-window'), (v) => {
    windowS = +v;
    for (const c of Object.values(charts)) { c.window = windowS; c.dirty = true; }
  });
  $('#en-reset').addEventListener('click', async () => {
    try {
      await api('api/battery/reset', {});
      for (const c of Object.values(charts)) c.clear();
      toast('Nueva sesión de energía: acumulados en cero');
    } catch (e) { toast(e.message, 'err'); }
  });
}

function buildKpis() {
  const box = $('#en-kpis');
  const kpi = (id, icon, label, unit, foot = '') => el('div', { class: 'en-kpi' },
    el('div', { class: 'label' }, el('i', { 'data-lucide': icon }), label),
    el('div', { class: 'big' }, el('span', { id: `${id}` }, '—'), el('small', {}, unit)),
    el('div', { class: 'foot', id: `${id}-foot` }, foot));
  box.append(
    kpi('ek-v', 'zap', 'Tensión', 'V'),
    kpi('ek-i', 'activity', 'Corriente', 'mA'),
    kpi('ek-p', 'flame', 'Potencia', 'W'),
    kpi('ek-used', 'battery', 'Consumido', 'mAh'),
    kpi('ek-wh', 'sigma', 'Energía', 'Wh', 'desde que se abrió la sesión'),
    kpi('ek-iavg', 'trending-up', 'Corriente media', 'mA', 'últimos ~30 s'),
    kpi('ek-peak', 'arrow-up-to-line', 'Pico de corriente', 'mA', 'arranques de los motores'),
    kpi('ek-vmin', 'arrow-down-to-line', 'Tensión mínima', 'V', 'la mayor caída bajo carga'),
  );
}

function buildCharts() {
  const box = $('#en-charts');
  const card = (title, sub, icon) => {
    const c = el('div', { class: 'card c4 chart-card' },
      el('div', { class: 'card-h' }, el('div', { class: 'ico' }, el('i', { 'data-lucide': icon })),
        el('div', {}, el('h2', {}, title), el('div', { class: 'sub' }, sub))));
    const chart = el('div', { class: 'chart' });
    c.append(chart);
    box.append(c);
    return chart;
  };
  const mk = (node, label, color, unit, decimals, range) => new LiveChart(node, {
    series: [{ label, color, width: 2 }], height: 190, window: windowS, decimals, xFmt, unit, gap: 3, range,
  });
  charts = {
    v: mk(card('Tensión', 'V de la alimentación del sistema', 'zap'), 'Tensión', 'ekf', 'V', 2, { minSpan: 0.3 }),
    i: mk(card('Corriente', 'mA que consume el sistema', 'activity'), 'Corriente', 'ukf', 'mA', 0, { zero: true, minMax: 50 }),
    p: mk(card('Potencia', 'W = V × I', 'flame'), 'Potencia', 'cmd', 'W', 2, { zero: true, minMax: 0.5 }),
  };
  icons();
}

// Historial del servidor (ya incluye hasta "ahora"): se carga una vez al
// iniciar y recien entonces se agregan puntos en vivo, para que el tiempo
// de las graficas vaya siempre en orden.
async function loadHistory() {
  try {
    const { points } = await api('api/battery/history');
    const nowWall = Date.now() / 1000, nowPerf = performance.now() / 1000;
    for (const c of Object.values(charts)) c.clear();
    for (const [t, v, i, p] of points) {
      const tr = nowPerf - (nowWall - t);
      charts.v.push(tr, [v]); charts.i.push(tr, [i]); charts.p.push(tr, [p]);
    }
    lastPush = nowPerf;
  } catch (e) { /* sin historial */ }
  histLoaded = true;
}

// ---------------------------------------------------------------------
// Bateria grande: nivel con superficie ondulada (animada)
let waveT = 0;
let shownSoc = null;
function drawBattery(B) {
  const cls = levelClass(B);
  const target = (B && B.soc !== null && B.soc !== undefined && cls !== 'none') ? B.soc : 0;
  shownSoc = shownSoc === null ? target : shownSoc + (target - shownSoc) * 0.2;
  waveT += 0.06;
  const top = 22, h = 218, x0 = 10, w = 120;
  const y = top + h * (1 - shownSoc / 100);
  const amp = shownSoc > 0.5 && shownSoc < 99.5 ? 3.2 : 0;
  const wave = (phase, a) => {
    let d = `M ${x0 - 2} ${top + h + 2} L ${x0 - 2} ${y}`;
    for (let x = 0; x <= w + 4; x += 6) {
      d += ` L ${x0 - 2 + x} ${(y + a * Math.sin((x / w) * Math.PI * 2.2 + waveT + phase)).toFixed(2)}`;
    }
    return `${d} L ${x0 + w + 2} ${top + h + 2} Z`;
  };
  $('#en-wave').setAttribute('d', wave(0, amp));
  $('#en-wave2').setAttribute('d', wave(2.1, amp * 0.8));
  $('#en-batt').dataset.level = cls;
}

function setStatus(B) {
  const st = (B && STATUS[B.status]) || STATUS.sin_datos;
  const key = B ? B.status : 'sin_datos';
  if (key !== lastStatus) {
    lastStatus = key;
    const s = $('#en-status');
    s.className = `en-status ${st.cls}`;
    s.innerHTML = `<i data-lucide="${st.icon}"></i><span>${st.label}</span>`;
    icons();
    const alert = $('#en-alert');
    if (key === 'baja' || key === 'critica') {
      alert.className = `energy-alert show ${key === 'critica' ? 'bad' : 'warn'}`;
      alert.querySelector('span').textContent = key === 'critica'
        ? 'Alimentación crítica: los motores pueden perder fuerza o el sistema apagarse. Carga las baterías Beston 9 V.'
        : 'Alimentación baja: queda poca autonomía. Ten lista la carga.';
    } else {
      alert.className = 'energy-alert';
    }
  }
}

function updatePill(B) {
  const pill = $('#batt-pill');
  const cls = levelClass(B);
  const ok = cls !== 'none';
  pill.className = `batt-pill ${ok ? cls : 'nodata'}`;
  const pct = ok ? Math.round(B.soc) : null;
  $('#batt-pill-pct').textContent = ok ? `${pct}%` : '—';
  const nb = $('#nb-energy');
  if (nb) { nb.textContent = ok ? `${pct}%` : ''; nb.className = `nav-badge ${ok ? cls : ''}`; }
  const full = 21.8;
  $('#batt-pill-fill').setAttribute('width', ok ? Math.max(1.5, (full * B.soc) / 100).toFixed(2) : 0);
  pill.title = ok
    ? `Alimentación del sistema: ${pct}% · ${fmt(B.v, 2)} V · ${fmt(B.i_ma, 0)} mA${B.runtime_min ? ` · ~${Math.round(B.runtime_min)} min` : ''} · clic para ver Energía`
    : `Alimentación del sistema: ${(STATUS[B && B.status] || STATUS.sin_datos).label} · clic para ver Energía`;
}

const txt = (id, v) => { const n = document.getElementById(id); if (n && n.textContent !== v) n.textContent = v; };

function fmtMin(m) {
  if (m === null || m === undefined || !Number.isFinite(m)) return null;
  if (m >= 600) return '> 10 h';
  if (m >= 60) return `${Math.floor(m / 60)} h ${String(Math.round(m % 60)).padStart(2, '0')} min`;
  return `${Math.max(0, Math.round(m))} min`;
}

// Se llama en cada mensaje del servidor (desde app.js)
export function tick(S) {
  const B = S.battery;
  updatePill(B);
  const live = B && B.status !== 'sin_datos' && B.status !== 'desconectada';
  // Graficas: un punto por muestra del INA219 (~10 Hz); el dibujo va en frame()
  const t = performance.now() / 1000;
  if (live && histLoaded && t - lastPush > 0.09) {
    lastPush = t;
    charts.v.push(t, [B.v]); charts.i.push(t, [B.i_ma]); charts.p.push(t, [B.p_w]);
  }
  if (ctxRef.view !== 'energy') return;

  drawBattery(B);
  setStatus(B);
  const ok = levelClass(B) !== 'none';
  txt('en-pct', ok ? `${Math.round(B.soc)}` : '—');
  const rt = ok ? fmtMin(B.runtime_min) : null;
  txt('en-runtime', rt || (ok ? 'en reposo' : '—'));
  txt('en-runtime-foot', ok && B.i_avg_ma ? `con ${fmt(B.i_avg_ma, 0)} mA de consumo medio (últimos ~30 s)` : 'con el consumo medio de los últimos 30 s');
  txt('en-remaining', ok ? `${fmt(B.remaining_mah, 0)} / ${fmt(B.capacity_mah, 0)} mAh` : '—');
  $('#en-remaining-bar').style.width = ok ? `${B.soc.toFixed(1)}%` : '0%';
  $('#en-remaining-bar').dataset.level = levelClass(B);

  txt('ek-v', live ? fmt(B.v, 2) : '—');
  txt('ek-v-foot', live ? `${fmt(B.v_cell, 3)} V por celda · ${B.cells} celdas Ni-MH` : '');
  txt('ek-i', live ? fmt(B.i_ma, 0) : '—');
  txt('ek-i-foot', live ? (B.i_ma < 15 ? 'motores parados' : 'motores en marcha') : '');
  txt('ek-p', live ? fmt(B.p_w, 2) : '—');
  txt('ek-p-foot', live ? `${fmt(B.p_w * 1000, 0)} mW` : '');
  txt('ek-used', live && B.used_mah !== null ? fmt(B.used_mah, 1) : '—');
  txt('ek-used-foot', live && B.used_mah !== null ? `${fmt((B.used_mah / B.capacity_mah) * 100, 1)} % de ${fmt(B.capacity_mah, 0)} mAh en esta sesión` : '');
  txt('ek-wh', live ? fmt(B.energy_wh, 3) : '—');
  txt('ek-iavg', live && B.i_avg_ma !== null ? fmt(B.i_avg_ma, 0) : '—');
  txt('ek-peak', live ? fmt(B.peak_ma, 0) : '—');
  txt('ek-vmin', live && B.v_min !== null ? fmt(B.v_min, 2) : '—');

  // Enlace con el INA219
  const L = S.links && S.links.battery;
  const chip = $('#en-link');
  const age = L ? L.age : null;
  chip.querySelector('.dot').className = `dot ${age === null ? '' : age < 1.5 ? 'ok' : age < 5 ? 'warn' : 'bad'}`;
  chip.querySelector('.hz').textContent = L && L.hz ? `${fmt(L.hz, 1)} Hz` : '—';

  renderSpec(B);
  updateRange(B, live);
}

// Las graficas se redibujan a ~30 cuadros/s aunque no llegue un punto nuevo:
// asi el eje de tiempo se desplaza continuo en vez de a saltos
let lastFrame = 0;
function frame(ms) {
  requestAnimationFrame(frame);
  if (!ctxRef || ctxRef.view !== 'energy' || ms - lastFrame < 33) return;
  lastFrame = ms;
  const t = performance.now() / 1000;
  for (const c of Object.values(charts)) { c.dirty = true; c.draw(t); }
}

let specDone = false;
function renderSpec(B) {
  if (specDone || !B) return;
  specDone = true;
  const packs = B.packs || 2;
  const tiles = [
    ['battery-full', 'Batería', 'Beston BST-9V', `${packs} × recargables Ni-MH`],
    ['git-merge', 'Conexión', 'En paralelo', `misma tensión, ${packs}× capacidad`],
    ['database', 'Capacidad', `${fmt(B.capacity_mah, 0)} mAh`, `${packs} × ${fmt(B.capacity_mah / packs, 0)} mAh`],
    ['layers', 'Celdas', `${B.cells} × 1.2 V`, '8.4 V nominal por batería'],
    ['omega', 'Resistencia interna', `≈ ${fmt(B.r_int, 1)} Ω`, 'la mitad de una sola'],
    ['cpu', 'Alimenta', 'Puente H DRV8833', 'dos motorreductores TT'],
    ['scan-line', 'Sensor', 'INA219 · 0x40', 'shunt 0.1 Ω · hasta 3.2 A'],
    ['radio-tower', 'Tópico ROS 2', 'battery/state', 'BatteryState · 10 Hz'],
  ];
  $('#en-spec').append(...tiles.map(([ic, k, v, sub]) => el('div', { class: 'en-tile' },
    el('span', { class: 'en-tile-ico' }, el('i', { 'data-lucide': ic })),
    el('div', { class: 'en-tile-txt' }, el('span', { class: 'k' }, k), el('b', {}, v), el('small', {}, sub)))));
  icons();
}

// Medidor de medio arco: tension actual en el rango 7.0 (vacia) - 9.8 V (cargada)
const GV0 = 7.0, GV1 = 9.8, GCX = 140, GCY = 150, GR = 112;
const gpt = (f, r = GR) => {               // f en [0,1] -> punto del arco (izquierda -> derecha)
  const a = Math.PI * (1 - f);
  return [GCX + r * Math.cos(a), GCY - r * Math.sin(a)];
};
const garc = (f0, f1, r = GR) => {
  const [x0, y0] = gpt(f0, r), [x1, y1] = gpt(f1, r);
  return `M ${x0.toFixed(2)} ${y0.toFixed(2)} A ${r} ${r} 0 0 1 ${x1.toFixed(2)} ${y1.toFixed(2)}`;
};
const fOfV = (v) => Math.max(0, Math.min(1, (v - GV0) / (GV1 - GV0)));
const ZONES = [[0, 0.08, 'bad'], [0.08, 0.22, 'warn'], [0.22, 1, 'ok']];   // < 7.2 V, < 7.6 V, normal
let gaugeBuilt = false;
function buildGauge() {
  if (gaugeBuilt) return;
  gaugeBuilt = true;
  $('#eg-track').setAttribute('d', garc(0, 1));
  const zr = GR + 17, gap = 0.008;
  $('#eg-zones').innerHTML = ZONES.map(([a, b, c], k) =>
    `<path d="${garc(a + (k ? gap : 0), b - (k < 2 ? gap : 0), zr)}" class="z-${c}"/>`).join('');
  const ticks = [];
  for (const v of [7.0, 8.4, 9.8]) {
    const f = fOfV(v);
    const [x0, y0] = gpt(f, zr + 5), [x1, y1] = gpt(f, zr + 11);
    ticks.push(`<line x1="${x0.toFixed(1)}" y1="${y0.toFixed(1)}" x2="${x1.toFixed(1)}" y2="${y1.toFixed(1)}"/>`);
  }
  // etiquetas: extremos bajo el arco y el nominal arriba
  const [lx, ly] = gpt(0, GR), [rx, ry] = gpt(1, GR);
  ticks.push(`<text x="${lx.toFixed(1)}" y="${(ly + 26).toFixed(1)}" text-anchor="middle">7.0</text>`);
  ticks.push(`<text x="${rx.toFixed(1)}" y="${(ry + 26).toFixed(1)}" text-anchor="middle">9.8</text>`);
  const [nx, ny] = gpt(0.5, zr + 18);
  ticks.push(`<text x="${nx.toFixed(1)}" y="${(ny + 1).toFixed(1)}" text-anchor="middle" class="nom">8.4 V nominal</text>`);
  $('#eg-ticks').innerHTML = ticks.join('');
}

let gaugeF = null;
function updateRange(B, live) {
  buildGauge();
  const g = $('#en-gauge');
  const knob = $('#eg-knob');
  if (!live || !B) {
    txt('en-range-v', '—'); txt('en-range-cell', 'sin datos del INA219'); txt('en-range-state', 'sin datos');
    g.dataset.level = 'none';
    $('#eg-prog').setAttribute('d', '');
    knob.style.opacity = 0;
    gaugeF = null;
    return;
  }
  const f = fOfV(B.v);
  gaugeF = gaugeF === null ? f : gaugeF + (f - gaugeF) * 0.3;      // avance suave pero rapido
  const ff = Math.max(gaugeF, 0.001);
  $('#eg-prog').setAttribute('d', garc(0, ff));
  const [kx, ky] = gpt(ff);
  knob.setAttribute('transform', `translate(${kx.toFixed(2)} ${ky.toFixed(2)})`);
  knob.style.opacity = 1;
  // el color del medidor sigue la zona de tension que se muestra
  g.dataset.level = f >= 0.22 ? 'ok' : f >= 0.08 ? 'warn' : 'bad';
  txt('en-range-v', fmt(B.v, 2));
  txt('en-range-cell', `${fmt(B.v_cell, 3)} V por celda · ${B.cells} celdas`);
  txt('en-range-state', f >= 0.22 ? 'Rango normal' : f >= 0.08 ? 'Tensión baja' : 'Casi vacía');
}

export function onShow(name) {
  if (name === 'energy' && charts) for (const c of Object.values(charts)) { c.resize(); c.dirty = true; }
}

export function onTheme() {
  if (charts) for (const c of Object.values(charts)) c.build();
}
