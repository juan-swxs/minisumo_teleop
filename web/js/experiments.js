// Vista "Pruebas": tablas de la guia (posicion esperada / medida / estimada).

import {
  $, api, toast, fmt, theme, icons, segmented, el, fitCanvas, planeView, drawPlaneGrid, drawDot,
} from './ui.js';
import * as targets from './targets.js';

const COND = {
  base: { name: 'Sin filtrado', src: 'odom', desc: 'Señales crudas · posición estimada = odometría del ESP32' },
  filtrado: { name: 'Filtrado', src: 'odom', desc: 'Filtros de bajo nivel activos · estimada = odometría del ESP32' },
  ekf: { name: 'EKF', src: 'ekf', desc: 'Filtro de Kalman extendido · estimada = EKF' },
  ukf: { name: 'UKF', src: 'ukf', desc: 'Filtro de Kalman sin esencia · estimada = UKF' },
};
let rows = [];
let strategy = 2;
let cond = 'base';
let ctxRef = null;

const dist = (a, b) => (a && b ? Math.hypot(a.x - b.x, a.y - b.y) : null);
const estOf = (r) => (r.estimates ? r.estimates[COND[r.condition].src] : null);

function errClass(e) {
  if (e === null) return '';
  return e < 0.03 ? 'good' : e < 0.08 ? 'mid' : 'badc';
}

async function load() {
  try { rows = (await api('api/experiments')).rows; } catch (e) { rows = []; }
  render();
  updatePins();   // el dojo siempre muestra lo que hay en la tabla
}

// Destinos quitados del dojo con "Reiniciar" (por id de fila: las filas
// siguen en la tabla de Pruebas; un punto nuevo tiene otro id y si aparece)
const HIDDEN_KEY = 'hiddenTargets';
let hidden = new Set();
try { hidden = new Set(JSON.parse(localStorage.getItem(HIDDEN_KEY) || '[]')); } catch (e) { /* sin almacenamiento */ }
function saveHidden() {
  // solo se guardan ids que siguen existiendo
  const ids = new Set(rows.map((r) => r.id));
  hidden = new Set([...hidden].filter((id) => ids.has(id)));
  try { localStorage.setItem(HIDDEN_KEY, JSON.stringify([...hidden])); } catch (e) { /* sin almacenamiento */ }
}

// Lo llama el boton Reiniciar: quita todos los destinos del dojo
export async function clearTargets() {
  try { rows = (await api('api/experiments')).rows; } catch (e) { /* usa las filas que hay */ }
  rows.forEach((r) => hidden.add(r.id));
  saveHidden();
  updatePins();
}

function current() { return rows.filter((r) => r.strategy === strategy && r.condition === cond); }

function render() {
  const list = current();
  $('#exp-title').textContent = `Estrategia ${strategy} · ${COND[cond].name}`;
  $('#exp-sub').textContent = COND[cond].desc;
  const box = $('#exp-table');
  box.innerHTML = '';
  if (!list.length) {
    box.append(el('div', { class: 'empty', html: '<i data-lucide="map-pin"></i>Agrega los puntos del dojo a los que llevarás el robot' }));
  } else {
    const t = el('table', { class: 'data' });
    t.innerHTML = `<thead>
      <tr><th rowspan="2">Punto</th><th class="grp" colspan="2">Esperada</th><th class="grp" colspan="2">Medida</th>
      <th class="grp" colspan="2">Estimada (${COND[cond].src.toUpperCase()})</th><th class="grp" colspan="2">Error (m)</th><th rowspan="2" class="bl"></th></tr>
      <tr><th class="bl">x</th><th>y</th><th class="bl">x</th><th>y</th><th class="bl">x</th><th>y</th><th class="bl">esp–med</th><th>med–est</th></tr></thead>`;
    const tb = el('tbody');
    for (const r of list) {
      const est = estOf(r);
      const eEM = dist(r.expected, r.measured), eME = dist(r.measured, est);
      const mx = el('input', { class: 'input xs', type: 'number', step: '0.005', value: r.measured ? r.measured.x : '', placeholder: 'x' });
      const my = el('input', { class: 'input xs', type: 'number', step: '0.005', value: r.measured ? r.measured.y : '', placeholder: 'y' });
      const save = async () => {
        if ((mx.value === '') !== (my.value === '')) return;
        try { await api('api/experiments/update', { id: r.id, mx: mx.value, my: my.value }); await load(); } catch (e) { toast(e.message, 'err'); }
      };
      mx.addEventListener('change', save);
      my.addEventListener('change', save);
      const tr = el('tr', {},
        el('td', {}, el('b', {}, r.point || '—')),
        el('td', { class: 'n bl' }, fmt(r.expected.x, 3)), el('td', { class: 'n' }, fmt(r.expected.y, 3)),
        el('td', { class: 'bl' }, mx), el('td', {}, my),
        el('td', { class: 'n bl' }, est ? fmt(est.x, 3) : '—'), el('td', { class: 'n' }, est ? fmt(est.y, 3) : '—'),
        el('td', { class: `n bl err ${errClass(eEM)}` }, fmt(eEM, 3)), el('td', { class: `n err ${errClass(eME)}` }, fmt(eME, 3)),
        el('td', { class: 'bl' },
          el('div', { class: 'row', style: 'gap:6px' },
            el('button', {
              class: `btn sm ${est ? '' : 'primary'}`, title: 'Guardar la estimación actual del robot',
              onclick: async () => {
                try { await api('api/experiments/capture', { id: r.id }); await load(); toast(`Estimación capturada en ${r.point}`); } catch (e) { toast(e.message, 'err'); }
              },
            }, el('i', { 'data-lucide': 'scan-line' }), est ? 'Recapturar' : 'Capturar'),
            el('button', {
              class: 'btn sm ghost', title: 'Eliminar',
              onclick: async () => {
                try { await api('api/experiments/delete', { id: r.id }); } catch (e) { toast(e.message, 'err'); }
                await load();
              },
            }, el('i', { 'data-lucide': 'x' })))));
      tb.append(tr);
    }
    t.append(tb);
    box.append(el('div', { class: 'table-wrap' }, t));
  }
  renderCopyOptions();
  renderSummary();
  drawMap();
  icons();
}

function renderCopyOptions() {
  const sel = $('#exp-copy-from');
  const opts = ['<option value="">Copiar puntos de…</option>'];
  for (const s of [1, 2]) {
    for (const c of Object.keys(COND)) {
      if (s === strategy && c === cond) continue;
      const n = rows.filter((r) => r.strategy === s && r.condition === c).length;
      if (n) opts.push(`<option value="${s}|${c}">E${s} · ${COND[c].name} (${n})</option>`);
    }
  }
  sel.innerHTML = opts.join('');
}

function mean(a) { const v = a.filter((x) => x !== null); return v.length ? v.reduce((s, x) => s + x, 0) / v.length : null; }

function renderSummary() {
  const th = theme();
  const stats = Object.keys(COND).map((c) => {
    const rs = rows.filter((r) => r.strategy === strategy && r.condition === c);
    return {
      c, n: rs.length,
      em: mean(rs.map((r) => dist(r.expected, r.measured))),
      me: mean(rs.map((r) => dist(r.measured, estOf(r)))),
    };
  });
  const max = Math.max(0.01, ...stats.flatMap((s) => [s.em || 0, s.me || 0]));
  $('#exp-summary').innerHTML = `
    <div class="row" style="gap:14px;margin-bottom:10px;font-size:12px">
      <span class="row" style="gap:6px"><span class="pill-tag" style="background:${th.exp}"></span>esperada–medida</span>
      <span class="row" style="gap:6px"><span class="pill-tag" style="background:${th.ekf}"></span>medida–estimada</span></div>
    <div class="bars">${stats.map((s) => `
      <div class="bar-row" style="grid-template-columns:96px 1fr 70px"><span>${COND[s.c].name}<span class="muted"> · ${s.n}</span></span>
        <div class="stack" style="gap:4px">
          <div class="track"><div class="fill" style="width:${((s.em || 0) / max) * 100}%;background:${th.exp}"></div></div>
          <div class="track"><div class="fill" style="width:${((s.me || 0) / max) * 100}%;background:${th.ekf}"></div></div></div>
        <span class="v" style="line-height:1.25">${s.em !== null ? (s.em * 100).toFixed(1) + ' cm' : '—'}<br>${s.me !== null ? (s.me * 100).toFixed(1) + ' cm' : '—'}</span></div>`).join('')}
    </div>`;
}

function drawMap() {
  const cv = $('#exp-map');
  if (cv.offsetParent === null) return;
  const { ctx: g, w, h } = fitCanvas(cv);
  const th = theme();
  g.clearRect(0, 0, w, h);
  const R = (ctxRef && ctxRef.S && ctxRef.S.remote.glove_teleop.dojo_radius) || 2.5;
  const view = planeView(w, h, 2 * R + 0.1);
  drawPlaneGrid(g, w, h, view, R > 1 ? 0.5 : 0.1, th);
  const [cx, cy] = view.px(0, 0);
  g.strokeStyle = th.text3; g.lineWidth = 2;
  g.beginPath(); g.arc(cx, cy, R * view.s, 0, 7); g.stroke();
  const srcCol = { odom: th.odom, ekf: th.ekf, ukf: th.ukf };
  for (const r of current()) {
    const est = estOf(r);
    const [ex, ey] = view.px(r.expected.x, r.expected.y);
    g.setLineDash([3, 3]); g.strokeStyle = th.text3; g.lineWidth = 1;
    for (const p of [r.measured, est]) {
      if (!p) continue;
      const [x, y] = view.px(p.x, p.y);
      g.beginPath(); g.moveTo(ex, ey); g.lineTo(x, y); g.stroke();
    }
    g.setLineDash([]);
    g.strokeStyle = th.exp; g.lineWidth = 2.2;
    g.beginPath(); g.arc(ex, ey, 7, 0, 7); g.stroke();
    if (r.measured) drawDot(g, view, r.measured.x, r.measured.y, th.meas, 4, false);
    if (est) drawDot(g, view, est.x, est.y, srcCol[COND[r.condition].src], 4, false);
    g.fillStyle = th.text2; g.font = '600 11px Inter, sans-serif';
    g.fillText(r.point, ex + 10, ey - 8);
  }
}

export function init(ctx) {
  ctxRef = ctx;
  segmented($('#seg-exp-strategy'), (v) => { strategy = +v; render(); updatePins(); });
  segmented($('#seg-exp-cond'), (v) => { cond = v; render(); updatePins(); });
  $('#exp-add').addEventListener('click', async () => {
    const x = parseFloat($('#exp-x').value), y = parseFloat($('#exp-y').value);
    if (!Number.isFinite(x) || !Number.isFinite(y)) { toast('Escribe x e y en metros', 'err'); return; }
    const point = $('#exp-point').value.trim() || `P${current().length + 1}`;
    await api('api/experiments/add', { strategy, condition: cond, point, ex: x, ey: y });
    $('#exp-point').value = ''; $('#exp-x').value = ''; $('#exp-y').value = '';
    await load();
    updatePins();
  });
  $('#exp-copy-from').addEventListener('change', async (e) => {
    if (!e.target.value) return;
    const [s, c] = e.target.value.split('|');
    const src = rows.filter((r) => r.strategy === +s && r.condition === c);
    for (const r of src) await api('api/experiments/add', { strategy, condition: cond, point: r.point, ex: r.expected.x, ey: r.expected.y });
    toast(`${src.length} puntos copiados`);
    await load();
    updatePins();
  });
  $('#exp-clear').addEventListener('click', async () => {
    if (!current().length || !confirm(`¿Vaciar la tabla Estrategia ${strategy} · ${COND[cond].name}?`)) return;
    await api('api/experiments/clear', { strategy, condition: cond });
    await load();
  });
  load().then(updatePins);
}

function updatePins() {
  if (!ctxRef || !ctxRef.dojo) return;
  // Destinos = filas de la tabla seleccionada (estrategia + condicion);
  // al borrar una fila o cambiar de tabla, el dojo se actualiza solo
  const pts = current()
    .filter((r) => !hidden.has(r.id))
    .map((r) => ({ x: r.expected.x, y: r.expected.y, label: r.point || 'Destino', id: r.id }));
  targets.setTargets(pts);
}

// Destino colocado con un clic en el dojo 3D
export async function addTarget(x, y) {
  x = Math.round(x * 100) / 100; y = Math.round(y * 100) / 100;
  const used = new Set(current().map((r) => r.point));
  let n = current().length + 1;
  while (used.has(`P${n}`)) n++;
  const point = `P${n}`;
  await api('api/experiments/add', { strategy, condition: cond, point, ex: x, ey: y });
  await load();
  return point;
}

export function onShow(name) {
  if (name === 'experiments') load();
}

export function update() { /* la tabla solo cambia por acciones del usuario */ }
