// Vista "Offline": re-ejecuta EKF y UKF sobre sesiones grabadas.

import {
  $, api, toast, fmt, theme, icons, segmented, el, fitCanvas, planeView, drawPlaneGrid, drawTrail, drawDot,
} from './ui.js';

let selected = null;
let robotMode = 'raw';
let handMode = 'filtered';
let last = null;

export async function refresh() {
  const box = $('#sessions');
  let list = [];
  try { list = (await api('api/sessions')).sessions; } catch (e) { /* servidor no disponible */ }
  box.innerHTML = '';
  if (!list.length) {
    box.append(el('div', { class: 'empty', html: '<i data-lucide="inbox"></i>Aún no hay sesiones grabadas' }));
  }
  for (const s of list) {
    const b = el('button', { class: s.name === selected ? 'on' : '' },
      el('i', { 'data-lucide': 'file-chart-column' }), el('span', {}, s.name), el('span', { class: 'meta' }, `${s.kb} KB`));
    b.addEventListener('click', () => {
      selected = s.name;
      [...box.children].forEach((x) => x.classList.toggle('on', x === b));
      $('#btn-offline').disabled = false;
      $('#off-sub').textContent = s.name;
    });
    box.append(b);
  }
  icons();
}

function plotCanvas(title, height = 320) {
  const cv = el('canvas', { class: 'plot2d', style: `height:${height}px` });
  return { node: el('div', { class: 'stack', style: 'gap:8px' }, el('div', { class: 'label' }, title), cv), cv };
}

function drawTraj(cv, sets, span, circleR) {
  const { ctx: g, w, h } = fitCanvas(cv);
  const th = theme();
  g.clearRect(0, 0, w, h);
  const view = planeView(w, h, span);
  drawPlaneGrid(g, w, h, view, span > 2 ? 0.5 : 0.1, th);
  if (circleR) {
    const [cx, cy] = view.px(0, 0);
    g.strokeStyle = th.text3; g.lineWidth = 2;
    g.beginPath(); g.arc(cx, cy, circleR * view.s, 0, 7); g.stroke();
  }
  for (const s of sets) {
    if (!s.pts || !s.pts.length) continue;
    drawTrail(g, view, s.pts, th[s.c], s.w || 2.2, s.a || 1);
    const p = s.pts[s.pts.length - 1];
    drawDot(g, view, p[0], p[1], th[s.c], 4.5);
  }
}

function render(res) {
  last = res;
  const th = theme();
  const out = $('#off-out');
  out.innerHTML = '';
  const r = res.robot, h = res.hand;
  const grid = el('div', { class: 'grid' });
  const rp = plotCanvas('Robot · trayectoria re-estimada');
  const hp = plotCanvas('Mano · trayectoria re-estimada');
  grid.append(el('div', { class: 'c6' }, rp.node), el('div', { class: 'c6' }, hp.node));

  const rowsR = r ? Object.entries(r.filters).map(([n, f]) => `
    <tr><td><b style="color:${n === 'EKF' ? th.ekf : th.ukf}">${n}</b></td><td class="n">${fmt(f.final.x, 3)}, ${fmt(f.final.y, 3)}</td>
    <td class="n">${fmt(f.path_m, 3)}</td><td class="n">${fmt(f.us_mean, 0)}</td><td class="n">${fmt(f.bias, 4)}</td>
    <td class="n">${f.vs_car_final_m !== null ? fmt(f.vs_car_final_m * 100, 1) + ' cm' : '—'}</td></tr>`).join('') : '';
  const carRow = r && r.car.final ? `<tr><td><b style="color:${th.odom}">ESP32</b></td><td class="n">${fmt(r.car.final.x, 3)}, ${fmt(r.car.final.y, 3)}</td>
    <td class="n">${fmt(r.car.path_m, 3)}</td><td class="n">—</td><td class="n">—</td><td class="n">—</td></tr>` : '';
  const rowsH = h ? Object.entries(h.filters).map(([n, f]) => `
    <tr><td><b style="color:${n === 'EKF' ? th.ekf : th.ukf}">${n}</b></td><td class="n">${fmt(f.path_m, 3)}</td>
    <td class="n">${fmt(f.v_peak, 3)}</td><td class="n">${fmt(f.us_mean, 0)}</td></tr>`).join('') : '';

  grid.append(el('div', {
    class: 'c12', html: `
    <div class="grid">
      <div class="c12"><div class="label" style="margin-bottom:8px">Robot · giroscopio ${r ? (r.mode === 'raw' ? 'crudo' : 'filtrado') : ''} · ${r ? r.samples : 0} pasos</div>
        ${r ? `<div class="table-wrap"><table class="data"><thead><tr><th>Fuente</th><th>Final x, y (m)</th><th>Recorrido (m)</th><th>µs/paso</th><th>Sesgo (rad/s)</th><th>Dif. con ESP32</th></tr></thead>
        <tbody>${carRow}${rowsR}</tbody></table></div>` : '<div class="empty">Sin datos del carro en esta sesión</div>'}</div>
      <div class="c12"><div class="label" style="margin-bottom:8px">Mano · aceleración ${h ? (h.mode === 'raw' ? 'cruda' : 'filtrada') : ''} · ${h ? h.samples : 0} pasos</div>
        ${h ? `<div class="table-wrap"><table class="data"><thead><tr><th>Filtro</th><th>Recorrido (m)</th><th>v pico (m/s)</th><th>µs/paso</th></tr></thead>
        <tbody>${rowsH}</tbody></table></div>` : '<div class="empty">Sin datos del guante en esta sesión</div>'}</div>
    </div>`,
  }));
  out.append(grid);
  requestAnimationFrame(() => {
    if (r) drawTraj(rp.cv, [{ pts: r.car.traj, c: 'odom', w: 1.6, a: 0.8 }, { pts: r.filters.EKF && r.filters.EKF.traj, c: 'ekf' }, { pts: r.filters.UKF && r.filters.UKF.traj, c: 'ukf', a: 0.85 }], 5.2, 2.5);
    if (h) drawTraj(hp.cv, [{ pts: h.filters.EKF && h.filters.EKF.traj, c: 'ekf' }, { pts: h.filters.UKF && h.filters.UKF.traj, c: 'ukf', a: 0.85 }], 0.8);
  });
}

export function init() {
  segmented($('#seg-off-robot'), (v) => { robotMode = v; });
  segmented($('#seg-off-hand'), (v) => { handMode = v; });
  $('#btn-sessions').addEventListener('click', refresh);
  $('#btn-offline').addEventListener('click', async () => {
    if (!selected) return;
    const b = $('#btn-offline');
    b.disabled = true;
    try {
      render(await api('api/offline', { session: selected, robot_mode: robotMode, hand_mode: handMode }));
    } catch (e) { toast(`Offline: ${e.message}`, 'err'); } finally { b.disabled = false; }
  });
  refresh();
}

export function onShow(name) {
  if (name === 'offline') { refresh(); if (last) render(last); }
}

export function update() { /* sin datos en vivo */ }
