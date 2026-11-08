// Vista "Kalman": EKF vs UKF en vivo + benchmark con verdad conocida.

import {
  $, api, toast, fmt, theme, icons, fitCanvas, planeView, drawPlaneGrid, drawTrail, drawDot, Ring,
} from './ui.js';
import { chartCard } from './charts.js';

const trails = {
  robot: { EKF: new Ring(1500), UKF: new Ring(1500), odom: new Ring(1500) },
  hand: { EKF: new Ring(800), UKF: new Ring(800) },
};
let charts = {};

export function clearTrails() {
  Object.values(trails).forEach((g) => Object.values(g).forEach((r) => r.clear()));
}

function kpi(label, color, value, unit, foot) {
  return `<div class="kpi"><div class="label"><span class="pill-tag" style="background:${color}"></span>${label}</div>
    <div class="big">${value}<small>${unit}</small></div><div class="foot">${foot}</div></div>`;
}

export function init() {
  const grid = $('#kalman-charts');
  charts.timing = chartCard(grid, {
    title: 'Tiempo de cómputo por paso', sub: 'predicción + correcciones · µs', icon: 'timer', span: 'c6', decimals: 0,
    series: [
      { label: 'robot EKF', color: 'ekf', width: 2 }, { label: 'robot UKF', color: 'ukf', width: 2 },
      { label: 'mano EKF', color: 'ekf', width: 1.4, dash: [5, 4] }, { label: 'mano UKF', color: 'ukf', width: 1.4, dash: [5, 4] },
    ],
  });
  charts.theta = chartCard(grid, {
    title: 'Rumbo del robot θ', sub: 'grados', icon: 'compass', span: 'c6', decimals: 1,
    series: [{ label: 'odometría', color: 'odom', width: 1.4 }, { label: 'EKF', color: 'ekf', width: 2 }, { label: 'UKF', color: 'ukf', width: 2, dash: [6, 4] }],
  });
  charts.handv = chartCard(grid, {
    title: 'Velocidad de la mano', sub: 'variable no medida · m/s', icon: 'move', span: 'c6', decimals: 3,
    series: [
      { label: 'vx EKF', color: 'ekf', width: 2 }, { label: 'vx UKF', color: 'ukf', width: 2, dash: [6, 4] },
      { label: 'vy EKF', color: 'meas', width: 1.6 }, { label: 'vy UKF', color: 'exp', width: 1.6, dash: [6, 4] },
    ],
  });
  charts.trace = chartCard(grid, {
    title: 'Incertidumbre de la posición del robot', sub: 'traza de P (x, y, θ) · crece sin tracking externo', icon: 'scan', span: 'c6', decimals: 5,
    series: [{ label: 'EKF', color: 'ekf', width: 2 }, { label: 'UKF', color: 'ukf', width: 2, dash: [6, 4] }],
  });
  icons();

  $('#btn-clear-traj').addEventListener('click', clearTrails);
  $('#btn-bench').addEventListener('click', runBench);
}

function drawRobotTraj(S) {
  const { ctx: g, w, h } = fitCanvas($('#robot-traj'));
  const th = theme();
  g.clearRect(0, 0, w, h);
  const R = (S.remote.glove_teleop && S.remote.glove_teleop.dojo_radius) || 2.5;
  const view = planeView(w, h, 2 * R + 0.12);
  const step = R > 1 ? 0.5 : 0.1;
  drawPlaneGrid(g, w, h, view, step, th);
  const [cx, cy] = view.px(0, 0);
  g.save();
  g.strokeStyle = th.text3; g.lineWidth = 2;
  g.beginPath(); g.arc(cx, cy, R * view.s, 0, 7); g.stroke();
  g.restore();
  const col = { EKF: th.ekf, UKF: th.ukf, odom: th.odom };
  drawTrail(g, view, trails.robot.odom.a, col.odom, 1.6, 0.8);
  drawTrail(g, view, trails.robot.EKF.a, col.EKF, 2.2);
  drawTrail(g, view, trails.robot.UKF.a, col.UKF, 2.2, 0.85);
  const cur = { EKF: S.robot.EKF, UKF: S.robot.UKF, odom: S.links.car.age !== null ? S.car.odom : null };
  for (const k of ['odom', 'EKF', 'UKF']) if (cur[k]) drawDot(g, view, cur[k].x, cur[k].y, col[k], 4.5);
  g.font = '11.5px Inter, sans-serif'; g.fillStyle = th.text3;
  g.fillText(`rejilla ${step * 100} cm · círculo = dojo`, 10, h - 8);
}

function drawHandTraj() {
  const { ctx: g, w, h } = fitCanvas($('#hand-traj'));
  const th = theme();
  g.clearRect(0, 0, w, h);
  const last = trails.hand.EKF.a.at(-1);
  const view = planeView(w, h, 0.8, last ? Math.round(last[0] / 0.2) * 0.2 : 0, last ? Math.round(last[1] / 0.2) * 0.2 : 0);
  drawPlaneGrid(g, w, h, view, 0.1, th);
  drawTrail(g, view, trails.hand.EKF.a, th.ekf, 2.2);
  drawTrail(g, view, trails.hand.UKF.a, th.ukf, 2.2, 0.85);
  const e = trails.hand.EKF.a.at(-1), u = trails.hand.UKF.a.at(-1);
  if (e) drawDot(g, view, e[0], e[1], th.ekf, 4.5);
  if (u) drawDot(g, view, u[0], u[1], th.ukf, 4.5);
}

function renderKpis(S) {
  const th = theme();
  const rs = S.robot.stats, hs = S.hand.stats;
  const us = (st, f) => (st && st[f] ? st[f].us : null);
  const ratio = (a, b) => (a && b ? `${fmt(b / a, 1)}×` : '—');
  const re = us(rs, 'EKF'), ru = us(rs, 'UKF'), he = us(hs, 'EKF'), hu = us(hs, 'UKF');
  const by = (st, f) => (st && st[f] ? st[f].bytes : null);
  $('#kpis').innerHTML = [
    kpi('Robot · EKF', th.ekf, fmt(re, 0), 'µs', `memoria del filtro ${by(rs, 'EKF') ?? '—'} B`),
    kpi('Robot · UKF', th.ukf, fmt(ru, 0), 'µs', `${ratio(re, ru)} el EKF · ${by(rs, 'UKF') ?? '—'} B`),
    kpi('Mano · EKF', th.ekf, fmt(he, 0), 'µs', `memoria del filtro ${by(hs, 'EKF') ?? '—'} B`),
    kpi('Mano · UKF', th.ukf, fmt(hu, 0), 'µs', `${ratio(he, hu)} el EKF · ${by(hs, 'UKF') ?? '—'} B`),
    kpi('Sesgo giroscopio', th.odom, fmt(S.robot.stats && S.robot.stats.EKF ? S.robot.stats.EKF.bias : null, 4), 'rad/s',
      `estimado por EKF · entrada ${(rs && rs.input_mode) || '—'}`),
    kpi('Diferencia EKF–UKF', th.accent,
      S.robot.EKF && S.robot.UKF ? fmt(Math.hypot(S.robot.EKF.x - S.robot.UKF.x, S.robot.EKF.y - S.robot.UKF.y) * 100, 2) : '—',
      'cm', 'posición del robot entre filtros'),
  ].join('');
}

let lastDraw = 0;
export function update(S, ctx) {
  if (S.robot.EKF) trails.robot.EKF.push([S.robot.EKF.x, S.robot.EKF.y], 0.003);
  if (S.robot.UKF) trails.robot.UKF.push([S.robot.UKF.x, S.robot.UKF.y], 0.003);
  if (S.links.car.age !== null) trails.robot.odom.push([S.car.odom.x, S.car.odom.y], 0.003);
  if (S.hand.EKF) trails.hand.EKF.push([S.hand.EKF.x, S.hand.EKF.y], 0.002);
  if (S.hand.UKF) trails.hand.UKF.push([S.hand.UKF.x, S.hand.UKF.y], 0.002);

  const rs = S.robot.stats || {}, hs = S.hand.stats || {};
  const g = (o, f, k) => (o[f] ? o[f][k] : null);
  const d = 180 / Math.PI;
  charts.timing.push(S.t, [g(rs, 'EKF', 'us'), g(rs, 'UKF', 'us'), g(hs, 'EKF', 'us'), g(hs, 'UKF', 'us')]);
  charts.theta.push(S.t, [S.links.car.age !== null ? S.car.odom.th * d : null,
    S.robot.EKF ? S.robot.EKF.th * d : null, S.robot.UKF ? S.robot.UKF.th * d : null]);
  charts.handv.push(S.t, [S.hand.EKF && S.hand.EKF.vx, S.hand.UKF && S.hand.UKF.vx, S.hand.EKF && S.hand.EKF.vy, S.hand.UKF && S.hand.UKF.vy]);
  charts.trace.push(S.t, [g(rs, 'EKF', 'trace'), g(rs, 'UKF', 'trace')]);

  if (ctx.view !== 'kalman') return;
  const t = performance.now();
  if (t - lastDraw < 66) return;
  lastDraw = t;
  renderKpis(S);
  drawRobotTraj(S);
  drawHandTraj();
  Object.values(charts).forEach((c) => c.draw(S.t));
}

export function onShow(name) {
  if (name === 'kalman') Object.values(charts).forEach((c) => { c.resize(); c.dirty = true; });
}

// ---------------------------------------------------------------------
// Benchmark
// ---------------------------------------------------------------------
function bars(title, rows, unit, digits, lowerBetter = true) {
  const max = Math.max(...rows.map((r) => r.v), 1e-12);
  const best = lowerBetter ? Math.min(...rows.map((r) => r.v)) : Math.max(...rows.map((r) => r.v));
  return `<div><div class="label" style="margin-bottom:10px">${title}</div><div class="bars">${rows.map((r) => `
    <div class="bar-row"><span>${r.name}${r.v === best ? ' <span style="color:var(--good)">●</span>' : ''}</span>
      <div class="track"><div class="fill" style="width:${(r.v / max) * 100}%;background:${r.c}"></div></div>
      <span class="v">${r.v.toFixed(digits)} ${unit}</span></div>`).join('')}</div></div>`;
}

async function runBench() {
  const btn = $('#btn-bench');
  btn.disabled = true;
  btn.innerHTML = '<i data-lucide="loader-circle"></i>Calculando…';
  icons();
  try {
    const r = await api('api/benchmark', { steps: +$('#bench-steps').value });
    const th = theme();
    const row = (sys, f, k) => ({ name: `${sys} ${f}`, v: r[sys === 'Robot' ? 'robot' : 'hand'][f][k], c: f === 'EKF' ? th.ekf : th.ukf });
    const pair = (k) => [row('Robot', 'EKF', k), row('Robot', 'UKF', k), row('Mano', 'EKF', k), row('Mano', 'UKF', k)];
    const rr = r.robot, hh = r.hand;
    const fast = rr.UKF.us_mean / rr.EKF.us_mean;
    const better = (e, u) => (Math.abs(e - u) / Math.max(e, u, 1e-12) < 0.02 ? 'empate' : e < u ? 'EKF' : 'UKF');
    const accR = better(rr.EKF.rmse_pos_m, rr.UKF.rmse_pos_m);
    const accH = better(hh.EKF.rmse_vel_ms, hh.UKF.rmse_vel_ms);
    const said = (sys, who) => (who === 'empate' ? `en ${sys} quedaron <b>empatados</b> (menos de 2 % de diferencia)` : `en ${sys} fue más preciso el <b>${who}</b>`);
    $('#bench-out').innerHTML = `
      <div class="grid">
        <div class="c6 stack">${bars('Tiempo medio por paso', pair('us_mean'), 'µs', 0)}${bars('Tiempo percentil 95', pair('us_p95'), 'µs', 0)}</div>
        <div class="c6 stack">${bars('Error RMS de posición (no medida)', pair('rmse_pos_m').map((x) => ({ ...x, v: x.v * 100 })), 'cm', 2)}
          ${bars('Memoria pico (200 pasos)', pair('peak_kb'), 'KB', 1)}</div>
        <div class="c12 table-wrap"><table class="data">
          <thead><tr><th>Filtro</th><th>µs/paso</th><th>p95 µs</th><th>Estado + P + sigma (B)</th><th>Pico (KB)</th><th>RMSE posición</th><th>RMSE velocidad</th></tr></thead>
          <tbody>${['robot', 'hand'].flatMap((s) => ['EKF', 'UKF'].map((f) => {
            const x = r[s][f];
            return `<tr><td>${s === 'robot' ? 'Robot' : 'Mano'} · <b style="color:${f === 'EKF' ? th.ekf : th.ukf}">${f}</b></td>
              <td class="n">${x.us_mean.toFixed(1)}</td><td class="n">${x.us_p95.toFixed(1)}</td><td class="n">${x.state_bytes}</td>
              <td class="n">${x.peak_kb.toFixed(1)}</td><td class="n">${(x.rmse_pos_m * 100).toFixed(2)} cm</td>
              <td class="n">${x.rmse_vel_ms !== undefined ? (x.rmse_vel_ms * 100).toFixed(2) + ' cm/s' : '—'}</td></tr>`;
          })).join('')}</tbody></table></div>
        <div class="c12 callout">
          Con ${r.steps} pasos (${(r.steps * r.dt).toFixed(0)} s simulados) el <b>UKF tarda ${fast.toFixed(1)}×</b> lo que el EKF
          (propaga ${2 * 6 + 1} puntos sigma en el robot y ${2 * 7 + 1} en la mano frente a un solo jacobiano) y guarda más memoria (los puntos sigma).
          ${said('posición del robot', accR).replace(/^en/, 'En')}, y ${said('velocidad de la mano', accH)}.
          ${(() => {
            const dR = Math.abs(rr.EKF.rmse_pos_m - rr.UKF.rmse_pos_m) / Math.max(rr.EKF.rmse_pos_m, rr.UKF.rmse_pos_m);
            const dH = Math.abs(hh.EKF.rmse_vel_ms - hh.UKF.rmse_vel_ms) / Math.max(hh.EKF.rmse_vel_ms, hh.UKF.rmse_vel_ms);
            return Math.max(dR, dH) < 0.15
              ? `La diferencia de precisión es pequeña (${(100 * Math.max(dR, dH)).toFixed(0)} % como máximo): con dt = ${r.dt * 1000} ms los modelos casi no se alejan de su linealización, así que el EKF da una precisión parecida con mucho menos cómputo.`
              : `La diferencia de precisión es notable (${(100 * Math.max(dR, dH)).toFixed(0)} %): hay que ponerla en la balanza contra el costo de cómputo del UKF.`;
          })()}
        </div>
      </div>`;
  } catch (e) {
    toast(`Benchmark: ${e.message}`, 'err');
  } finally {
    btn.disabled = false;
    btn.innerHTML = '<i data-lucide="play"></i>Ejecutar';
    icons();
  }
}
