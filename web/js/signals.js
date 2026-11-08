// Vista "Senales": crudo vs filtrado en los dos microcontroladores.

import { $, fmt, segmented, icons } from './ui.js';
import { chartCard, setWindowAll } from './charts.js';

const G = 9.80665;
let charts = [];
let axis = 0;
let paused = false;
const AX = ['X', 'Y', 'Z'];

function quatToRP(q) {
  const [x, y, z, w] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  return [roll, pitch];
}

// Gravedad expresada en el cuerpo: g_b = R(q)^T [0, 0, g]
function gravityBody(q) {
  const [x, y, z, w] = q;
  return [2 * (x * z - w * y) * G, 2 * (y * z + w * x) * G, (1 - 2 * (x * x + y * y)) * G];
}

const DEFS = [
  {
    key: 'glove_gyro', title: 'Guante · giroscopio', sub: 'MPU6050 crudo vs pasa-bajos EMA (ESP32)', icon: 'hand', unit: 'rad/s',
    series: () => [{ label: `ω${AX[axis]} crudo`, color: 'raw', width: 1.2 }, { label: `ω${AX[axis]} filtrado`, color: 'filt', width: 2 }],
    get: (S) => [S.glove.raw.g[axis], S.glove.filt.g[axis]],
    noise: [0, 1],
  },
  {
    key: 'glove_acc', title: 'Guante · aceleración lineal', sub: 'acelerómetro − gravedad vs fusión Mahony + EMA', icon: 'hand', unit: 'm/s²',
    series: () => [{ label: `a${AX[axis]} crudo`, color: 'raw', width: 1.2 }, { label: `a${AX[axis]} filtrado`, color: 'filt', width: 2 }],
    get: (S) => {
      const gb = gravityBody(S.glove.filt.q);
      return [S.glove.raw.a[axis] - gb[axis], S.glove.filt.a[axis]];
    },
    noise: [0, 1],
  },
  {
    key: 'glove_tilt', title: 'Guante · inclinación', sub: 'ángulo del acelerómetro crudo vs fusión Mahony (estrategia 2)', icon: 'rotate-3d', unit: '°', span: 'c12',
    series: () => [
      { label: 'roll acelerómetro', color: 'raw', width: 1.2 }, { label: 'roll fusión', color: 'ekf', width: 2 },
      { label: 'pitch acelerómetro', color: 'raw', width: 1.2, dash: [4, 4] }, { label: 'pitch fusión', color: 'ukf', width: 2 },
    ],
    get: (S) => {
      const a = S.glove.raw.a;
      const rollA = Math.atan2(a[1], a[2]), pitchA = Math.atan2(-a[0], Math.hypot(a[1], a[2]));
      const [r, p] = quatToRP(S.glove.filt.q);
      const d = 180 / Math.PI;
      return [rollA * d, r * d, pitchA * d, p * d];
    },
    noise: [0, 1],
    decimals: 1,
  },
  {
    key: 'car_gyro', title: 'Carro · giroscopio Z', sub: 'MPU6050 crudo (con sesgo) vs sin sesgo + EMA', icon: 'car', unit: 'rad/s',
    series: () => [{ label: 'ωz crudo', color: 'raw', width: 1.2 }, { label: 'ωz filtrado', color: 'filt', width: 2 }],
    get: (S) => [S.car.raw.g[2], S.car.filt.g[2]],
    noise: [0, 1],
  },
  {
    key: 'car_acc', title: 'Carro · aceleración', sub: 'MPU6050 crudo vs lineal (complementario) + EMA', icon: 'car', unit: 'm/s²',
    series: () => [{ label: `a${AX[axis]} crudo`, color: 'raw', width: 1.2 }, { label: `a${AX[axis]} lineal filtrada`, color: 'filt', width: 2 }],
    get: (S) => [S.car.raw.a[axis] - (axis === 2 ? G : 0), S.car.filt.a[axis]],
    noise: [0, 1],
  },
  {
    key: 'wheels', title: 'Carro · velocidad de ruedas', sub: 'encoders: conteo de pulsos vs EMA', icon: 'cog', unit: 'rad/s',
    series: () => [
      { label: 'izq. cruda', color: 'raw', width: 1.2 }, { label: 'izq. filtrada', color: 'ekf', width: 2 },
      { label: 'der. cruda', color: 'raw', width: 1.2, dash: [4, 4] }, { label: 'der. filtrada', color: 'ukf', width: 2 },
    ],
    get: (S) => [S.car.wheels.wl_raw, S.car.wheels.wl_filt, S.car.wheels.wr_raw, S.car.wheels.wr_filt],
    noise: [0, 1],
    decimals: 1,
  },
  {
    key: 'speed', title: 'Velocidad lineal del robot', sub: 'comando vs encoders vs estimadores', icon: 'gauge', unit: 'm/s',
    series: () => [
      { label: 'comando', color: 'cmd', width: 1.6, dash: [6, 4] }, { label: 'encoders', color: 'odom', width: 1.4 },
      { label: 'EKF', color: 'ekf', width: 2 }, { label: 'UKF', color: 'ukf', width: 2 },
    ],
    get: (S) => [S.cmd.v, S.car.odom.v, S.robot.EKF && S.robot.EKF.v, S.robot.UKF && S.robot.UKF.v],
  },
  {
    key: 'omega', title: 'Velocidad angular del robot', sub: 'comando vs giroscopio vs estimadores', icon: 'rotate-cw', unit: 'rad/s',
    series: () => [
      { label: 'comando', color: 'cmd', width: 1.6, dash: [6, 4] }, { label: 'giroscopio', color: 'odom', width: 1.4 },
      { label: 'EKF', color: 'ekf', width: 2 }, { label: 'UKF', color: 'ukf', width: 2 },
    ],
    get: (S) => [S.cmd.w, S.car.odom.w, S.robot.EKF && S.robot.EKF.w, S.robot.UKF && S.robot.UKF.w],
  },
];

function build() {
  const grid = $('#signals-grid');
  grid.innerHTML = '';
  charts = DEFS.map((d) => {
    const c = chartCard(grid, { title: d.title, sub: `${d.sub} · ${d.unit}`, icon: d.icon, span: d.span || 'c6', series: d.series(), decimals: d.decimals || 3 });
    c.def0 = d;
    return c;
  });
  icons();
}

export function init() {
  build();
  segmented($('#seg-axis'), (v) => { axis = +v; build(); });
  segmented($('#seg-window'), (v) => setWindowAll(+v));
  $('#btn-pause').addEventListener('click', () => {
    paused = !paused;
    $('#btn-pause span').textContent = paused ? 'Reanudar' : 'Pausar';
  });
}

let lastDraw = 0;
export function update(S, ctx) {
  if (paused) return;
  const now = S.t;
  for (const c of charts) c.push(now, c.def0.get(S));
  if (ctx.view !== 'signals') return;
  const t = performance.now();
  if (t - lastDraw < 66) return;   // ~15 cuadros por segundo
  lastDraw = t;
  for (const c of charts) {
    c.draw(now);
    const d = c.def0;
    if (d.noise) {
      const sd = c.std();
      const [i, j] = d.noise;
      const red = sd[i] && sd[j] !== null ? (1 - sd[j] / sd[i]) * 100 : null;
      c.statsEl.innerHTML = `ruido σΔ crudo <b>${fmt(sd[i], 4)}</b> · filtrado <b>${fmt(sd[j], 4)}</b>` +
        (red !== null ? ` · reducción de ruido <b>${fmt(red, 0)} %</b>` : '');
    }
  }
}

export function onShow(name) {
  if (name === 'signals') charts.forEach((c) => { c.resize(); c.dirty = true; });
}
