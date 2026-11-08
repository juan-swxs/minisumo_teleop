// Graficas en tiempo real con uPlot.

import { theme, el } from './ui.js';

const charts = new Set();
window.__charts = charts;   // para depurar desde la consola del navegador

export class LiveChart {
  // series: [{label, color: 'ekf' | '#hex', width, dash, unitless}]
  // xFmt: formato del eje de tiempo (segundos relativos -> texto)
  // gap: si pasan mas de 'gap' s sin datos se corta la linea (no se une el hueco)
  // range: {minSpan} alto minimo del eje, o {zero, minMax} eje desde 0 con un tope minimo
  constructor(container, { series, height = 220, window = 20, yRange = null, decimals = 2, xFmt = null, unit = '', gap = 0, range = null }) {
    this.range = range;
    this.xFmt = xFmt;
    this.gap = gap;
    this.unit = unit;
    this.container = container;
    this.def = series;
    this.height = height;
    this.window = window;
    this.yRange = yRange;
    this.decimals = decimals;
    this.t = [];
    this.y = series.map(() => []);
    this.dirty = true;
    this.build();
    charts.add(this);
    new ResizeObserver(() => this.resize()).observe(container);
  }

  build() {
    if (this.u) this.u.destroy();
    const th = theme();
    const color = (c) => (th[c] || c);
    const dec = this.decimals;
    const opts = {
      width: Math.max(200, this.container.clientWidth),
      height: this.height,
      pxAlign: false,
      cursor: { drag: { x: false, y: false }, points: { size: 6 } },
      legend: { show: true, live: true },
      scales: { x: { time: false }, y: this.yRange ? { range: this.yRange } : this.range ? { auto: true, range: (u, mn, mx) => this.yRangeFn(mn, mx) } : { auto: true } },
      axes: [
        {
          stroke: th.axis, font: '11px Inter, sans-serif', size: 30,
          grid: { stroke: th.grid, width: 1 }, ticks: { show: false },
          values: (u, v) => v.map((x) => (this.xFmt ? this.xFmt(x) : `${x.toFixed(0)} s`)),
        },
        {
          stroke: th.axis, font: '11px JetBrains Mono, monospace', size: 52,
          grid: { stroke: th.grid, width: 1 }, ticks: { show: false },
          values: (u, v) => v.map((x) => +x.toFixed(3)),
        },
      ],
      series: [
        { label: 't', value: (u, v) => (v == null ? '—' : (this.xFmt ? this.xFmt(v) : `${v.toFixed(1)} s`)) },
        ...this.def.map((s) => ({
          label: s.label,
          stroke: color(s.color),
          width: s.width || 1.6,
          dash: s.dash,
          points: { show: false },
          value: (u, v) => (v == null ? '—' : `${v.toFixed(dec)}${this.unit ? ' ' + this.unit : ''}`),
        })),
      ],
    };
    this.u = new uPlot(opts, [[], ...this.def.map(() => [])], this.container);
    // Con el mouse encima la grafica se congela: si siguiera desplazandose,
    // un pico pasaria por debajo del cursor y no se podria leer su valor.
    // Los datos se siguen guardando y al salir se pone al dia.
    const over = this.u.over;
    over.addEventListener('pointerenter', () => this.setPaused(true));
    over.addEventListener('pointerleave', () => this.setPaused(false));
    this.paused = false;
    this.container.classList.remove('hover-paused');
    this.dirty = true;
  }

  setPaused(p) {
    this.paused = p;
    this.container.classList.toggle('hover-paused', p);
    if (!p) this.dirty = true;
  }

  // Eje Y con alto minimo: la cuantizacion del sensor (1 paso) no debe
  // llenar toda la grafica cuando la senal esta quieta
  yRangeFn(mn, mx) {
    const r = this.range;
    if (mn === null || mx === null || !Number.isFinite(mn) || !Number.isFinite(mx)) return [0, 1];
    if (r.zero) {
      const top = Math.max(mx * 1.15, r.minMax || 1);
      return [Math.min(0, mn), top];
    }
    const span = Math.max(mx - mn, r.minSpan || 0);
    const mid = (mx + mn) / 2;
    return [mid - span / 2 - span * 0.08, mid + span / 2 + span * 0.08];
  }

  resize() {
    if (!this.u || !this.container.clientWidth) return;
    this.u.setSize({ width: this.container.clientWidth, height: this.height });
  }

  push(t, values) {
    if (this.gap && this.t.length && t - this.t[this.t.length - 1] > this.gap) {
      this.t.push(this.t[this.t.length - 1] + 1e-3);          // punto nulo: uPlot corta la linea
      this.y.forEach((a) => a.push(null));
    }
    this.t.push(t);
    values.forEach((v, i) => this.y[i].push(v === undefined ? null : v));
    const cut = t - this.window - 1;
    let n = 0;
    while (n < this.t.length && this.t[n] < cut) n++;
    if (n) { this.t.splice(0, n); this.y.forEach((a) => a.splice(0, n)); }
    this.dirty = true;
  }

  clear() { this.t = []; this.y = this.y.map(() => []); this.dirty = true; }

  draw(now) {
    if (this.paused || !this.dirty || this.container.offsetParent === null) return;
    const x = this.t.map((t) => t - now);
    this.u.setData([x, ...this.y], true);
    this.u.setScale('x', { min: -this.window, max: 0 });
    this.dirty = false;
  }

  // Ruido de alta frecuencia de cada serie: desviacion estandar de la
  // diferencia entre muestras consecutivas (no la infla el movimiento
  // lento real, solo las variaciones rapidas).
  std() {
    return this.y.map((a) => {
      const d = [];
      for (let i = 1; i < a.length; i++) {
        if (a[i] !== null && a[i - 1] !== null && Number.isFinite(a[i]) && Number.isFinite(a[i - 1])) d.push(a[i] - a[i - 1]);
      }
      if (d.length < 3) return null;
      const m = d.reduce((s, x) => s + x, 0) / d.length;
      return Math.sqrt(d.reduce((s, x) => s + (x - m) ** 2, 0) / (d.length - 1));
    });
  }
}

export function rebuildAll() { charts.forEach((c) => c.build()); }
export function setWindowAll(w) { charts.forEach((c) => { c.window = w; c.dirty = true; }); }

// Tarjeta con titulo, grafica y linea de estadisticas.
export function chartCard(parent, { title, sub, icon = 'activity', span = 'c6', height = 220, series, decimals, yRange }) {
  const card = el('div', { class: `card chart-card ${span}` },
    el('div', { class: 'card-h' },
      el('div', { class: 'ico' }, el('i', { 'data-lucide': icon })),
      el('div', {}, el('h2', {}, title), sub ? el('div', { class: 'sub' }, sub) : null)));
  const box = el('div', { class: 'chart' });
  const stats = el('div', { class: 'stats' });
  card.append(box, stats);
  parent.append(card);
  const chart = new LiveChart(box, { series, height, decimals, yRange });
  chart.statsEl = stats;
  return chart;
}
