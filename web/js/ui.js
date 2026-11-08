// Utilidades compartidas por todas las vistas.

export const $ = (s, root = document) => root.querySelector(s);
export const $$ = (s, root = document) => [...root.querySelectorAll(s)];

export const DEG = 180 / Math.PI;
export const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? '—' : Number(v).toFixed(d);
export const fmtDeg = (rad, d = 1) => (rad === null || rad === undefined) ? '—' : `${(rad * DEG).toFixed(d)}°`;
export const clamp = (x, a, b) => Math.max(a, Math.min(b, x));

export function icons() {
  if (window.lucide) window.lucide.createIcons({ attrs: { 'stroke-width': 1.9 } });
}

export function el(tag, attrs = {}, ...children) {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') n.className = v;
    else if (k === 'html') n.innerHTML = v;
    else if (k.startsWith('on')) n.addEventListener(k.slice(2), v);
    else if (v !== false && v !== null && v !== undefined) n.setAttribute(k, v === true ? '' : v);
  }
  for (const c of children.flat()) if (c !== null && c !== undefined) n.append(c.nodeType ? c : document.createTextNode(c));
  return n;
}

export async function api(path, body) {
  const res = await fetch(path, body === undefined ? {} : {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  });
  let data = null;
  try { data = await res.json(); } catch (e) { /* respuesta sin cuerpo JSON */ }
  if (!res.ok) throw new Error((data && data.error) || `HTTP ${res.status}`);
  return data;
}

export function toast(msg, kind = 'ok') {
  const icon = kind === 'ok' ? 'circle-check' : 'circle-alert';
  const t = el('div', { class: `toast ${kind}` }, el('i', { 'data-lucide': icon }), el('span', {}, msg));
  $('#toasts').append(t);
  icons();
  setTimeout(() => { t.style.transition = 'opacity .3s, transform .3s'; t.style.opacity = '0'; t.style.transform = 'translateY(6px)'; }, 2600);
  setTimeout(() => t.remove(), 3000);
}

// Lee los colores del tema actual (variables CSS) para canvas y graficas.
export function theme() {
  const cs = getComputedStyle(document.documentElement);
  const v = (n) => cs.getPropertyValue(n).trim();
  return {
    text: v('--text'), text2: v('--text-2'), text3: v('--text-3'),
    grid: v('--grid'), axis: v('--axis'), border: v('--border-strong'),
    ekf: v('--c-ekf'), ukf: v('--c-ukf'), odom: v('--c-odom'), meas: v('--c-meas'),
    exp: v('--c-exp'), raw: v('--c-raw'), filt: v('--c-filt'), cmd: v('--c-cmd'),
    accent: v('--accent'), good: v('--good'), warn: v('--warn'), bad: v('--bad'),
    surface: v('--surface-solid'), dark: document.documentElement.dataset.theme !== 'light',
  };
}

// Segmentado: marca el boton activo y avisa al cambiar.
// Segmentado con una pildora que se desliza hasta la opcion elegida.
// Tras un clic se respeta la eleccion hasta que el servidor la confirme
// (o 1.5 s): antes, el siguiente mensaje traia todavia el valor viejo y el
// boton saltaba atras y adelante.
export function segmented(root, onChange) {
  const btns = $$('button', root);
  root.classList.add('seg-anim');
  const ind = document.createElement('span');
  ind.className = 'seg-ind';
  root.prepend(ind);
  let lockVal = null, lockUntil = 0;
  const place = (instant) => {
    const b = btns.find((x) => x.classList.contains('on'));
    if (!b || !b.offsetWidth) { ind.style.opacity = 0; return; }
    if (instant) ind.style.transition = 'none';
    ind.style.opacity = 1;
    ind.style.width = `${b.offsetWidth}px`;
    ind.style.height = `${b.offsetHeight}px`;
    ind.style.transform = `translate(${b.offsetLeft}px, ${b.offsetTop}px)`;
    if (instant) { void ind.offsetWidth; ind.style.transition = ''; }
  };
  const mark = (v) => {
    let changed = false;
    btns.forEach((x) => {
      const on = x.dataset.v === String(v);
      if (on !== x.classList.contains('on')) changed = true;
      x.classList.toggle('on', on);
    });
    if (changed) place(false);
  };
  btns.forEach((b) => b.addEventListener('click', () => {
    lockVal = b.dataset.v;
    lockUntil = performance.now() + 1500;
    mark(b.dataset.v);
    onChange(b.dataset.v);
  }));
  new ResizeObserver(() => place(true)).observe(root);
  requestAnimationFrame(() => place(true));
  return {
    set(v) {
      if (lockVal !== null) {
        if (String(v) === lockVal || performance.now() > lockUntil) lockVal = null;
        else return;                       // el servidor aun no aplica el clic
      }
      mark(v);
    },
    get() { const b = btns.find((x) => x.classList.contains('on')); return b ? b.dataset.v : null; },
  };
}

// Canvas nitido en pantallas HiDPI; devuelve contexto escalado en px CSS.
export function fitCanvas(cv) {
  const r = cv.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: r.width, h: r.height };
}

// Plano cartesiano de robotica (x hacia arriba, y hacia la izquierda)
// centrado en (cx, cy) metros y con "span" metros visibles en el lado corto.
export function planeView(w, h, span, cx = 0, cy = 0) {
  const s = Math.min(w, h) / span;
  return {
    s, cx, cy, w, h,
    px: (x, y) => [w / 2 - (y - cy) * s, h / 2 - (x - cx) * s],
  };
}

export function drawPlaneGrid(ctx, w, h, view, step, th) {
  const { s, cx, cy } = view;
  const hx = h / (2 * s) + step, hy = w / (2 * s) + step;   // semiancho visible (m)
  ctx.save();
  ctx.strokeStyle = th.grid;
  ctx.lineWidth = 1;
  for (let x = Math.floor((cx - hx) / step) * step; x <= cx + hx; x += step) {
    const [, py] = view.px(x, cy);
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke();
  }
  for (let y = Math.floor((cy - hy) / step) * step; y <= cy + hy; y += step) {
    const [px] = view.px(cx, y);
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke();
  }
  // Ejes (si el origen esta a la vista)
  ctx.strokeStyle = th.border;
  const [ox, oy] = view.px(0, 0);
  ctx.beginPath();
  if (ox >= 0 && ox <= w) { ctx.moveTo(ox, 0); ctx.lineTo(ox, h); }
  if (oy >= 0 && oy <= h) { ctx.moveTo(0, oy); ctx.lineTo(w, oy); }
  ctx.stroke();
  ctx.fillStyle = th.text3;
  ctx.font = '11px Inter, sans-serif';
  if (ox >= 0 && ox <= w) ctx.fillText('x', ox + 6, 14);
  if (oy >= 0 && oy <= h) ctx.fillText('y', 8, oy - 6);
  ctx.restore();
}

export function drawTrail(ctx, view, pts, color, width = 2, alpha = 1) {
  if (pts.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.globalAlpha = alpha;
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.beginPath();
  pts.forEach((p, i) => { const [x, y] = view.px(p[0], p[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.stroke();
  ctx.restore();
}

export function drawDot(ctx, view, x, y, color, r = 5, ring = true) {
  const [px, py] = view.px(x, y);
  ctx.save();
  if (ring) {
    ctx.fillStyle = color; ctx.globalAlpha = 0.18;
    ctx.beginPath(); ctx.arc(px, py, r * 2.2, 0, Math.PI * 2); ctx.fill();
  }
  ctx.globalAlpha = 1; ctx.fillStyle = color;
  ctx.beginPath(); ctx.arc(px, py, r, 0, Math.PI * 2); ctx.fill();
  ctx.restore();
}

export function drawArrow(ctx, view, x, y, dx, dy, color, width = 2.5) {
  const [x0, y0] = view.px(x, y);
  const [x1, y1] = view.px(x + dx, y + dy);
  const a = Math.atan2(y1 - y0, x1 - x0), L = Math.hypot(x1 - x0, y1 - y0);
  if (L < 2) return;
  ctx.save();
  ctx.strokeStyle = ctx.fillStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
  const hl = Math.min(10, L * 0.45);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - hl * Math.cos(a - 0.45), y1 - hl * Math.sin(a - 0.45));
  ctx.lineTo(x1 - hl * Math.cos(a + 0.45), y1 - hl * Math.sin(a + 0.45));
  ctx.closePath(); ctx.fill();
  ctx.restore();
}

export class Ring {
  // Buffer circular de puntos [x, y] (rastros).
  constructor(n = 600) { this.n = n; this.a = []; }
  push(p, minDist = 0) {
    const l = this.a[this.a.length - 1];
    if (l && minDist && Math.hypot(p[0] - l[0], p[1] - l[1]) < minDist) return;
    this.a.push(p);
    if (this.a.length > this.n) this.a.shift();
  }
  clear() { this.a = []; }
}
