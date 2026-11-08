// Vista "Ajustes": parametros en vivo de los nodos (sin reiniciar nada).

import { $, api, toast, el, icons } from './ui.js';

const TELEOP = [
  { k: 'v_max', name: 'Velocidad lineal máxima', unit: 'm/s', min: 0.05, max: 1.0, step: 0.01, desc: 'Saturación de v enviada al robot' },
  { k: 'w_max', name: 'Velocidad angular máxima', unit: 'rad/s', min: 0.5, max: 8, step: 0.1 },
  { k: 'v_min', name: 'Velocidad lineal mínima', unit: 'm/s', min: 0, max: 0.4, step: 0.01, desc: 'E2 · con la que arranca al salir de la zona muerta (por debajo las ruedas van a tirones)' },
  { k: 'w_min', name: 'Velocidad angular mínima', unit: 'rad/s', min: 0, max: 4, step: 0.1, desc: 'E2' },
  { k: 'deadzone_deg', name: 'Zona muerta', unit: '°', min: 0, max: 25, step: 0.5, desc: 'E2 · inclinación que se ignora alrededor de la postura neutra' },
  { k: 'deadzone_exit_deg', name: 'Salida de la zona muerta', unit: '°', min: 0, max: 25, step: 0.5, desc: 'E2 · histéresis: ya en marcha, se detiene por debajo de este ángulo' },
  { k: 'yaw_loop', name: 'Lazo de giro con el giroscopio', type: 'bool', desc: 'E2 · corrige el giro con el giroscopio del carro (rueda más floja, piso)' },
  { k: 'full_tilt_deg', name: 'Inclinación para velocidad máxima', unit: '°', min: 10, max: 70, step: 1, desc: 'E2' },
  { k: 'expo', name: 'Curva exponencial', unit: '', min: 1, max: 3, step: 0.1, desc: 'E2 · más alto = más fino cerca del centro' },
  { k: 'invert_pitch', name: 'Invertir pitch', type: 'bool', desc: 'Si inclinar la mano hacia adelante hace retroceder al robot' },
  { k: 'invert_roll', name: 'Invertir roll', type: 'bool' },
  { k: 'plane_gain', name: 'Ganancia mano → robot', unit: '×', min: 0.5, max: 6, step: 0.1, desc: 'E1 · velocidad del robot por cada m/s de la mano' },
  { k: 'plane_deadband', name: 'Umbral de movimiento de la mano', unit: 'm/s', min: 0, max: 0.2, step: 0.005, desc: 'E1' },
  { k: 'plane_stroke_max', name: 'Duración máxima de un trazo', unit: 's', min: 0.3, max: 4, step: 0.1, desc: 'E1 · el carro sigue la dirección de cada trazo de la mano hasta que esta queda quieta' },
  { k: 'heading_kp', name: 'Ganancia de rumbo', unit: '1/s', min: 0.5, max: 8, step: 0.1, desc: 'E1 · qué tan rápido gira hacia la dirección de la mano' },
  { k: 'smoothing_tau', name: 'Suavizado del comando', unit: 's', min: 0, max: 0.6, step: 0.01 },
  { k: 'dojo_guard', name: 'Protección del borde', type: 'bool', desc: 'Anula el avance que saca al robot del dojo (el giro sigue permitido)' },
  { k: 'dojo_radius', name: 'Radio del dojo', unit: 'm', min: 0.3, max: 5, step: 0.05 },
  { k: 'guard_margin', name: 'Margen al borde', unit: 'm', min: 0, max: 0.2, step: 0.005 },
  { k: 'guard_lookahead', name: 'Anticipación del borde', unit: 's', min: 0, max: 1.2, step: 0.05, desc: 'Cuánto sigue rodando el robot tras pedir v = 0' },
  { k: 'guard_slow', name: 'Velocidad cerca del borde', unit: 'm/s', min: 0.05, max: 0.5, step: 0.01 },
];

const ROBOT = [
  { k: 'input_mode', name: 'Giroscopio', type: 'seg', opts: [['raw', 'Crudo + sesgo'], ['filtered', 'Filtrado']] },
  { k: 'tf_source', name: 'Fuente de TF (RViz)', type: 'seg', opts: [['ekf', 'EKF'], ['ukf', 'UKF']] },
  { k: 'q_v', name: 'Q · velocidad lineal', type: 'num', desc: '(m/s)²/s · cuánto se permite a v alejarse del modelo' },
  { k: 'q_w', name: 'Q · velocidad angular', type: 'num', desc: '(rad/s)²/s' },
  { k: 'q_theta', name: 'Q · rumbo', type: 'num' },
  { k: 'q_pos', name: 'Q · posición', type: 'num' },
  { k: 'q_bias', name: 'Q · sesgo del giroscopio', type: 'num' },
  { k: 'r_v', name: 'R · encoders (v)', type: 'num', desc: '(m/s)² · 40 pulsos por vuelta: medición gruesa' },
  { k: 'r_gyro', name: 'R · giroscopio', type: 'num', desc: '(rad/s)²' },
  { k: 'tau_v', name: 'Retardo del PID (v)', type: 'num', desc: 's · respuesta de primer orden al comando' },
  { k: 'tau_w', name: 'Retardo del PID (ω)', type: 'num' },
];

const HAND = [
  { k: 'input_mode', name: 'Aceleración', type: 'seg', opts: [['raw', 'Cruda'], ['filtered', 'Filtrada']] },
  { k: 'tau_a', name: 'Correlación de la aceleración', type: 'num', desc: 's · modelo de Singer' },
  { k: 'q_acc', name: 'Q · aceleración', type: 'num', desc: '(m/s²)²/s · agilidad del movimiento de la mano' },
  { k: 'q_vel', name: 'Q · velocidad', type: 'num' },
  { k: 'r_acc', name: 'R · acelerómetro', type: 'num', desc: '(m/s²)²' },
  { k: 'r_psi', name: 'R · rumbo de la fusión', type: 'num', desc: 'rad²' },
  { k: 'r_zupt', name: 'R · ZUPT', type: 'num', desc: '(m/s)² · confianza en "mano quieta ⇒ v = 0"' },
  { k: 'zupt_gyro', name: 'Umbral ZUPT · giro', type: 'num', desc: 'rad/s' },
  { k: 'zupt_acc', name: 'Umbral ZUPT · aceleración', type: 'num', desc: 'm/s²' },
  { k: 'zupt_time', name: 'Tiempo quieta para ZUPT', type: 'num', desc: 's' },
  { k: 'zupt_vmax', name: 'ZUPT · velocidad máxima', type: 'num', desc: 'm/s · evita frenar la mano a velocidad constante' },
  { k: 'zupt_long', name: 'ZUPT · quietud larga', type: 'num', desc: 's · aplica ZUPT aunque la velocidad estimada sea alta' },
  { k: 'leak_tau', name: 'Fuga de velocidad', type: 'num', desc: 's · 0 = desactivada; limita la deriva de la posición' },
];

const controls = {};   // `${grupo}.${clave}` -> {set(v)}
const editing = new Set();

function send(group, k, v) {
  const path = group === 'teleop' ? 'api/teleop' : 'api/estimator';
  const body = group === 'teleop' ? { [k]: v } : { target: group, [k]: v };
  return api(path, body).catch((e) => toast(e.message, 'err'));
}

function build(container, group, defs) {
  for (const d of defs) {
    const id = `${group}.${d.k}`;
    const row = el('div', { class: 'setting' });
    const name = el('div', {}, el('div', { class: 'name' }, d.name));
    row.append(name);
    if (d.type === 'bool') {
      const inp = el('input', { type: 'checkbox' });
      inp.addEventListener('change', () => send(group, d.k, inp.checked));
      row.append(el('label', { class: 'switch' }, inp, el('span')));
      controls[id] = { set: (v) => { inp.checked = !!v; } };
    } else if (d.type === 'seg') {
      const seg = el('div', { class: 'seg' }, d.opts.map(([v, l]) => el('button', { 'data-v': v }, l)));
      seg.querySelectorAll('button').forEach((b) => b.addEventListener('click', () => {
        seg.querySelectorAll('button').forEach((x) => x.classList.toggle('on', x === b));
        send(group, d.k, b.dataset.v);
      }));
      row.append(seg);
      controls[id] = { set: (v) => seg.querySelectorAll('button').forEach((x) => x.classList.toggle('on', x.dataset.v === v)) };
    } else if (d.type === 'num') {
      const inp = el('input', { class: 'input xs', type: 'number', step: 'any', style: 'width:110px' });
      inp.addEventListener('focus', () => editing.add(id));
      inp.addEventListener('blur', () => editing.delete(id));
      inp.addEventListener('change', () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) send(group, d.k, v).then(() => toast(`${d.name} = ${v}`)); });
      row.append(inp);
      controls[id] = { set: (v) => { if (!editing.has(id)) inp.value = v; } };
    } else {
      const val = el('span', { class: 'val' });
      const inp = el('input', { type: 'range', min: d.min, max: d.max, step: d.step });
      let timer = null;
      inp.addEventListener('input', () => {
        editing.add(id);
        val.textContent = `${(+inp.value).toFixed(d.step < 0.01 ? 3 : 2)} ${d.unit}`;
        clearTimeout(timer);
        timer = setTimeout(() => send(group, d.k, +inp.value), 120);
      });
      inp.addEventListener('change', () => setTimeout(() => editing.delete(id), 600));
      row.append(val, el('div', { class: 'range' }, inp));
      controls[id] = {
        set: (v) => {
          if (editing.has(id)) return;
          inp.value = v;
          val.textContent = `${(+v).toFixed(d.step < 0.01 ? 3 : 2)} ${d.unit}`;
        },
      };
    }
    if (d.desc) row.append(el('div', { class: 'desc' }, d.desc));
    container.append(row);
  }
}

// ---------- Acceso remoto (clave de invitados) ----------
let owner = false;
let accessBox = null;
let lastAccess = 0;

function ago(s) {
  if (s < 60) return 'activo ahora';
  if (s < 3600) return `hace ${Math.round(s / 60)} min`;
  return `hace ${Math.round(s / 3600)} h`;
}

function renderGuestView(name) {
  accessBox.replaceChildren(
    el('p', { class: 'acc-note' }, `Entraste como invitado: `, el('b', {}, name || 'Invitado'),
      '. Puedes usar el tablero igual que el dueño.'),
    el('button', { class: 'btn', onclick: async () => {
      try { await api('api/logout', {}); } catch (e) { /* ya sin sesion */ }
      location.reload();
    } }, el('i', { 'data-lucide': 'log-out' }), 'Cerrar sesión'),
  );
  icons();
}

function renderOwner(st) {
  const keyText = el('code', { class: 'acc-key', 'data-hidden': '1' }, '••••••••••••');
  const show = el('button', { class: 'btn sm', title: 'Mostrar u ocultar la clave', onclick: () => {
    const hidden = keyText.dataset.hidden === '1';
    keyText.textContent = hidden ? st.key : '••••••••••••';
    keyText.dataset.hidden = hidden ? '0' : '1';
  } }, el('i', { 'data-lucide': 'eye' }), 'Mostrar');
  const copy = el('button', { class: 'btn sm', onclick: async () => {
    try { await navigator.clipboard.writeText(st.key); toast('Clave copiada'); } catch (e) { toast('No se pudo copiar', 'err'); }
  } }, el('i', { 'data-lucide': 'copy' }), 'Copiar');
  const custom = el('input', { class: 'acc-input', placeholder: 'Clave nueva (vacío = generar una)', maxlength: '40' });
  const change = el('button', { class: 'btn primary', onclick: async () => {
    if (!confirm('Cambiar la clave expulsa a todos los invitados conectados. ¿Continuar?')) return;
    try {
      const r = await api('api/access/key', { key: custom.value });
      custom.value = '';
      toast(`Clave nueva: ${r.key}`);
      refreshAccess(true);
    } catch (e) { toast(e.message, 'err'); }
  } }, el('i', { 'data-lucide': 'refresh-cw' }), 'Cambiar clave');

  const addr = [];
  if (st.tailscale_ip) addr.push(el('div', {}, el('span', { class: 'acc-lbl' }, 'Por Tailscale'), el('code', {}, `http://${st.tailscale_ip}:${st.port}`)));
  if (st.lan_ip) addr.push(el('div', {}, el('span', { class: 'acc-lbl' }, 'Misma red WiFi'), el('code', {}, `http://${st.lan_ip}:${st.port}`)));
  if (!st.tailscale_ip) addr.push(el('div', { class: 'acc-note' }, 'Tailscale no está conectado en este PC.'));

  const list = st.guests.length
    ? st.guests.map((g) => el('div', { class: 'acc-guest' },
      el('i', { 'data-lucide': 'user' }),
      el('span', { class: 'acc-gname' }, g.name),
      el('span', { class: 'acc-gmeta' }, `${g.ip} · ${ago(g.idle_s)}`),
      el('button', { class: 'btn sm acc-kick', onclick: async () => {
        try { await api('api/access/kick', { token: g.token }); toast(`${g.name} expulsado`); refreshAccess(true); } catch (e) { toast(e.message, 'err'); }
      } }, el('i', { 'data-lucide': 'user-x' }), 'Expulsar')))
    : [el('div', { class: 'acc-note' }, 'Nadie conectado.')];

  accessBox.replaceChildren(
    el('div', { class: 'acc-grid' },
      el('div', {},
        el('div', { class: 'acc-title' }, 'Clave de invitados'),
        el('p', { class: 'acc-note' }, 'Quien entre desde otro equipo debe escribir esta clave. Con ella puede hacer lo mismo que tú. Solo tú, desde este PC, puedes verla o cambiarla.'),
        el('div', { class: 'acc-row' }, keyText, show, copy),
        el('div', { class: 'acc-row' }, custom, change)),
      el('div', {},
        el('div', { class: 'acc-title' }, 'Dirección para el invitado'),
        ...addr),
      el('div', {},
        el('div', { class: 'acc-title' }, `Conectados (${st.guests.length})`),
        ...list,
        st.guests.length > 1 ? el('button', { class: 'btn sm acc-kick', onclick: async () => {
          try { await api('api/access/kick', {}); toast('Todos expulsados'); refreshAccess(true); } catch (e) { toast(e.message, 'err'); }
        } }, 'Expulsar a todos') : null)),
  );
  icons();
}

async function refreshAccess(force = false) {
  if (!owner || !accessBox) return;
  const t = performance.now();
  if (!force && t - lastAccess < 3000) return;
  lastAccess = t;
  try { renderOwner(await api('api/access')); } catch (e) { /* tablero reiniciandose */ }
}

async function initAccess() {
  accessBox = $('#set-access');
  try {
    const who = await api('api/whoami');
    owner = !!who.owner;
    if (owner) refreshAccess(true); else renderGuestView(who.name);
  } catch (e) { accessBox.replaceChildren(el('p', { class: 'acc-note' }, 'No disponible.')); }
}

export function init() {
  build($('#set-teleop'), 'teleop', TELEOP);
  build($('#set-robot'), 'robot', ROBOT);
  build($('#set-hand'), 'hand', HAND);
  initAccess();
  icons();
}

let lastSync = 0;
export function update(S, ctx) {
  if (ctx.view !== 'settings') return;
  refreshAccess();
  const t = performance.now();
  if (t - lastSync < 300) return;
  lastSync = t;
  const src = {
    teleop: (S.teleop && S.teleop.params) || S.remote.glove_teleop,
    robot: S.remote.robot_estimator,
    hand: S.remote.hand_estimator,
  };
  for (const [id, c] of Object.entries(controls)) {
    const [g, k] = id.split('.');
    if (src[g] && src[g][k] !== undefined) c.set(src[g][k]);
  }
}
