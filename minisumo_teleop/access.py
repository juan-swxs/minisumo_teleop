"""Acceso de invitados al tablero web.

El dueno usa el tablero desde el mismo PC (localhost) y entra sin clave.
Cualquier otro equipo (la red local o Tailscale) tiene que escribir la
clave de invitados; con ella puede hacer lo mismo que el dueno, salvo ver
o cambiar la clave y expulsar invitados (solo desde el PC).

La clave se guarda en ~/.config/minisumo_teleop/clave_invitados.txt (solo
lectura del usuario). Las sesiones viven en memoria: al reiniciar el
tablero los invitados vuelven a escribir la clave.
"""

import hmac
import os
import secrets
import threading
import time

KEY_FILE = os.path.expanduser('~/.config/minisumo_teleop/clave_invitados.txt')
COOKIE = 'minisumo_sesion'
SESSION_TTL_S = 12 * 3600
MAX_FAILS = 5            # intentos fallidos seguidos antes de bloquear el equipo
LOCK_S = 60              # bloqueo inicial; se duplica en cada bloqueo nuevo
LOCAL_IPS = ('127.0.0.1', '::1', '::ffff:127.0.0.1')

_WORDS = ('sumo', 'dojo', 'rueda', 'guante', 'motor', 'kalman', 'giro', 'pista',
          'robot', 'pulso', 'chasis', 'ruta', 'radar', 'nodo', 'brazo', 'senal')


def new_key():
    """Clave facil de dictar: dos palabras y cuatro numeros (sumo-giro-4821)."""
    a, b = secrets.choice(_WORDS), secrets.choice(_WORDS)
    return f'{a}-{b}-{secrets.randbelow(10000):04d}'


class Access:
    def __init__(self, logger=None):
        self.log = logger
        self.lock = threading.Lock()
        self.sessions = {}    # token -> {name, ip, since, seen}
        self.fails = {}       # ip -> {n, until, lock_s}
        self.key = self._load_or_create()

    # ---------- clave ----------
    def _load_or_create(self):
        try:
            with open(KEY_FILE, encoding='utf-8') as f:
                key = f.read().strip()
            if key:
                return key
        except OSError:
            pass
        key = new_key()
        self._save(key)
        return key

    def _save(self, key):
        os.makedirs(os.path.dirname(KEY_FILE), exist_ok=True)
        fd = os.open(KEY_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(key + '\n')

    def set_key(self, key=None):
        """Cambia la clave y expulsa a todos los invitados."""
        key = (key or '').strip() or new_key()
        if len(key) < 6:
            raise ValueError('la clave debe tener al menos 6 caracteres')
        with self.lock:
            self.key = key
            self.sessions.clear()
            self.fails.clear()
        self._save(key)
        return key

    # ---------- quien entra ----------
    @staticmethod
    def is_local(ip):
        return ip in LOCAL_IPS

    @staticmethod
    def _cookie_token(cookie_header):
        for part in (cookie_header or '').split(';'):
            k, _, v = part.strip().partition('=')
            if k == COOKIE:
                return v
        return None

    def guest(self, ip, cookie_header):
        """Sesion valida del invitado, o None."""
        tok = self._cookie_token(cookie_header)
        if not tok:
            return None
        now = time.time()
        with self.lock:
            s = self.sessions.get(tok)
            if s is None or now - s['since'] > SESSION_TTL_S:
                self.sessions.pop(tok, None)
                return None
            s['seen'] = now
            s['ip'] = ip
            return dict(s, token=tok)

    def allowed(self, ip, cookie_header):
        return self.is_local(ip) or self.guest(ip, cookie_header) is not None

    def login(self, ip, key, name):
        """Devuelve (token, None) o (None, mensaje de error)."""
        now = time.time()
        with self.lock:
            f = self.fails.get(ip)
            if f and now < f['until']:
                return None, f'Demasiados intentos. Espera {int(f["until"] - now) + 1} s.'
            ok = hmac.compare_digest((key or '').strip().encode(), self.key.encode())
            if not ok:
                f = f or {'n': 0, 'until': 0.0, 'lock_s': LOCK_S}
                f['n'] += 1
                if f['n'] >= MAX_FAILS:
                    f['until'] = now + f['lock_s']
                    f['lock_s'] *= 2
                    f['n'] = 0
                self.fails[ip] = f
                left = MAX_FAILS - f['n'] if f['n'] else 0
                if not left:
                    msg = 'Clave incorrecta. Equipo bloqueado un momento.'
                else:
                    msg = 'Clave incorrecta. ' + ('Te queda 1 intento.' if left == 1 else f'Te quedan {left} intentos.')
                if self.log:
                    self.log.warning(f'Acceso: clave incorrecta desde {ip}')
                return None, msg
            self.fails.pop(ip, None)
            tok = secrets.token_urlsafe(24)
            name = (name or '').strip()[:40] or 'Invitado'
            self.sessions[tok] = {'name': name, 'ip': ip, 'since': now, 'seen': now}
        if self.log:
            self.log.info(f'Acceso: entro "{name}" desde {ip}')
        return tok, None

    def logout(self, cookie_header):
        tok = self._cookie_token(cookie_header)
        with self.lock:
            self.sessions.pop(tok, None)

    def kick(self, token=None):
        with self.lock:
            if token:
                self.sessions.pop(token, None)
            else:
                self.sessions.clear()

    def status(self):
        now = time.time()
        with self.lock:
            guests = [{'token': t, 'name': s['name'], 'ip': s['ip'],
                       'since': s['since'], 'idle_s': round(now - s['seen'])}
                      for t, s in self.sessions.items()]
        return {'key': self.key, 'guests': sorted(guests, key=lambda g: g['since'])}

    @staticmethod
    def cookie(token, clear=False):
        if clear:
            return f'{COOKIE}=; Path=/; Max-Age=0; HttpOnly; SameSite=Strict'
        return f'{COOKIE}={token}; Path=/; Max-Age={SESSION_TTL_S}; HttpOnly; SameSite=Strict'


LOGIN_PAGE = """<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Teleoperación minisumo · acceso</title>
<style>
  :root { --bg:#07080c; --card:#12141c; --line:#262a38; --text:#eef0f7; --muted:#9aa0b4; --accent:#7c6cff; --pink:#ff4fa3; --bad:#ff6b6b; }
  @media (prefers-color-scheme: light) { :root { --bg:#eef0f6; --card:#ffffff; --line:#dde1ec; --text:#0f1222; --muted:#5c6379; --accent:#5b4cf0; --bad:#d6336c; } }
  * { box-sizing:border-box; }
  body { margin:0; min-height:100vh; display:grid; place-items:center; padding:16px;
         background:var(--bg); color:var(--text); font:15px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; }
  .card { width:100%; max-width:380px; background:var(--card); border:1px solid var(--line); border-radius:16px; padding:28px; }
  .bar { height:4px; border-radius:4px; background:linear-gradient(90deg, var(--accent), var(--pink)); margin:-8px 0 22px; }
  h1 { font-size:20px; margin:0 0 6px; }
  p { color:var(--muted); margin:0 0 20px; font-size:14px; }
  label { display:block; font-size:13px; color:var(--muted); margin:14px 0 6px; }
  input { width:100%; padding:11px 12px; border-radius:10px; border:1px solid var(--line); background:transparent; color:var(--text); font-size:15px; }
  input:focus { outline:2px solid var(--accent); outline-offset:1px; border-color:transparent; }
  button { width:100%; margin-top:22px; padding:12px; border:0; border-radius:10px; background:var(--accent); color:#fff; font-size:15px; font-weight:600; cursor:pointer; }
  button:disabled { opacity:.6; cursor:default; }
  .err { color:var(--bad); font-size:13px; min-height:18px; margin-top:12px; }
</style></head>
<body>
<form class="card" id="f" autocomplete="off">
  <div class="bar"></div>
  <h1>Teleoperación minisumo</h1>
  <p>Este tablero es privado. Escribe la clave que te dio el dueño para entrar.</p>
  <label for="name">Tu nombre</label>
  <input id="name" maxlength="40" placeholder="Ej: Juan" required>
  <label for="key">Clave de acceso</label>
  <input id="key" type="password" placeholder="sumo-giro-1234" required>
  <button id="go" type="submit">Entrar</button>
  <div class="err" id="err" role="alert"></div>
</form>
<script>
  const f = document.getElementById('f'), go = document.getElementById('go'), err = document.getElementById('err');
  try { document.getElementById('name').value = localStorage.getItem('minisumo_nombre') || ''; } catch (e) {}
  f.addEventListener('submit', async (ev) => {
    ev.preventDefault();
    go.disabled = true; err.textContent = '';
    const name = document.getElementById('name').value, key = document.getElementById('key').value;
    try { localStorage.setItem('minisumo_nombre', name); } catch (e) {}
    try {
      const r = await fetch('/api/login', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ name, key }) });
      const d = await r.json().catch(() => ({}));
      if (r.ok) { location.reload(); return; }
      err.textContent = d.error || 'No se pudo entrar.';
    } catch (e) { err.textContent = 'Sin conexión con el tablero.'; }
    go.disabled = false;
  });
</script>
</body></html>
"""
