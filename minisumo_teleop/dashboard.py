"""Tablero web de tele-operacion (Practica 3: filtro de Kalman).

Puente entre ROS 2 y el navegador:
  - Se suscribe a todos los topicos del guante, del carro, de los
    estimadores y de la tele-operacion, y envia un resumen al navegador
    ~25 veces por segundo (Server-Sent Events).
  - Recibe ordenes del navegador (armar/desarmar, estrategia, ajustes,
    cero del guante, reinicios) y las aplica con parametros y topicos.
  - Graba sesiones en CSV, guarda las tablas de experimentos de la guia
    y corre el benchmark y la evaluacion offline EKF vs UKF.

Uso:
    ros2 run minisumo_teleop dashboard --ros-args -p port:=8090
y abrir http://localhost:8090
"""

import json
import math
import mimetypes
import os
import queue
import socket
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, Twist, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Empty, Float32, String

from .battery import BatteryMonitor, DEFAULTS as BATTERY_DEFAULTS
from . import glove_teleop, hand_estimator, robot_estimator
from .common import quat_to_rpy, stamp_to_sec
from .models import HAND_DEFAULTS, ROBOT_DEFAULTS
from .offline_eval import evaluate
from .access import Access, LOGIN_PAGE
from .recording import DATA_ROOT, SessionRecorder, list_sessions

PUSH_HZ = 25.0
EXPERIMENTS_FILE = os.path.join(DATA_ROOT, 'experiments.json')

# Tipos de cada parametro remoto (el cliente de parametros exige el tipo
# exacto con el que se declaro).
REMOTE = {
    'glove_teleop': dict(glove_teleop.PARAMS),
    'robot_estimator': {**robot_estimator.NODE_DEFAULTS,
                        **{k: float(v) for k, v in ROBOT_DEFAULTS.items() if k != 'gyro_has_bias'}},
    'hand_estimator': {**hand_estimator.NODE_DEFAULTS, **{k: float(v) for k, v in HAND_DEFAULTS.items()}},
}


def _coerce(default, value):
    if isinstance(default, bool):
        return bool(value)
    if isinstance(default, int):
        return int(round(float(value)))
    if isinstance(default, float):
        return float(value)
    return str(value)


def _web_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('minisumo_teleop'), 'web')
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


class Rate:
    """Frecuencia de llegada de un topico (Hz) y edad del ultimo mensaje."""

    def __init__(self):
        self.count = 0
        self.hz = 0.0
        self.t_last = 0.0
        self._t0 = time.monotonic()

    def tick(self):
        self.count += 1
        self.t_last = time.monotonic()

    def update(self, now):
        dt = now - self._t0
        if dt >= 1.0:
            self.hz = self.count / dt
            self.count = 0
            self._t0 = now

    def age(self, now):
        return (now - self.t_last) if self.t_last else None


class Experiments:
    """Tablas de la guia: punto, posicion esperada / medida / estimada."""

    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()
        self.rows = []
        try:
            with open(path) as f:
                self.rows = json.load(f).get('rows', [])
        except (OSError, ValueError):
            self.rows = []

    def _save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump({'rows': self.rows}, f, indent=1)
        os.replace(tmp, self.path)

    def add(self, d):
        with self.lock:
            row = {
                'id': uuid.uuid4().hex[:10],
                'strategy': int(d.get('strategy', 2)),
                'condition': str(d.get('condition', 'base')),
                'point': str(d.get('point', '')),
                'expected': {'x': float(d.get('ex', 0.0)), 'y': float(d.get('ey', 0.0))},
                'measured': None,
                'estimates': None,
                'created': time.time(),
            }
            self.rows.append(row)
            self._save()
            return row

    def update(self, d):
        with self.lock:
            for r in self.rows:
                if r['id'] == d.get('id'):
                    if 'point' in d:
                        r['point'] = str(d['point'])
                    if 'ex' in d or 'ey' in d:
                        r['expected'] = {'x': float(d.get('ex', r['expected']['x'])),
                                         'y': float(d.get('ey', r['expected']['y']))}
                    if 'mx' in d and 'my' in d:
                        if d['mx'] in (None, '') or d['my'] in (None, ''):
                            r['measured'] = None
                        else:
                            r['measured'] = {'x': float(d['mx']), 'y': float(d['my'])}
                    self._save()
                    return r
        return None

    def capture(self, rid, estimates):
        with self.lock:
            for r in self.rows:
                if r['id'] == rid:
                    r['estimates'] = estimates
                    r['captured_at'] = time.time()
                    self._save()
                    return r
        return None

    def delete(self, rid):
        with self.lock:
            self.rows = [r for r in self.rows if r['id'] != rid]
            self._save()

    def clear(self, strategy=None, condition=None):
        with self.lock:
            self.rows = [r for r in self.rows
                         if not ((strategy is None or r['strategy'] == strategy)
                                 and (condition is None or r['condition'] == condition))]
            self._save()

    def csv(self):
        lines = ['estrategia,condicion,punto,esperada_x,esperada_y,medida_x,medida_y,'
                 'estimada_fuente,estimada_x,estimada_y,odom_x,odom_y,ekf_x,ekf_y,ukf_x,ukf_y,'
                 'error_esperada_medida_m,error_medida_estimada_m']
        src_for = {'base': 'odom', 'filtrado': 'odom', 'ekf': 'ekf', 'ukf': 'ukf'}
        with self.lock:
            for r in self.rows:
                m, e = r['measured'], r['estimates'] or {}
                src = src_for.get(r['condition'], 'ekf')
                est = e.get(src)

                def f(v):
                    return '' if v is None else f'{v:.4f}'

                def g(dct, k):
                    return f(dct[k]) if dct else ''
                e_em = math.hypot(r['expected']['x'] - m['x'], r['expected']['y'] - m['y']) if m else None
                e_me = math.hypot(m['x'] - est['x'], m['y'] - est['y']) if (m and est) else None
                lines.append(','.join([
                    str(r['strategy']), r['condition'], '"' + r['point'].replace('"', "'") + '"',
                    f(r['expected']['x']), f(r['expected']['y']), g(m, 'x'), g(m, 'y'),
                    src, g(est, 'x'), g(est, 'y'),
                    g(e.get('odom'), 'x'), g(e.get('odom'), 'y'),
                    g(e.get('ekf'), 'x'), g(e.get('ekf'), 'y'),
                    g(e.get('ukf'), 'x'), g(e.get('ukf'), 'y'),
                    f(e_em), f(e_me)]))
        return '\n'.join(lines) + '\n'


class Dashboard(Node):
    def __init__(self):
        super().__init__('teleop_dashboard')
        self.declare_parameter('port', 8090)
        self.declare_parameter('guest_port', 8091)
        for k, v in BATTERY_DEFAULTS.items():
            self.declare_parameter(k, v)
        self.battery = BatteryMonitor({k: self.get_parameter(k).value for k in BATTERY_DEFAULTS})
        self.lock = threading.Lock()
        self.rec = SessionRecorder()
        self.exp = Experiments(EXPERIMENTS_FILE)
        self.rates = {k: Rate() for k in (
            'glove_raw', 'glove_filtered', 'glove_rpy', 'car_imu_raw', 'car_imu_filtered', 'wheels',
            'odom_twist', 'hand_ekf', 'robot_ekf', 'teleop', 'cmd_vel', 'espnow', 'battery')}
        # Enlace que mueve al carro: 'espnow' (guante -> carro directo) o
        # 'pc' (glove_teleop -> /cmd_vel; necesario para la estrategia 1)
        self.mode = 'espnow'

        z3 = [0.0, 0.0, 0.0]
        self.s = {
            'glove': {'raw': {'a': z3, 'g': z3}, 'filt': {'q': [0, 0, 0, 1], 'a': z3, 'g': z3}, 'rpy': z3},
            'car': {'raw': {'a': z3, 'g': z3}, 'filt': {'q': [0, 0, 0, 1], 'a': z3, 'g': z3},
                    'wheels': {'wl_raw': 0.0, 'wl_filt': 0.0, 'wr_raw': 0.0, 'wr_filt': 0.0},
                    'odom': {'x': 0.0, 'y': 0.0, 'th': 0.0, 'v': 0.0, 'w': 0.0},
                    'goto': {'seq': 0, 'state': 0}},
            'hand': {'EKF': None, 'UKF': None, 'stats': None},
            'robot': {'EKF': None, 'UKF': None, 'stats': None},
            'teleop': None,
            'cmd': {'v': 0.0, 'w': 0.0},
            'glove_armed': False,
            'espnow': {'link': 0, 'v': 0.0, 'w': 0.0, 'rate': 0.0, 'rssi': 0, 'guard': 0},
        }
        self.remote = {k: dict(v) for k, v in REMOTE.items()}
        self.param_clients = {k: AsyncParameterClient(self, k) for k in REMOTE}

        sq = qos_profile_sensor_data
        sub = self.create_subscription
        sub(Imu, 'glove/imu/raw', self._glove_raw, sq)
        sub(Imu, 'glove/imu/filtered', self._glove_filt, sq)
        sub(Vector3, 'glove/rpy', self._glove_rpy, sq)
        sub(Imu, 'imu/raw', self._car_raw, sq)
        sub(Imu, 'imu/filtered', self._car_filt, sq)
        for key in ('wl_raw', 'wl_filt', 'wr_raw', 'wr_filt'):
            side = 'left' if key.startswith('wl') else 'right'
            kind = 'raw' if key.endswith('raw') else 'filtered'
            sub(Float32, f'wheel_{side}/{kind}', lambda m, k=key: self._wheel(k, m), sq)
        sub(Pose, 'odom/pose', self._odom_pose, sq)
        sub(Twist, 'odom/twist', self._odom_twist, sq)
        sub(Twist, 'cmd_vel', self._cmd_vel, 10)
        for f in ('EKF', 'UKF'):
            sub(Odometry, f'hand/{f.lower()}/odom', lambda m, f=f: self._hand(f, m), 10)
            sub(Vector3Stamped, f'hand/{f.lower()}/accel', lambda m, f=f: self._hand_acc(f, m), 10)
            sub(Odometry, f'robot/{f.lower()}/odom', lambda m, f=f: self._robot(f, m), 10)
        sub(String, 'hand/stats', lambda m: self._stats('hand', m), 10)
        sub(String, 'robot/stats', lambda m: self._stats('robot', m), 10)
        sub(String, 'teleop/state', self._teleop, 10)
        sub(Bool, 'glove/armed', self._glove_armed, sq)
        sub(Twist, 'teleop/espnow', self._espnow, sq)
        sub(BatteryState, 'battery/state', self._battery, sq)

        self.pub_zero = self.create_publisher(Empty, 'glove/zero', 10)
        self.pub_hand_reset = self.create_publisher(Empty, 'hand/reset', 10)
        self.pub_robot_reset = self.create_publisher(Empty, 'robot/reset', 10)
        self.pub_goto = self.create_publisher(Twist, 'goto_cmd', 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_arm = self.create_publisher(Bool, 'glove/arm', 10)

    # ---------------- callbacks de ROS ----------------
    def _t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _v3(v):
        return [v.x, v.y, v.z]

    def _glove_raw(self, m):
        self.rates['glove_raw'].tick()
        a, g = self._v3(m.linear_acceleration), self._v3(m.angular_velocity)
        with self.lock:
            self.s['glove']['raw'] = {'a': a, 'g': g}
            self.rec.write('glove_raw', [self._t(), stamp_to_sec(m.header.stamp), *a, *g])

    def _glove_filt(self, m):
        self.rates['glove_filtered'].tick()
        q = [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]
        a, g = self._v3(m.linear_acceleration), self._v3(m.angular_velocity)
        with self.lock:
            self.s['glove']['filt'] = {'q': q, 'a': a, 'g': g}
            self.rec.write('glove_filtered', [self._t(), stamp_to_sec(m.header.stamp), *q, *a, *g])

    def _glove_rpy(self, m):
        self.rates['glove_rpy'].tick()
        with self.lock:
            self.s['glove']['rpy'] = [m.x, m.y, m.z]
            self.rec.write('glove_rpy', [self._t(), m.x, m.y, m.z])

    def _car_raw(self, m):
        self.rates['car_imu_raw'].tick()
        a, g = self._v3(m.linear_acceleration), self._v3(m.angular_velocity)
        with self.lock:
            self.s['car']['raw'] = {'a': a, 'g': g}
            self.rec.write('car_imu_raw', [self._t(), *a, *g])

    def _car_filt(self, m):
        self.rates['car_imu_filtered'].tick()
        q = [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]
        a, g = self._v3(m.linear_acceleration), self._v3(m.angular_velocity)
        with self.lock:
            self.s['car']['filt'] = {'q': q, 'a': a, 'g': g}
            self.rec.write('car_imu_filtered', [self._t(), *q, *a, *g])

    def _wheel(self, key, m):
        with self.lock:
            w = self.s['car']['wheels']
            w[key] = m.data
            if key == 'wr_filt':   # ultimo de los 4 en cada ciclo del ESP32
                self.rates['wheels'].tick()
                self.rec.write('wheels', [self._t(), w['wl_raw'], w['wl_filt'], w['wr_raw'], w['wr_filt']])

    def _odom_pose(self, m):
        _, _, th = quat_to_rpy(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)
        with self.lock:
            o = self.s['car']['odom']
            o['x'], o['y'], o['th'] = m.position.x, m.position.y, th

    def _odom_twist(self, m):
        self.rates['odom_twist'].tick()
        with self.lock:
            o = self.s['car']['odom']
            o['v'], o['w'] = m.linear.x, m.angular.z
            self.s['car']['goto'] = {'seq': int(m.linear.y), 'state': int(m.linear.z)}
            self.rec.write('odom', [self._t(), o['x'], o['y'], o['th'], o['v'], o['w']])

    def _cmd_vel(self, m):
        self.rates['cmd_vel'].tick()
        with self.lock:
            self.s['cmd'] = {'v': m.linear.x, 'w': m.angular.z}
            self.rec.write('cmd_vel', [self._t(), m.linear.x, m.angular.z])

    def _hand(self, f, m):
        if f == 'EKF':
            self.rates['hand_ekf'].tick()
        q = m.pose.pose.orientation
        _, _, psi = quat_to_rpy(q.x, q.y, q.z, q.w)
        p, v = m.pose.pose.position, m.twist.twist.linear
        with self.lock:
            prev = self.s['hand'][f] or {}
            self.s['hand'][f] = {'x': p.x, 'y': p.y, 'vx': v.x, 'vy': v.y, 'psi': psi,
                                 'ax': prev.get('ax', 0.0), 'ay': prev.get('ay', 0.0)}
            if f == 'UKF' and self.s['hand']['EKF']:
                e, u = self.s['hand']['EKF'], self.s['hand']['UKF']
                st = self.s['hand']['stats'] or {}
                self.rec.write('hand_est', [self._t(), e['x'], e['y'], e['vx'], e['vy'], u['x'], u['y'], u['vx'], u['vy'],
                                            st.get('EKF', {}).get('us', 0.0), st.get('UKF', {}).get('us', 0.0),
                                            int(bool(st.get('zupt')))])

    def _hand_acc(self, f, m):
        with self.lock:
            if self.s['hand'][f]:
                self.s['hand'][f]['ax'], self.s['hand'][f]['ay'] = m.vector.x, m.vector.y

    def _robot(self, f, m):
        if f == 'EKF':
            self.rates['robot_ekf'].tick()
        q = m.pose.pose.orientation
        _, _, th = quat_to_rpy(q.x, q.y, q.z, q.w)
        c = m.pose.covariance
        with self.lock:
            self.s['robot'][f] = {'x': m.pose.pose.position.x, 'y': m.pose.pose.position.y, 'th': th,
                                  'v': m.twist.twist.linear.x, 'w': m.twist.twist.angular.z,
                                  'cov': [c[0], c[7], c[1], c[35]]}
            if f == 'UKF' and self.s['robot']['EKF']:
                e, u = self.s['robot']['EKF'], self.s['robot']['UKF']
                st = self.s['robot']['stats'] or {}
                self.rec.write('robot_est', [self._t(), e['x'], e['y'], e['th'], e['v'], u['x'], u['y'], u['th'], u['v'],
                                             st.get('EKF', {}).get('us', 0.0), st.get('UKF', {}).get('us', 0.0)])

    def _stats(self, which, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.s[which]['stats'] = d

    def _teleop(self, m):
        self.rates['teleop'].tick()
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.s['teleop'] = d
            if 'params' in d:
                self.remote['glove_teleop'].update(d['params'])
            self.rec.write('teleop', [self._t(), int(bool(d.get('enabled'))), d.get('strategy', 0),
                                      d.get('v', 0.0), d.get('w', 0.0), d.get('reason', '')])

    def _glove_armed(self, m):
        with self.lock:
            self.s['glove_armed'] = bool(m.data)

    def _espnow(self, m):
        self.rates['espnow'].tick()
        e = {'link': int(m.linear.z), 'v': m.linear.x, 'w': m.angular.z,
             'rate': m.linear.y, 'rssi': int(m.angular.x), 'guard': int(m.angular.y)}
        with self.lock:
            self.s['espnow'] = e
            self.rec.write('espnow', [self._t(), e['link'], e['v'], e['w'], e['rate'], e['rssi'], e['guard']])

    def _battery(self, m):
        self.rates['battery'].tick()
        with self.lock:
            self.battery.update(m.voltage, m.current, m.charge, m.capacity, m.present)
            b = self.battery
            self.rec.write('battery', [self._t(), b.v, b.i, b.p, b.used_mah, b.soc if b.soc is not None else ''])

    # ---------------- snapshot para el navegador ----------------
    def snapshot(self):
        now = time.monotonic()
        for r in self.rates.values():
            r.update(now)

        def link(*keys):
            ages = [self.rates[k].age(now) for k in keys]
            ages = [a for a in ages if a is not None]
            return {'age': min(ages) if ages else None, 'hz': max(self.rates[k].hz for k in keys)}

        with self.lock:
            snap = json.loads(json.dumps(self.s))
            snap['remote'] = {k: dict(v) for k, v in self.remote.items()}
            snap['mode'] = self.mode
            snap['battery'] = self.battery.state()
            snap['recording'] = {'active': self.rec.active,
                                 'name': os.path.basename(self.rec.dir) if self.rec.dir else None,
                                 'rows': self.rec.rows}
        snap['t'] = time.time()
        snap['links'] = {
            'glove': link('glove_raw', 'glove_filtered'),
            'car': link('car_imu_raw', 'odom_twist'),
            'hand_est': link('hand_ekf'),
            'robot_est': link('robot_ekf'),
            'teleop': link('teleop'),
            'espnow': link('espnow'),
            'battery': link('battery'),
        }
        snap['rates'] = {k: round(r.hz, 1) for k, r in self.rates.items()}
        return snap

    # ---------------- acciones ----------------
    def set_remote(self, node_name, values):
        defaults = REMOTE[node_name]
        params = []
        for k, v in values.items():
            if k not in defaults:
                continue
            val = _coerce(defaults[k], v)
            params.append(Parameter(k, value=val))
            with self.lock:
                self.remote[node_name][k] = val
        if not params:
            return False
        client = self.param_clients[node_name]
        if not client.services_are_ready():
            return False
        client.set_parameters(params)
        return True

    def arm_glove(self, on):
        for _ in range(3):   # el guante lo recibe por micro-ROS (fiable)
            self.pub_arm.publish(Bool(data=bool(on)))

    def set_mode(self, mode):
        """Un solo enlace mueve al carro a la vez."""
        self.mode = 'pc' if mode == 'pc' else 'espnow'
        if self.mode == 'pc':
            self.arm_glove(False)
        else:
            self.set_remote('glove_teleop', {'enabled': False})

    def estop(self):
        self.set_remote('glove_teleop', {'enabled': False})
        self.arm_glove(False)
        for _ in range(3):
            self.pub_cmd.publish(Twist())

    def zero_glove(self):
        self.pub_zero.publish(Empty())
        self.pub_hand_reset.publish(Empty())

    def reset_robot(self, also_car=True):
        self.pub_robot_reset.publish(Empty())
        if also_car:
            # goto_cmd modo 2 = reset del cero en el ESP32 (con secuencia
            # nueva para que no la descarte como duplicada)
            m = Twist()
            m.angular.z = 2.0
            m.angular.y = float(int(time.time() * 10) % 2_000_000_000)
            for _ in range(3):
                self.pub_goto.publish(m)

    def reset_system(self):
        """Detiene y desarma todo y toma la posicion actual del carro como
        nuevo origen: odometria del ESP32 (x, y y rumbo a 0), EKF/UKF del
        robot y estimador de la mano."""
        self.estop()
        self.reset_robot(also_car=True)
        self.pub_hand_reset.publish(Empty())

    def current_estimates(self):
        with self.lock:
            o = self.s['car']['odom']
            out = {'odom': {'x': o['x'], 'y': o['y'], 'th': o['th']}}
            for f in ('EKF', 'UKF'):
                r = self.s['robot'][f]
                if r:
                    out[f.lower()] = {'x': r['x'], 'y': r['y'], 'th': r['th']}
        return out


# ======================================================================
# Servidor HTTP
# ======================================================================
def make_handler(node, web_dir, guest_only=False):
    """guest_only: puerto de invitados, la clave se pide siempre (aunque la
    conexion llegue de 127.0.0.1, como pasa con Tailscale en modo usuario)."""
    subscribers = node._sse_queues
    access = node.access

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype='application/json', extra=None):
            if isinstance(body, (dict, list)):
                body = json.dumps(body)
            if isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json_body(self):
            n = int(self.headers.get('Content-Length') or 0)
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n) or b'{}')
            except ValueError:
                return {}

        # ---------- Acceso (dueno = este PC; invitados = clave) ----------
        def _ip(self):
            return self.client_address[0]

        def _is_owner(self):
            # Dueno: puerto principal, conexion desde este PC y el navegador
            # abrio localhost. Tailscale en modo usuario entrega las conexiones
            # remotas desde 127.0.0.1, pero con Host = 100.x / nombre del PC.
            if guest_only or not access.is_local(self._ip()):
                return False
            host = (self.headers.get('Host') or '').rsplit(':', 1)[0].strip('[]').lower()
            return host in ('localhost', '127.0.0.1', '::1')

        def _allowed(self):
            return self._is_owner() or access.guest(self._ip(), self.headers.get('Cookie')) is not None

        def _access_get(self, path):
            if path == '/api/whoami':
                if self._is_owner():
                    return self._send(200, {'owner': True})
                g = access.guest(self._ip(), self.headers.get('Cookie'))
                return self._send(200, {'owner': False, 'name': g['name'] if g else None})
            if path == '/api/access':
                if not self._is_owner():
                    return self._send(403, {'error': 'solo desde el PC del dueño'})
                st = access.status()
                st.update(port=node.guest_port, lan_ip=_lan_ip(), tailscale_ip=_tailscale_ip())
                return self._send(200, st)
            return None

        # ---------- GET ----------
        def do_GET(self):
            path = urlparse(self.path).path
            if not self._allowed():
                if path in ('', '/', '/index.html'):
                    return self._send(200, LOGIN_PAGE, 'text/html; charset=utf-8')
                return self._send(401, {'error': 'necesitas la clave de acceso'})
            if path in ('/api/whoami', '/api/access'):
                return self._access_get(path)
            if path == '/events':
                return self._events()
            if path == '/api/experiments':
                return self._send(200, {'rows': node.exp.rows})
            if path == '/api/experiments.csv':
                return self._send(200, node.exp.csv(), 'text/csv; charset=utf-8',
                                  {'Content-Disposition': 'attachment; filename="experimentos_teleoperacion.csv"'})
            if path == '/api/sessions':
                return self._send(200, {'sessions': list_sessions()})
            if path == '/api/snapshot':
                return self._send(200, node.snapshot())
            if path == '/api/battery/history':
                with node.lock:
                    return self._send(200, {'points': node.battery.history()})
            return self._static(path)

        def _static(self, path):
            if path in ('', '/'):
                path = '/index.html'
            full = os.path.normpath(os.path.join(web_dir, path.lstrip('/')))
            if not full.startswith(os.path.normpath(web_dir)) or not os.path.isfile(full):
                return self._send(404, {'error': 'no encontrado'})
            ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
            if full.endswith('.js'):
                ctype = 'text/javascript; charset=utf-8'
            with open(full, 'rb') as f:
                data = f.read()
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache' if '/vendor/' not in full else 'max-age=86400')
            self.end_headers()
            self.wfile.write(data)

        def _events(self):
            q = queue.Queue(maxsize=4)
            subscribers.append(q)
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while True:
                    try:
                        data = q.get(timeout=5.0)
                    except queue.Empty:
                        data = None
                    chunk = (f'data: {data}\n\n' if data else ': ping\n\n').encode()
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                if q in subscribers:
                    subscribers.remove(q)

        # ---------- POST ----------
        def do_POST(self):
            path = urlparse(self.path).path
            d = self._json_body()
            if path == '/api/login':
                tok, err = access.login(self._ip(), d.get('key'), d.get('name'))
                if err:
                    return self._send(401, {'error': err})
                return self._send(200, {'ok': True}, extra={'Set-Cookie': access.cookie(tok)})
            if path == '/api/logout':
                access.logout(self.headers.get('Cookie'))
                return self._send(200, {'ok': True}, extra={'Set-Cookie': access.cookie(None, clear=True)})
            if not self._allowed():
                return self._send(401, {'error': 'necesitas la clave de acceso'})
            if path in ('/api/access/key', '/api/access/kick'):
                if not self._is_owner():
                    return self._send(403, {'error': 'solo desde el PC del dueño'})
                try:
                    if path == '/api/access/key':
                        return self._send(200, {'key': access.set_key(d.get('key'))})
                    access.kick(d.get('token'))
                    return self._send(200, {'ok': True})
                except ValueError as e:
                    return self._send(400, {'error': str(e)})
            try:
                return self._post(path, d)
            except Exception as e:   # el tablero no debe caerse por una orden
                node.get_logger().error(f'{path}: {e}')
                return self._send(500, {'error': str(e)})

        def _post(self, path, d):
            if path == '/api/teleop':
                ok = node.set_remote('glove_teleop', d)
                return self._send(200 if ok else 503, {'ok': ok, 'error': None if ok else 'glove_teleop no responde'})
            if path == '/api/estimator':
                target = d.pop('target', 'robot')
                name = 'robot_estimator' if target == 'robot' else 'hand_estimator'
                ok = node.set_remote(name, d)
                return self._send(200 if ok else 503, {'ok': ok, 'error': None if ok else f'{name} no responde'})
            if path == '/api/mode':
                node.set_mode(d.get('mode', 'espnow'))
                return self._send(200, {'ok': True, 'mode': node.mode})
            if path == '/api/glove/arm':
                node.arm_glove(bool(d.get('on')))
                return self._send(200, {'ok': True})
            if path == '/api/estop':
                node.estop()
                return self._send(200, {'ok': True})
            if path == '/api/glove/zero':
                node.zero_glove()
                return self._send(200, {'ok': True})
            if path == '/api/hand/reset':
                node.pub_hand_reset.publish(Empty())
                return self._send(200, {'ok': True})
            if path == '/api/system/reset':
                node.reset_system()
                return self._send(200, {'ok': True})
            if path == '/api/battery/reset':
                with node.lock:
                    node.battery.reset()
                return self._send(200, {'ok': True})
            if path == '/api/robot/reset':
                node.reset_robot(bool(d.get('car', True)))
                return self._send(200, {'ok': True})
            if path == '/api/record':
                if d.get('on'):
                    with node.lock:
                        name = node.rec.start(d.get('label', ''))
                    return self._send(200, {'ok': True, 'name': name})
                with node.lock:
                    name = node.rec.stop()
                return self._send(200, {'ok': True, 'name': name})
            if path == '/api/experiments/add':
                return self._send(200, node.exp.add(d))
            if path == '/api/experiments/update':
                r = node.exp.update(d)
                return self._send(200 if r else 404, r or {'error': 'no existe'})
            if path == '/api/experiments/capture':
                r = node.exp.capture(d.get('id'), node.current_estimates())
                return self._send(200 if r else 404, r or {'error': 'no existe'})
            if path == '/api/experiments/delete':
                node.exp.delete(d.get('id'))
                return self._send(200, {'ok': True})
            if path == '/api/experiments/clear':
                st = d.get('strategy')
                node.exp.clear(int(st) if st is not None else None, d.get('condition'))
                return self._send(200, {'ok': True})
            if path == '/api/benchmark':
                steps = int(max(200, min(5000, d.get('steps', 1500))))
                # En un proceso aparte: dentro del tablero los hilos de ROS
                # inflarian el tiempo y la memoria medidos.
                out = subprocess.run([sys.executable, '-m', 'minisumo_teleop.models', str(steps)],
                                     capture_output=True, text=True, timeout=300)
                if out.returncode != 0:
                    return self._send(500, {'error': out.stderr.strip().splitlines()[-1] if out.stderr else 'benchmark fallo'})
                return self._send(200, json.loads(out.stdout))
            if path == '/api/offline':
                res = evaluate(d['session'], d.get('robot_mode', 'raw'), d.get('hand_mode', 'filtered'))
                return self._send(200, res)
            return self._send(404, {'error': 'ruta desconocida'})

    return Handler


def _lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return None


def _tailscale_ip():
    """IP de este PC en Tailscale (100.x.y.z), o None si no esta instalado/conectado."""
    import subprocess
    exe = os.path.expanduser('~/.local/bin/tailscale')   # instalacion sin root (modo usuario)
    try:
        out = subprocess.run([exe if os.path.exists(exe) else 'tailscale', 'ip', '-4'],
                             capture_output=True, text=True, timeout=2)
        ip = out.stdout.strip().splitlines()[0] if out.returncode == 0 and out.stdout.strip() else None
        return ip
    except (OSError, subprocess.SubprocessError, IndexError):
        return None


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    node._sse_queues = []
    node.access = Access(node.get_logger())
    port = int(node.get_parameter('port').value)
    web_dir = _web_dir()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(node, web_dir))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    # Puerto de invitados (LAN o Tailscale): siempre pide la clave
    node.guest_port = int(node.get_parameter('guest_port').value)
    gserver = ThreadingHTTPServer(('0.0.0.0', node.guest_port), make_handler(node, web_dir, guest_only=True))
    gserver.daemon_threads = True
    threading.Thread(target=gserver.serve_forever, daemon=True).start()

    node.get_logger().info(f'Tablero en http://localhost:{port}')
    ip = _lan_ip()
    if ip:
        node.get_logger().info(f'Invitados (piden la clave): http://{ip}:{node.guest_port}')

    try:
        period = 1.0 / PUSH_HZ
        while rclpy.ok():
            t0 = time.monotonic()
            if node._sse_queues:
                data = json.dumps(node.snapshot(), separators=(',', ':'))
                for q in list(node._sse_queues):
                    try:
                        q.put_nowait(data)
                    except queue.Full:
                        try:
                            q.get_nowait()
                            q.put_nowait(data)
                        except (queue.Empty, queue.Full):
                            pass
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with node.lock:
            node.rec.stop()
        server.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
