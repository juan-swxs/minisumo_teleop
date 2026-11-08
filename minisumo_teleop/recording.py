"""Formato de las sesiones grabadas (un CSV por fuente de datos).

El tablero escribe estas columnas y la evaluacion offline las lee, asi
que cualquier cambio aqui aplica a ambos. "t" es la hora ROS de llegada
al PC (s); "stamp" es la hora de muestreo del guante (sincronizada con
el agente micro-ROS).
"""

import csv
import os
import time

SCHEMAS = {
    'glove_raw': ['t', 'stamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'],
    'glove_filtered': ['t', 'stamp', 'qx', 'qy', 'qz', 'qw', 'lax', 'lay', 'laz', 'gx', 'gy', 'gz'],
    'glove_rpy': ['t', 'roll', 'pitch', 'yaw'],
    'car_imu_raw': ['t', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'],
    'car_imu_filtered': ['t', 'qx', 'qy', 'qz', 'qw', 'lax', 'lay', 'laz', 'gx', 'gy', 'gz'],
    'wheels': ['t', 'wl_raw', 'wl_filt', 'wr_raw', 'wr_filt'],
    'odom': ['t', 'x', 'y', 'th', 'v', 'w'],
    'cmd_vel': ['t', 'v', 'w'],
    'robot_est': ['t', 'ekf_x', 'ekf_y', 'ekf_th', 'ekf_v', 'ukf_x', 'ukf_y', 'ukf_th', 'ukf_v', 'ekf_us', 'ukf_us'],
    'hand_est': ['t', 'ekf_x', 'ekf_y', 'ekf_vx', 'ekf_vy', 'ukf_x', 'ukf_y', 'ukf_vx', 'ukf_vy', 'ekf_us', 'ukf_us', 'zupt'],
    'teleop': ['t', 'enabled', 'strategy', 'v', 'w', 'reason'],
    'espnow': ['t', 'link', 'v', 'w', 'rate', 'rssi', 'guard'],
    'battery': ['t', 'v', 'i_ma', 'p_w', 'used_mah', 'soc'],
}

DATA_ROOT = os.path.expanduser('~/teleop_data')


class SessionRecorder:
    """Graba cada fuente en su CSV dentro de ~/teleop_data/<fecha>_<nombre>/."""

    def __init__(self):
        self.dir = None
        self._files = {}
        self._writers = {}
        self.rows = 0

    @property
    def active(self):
        return self.dir is not None

    def start(self, label=''):
        self.stop()
        safe = ''.join(c if c.isalnum() or c in '-_' else '_' for c in label.strip())[:40]
        name = time.strftime('%Y%m%d_%H%M%S') + (f'_{safe}' if safe else '')
        self.dir = os.path.join(DATA_ROOT, 'sessions', name)
        os.makedirs(self.dir, exist_ok=True)
        for key, cols in SCHEMAS.items():
            f = open(os.path.join(self.dir, f'{key}.csv'), 'w', newline='')
            w = csv.writer(f)
            w.writerow(cols)
            self._files[key] = f
            self._writers[key] = w
        self.rows = 0
        return name

    def write(self, key, row):
        w = self._writers.get(key)
        if w is not None:
            w.writerow([f'{v:.6f}' if isinstance(v, float) else v for v in row])
            self.rows += 1

    def stop(self):
        for f in self._files.values():
            f.close()
        self._files.clear()
        self._writers.clear()
        name = os.path.basename(self.dir) if self.dir else None
        self.dir = None
        return name


def list_sessions():
    root = os.path.join(DATA_ROOT, 'sessions')
    if not os.path.isdir(root):
        return []
    out = []
    for name in sorted(os.listdir(root), reverse=True):
        d = os.path.join(root, name)
        if not os.path.isdir(d):
            continue
        size = sum(os.path.getsize(os.path.join(d, f)) for f in os.listdir(d))
        out.append({'name': name, 'kb': round(size / 1024.0, 1)})
    return out


def read_csv(session_dir, key):
    path = os.path.join(session_dir, f'{key}.csv')
    if not os.path.exists(path):
        return []
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            row = {}
            for k, v in r.items():
                try:
                    row[k] = float(v)
                except (TypeError, ValueError):
                    row[k] = v
            rows.append(row)
    return rows
