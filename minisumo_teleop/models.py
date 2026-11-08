"""Modelos de sistema y de sensores para los filtros de Kalman.

Cada estimador es un "nucleo" sin dependencias de ROS: lo usan los nodos
en vivo, la evaluacion offline sobre datos grabados y el benchmark, asi
los tres corren exactamente el mismo codigo.

-----------------------------------------------------------------------
ROBOT MINISUMO (plataforma diferencial)
-----------------------------------------------------------------------
Marco: dojo (x adelante al arrancar, y a la izquierda, theta antihorario).

Estado      x = [px, py, theta, v, w, bg]
            px, py  posicion del centro del robot (m)
            theta   rumbo (rad)
            v, w    velocidad lineal (m/s) y angular (rad/s) del chasis
            bg      sesgo del giroscopio z del MPU6050 (rad/s)

Entradas    u = (v_cmd, w_cmd) de /cmd_vel. El PID de cada rueda hace que
            la velocidad real siga al comando con un retardo de primer
            orden (tau_v, tau_w). Sin comando reciente (modo goto o PWM
            directo) v y w evolucionan como caminata aleatoria.

Proceso     px' = px + v cos(theta + w dt/2) dt
            py' = py + v sin(theta + w dt/2) dt         (no lineal)
            theta' = theta + w dt
            v' = v + (dt/tau_v)(v_cmd - v),  w' = w + (dt/tau_w)(w_cmd - w)
            bg' = bg

Sensores    encoders -> v_enc = v                       (marco del robot)
            giroscopio z (crudo) -> g = w + bg          (marco del sensor)
            giroscopio z (filtrado en el ESP32, ya sin sesgo) -> g = w
            tracking externo opcional -> [px, py, theta] (marco del dojo)
            ZARU: ruedas detenidas y sin comando -> pseudo-medicion w = 0
            (hace observable el sesgo bg aunque el robot no se mueva)

-----------------------------------------------------------------------
MANO DEL OPERADOR (guante con MPU6050 + filtro de Mahony en el ESP32)
-----------------------------------------------------------------------
Marco: plano horizontal del operador (x hacia adelante en la postura
neutra fijada con glove/zero, y a la izquierda).

Estado      x = [px, py, vx, vy, ax, ay, psi]
            posicion, velocidad y aceleracion de la mano en el plano y
            rumbo psi de la mano respecto a la postura neutra.

Entradas    w_z: velocidad angular vertical (giroscopio llevado al marco
            del mundo con la orientacion del guante). Mueve psi.

Proceso     modelo de Singer (aceleracion correlacionada en el tiempo):
            p' = p + v dt + a dt^2/2
            v' = (1 - leak) v + a dt
            a' = exp(-dt/tau_a) a
            psi' = psi + w_z dt

Sensores    acelerometro (aceleracion lineal, sin gravedad) compensado en
            inclinacion -> marco de rumbo de la mano:
                z_a = Rz(psi)^T [ax, ay]                (no lineal en psi)
            rumbo de la fusion del guante -> z_psi = psi
            ZUPT (mano quieta) -> pseudo-medicion v = 0
"""

import math
import time
import tracemalloc

import numpy as np

from .kalman import EKF, UKF, wrap_pi


# ======================================================================
# Robot
# ======================================================================
ROBOT_DEFAULTS = {
    'q_pos': 1e-5,        # m^2/s      error de modelo en posicion
    'q_theta': 1e-4,      # rad^2/s
    'q_v': 0.1,           # (m/s)^2/s  cuanto puede cambiar v fuera del modelo
    'q_w': 0.5,           # (rad/s)^2/s
    'q_bias': 1e-6,       # (rad/s)^2/s deriva del sesgo
    'r_v': 0.003,         # (m/s)^2    encoders de 40 pulsos/vuelta: cuantizados
    'r_gyro': 4e-4,       # (rad/s)^2
    'r_pose_xy': 4e-4,    # m^2        tracking externo
    'r_pose_th': 4e-3,    # rad^2
    'r_zaru': 1e-5,       # (rad/s)^2  robot quieto -> w = 0
    'still_wheel': 0.3,   # rad/s     por debajo, la rueda esta detenida
    'tau_v': 0.25,        # s
    'tau_w': 0.20,        # s
    'cmd_timeout': 0.5,   # s
    'gyro_has_bias': 1.0, # 1: giroscopio crudo (con sesgo), 0: ya filtrado
}


def robot_f(x, u, dt):
    vc, wc, av, aw = u
    px, py, th, v, w, b = x
    thm = th + 0.5 * w * dt
    return np.array([
        px + v * math.cos(thm) * dt,
        py + v * math.sin(thm) * dt,
        wrap_pi(th + w * dt),
        v + av * (vc - v),
        w + aw * (wc - w),
        b,
    ])


def robot_F(x, u, dt):
    _, _, av, aw = u
    _, _, th, v, w, _ = x
    thm = th + 0.5 * w * dt
    c, s = math.cos(thm), math.sin(thm)
    F = np.eye(6)
    F[0, 2] = -v * s * dt
    F[0, 3] = c * dt
    F[0, 4] = -v * s * 0.5 * dt * dt
    F[1, 2] = v * c * dt
    F[1, 3] = s * dt
    F[1, 4] = v * c * 0.5 * dt * dt
    F[2, 4] = dt
    F[3, 3] = 1.0 - av
    F[4, 4] = 1.0 - aw
    return F


_H_V = np.array([[0, 0, 0, 1.0, 0, 0]])
_H_G_BIAS = np.array([[0, 0, 0, 0, 1.0, 1.0]])
_H_G = np.array([[0, 0, 0, 0, 1.0, 0]])
_H_W = np.array([[0, 0, 0, 0, 1.0, 0]])
_H_POSE = np.array([[1.0, 0, 0, 0, 0, 0], [0, 1.0, 0, 0, 0, 0], [0, 0, 1.0, 0, 0, 0]])


class RobotEstimatorCore:
    """EKF y UKF del robot corriendo en paralelo con los mismos datos."""

    def __init__(self, params=None):
        self.p = dict(ROBOT_DEFAULTS)
        if params:
            self.p.update(params)
        self.t_last = None
        self._cmd = (0.0, 0.0, -1e9)
        self._gyro = None
        self._speed = None
        self._pose = None
        self._wheels = None
        self.still = False
        self.timing_us = {'EKF': 0.0, 'UKF': 0.0}
        self.filters = {}
        self.reset()

    # ---------------- configuracion ----------------
    def set_params(self, **kw):
        self.p.update({k: float(v) for k, v in kw.items() if k in self.p})

    def _Q(self, dt):
        p = self.p
        return np.diag([p['q_pos'], p['q_pos'], p['q_theta'], p['q_v'], p['q_w'], p['q_bias']]) * max(dt, 1e-4)

    def reset(self, x=0.0, y=0.0, th=0.0, bias=None):
        x0 = [x, y, th, 0.0, 0.0, 0.0 if bias is None else bias]
        P0 = np.diag([1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 2.5e-3])
        self.filters = {
            'EKF': EKF(x0, P0, self._Q, angle_idx=(2,)),
            'UKF': UKF(x0, P0, self._Q, angle_idx=(2,)),
        }

    # ---------------- entradas de datos ----------------
    def set_cmd(self, t, v, w):
        self._cmd = (float(v), float(w), float(t))

    def add_gyro(self, gz):
        self._gyro = float(gz)

    def add_speed(self, v):
        self._speed = float(v)

    def add_wheels(self, wl, wr):
        """Rapidez de cada rueda (rad/s, sin signo: encoders de un canal)."""
        self._wheels = (abs(float(wl)), abs(float(wr)))

    def add_pose(self, x, y, th):
        self._pose = (float(x), float(y), float(th))

    # ---------------- ciclo del filtro ----------------
    def step(self, t):
        """Prediccion hasta t + correccion con las mediciones nuevas."""
        if self.t_last is None:
            self.t_last = t
            return
        dt = t - self.t_last
        self.t_last = t
        if dt <= 0.0:
            return
        dt = min(dt, 0.2)
        p = self.p

        vc, wc, tc = self._cmd
        if t - tc <= p['cmd_timeout']:
            u = (vc, wc, min(dt / p['tau_v'], 1.0), min(dt / p['tau_w'], 1.0))
        else:
            u = (0.0, 0.0, 0.0, 0.0)

        gyro, speed, pose, wheels = self._gyro, self._speed, self._pose, self._wheels
        self._gyro = self._speed = self._pose = self._wheels = None

        # Robot quieto: ruedas detenidas y sin comando de movimiento
        if wheels is not None:
            no_cmd = u[2] == 0.0 or (abs(vc) < 1e-3 and abs(wc) < 1e-3)
            self.still = no_cmd and max(wheels) < p['still_wheel']

        H_g = _H_G_BIAS if p['gyro_has_bias'] >= 0.5 else _H_G
        R_v = np.array([[p['r_v']]])
        R_g = np.array([[p['r_gyro']]])
        R_pose = np.diag([p['r_pose_xy'], p['r_pose_xy'], p['r_pose_th']])
        R_w = np.array([[p['r_zaru']]])

        for name, kf in self.filters.items():
            t0 = time.perf_counter_ns()
            kf.predict(robot_f, dt, u, robot_F)
            if speed is not None:
                kf.update([speed], lambda s: _H_V @ s, R_v, lambda s: _H_V)
            if gyro is not None:
                kf.update([gyro], lambda s, H=H_g: H @ s, R_g, lambda s, H=H_g: H)
            if pose is not None:
                kf.update(list(pose), lambda s: _H_POSE @ s, R_pose, lambda s: _H_POSE, z_angles=(2,))
            if self.still and gyro is not None:
                kf.update([0.0], lambda s: _H_W @ s, R_w, lambda s: _H_W)
            us = (time.perf_counter_ns() - t0) / 1000.0
            # promedio movil para que el numero en pantalla sea legible
            self.timing_us[name] = us if self.timing_us[name] == 0.0 else 0.9 * self.timing_us[name] + 0.1 * us

    def state(self, name):
        kf = self.filters[name]
        x = kf.x
        return {
            'x': float(x[0]), 'y': float(x[1]), 'th': float(x[2]),
            'v': float(x[3]), 'w': float(x[4]), 'bias': float(x[5]),
            'cov': [float(kf.P[0, 0]), float(kf.P[1, 1]), float(kf.P[0, 1]), float(kf.P[2, 2])],
            'trace': float(np.trace(kf.P[:3, :3])),
            'nis': float(kf.last_nis),
            'us': float(self.timing_us[name]),
            'bytes': int(kf.nbytes()),
        }


# ======================================================================
# Mano
# ======================================================================
HAND_DEFAULTS = {
    'tau_a': 0.4,         # s   correlacion de la aceleracion (Singer)
    'leak_tau': 0.0,      # s   fuga de velocidad (0 = desactivada). Con 0.6 la velocidad cambiaba de
                          #     signo al frenar la mano (2026-10-26); el modo plano usa trazos
    'q_pos': 1e-6,
    'q_vel': 1e-3,
    'q_acc': 8.0,         # (m/s^2)^2/s  "jerk" del movimiento de la mano
    'q_psi': 1e-4,
    'r_acc': 0.04,        # (m/s^2)^2
    'r_psi': 3e-4,        # rad^2
    'r_zupt': 1e-4,       # (m/s)^2
    'zupt_gyro': 0.25,    # rad/s  por debajo: mano sin girar
    'zupt_acc': 0.35,     # m/s^2  por debajo: mano sin acelerar
    'zupt_time': 0.15,    # s      tiempo quieta para aplicar ZUPT
    'zupt_vmax': 0.15,    # m/s    ZUPT solo si la velocidad estimada ya es baja...
    'zupt_long': 0.3,     # s      ...o si la quietud dura al menos esto (antes 0.08 / 0.6)
}


def hand_f(x, u, dt):
    wz, leak, dec = u
    px, py, vx, vy, ax, ay, psi = x
    h2 = 0.5 * dt * dt
    return np.array([
        px + vx * dt + ax * h2,
        py + vy * dt + ay * h2,
        (1.0 - leak) * vx + ax * dt,
        (1.0 - leak) * vy + ay * dt,
        dec * ax,
        dec * ay,
        wrap_pi(psi + wz * dt),
    ])


def hand_F(x, u, dt):
    _, leak, dec = u
    F = np.eye(7)
    F[0, 2] = F[1, 3] = dt
    F[0, 4] = F[1, 5] = 0.5 * dt * dt
    F[2, 2] = F[3, 3] = 1.0 - leak
    F[2, 4] = F[3, 5] = dt
    F[4, 4] = F[5, 5] = dec
    return F


def hand_h_acc(x):
    c, s = math.cos(x[6]), math.sin(x[6])
    return np.array([c * x[4] + s * x[5], -s * x[4] + c * x[5]])


def hand_H_acc(x):
    c, s = math.cos(x[6]), math.sin(x[6])
    ax, ay = x[4], x[5]
    H = np.zeros((2, 7))
    H[0, 4], H[0, 5], H[0, 6] = c, s, -s * ax + c * ay
    H[1, 4], H[1, 5], H[1, 6] = -s, c, -c * ax - s * ay
    return H


_H_PSI = np.zeros((1, 7))
_H_PSI[0, 6] = 1.0
_H_VEL = np.zeros((2, 7))
_H_VEL[0, 2] = _H_VEL[1, 3] = 1.0


class HandEstimatorCore:
    """EKF y UKF de la mano corriendo en paralelo con los mismos datos."""

    def __init__(self, params=None):
        self.p = dict(HAND_DEFAULTS)
        if params:
            self.p.update(params)
        self.t_last = None
        self.still_since = None
        self.zupt = False
        self.timing_us = {'EKF': 0.0, 'UKF': 0.0}
        self.filters = {}
        self.reset()

    def set_params(self, **kw):
        self.p.update({k: float(v) for k, v in kw.items() if k in self.p})

    def _Q(self, dt):
        p = self.p
        return np.diag([p['q_pos']] * 2 + [p['q_vel']] * 2 + [p['q_acc']] * 2 + [p['q_psi']]) * max(dt, 1e-4)

    def reset(self, psi=0.0):
        x0 = [0, 0, 0, 0, 0, 0, psi]
        P0 = np.diag([1e-6, 1e-6, 1e-4, 1e-4, 0.1, 0.1, 1e-3])
        self.filters = {
            'EKF': EKF(x0, P0, self._Q, angle_idx=(6,)),
            'UKF': UKF(x0, P0, self._Q, angle_idx=(6,)),
        }
        self.still_since = None
        self.zupt = False

    def step(self, t, a_head, psi_meas, wz, gyro_norm):
        """a_head: aceleracion lineal en el marco de rumbo de la mano (2,).
        psi_meas: rumbo de la fusion (rad). wz: giro vertical (rad/s)."""
        if self.t_last is None:
            self.t_last = t
            return
        dt = t - self.t_last
        self.t_last = t
        if dt <= 0.0:
            return
        dt = min(dt, 0.1)
        p = self.p

        # Deteccion de mano quieta para el ZUPT. Aceleracion baja y sin
        # giro tambien ocurre moviendose a velocidad constante, por eso se
        # exige ademas velocidad estimada baja (o una quietud larga).
        a_norm = math.hypot(a_head[0], a_head[1])
        if gyro_norm < p['zupt_gyro'] and a_norm < p['zupt_acc']:
            if self.still_since is None:
                self.still_since = t
            held = t - self.still_since
            x = next(iter(self.filters.values())).x   # el EKF si existe (benchmark: el unico)
            slow = math.hypot(x[2], x[3]) < p['zupt_vmax']
            self.zupt = held >= p['zupt_time'] and (slow or held >= p['zupt_long'])
        else:
            self.still_since = None
            self.zupt = False

        leak = min(dt / p['leak_tau'], 1.0) if p['leak_tau'] > 0 else 0.0
        u = (wz, leak, math.exp(-dt / max(p['tau_a'], 1e-3)))
        R_a = np.eye(2) * p['r_acc']
        R_psi = np.array([[p['r_psi']]])
        R_z = np.eye(2) * p['r_zupt']

        for name, kf in self.filters.items():
            t0 = time.perf_counter_ns()
            kf.predict(hand_f, dt, u, hand_F)
            kf.update(a_head, hand_h_acc, R_a, hand_H_acc)
            kf.update([psi_meas], lambda s: _H_PSI @ s, R_psi, lambda s: _H_PSI, z_angles=(0,))
            if self.zupt:
                kf.update([0.0, 0.0], lambda s: _H_VEL @ s, R_z, lambda s: _H_VEL)
            us = (time.perf_counter_ns() - t0) / 1000.0
            self.timing_us[name] = us if self.timing_us[name] == 0.0 else 0.9 * self.timing_us[name] + 0.1 * us

    def state(self, name):
        kf = self.filters[name]
        x = kf.x
        return {
            'x': float(x[0]), 'y': float(x[1]), 'vx': float(x[2]), 'vy': float(x[3]),
            'ax': float(x[4]), 'ay': float(x[5]), 'psi': float(x[6]),
            'trace': float(np.trace(kf.P[:4, :4])),
            'nis': float(kf.last_nis),
            'us': float(self.timing_us[name]),
            'bytes': int(kf.nbytes()),
        }


# ======================================================================
# Benchmark con datos sinteticos (verdad conocida)
# ======================================================================
def _simulate_robot(steps, dt, rng):
    """Robot recorriendo una curva en S con encoders y giroscopio ruidosos."""
    truth, meas = [], []
    x = np.zeros(6)
    x[5] = 0.03  # sesgo real del giroscopio
    for k in range(steps):
        t = k * dt
        vc = 0.35 if (k // 150) % 4 != 3 else 0.0
        wc = 2.2 * math.sin(0.6 * t)
        u = (vc, wc, dt / 0.25, dt / 0.2)
        x = robot_f(x, u, dt)
        truth.append(x.copy())
        wl = abs(x[3] - x[4] * 0.0765) / 0.0335   # L/2 = 15.3/2 cm, R = 3.35 cm (medidos)
        wr = abs(x[3] + x[4] * 0.0765) / 0.0335
        meas.append((t, vc, wc, x[3] + rng.normal(0, 0.05), x[4] + x[5] + rng.normal(0, 0.02), wl, wr))
    return np.array(truth), meas


def _simulate_hand(steps, dt, rng):
    """Mano haciendo trazos en el plano (1.6 s, perfil suave sin^2 de
    velocidad) separados por pausas de 0.8 s, girando la muneca. La
    aceleracion se mide en el marco de rumbo (no lineal) con ruido."""
    truth, meas = [], []
    stroke, pause = 1.6, 0.8
    period = stroke + pause
    px = py = vx = vy = 0.0
    psi = 0.0
    dirs = [(1, 0), (0, 1), (-1, 0.4), (0.3, -1), (-0.6, -0.6)]
    for k in range(steps):
        t = k * dt
        n, ph = int(t // period), t % period
        dx, dy = dirs[n % len(dirs)]
        norm = math.hypot(dx, dy)
        if ph < stroke:
            # v(t) = V sin^2(pi t / T)  ->  a(t) = V (pi/T) sin(2 pi t / T)
            V = 0.25
            sp = V * math.sin(math.pi * ph / stroke) ** 2
            ac = V * (math.pi / stroke) * math.sin(2 * math.pi * ph / stroke)
            vx, vy = sp * dx / norm, sp * dy / norm
            ax, ay = ac * dx / norm, ac * dy / norm
            wz = 0.8 * math.sin(2 * math.pi * ph / stroke)
        else:
            vx = vy = ax = ay = wz = 0.0
        px += vx * dt
        py += vy * dt
        psi = wrap_pi(psi + wz * dt)
        c, s = math.cos(psi), math.sin(psi)
        a_h = np.array([c * ax + s * ay, -s * ax + c * ay]) + rng.normal(0, 0.15, 2)
        truth.append((px, py, vx, vy))
        meas.append((t, a_h, psi + rng.normal(0, 0.015), wz + rng.normal(0, 0.01),
                     abs(wz) + abs(rng.normal(0, 0.02))))
    return np.array(truth), meas


def _run_robot(name, meas_r):
    core = RobotEstimatorCore()
    core.filters = {name: core.filters[name]}
    core.timing_us = {name: 0.0}
    est, times = [], []
    for (t, vc, wc, v, g, wl, wr) in meas_r:
        core.set_cmd(t, vc, wc)
        core.add_speed(v)
        core.add_gyro(g)
        core.add_wheels(wl, wr)
        t0 = time.perf_counter_ns()
        core.step(t)
        times.append((time.perf_counter_ns() - t0) / 1000.0)
        est.append(core.filters[name].x[:2].copy())
    return core, np.array(est), np.array(times[1:])


def _run_hand(name, meas_h):
    core = HandEstimatorCore()
    core.filters = {name: core.filters[name]}
    core.timing_us = {name: 0.0}
    est, times = [], []
    for (t, a_h, psi, wz, g) in meas_h:
        t0 = time.perf_counter_ns()
        core.step(t, a_h, psi, wz, g)
        times.append((time.perf_counter_ns() - t0) / 1000.0)
        est.append(core.filters[name].x[:4].copy())
    return core, np.array(est), np.array(times[1:])


def _peak_kb(fn, *args):
    """Pico de memoria reservada durante fn (corrida aparte: tracemalloc
    hace mas lento el codigo y falsearia los tiempos)."""
    tracemalloc.start()
    tracemalloc.reset_peak()
    base, _ = tracemalloc.get_traced_memory()
    fn(*args)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return (peak - base) / 1024.0


def run_benchmark(steps=1500, seed=7):
    """Compara EKF y UKF: tiempo por paso, memoria y error contra la
    verdad simulada en variables NO medidas (posicion de robot y mano,
    velocidad de la mano)."""
    dt = 0.02
    out = {'steps': steps, 'dt': dt, 'robot': {}, 'hand': {}}

    truth_r, meas_r = _simulate_robot(steps, dt, np.random.default_rng(seed))
    truth_h, meas_h = _simulate_hand(steps, dt, np.random.default_rng(seed + 1))
    # Los pasos de memoria guardan las estimaciones de cada paso; se
    # mide con pocas muestras para que domine el filtro, no la lista.
    mem_r, mem_h = meas_r[:200], meas_h[:200]

    for name in ('EKF', 'UKF'):
        core, est, times = _run_robot(name, meas_r)
        err = np.linalg.norm(est - truth_r[:, :2], axis=1)
        out['robot'][name] = {
            'us_mean': float(times.mean()), 'us_p95': float(np.percentile(times, 95)),
            'peak_kb': _peak_kb(_run_robot, name, mem_r),
            'state_bytes': int(core.filters[name].nbytes()),
            'rmse_pos_m': float(math.sqrt(np.mean(err ** 2))), 'final_err_m': float(err[-1]),
        }

        core, est, times = _run_hand(name, meas_h)
        err_p = np.linalg.norm(est[:, :2] - truth_h[:, :2], axis=1)
        err_v = np.linalg.norm(est[:, 2:4] - truth_h[:, 2:4], axis=1)
        out['hand'][name] = {
            'us_mean': float(times.mean()), 'us_p95': float(np.percentile(times, 95)),
            'peak_kb': _peak_kb(_run_hand, name, mem_h),
            'state_bytes': int(core.filters[name].nbytes()),
            'rmse_pos_m': float(math.sqrt(np.mean(err_p ** 2))),
            'rmse_vel_ms': float(math.sqrt(np.mean(err_v ** 2))),
        }
    return out


if __name__ == '__main__':
    # Proceso aparte para el tablero: mide sin la interferencia de los
    # hilos de ROS (tiempo y memoria limpios).
    import json
    import sys
    print(json.dumps(run_benchmark(int(sys.argv[1]) if len(sys.argv) > 1 else 1500)))
