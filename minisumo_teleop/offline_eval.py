"""Evaluacion offline de los filtros de Kalman sobre una sesion grabada.

Re-ejecuta EKF y UKF del robot y de la mano con los datos guardados por
el tablero (boton "Grabar"), con la entrada cruda o filtrada, para
ajustar modelos y parametros sin tener el robot encendido.

Uso:
    ros2 run minisumo_teleop offline_eval <sesion> [--robot-mode raw|filtered]
                                          [--hand-mode raw|filtered]
<sesion> puede ser el nombre de la carpeta en ~/teleop_data/sessions o
una ruta. Escribe offline_result.json en la carpeta de la sesion.
"""

import argparse
import json
import math
import os

import numpy as np

from .common import hand_measurements
from .models import HandEstimatorCore, RobotEstimatorCore
from .recording import DATA_ROOT, read_csv


def _downsample(arr, n=600):
    if len(arr) <= n:
        return arr
    idx = np.linspace(0, len(arr) - 1, n).astype(int)
    return [arr[i] for i in idx]


def _path_len(xy):
    if len(xy) < 2:
        return 0.0
    a = np.asarray(xy)
    return float(np.sum(np.linalg.norm(np.diff(a, axis=0), axis=1)))


def resolve(session):
    if os.path.isdir(session):
        return session
    return os.path.join(DATA_ROOT, 'sessions', session)


def eval_robot(sdir, mode='raw', params=None):
    imu_key = 'car_imu_raw' if mode == 'raw' else 'car_imu_filtered'
    events = []
    for r in read_csv(sdir, imu_key):
        events.append((r['t'], 1, r['gz']))
    for r in read_csv(sdir, 'odom'):
        events.append((r['t'], 0, r))
    for r in read_csv(sdir, 'cmd_vel'):
        events.append((r['t'], 2, r))
    for r in read_csv(sdir, 'wheels'):
        events.append((r['t'], 3, r))
    for r in read_csv(sdir, 'espnow'):
        if int(r['link']) == 2:   # comando del guante por ESP-NOW
            events.append((r['t'], 2, r))
    if not events:
        return None
    events.sort(key=lambda e: (e[0], e[1]))

    core = RobotEstimatorCore(params)
    core.p['gyro_has_bias'] = 1.0 if mode == 'raw' else 0.0
    traj = {'EKF': [], 'UKF': []}
    car = []
    times = {'EKF': [], 'UKF': []}
    for t, kind, val in events:
        if kind == 2:
            core.set_cmd(t, val['v'], val['w'])
        elif kind == 3:
            core.add_wheels(val['wl_filt'], val['wr_filt'])
        elif kind == 0:
            core.add_speed(val['v'])
            car.append((val['x'], val['y']))
        else:
            core.add_gyro(val)
            core.step(t)
            for name in traj:
                s = core.state(name)
                traj[name].append((s['x'], s['y']))
                times[name].append(core.timing_us[name])

    out = {'mode': mode, 'samples': len(traj['EKF']), 'filters': {}}
    car_final = car[-1] if car else None
    for name in traj:
        xy = traj[name]
        if not xy:
            continue
        f = core.state(name)
        out['filters'][name] = {
            'final': {'x': f['x'], 'y': f['y'], 'th': f['th']},
            'path_m': _path_len(xy),
            'us_mean': float(np.mean(times[name])) if times[name] else 0.0,
            'bias': f['bias'],
            'vs_car_final_m': (math.hypot(f['x'] - car_final[0], f['y'] - car_final[1]) if car_final else None),
            'traj': _downsample(xy),
        }
    out['car'] = {'final': {'x': car_final[0], 'y': car_final[1]} if car_final else None,
                  'path_m': _path_len(car), 'traj': _downsample(car)}
    return out


def eval_hand(sdir, mode='filtered', params=None):
    filt = read_csv(sdir, 'glove_filtered')
    if not filt:
        return None
    raw = read_csv(sdir, 'glove_raw')
    rpy = read_csv(sdir, 'glove_rpy')
    core = HandEstimatorCore(params)
    traj = {'EKF': [], 'UKF': []}
    vel = {'EKF': [], 'UKF': []}
    times = {'EKF': [], 'UKF': []}
    i_raw = i_rpy = 0
    last_raw = last_yaw = None
    t0 = None
    for r in filt:
        t = r['t']
        while i_raw < len(raw) and raw[i_raw]['t'] <= t:
            last_raw = raw[i_raw]
            i_raw += 1
        while i_rpy < len(rpy) and rpy[i_rpy]['t'] <= t:
            last_yaw = rpy[i_rpy]['yaw']
            i_rpy += 1
        a_raw = g_raw = None
        if last_raw is not None:
            a_raw = (last_raw['ax'], last_raw['ay'], last_raw['az'])
            g_raw = (last_raw['gx'], last_raw['gy'], last_raw['gz'])
        m = hand_measurements((r['qx'], r['qy'], r['qz'], r['qw']), (r['lax'], r['lay'], r['laz']),
                              (r['gx'], r['gy'], r['gz']), a_raw, g_raw, last_yaw, mode)
        if m is None:
            continue
        ts = r['stamp'] if r.get('stamp', 0) and r['stamp'] > 0 else t
        if t0 is None:
            t0 = ts
        core.step(ts, *m)
        for name in traj:
            s = core.state(name)
            traj[name].append((s['x'], s['y']))
            vel[name].append((ts - t0, s['vx'], s['vy']))
            times[name].append(core.timing_us[name])

    out = {'mode': mode, 'samples': len(traj['EKF']), 'filters': {}}
    for name in traj:
        if not traj[name]:
            continue
        v = np.asarray([(a, b) for _, a, b in vel[name]])
        out['filters'][name] = {
            'path_m': _path_len(traj[name]),
            'v_peak': float(np.max(np.linalg.norm(v, axis=1))) if len(v) else 0.0,
            'us_mean': float(np.mean(times[name])),
            'traj': _downsample(traj[name]),
            'vel': _downsample(vel[name]),
        }
    return out


def evaluate(session, robot_mode='raw', hand_mode='filtered', robot_params=None, hand_params=None):
    sdir = resolve(session)
    if not os.path.isdir(sdir):
        raise FileNotFoundError(sdir)
    return {
        'session': os.path.basename(os.path.normpath(sdir)),
        'robot': eval_robot(sdir, robot_mode, robot_params),
        'hand': eval_hand(sdir, hand_mode, hand_params),
    }


def main():
    ap = argparse.ArgumentParser(description='EKF vs UKF sobre una sesion grabada')
    ap.add_argument('session')
    ap.add_argument('--robot-mode', default='raw', choices=['raw', 'filtered'])
    ap.add_argument('--hand-mode', default='filtered', choices=['raw', 'filtered'])
    a = ap.parse_args()

    res = evaluate(a.session, a.robot_mode, a.hand_mode)
    sdir = resolve(a.session)
    with open(os.path.join(sdir, 'offline_result.json'), 'w') as f:
        json.dump(res, f)

    print(f"Sesion: {res['session']}")
    if res['robot']:
        r = res['robot']
        print(f"\nROBOT (giroscopio {r['mode']}, {r['samples']} pasos)")
        if r['car']['final']:
            print(f"  Odometria ESP32  final=({r['car']['final']['x']:+.3f}, {r['car']['final']['y']:+.3f}) m  recorrido={r['car']['path_m']:.3f} m")
        for n, f in r['filters'].items():
            d = f['vs_car_final_m']
            print(f"  {n}  final=({f['final']['x']:+.3f}, {f['final']['y']:+.3f}) m  recorrido={f['path_m']:.3f} m"
                  f"  t={f['us_mean']:.0f} us/paso  sesgo={f['bias']:+.4f} rad/s"
                  + (f"  dif. con ESP32={d:.3f} m" if d is not None else ''))
    else:
        print('\nROBOT: sin datos del carro en la sesion')
    if res['hand']:
        h = res['hand']
        print(f"\nMANO (entrada {h['mode']}, {h['samples']} pasos)")
        for n, f in h['filters'].items():
            print(f"  {n}  recorrido={f['path_m']:.3f} m  v_pico={f['v_peak']:.3f} m/s  t={f['us_mean']:.0f} us/paso")
    else:
        print('\nMANO: sin datos del guante en la sesion')
    print(f"\nResultado completo: {os.path.join(sdir, 'offline_result.json')}")


if __name__ == '__main__':
    main()
