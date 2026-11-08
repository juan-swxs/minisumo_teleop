"""Utilidades compartidas por los nodos de tele-operacion."""

import math

import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

GRAVITY = 9.80665

# Ultimo valor "pegado" para topicos de estado (los que se conectan tarde
# reciben el ultimo mensaje).
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


def quat_to_matrix(x, y, z, w):
    """Matriz de rotacion cuerpo -> mundo de un quaternion (x, y, z, w)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def declare_params(node, defaults):
    """Declara un dict de parametros y devuelve un dict con sus valores."""
    for name, value in defaults.items():
        node.declare_parameter(name, value)
    return {name: node.get_parameter(name).value for name in defaults}


def odom_cov6(P_xy_th):
    """Covarianza 6x6 de nav_msgs/Odometry a partir de [Pxx, Pyy, Pxy, Pthth]."""
    pxx, pyy, pxy, pth = P_xy_th
    c = [0.0] * 36
    c[0], c[7], c[1], c[6] = pxx, pyy, pxy, pxy
    c[14] = c[21] = c[28] = 1e6   # z, roll, pitch: no se estiman
    c[35] = pth
    return c


def hand_measurements(q, a_filt, g_filt, a_raw, g_raw, rel_yaw, mode):
    """Lleva las lecturas del guante al marco de rumbo de la mano.

    q: quaternion (x, y, z, w) de la fusion del guante.
    a_filt/g_filt: aceleracion lineal y giroscopio filtrados (cuerpo).
    a_raw/g_raw: acelerometro (con gravedad) y giroscopio crudos (cuerpo).
    rel_yaw: rumbo relativo a la postura neutra (glove/rpy.z) o None.
    mode: "raw" o "filtered".

    Devuelve (a_rumbo[2], psi, w_z, |giro|) o None si q no es valido.
    """
    if q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] < 0.5:
        return None   # la fusion del guante aun no arranca
    R = quat_to_matrix(*q)
    _, _, yaw_abs = quat_to_rpy(*q)
    if mode == 'raw' and a_raw is not None and g_raw is not None:
        a_body = np.asarray(a_raw, dtype=float) - R.T @ np.array([0.0, 0.0, GRAVITY])
        gyro = np.asarray(g_raw, dtype=float)
    else:
        a_body = np.asarray(a_filt, dtype=float)
        gyro = np.asarray(g_filt, dtype=float)
    a_world = R @ a_body
    c, s = math.cos(yaw_abs), math.sin(yaw_abs)
    a_head = np.array([c * a_world[0] + s * a_world[1], -s * a_world[0] + c * a_world[1]])
    wz = float((R @ gyro)[2])
    psi = rel_yaw if rel_yaw is not None else yaw_abs
    return a_head, psi, wz, float(np.linalg.norm(gyro))
