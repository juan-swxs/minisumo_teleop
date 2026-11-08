"""Simulador del guante y del carro (mismos topicos que los ESP32).

Sirve para probar el sistema completo sin hardware:
    ros2 run minisumo_teleop fake_hardware --ros-args -p hand_motion:=tilt

Guante (50 Hz): glove/imu/raw, glove/imu/filtered, glove/rpy
    hand_motion = tilt   la mano se inclina (pitch/roll) -> estrategia 2
                  plane  la mano se desplaza en circulos  -> estrategia 1
                  idle   mano quieta
Carro (20 Hz): imu/raw, imu/filtered, wheel_*/raw|filtered, odom/pose,
    odom/twist. Sigue /cmd_vel con un retardo de primer orden, como el
    PID de ruedas; el giroscopio tiene sesgo y los encoders cuantizan.
ESP-NOW: imita el enlace directo guante -> carro de los firmwares. El
    guante se arma con glove/arm (publica glove/armed); armado, el carro
    convierte la inclinacion en (v, w) como main.c, con prioridad sobre
    /cmd_vel, y publica teleop/espnow.
"""

import math
import random

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Empty, Float32

from .common import GRAVITY, quat_to_matrix
from .glove_teleop import PARAMS as TELEOP_DEFAULTS, GloveTeleop, shape
from .battery import NIMH_CURVE

R_WHEEL = 0.0335   # m (medido: 6.7 cm de diametro)
L_WHEELS = 0.153   # m (medido)
PPR = 40         # pulsos por vuelta (medido)


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class FakeHardware(Node):
    def __init__(self):
        super().__init__('fake_hardware')
        self.declare_parameter('hand_motion', 'tilt')
        self.t = 0.0
        self.yaw0 = 0.7          # rumbo absoluto del guante (norte arbitrario)
        self.rpy0 = (0.0, 0.0, 0.0)
        self.abs_rpy = (0.0, 0.0, 0.0)
        self.prev_rpy = None
        self.prev_v = np.zeros(3)
        self.ema = {}

        # Carro
        self.car = dict(x=0.0, y=0.0, th=0.0, v=0.0, w=0.0)
        # Alimentacion del sistema simulada (2 x Beston 9 V Ni-MH en paralelo: 7 celdas, 500 mAh)
        self.batt = dict(soc=82.0, used_mah=0.0, i_ma=45.0, wl=0.0, wr=0.0)
        self.odom = dict(x=0.0, y=0.0, th=0.0)
        self.cmd = (0.0, 0.0, -1.0)
        self.bias = 0.025
        self.pulses = [0.0, 0.0]

        sq = qos_profile_sensor_data
        self.p_graw = self.create_publisher(Imu, 'glove/imu/raw', sq)
        self.p_gfilt = self.create_publisher(Imu, 'glove/imu/filtered', sq)
        self.p_rpy = self.create_publisher(Vector3, 'glove/rpy', sq)
        self.p_craw = self.create_publisher(Imu, 'imu/raw', sq)
        self.p_cfilt = self.create_publisher(Imu, 'imu/filtered', sq)
        self.p_wheels = {k: self.create_publisher(Float32, t, sq) for k, t in (
            ('wl_raw', 'wheel_left/raw'), ('wl_filt', 'wheel_left/filtered'),
            ('wr_raw', 'wheel_right/raw'), ('wr_filt', 'wheel_right/filtered'))}
        self.p_pose = self.create_publisher(Pose, 'odom/pose', sq)
        self.p_twist = self.create_publisher(Twist, 'odom/twist', sq)

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(Twist, 'goto_cmd', self._on_goto, 10)
        self.create_subscription(Empty, 'glove/zero', self._on_zero, 10)
        self.create_subscription(Bool, 'glove/arm', self._on_arm, 10)

        # Enlace ESP-NOW simulado (mismos valores por defecto que main.c)
        self.glove_armed = False
        self.rel_rpy = (0.0, 0.0, 0.0)
        self.glove_active = False
        self.esp_v = self.esp_w = 0.0
        self.esp_guard = 0
        self.p_armed = self.create_publisher(Bool, 'glove/armed', sq)
        self.p_espnow = self.create_publisher(Twist, 'teleop/espnow', sq)
        self.p_batt = self.create_publisher(BatteryState, 'battery/state', sq)
        self.create_timer(0.2, self._battery_publish)
        self._armed_div = 0
        self.create_timer(0.02, self._espnow_step)

        self.create_timer(0.02, self._glove_tick)
        self.create_timer(0.01, self._car_physics)
        self.create_timer(0.05, self._car_publish)
        self.get_logger().info('Simulador de guante y carro corriendo (sin hardware)')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _lp(self, key, value, alpha):
        v = np.asarray(value, dtype=float)
        prev = self.ema.get(key)
        self.ema[key] = v if prev is None else prev + alpha * (v - prev)
        return self.ema[key]

    def _on_arm(self, m):
        self.glove_armed = bool(m.data)
        self.get_logger().info(f"Guante simulado {'ARMADO' if self.glove_armed else 'desarmado'}")

    def _espnow_step(self):
        """Copia de glove_teleop_step() del firmware del carro."""
        p = TELEOP_DEFAULTS
        if not self.glove_armed:
            if self.glove_active:
                self.cmd = (0.0, 0.0, self._now())
            self.glove_active = False
            self.esp_v = self.esp_w = 0.0
            self.esp_guard = 0
            return
        roll, pitch, _ = self.rel_rpy
        dz, full = math.radians(p['deadzone_deg']), math.radians(p['full_tilt_deg'])
        v = p['v_max'] * shape(pitch, dz, full, p['expo'])
        w = -p['w_max'] * shape(roll, dz, full, p['expo'])
        o = self.odom
        self.esp_guard = 0
        guard_hit = False
        if v != 0.0:
            speed = max(abs(v), abs(self.car['v']), abs(self.esp_v))
            if GloveTeleop._arc_leaves(o['x'], o['y'], o['th'], math.copysign(speed, v), w, p):
                v, guard_hit, self.esp_guard = 0.0, True, 2
            elif math.hypot(o['x'], o['y']) > p['dojo_radius'] - p['guard_margin'] - 0.05:
                v = max(-p['guard_slow'], min(p['guard_slow'], v))
                self.esp_guard = 1
        a = min(1.0, 0.02 / p['smoothing_tau'])
        self.esp_v = 0.0 if guard_hit else self.esp_v + a * (v - self.esp_v)
        self.esp_w += a * (w - self.esp_w)
        self.glove_active = True
        self.cmd = (self.esp_v, self.esp_w, self._now())

    def _on_cmd(self, m):
        if self.glove_active:   # el guante armado tiene prioridad, como en main.c
            return
        self.cmd = (m.linear.x, m.angular.z, self._now())

    def _on_goto(self, m):
        if int(m.angular.z + 0.5) == 2:   # reset del cero, como el ESP32
            self.car.update(x=0.0, y=0.0, th=0.0)
            self.odom.update(x=0.0, y=0.0, th=0.0)

    def _on_zero(self, _m):
        self.rpy0 = self.abs_rpy

    # ------------------------------------------------------------------
    # Guante
    # ------------------------------------------------------------------
    def _hand_pose(self, t):
        mode = self.get_parameter('hand_motion').value
        if mode == 'tilt':
            pitch = math.radians(28) * math.sin(0.35 * t) if (t % 24) < 18 else 0.0
            roll = math.radians(22) * math.sin(0.21 * t + 1.0)
            return np.zeros(3), (roll, pitch, 0.15 * math.sin(0.05 * t))
        if mode == 'plane':
            # Ida y vuelta adelante/atras con deriva lateral y pausa. La
            # envolvente sin^2 arranca y termina con velocidad y aceleracion
            # cero, como una mano real (sin saltos imposibles).
            phase = t % 12.0
            if phase < 8.0:
                env = math.sin(math.pi * phase / 8.0) ** 2
                pos = np.array([0.12 * math.sin(2.0 * phase) * env, 0.08 * math.sin(0.8 * phase) * env, 0.0])
            else:
                pos = np.zeros(3)
            return pos, (0.03 * math.sin(2 * t), 0.03 * math.cos(1.7 * t), 0.2 * math.sin(0.1 * t))
        return np.zeros(3), (0.0, 0.0, 0.0)

    def _glove_tick(self):
        dt = 0.02
        self.t += dt
        pos, (r, p, y) = self._hand_pose(self.t)
        self.abs_rpy = (r, p, y)
        # Aceleracion del mundo por diferencias finitas de la posicion
        prev_pos = getattr(self, '_prev_pos', pos)
        v = (pos - prev_pos) / dt
        a_world = (v - self.prev_v) / dt
        self._prev_pos, self.prev_v = pos, v

        q = quat_from_rpy(r, p, y + self.yaw0)
        R = quat_to_matrix(*q)
        a_lin_body = R.T @ a_world
        g_body = R.T @ np.array([0.0, 0.0, GRAVITY])
        if self.prev_rpy is None:
            self.prev_rpy = (r, p, y)
        gyro = np.array([(r - self.prev_rpy[0]) / dt, (p - self.prev_rpy[1]) / dt, (y - self.prev_rpy[2]) / dt])
        self.prev_rpy = (r, p, y)

        acc_raw = a_lin_body + g_body + np.random.normal(0, 0.18, 3)
        gyro_raw = gyro + np.random.normal(0, 0.03, 3)
        lin_f = self._lp('lin', a_lin_body + np.random.normal(0, 0.08, 3), 0.2)
        gyro_f = self._lp('gyro', gyro_raw, 0.2)

        stamp = self.get_clock().now().to_msg()
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = 'glove_imu'
        m.orientation_covariance[0] = -1.0
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, acc_raw)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, gyro_raw)
        self.p_graw.publish(m)

        f = Imu()
        f.header = m.header
        f.orientation.x, f.orientation.y, f.orientation.z, f.orientation.w = q
        f.linear_acceleration.x, f.linear_acceleration.y, f.linear_acceleration.z = map(float, lin_f)
        f.angular_velocity.x, f.angular_velocity.y, f.angular_velocity.z = map(float, gyro_f)
        self.p_gfilt.publish(f)

        r0, p0, y0 = self.rpy0
        wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi  # noqa: E731
        self.rel_rpy = (wrap(r - r0), p - p0, wrap(y - y0))
        self.p_rpy.publish(Vector3(x=self.rel_rpy[0], y=self.rel_rpy[1], z=self.rel_rpy[2]))
        self._armed_div += 1
        if self._armed_div >= 5:
            self._armed_div = 0
            self.p_armed.publish(Bool(data=self.glove_armed))

    # ------------------------------------------------------------------
    # Carro
    # ------------------------------------------------------------------
    def _car_physics(self):
        dt = 0.01
        c = self.car
        vc, wc, tc = self.cmd
        if self._now() - tc > 0.5:   # watchdog del ESP32
            vc = wc = 0.0
        c['v'] += (dt / 0.25) * (vc - c['v'])
        c['w'] += (dt / 0.20) * (wc - c['w'])
        # Patinaje leve: el robot real nunca sigue exacto al modelo
        v_real = c['v'] * 0.97
        c['x'] += v_real * math.cos(c['th']) * dt
        c['y'] += v_real * math.sin(c['th']) * dt
        c['th'] = (c['th'] + c['w'] * dt + math.pi) % (2 * math.pi) - math.pi
        wl = (c['v'] - c['w'] * L_WHEELS / 2) / R_WHEEL
        wr = (c['v'] + c['w'] * L_WHEELS / 2) / R_WHEEL
        self.pulses[0] += abs(wl) * dt * PPR / (2 * math.pi)
        self.pulses[1] += abs(wr) * dt * PPR / (2 * math.pi)
        self._signs = (math.copysign(1, wl), math.copysign(1, wr))
        # Consumo medido con el DRV8833: ~3 mA en reposo y ~6 mA por rad/s de
        # rueda (una rueda a 24 rad/s ~150 mA, dos a ~16 rad/s ~240 mA)
        b = self.batt
        i_target = 3.0 + 6.0 * (abs(wl) + abs(wr)) + 900.0 * (abs(wl - b['wl']) + abs(wr - b['wr'])) * 0.01
        b['wl'], b['wr'] = wl, wr
        b['i_ma'] += (dt / 0.15) * (i_target - b['i_ma'])
        b['used_mah'] += b['i_ma'] * dt / 3600.0

    def _battery_publish(self):
        b = self.batt
        soc = max(0.0, 82.0 - b['used_mah'] / 500.0 * 100.0)
        # tension de celda por la curva de Ni-MH, menos la caida interna
        pts = NIMH_CURVE
        v_cell = pts[0][0]
        for (v0, s0), (v1, s1) in zip(pts, pts[1:]):
            if soc <= s1:
                v_cell = v0 + (v1 - v0) * (soc - s0) / max(s1 - s0, 1e-6)
                break
        i_meas = b['i_ma'] + random.gauss(0.0, 3.0)
        v = 7 * v_cell - i_meas / 1000.0 * 0.7 + random.gauss(0.0, 0.01)
        m = BatteryState()
        m.voltage = float(v)
        m.current = float(-i_meas / 1000.0)
        m.charge = float((500.0 - b['used_mah']) / 1000.0)
        m.capacity = 0.5
        m.design_capacity = 0.5
        m.percentage = float('nan')
        m.present = True
        m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        m.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_NIMH
        self.p_batt.publish(m)

    def _car_publish(self):
        dt = 0.05
        c = self.car
        # Encoders: pulsos enteros contados en la ventana (cuantizacion)
        nl, nr = int(self.pulses[0]), int(self.pulses[1])
        self.pulses[0] -= nl
        self.pulses[1] -= nr
        wl_raw = nl * 2 * math.pi / PPR / dt
        wr_raw = nr * 2 * math.pi / PPR / dt
        wl_f = float(self._lp('wl', wl_raw, 0.3))
        wr_f = float(self._lp('wr', wr_raw, 0.3))
        for k, v in (('wl_raw', wl_raw), ('wl_filt', wl_f), ('wr_raw', wr_raw), ('wr_filt', wr_f)):
            self.p_wheels[k].publish(Float32(data=float(v)))

        sl, sr = getattr(self, '_signs', (1.0, 1.0))
        v_enc = 0.5 * (wl_f * sl + wr_f * sr) * R_WHEEL

        gz_raw = c['w'] + self.bias + random.gauss(0, 0.02)
        gz_f = float(self._lp('gz', gz_raw - self.bias, 0.1))
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.orientation_covariance[0] = -1.0
        m.linear_acceleration.x = random.gauss(0, 0.15)
        m.linear_acceleration.y = random.gauss(0, 0.15)
        m.linear_acceleration.z = GRAVITY + random.gauss(0, 0.15)
        m.angular_velocity.z = gz_raw
        self.p_craw.publish(m)
        f = Imu()
        f.header = m.header
        f.orientation.z, f.orientation.w = math.sin(c['th'] / 2), math.cos(c['th'] / 2)
        f.angular_velocity.z = gz_f
        f.linear_acceleration.x = float(self._lp('ax', m.linear_acceleration.x, 0.1))
        self.p_cfilt.publish(f)

        # Odometria del ESP32: encoders + giroscopio (como en main.c)
        o = self.odom
        o['th'] = (o['th'] + (gz_raw - self.bias) * dt + math.pi) % (2 * math.pi) - math.pi
        o['x'] += v_enc * math.cos(o['th']) * dt
        o['y'] += v_enc * math.sin(o['th']) * dt
        pose = Pose()
        pose.position.x, pose.position.y = o['x'], o['y']
        pose.orientation.z, pose.orientation.w = math.sin(o['th'] / 2), math.cos(o['th'] / 2)
        self.p_pose.publish(pose)
        tw = Twist()
        tw.linear.x, tw.angular.z = v_enc, gz_raw - self.bias
        self.p_twist.publish(tw)

        e = Twist()
        e.linear.x, e.angular.z = self.esp_v, self.esp_w
        e.linear.y = 50.0                                  # paquetes/s del guante
        e.linear.z = 2.0 if self.glove_armed else 1.0      # 1 enlace desarmado, 2 armado
        e.angular.x = -48.0 + random.gauss(0, 2)           # RSSI (dBm)
        e.angular.y = float(self.esp_guard)
        self.p_espnow.publish(e)


def main(args=None):
    rclpy.init(args=args)
    node = FakeHardware()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
