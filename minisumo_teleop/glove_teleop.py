"""Coordinacion de movimiento: mano del operador -> velocidades del robot.

Estrategia 1 (plano):      la mano se desplaza en un plano horizontal; su
    velocidad (estimada con EKF/UKF) escalada es la velocidad deseada del
    robot en el dojo. Se maneja por trazos: al empezar a moverse la mano se
    fija la direccion del trazo (al principio la integracion del
    acelerometro es confiable) y se usa la mayor velocidad alcanzada en esa
    direccion; el trazo termina con la mano quieta (ZUPT) o a los
    plane_stroke_max s. Asi el rebote de la integracion al frenar la mano
    (la velocidad estimada cambiaba de signo) no hace retroceder al carro.
    Como el robot es diferencial (no puede moverse de lado), se convierte a
    (v, w) con el rumbo estimado del robot:
        V = k (vx_mano, vy_mano)
        e = rumbo(V) - theta_robot
        v = |V| cos(e)              (negativo: avanza en reversa)
        w = Kp * e  (e se toma respecto a la reversa si |e| > 90 grados)

Estrategia 2 (inclinacion): pitch de la mano -> v, roll -> w, con zona
    muerta (con histeresis), curva exponencial y saturacion. Al salir de la
    zona muerta arranca ya en una velocidad minima que las ruedas pueden
    mantener (por debajo avanzan a tirones):
        v = signo * (v_min + (v_max - v_min) * forma(pitch))
    Igual que el mando directo por ESP-NOW del firmware, mas un lazo de giro
    con el giroscopio del carro (odom/twist) para que el carro gire lo que
    pide la mano aunque una rueda sea mas floja.

Seguridad: hay que "armar" el sistema (enabled); si el guante deja de
enviar datos el robot se detiene; cerca del borde del dojo se anula la
velocidad que lo saca (la de giro sigue permitida).

Entradas: glove/rpy, glove/imu/raw, glove/zero, hand/{ekf,ukf}/odom,
          robot/{ekf,ukf}/odom, odom/pose (odometria del ESP32)
Salidas:  cmd_vel (geometry_msgs/Twist), teleop/state (String JSON)
"""

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, String

from .common import declare_params, quat_to_rpy

PARAMS = {
    'enabled': False,
    'strategy': 2,                # 1 = plano, 2 = inclinacion
    'source': 'ekf',              # estimador para estrategia 1 y borde: ekf | ukf | odom
    'glove_input': 'filtered',    # filtered: fusion del guante (Mahony) | raw: inclinacion del acelerometro crudo
    'rate_hz': 25.0,
    'glove_timeout': 0.3,         # s
    'v_max': 0.45,                # m/s (ruedas a ~13 rad/s, PWM ~33 con el DRV8833)
    'w_max': 4.5,                 # rad/s
    'wheel_max': 30.0,            # rad/s por rueda (medido con PWM 50): si se pasa, se escalan v y w juntas
    # estrategia 2 (mismos ajustes que el mando por ESP-NOW del carro)
    'v_min': 0.14,                # m/s al salir de la zona muerta (~4.2 rad/s por rueda)
    'w_min': 1.8,                 # rad/s al salir de la zona muerta (~4.1 rad/s por rueda)
    'deadzone_exit_deg': 5.0,     # histeresis: ya en marcha, se para por debajo de esto
    'yaw_loop': True,             # lazo de giro con el giroscopio del carro
    'yaw_kp': 0.5,
    'yaw_ki': 1.2,
    'yaw_corr_max': 1.5,          # rad/s
    'deadzone_deg': 8.0,
    'full_tilt_deg': 35.0,
    'expo': 1.6,
    'invert_pitch': False,
    'invert_roll': False,
    # estrategia 1
    'plane_gain': 1.0,            # (m/s robot) / (m/s mano)
    'plane_deadband': 0.04,       # m/s de la mano
    'plane_stroke_max': 0.8,      # s: duracion maxima de un trazo
    'heading_kp': 3.5,            # rad/s por rad
    # comun
    'smoothing_tau': 0.12,        # s, pasa-bajos del comando
    'dojo_guard': True,
    'dojo_radius': 2.5,           # m (dojo de 5 m de diametro)
    'guard_margin': 0.05,         # m
    'guard_lookahead': 0.45,      # s, cuanto avanza el robot tras pedir v = 0 (retardo PID + inercia)
    'guard_slow': 0.15,           # m/s maximos dentro de la franja de peligro (menos inercia)
}


WHEEL_R = 0.0335   # m, radio de la rueda (medido: 6.7 cm de diametro, igual que el firmware)
WHEEL_L = 0.153    # m, distancia entre ruedas (medida)


def shape(x, dz, full, expo):
    """Zona muerta + curva exponencial + saturacion en [-1, 1]."""
    a = abs(x)
    if a <= dz:
        return 0.0
    n = min((a - dz) / max(full - dz, 1e-6), 1.0)
    return math.copysign(n ** expo, x)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class GloveTeleop(Node):
    def __init__(self):
        super().__init__('glove_teleop')
        self.p = declare_params(self, PARAMS)
        self.add_post_set_parameters_callback(self._on_params)

        self.rpy = None
        self.t_rpy = -1e9
        self.raw_tilt = None
        self.raw_neutral = (0.0, 0.0)
        self.hand = {}
        self.robot = {}
        self.v_out = 0.0
        self.w_out = 0.0
        self.was_enabled = False
        self.stop_burst = 0
        self.v_on = self.w_on = False   # histeresis de la zona muerta
        self.gz = None                  # giro medido por el carro (rad/s, sin sesgo)
        self.t_gz = -1e9
        self.yaw_i = 0.0
        self.stroke = None              # trazo de la estrategia 1: {'ux','uy','peak','t0'}
        self.need_still = False         # tras un trazo, el siguiente espera a que la mano quede quieta
        self.hand_zupt = False          # mano quieta segun el estimador (hand/stats)

        self.create_subscription(Vector3, 'glove/rpy', self._on_rpy, qos_profile_sensor_data)
        self.create_subscription(String, 'hand/stats', self._on_hand_stats, 10)
        self.create_subscription(Imu, 'glove/imu/raw', self._on_raw, qos_profile_sensor_data)
        self.create_subscription(Empty, 'glove/zero', self._on_zero, 10)
        for src in ('ekf', 'ukf'):
            self.create_subscription(Odometry, f'hand/{src}/odom', lambda m, s=src: self._on_hand(s, m), 10)
            self.create_subscription(Odometry, f'robot/{src}/odom',
                                     lambda m, s=src: self._on_robot(s, m.pose.pose, m.twist.twist.linear.x), 10)
        self.create_subscription(Pose, 'odom/pose', lambda m: self._on_robot('odom', m), qos_profile_sensor_data)
        self.create_subscription(Twist, 'odom/twist', self._on_odom_twist, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'teleop/state', 10)
        self.create_timer(1.0 / float(self.p['rate_hz']), self._tick)
        self.get_logger().info('Tele-operacion lista (desarmada). Armala desde el tablero.')

    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_params(self, params):
        for prm in params:
            if prm.name in self.p:
                self.p[prm.name] = prm.value

    def _on_rpy(self, m):
        self.rpy = (m.x, m.y, m.z)
        self.t_rpy = self._now()

    def _on_odom_twist(self, m):
        self.gz = m.angular.z   # el ESP32 ya le resta el sesgo del giroscopio
        self.t_gz = self._now()

    def _on_raw(self, m):
        a = m.linear_acceleration
        self.raw_tilt = (math.atan2(a.y, a.z), math.atan2(-a.x, math.hypot(a.y, a.z)))

    def _on_zero(self, _m):
        if self.raw_tilt is not None:
            self.raw_neutral = self.raw_tilt

    def _on_hand_stats(self, m):
        try:
            self.hand_zupt = bool(json.loads(m.data).get('zupt', False))
        except ValueError:
            pass

    def _on_hand(self, src, m):
        self.hand[src] = (m.twist.twist.linear.x, m.twist.twist.linear.y, m.pose.pose.position.x, m.pose.pose.position.y)

    def _on_robot(self, src, pose, v=None):
        q = pose.orientation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.robot[src] = (pose.position.x, pose.position.y, yaw, v)

    # ------------------------------------------------------------------
    def _tilt(self):
        """(roll, pitch) relativos a la postura neutra, segun la entrada."""
        if self.p['glove_input'] == 'raw' and self.raw_tilt is not None:
            return (wrap_pi(self.raw_tilt[0] - self.raw_neutral[0]), self.raw_tilt[1] - self.raw_neutral[1])
        return (self.rpy[0], self.rpy[1])

    def _strategy_tilt(self, st):
        p = self.p
        roll, pitch = self._tilt()
        dz, full = math.radians(p['deadzone_deg']), math.radians(p['full_tilt_deg'])
        sp = -1.0 if p['invert_pitch'] else 1.0
        sr = -1.0 if p['invert_roll'] else 1.0
        dz_exit = math.radians(min(p['deadzone_exit_deg'], p['deadzone_deg']))
        sv = self._shape_hyst(sp * pitch, dz, dz_exit, full, 'v_on')
        sw = self._shape_hyst(sr * roll, dz, dz_exit, full, 'w_on')
        v = 0.0 if sv == 0.0 else math.copysign(p['v_min'] + (p['v_max'] - p['v_min']) * abs(sv), sv)
        w = 0.0 if sw == 0.0 else -math.copysign(p['w_min'] + (p['w_max'] - p['w_min']) * abs(sw), sw)
        if v == 0.0 and w == 0.0:
            st['reason'] = 'Mano en zona muerta'
        return v, w

    def _shape_hyst(self, x, dz, dz_exit, full, flag):
        """Zona muerta con histeresis: arranca al pasar dz y solo vuelve a 0
        por debajo de dz_exit (cerca del borde el comando ya no se prende y
        apaga varias veces por segundo)."""
        a = abs(x)
        on = getattr(self, flag)
        if not on and a > dz:
            on = True
        elif on and a < dz_exit:
            on = False
        setattr(self, flag, on)
        if not on:
            return 0.0
        n = 0.0 if a <= dz else min((a - dz) / max(full - dz, 1e-6), 1.0)
        return math.copysign(max(n ** self.p['expo'], 1e-6), x)

    def _strategy_plane(self, st):
        p = self.p
        src = p['source'] if p['source'] in ('ekf', 'ukf') else 'ekf'
        hand = self.hand.get(src)
        robot = self.robot.get(p['source']) or self.robot.get('odom')
        if hand is None:
            st['reason'] = 'Sin estimacion de la mano'
            return 0.0, 0.0
        if robot is None:
            st['reason'] = 'Sin estimacion del robot'
            return 0.0, 0.0
        vx, vy = hand[0], hand[1]
        speed_hand = math.hypot(vx, vy)
        st.update(hand_vx=vx, hand_vy=vy)
        now = self._now()
        if self.hand_zupt:
            self.need_still = False
        if self.stroke is not None and (self.hand_zupt or now - self.stroke['t0'] > p['plane_stroke_max']):
            self.stroke = None
            # Si el trazo acabo por tiempo la mano aun se mueve (frenando): sin
            # esto arrancaba otro trazo con el rebote de la integracion, en el
            # sentido contrario, y un empujon largo salia partido en varios
            self.need_still = True
        if self.stroke is None:
            if self.need_still:
                st['reason'] = 'Esperando mano quieta'
                return 0.0, 0.0
            if speed_hand < p['plane_deadband'] or self.hand_zupt:
                st['reason'] = 'Mano quieta'
                return 0.0, 0.0
            self.stroke = {'ux': vx / speed_hand, 'uy': vy / speed_hand, 'peak': 0.0, 't0': now}
        s = self.stroke
        # Solo la componente en la direccion del trazo, y nunca menos que lo
        # ya alcanzado: el rebote al frenar no cambia el sentido
        s['peak'] = max(s['peak'], vx * s['ux'] + vy * s['uy'])
        st.update(stroke_dir=math.degrees(math.atan2(s['uy'], s['ux'])), stroke_peak=s['peak'])
        Vx, Vy = p['plane_gain'] * s['peak'] * s['ux'], p['plane_gain'] * s['peak'] * s['uy']
        speed = math.hypot(Vx, Vy)
        e = wrap_pi(math.atan2(Vy, Vx) - robot[2])
        v = speed * math.cos(e)
        e_turn = e if abs(e) <= math.pi / 2 else wrap_pi(e - math.pi)
        w = p['heading_kp'] * e_turn
        # Por debajo de ~4 rad/s una rueda no gira (DRV8833): igual que en la
        # estrategia 2, al pedir movimiento se arranca ya en el minimo
        if abs(e_turn) < math.pi / 4 and 0.0 < abs(v) < p['v_min']:
            v = math.copysign(p['v_min'], v)
        elif abs(v) < p['v_min'] and 0.0 < abs(w) < p['w_min']:
            w = math.copysign(p['w_min'], w)
        st.update(heading_err=e)
        return v, w

    @staticmethod
    def _arc_leaves(x, y, th, v, w, p, steps=12):
        """True si avanzar con (v, w) durante guard_lookahead acerca al robot
        al borde: el arco pasa de dojo_radius - guard_margin alejandose del
        centro, o el robot ya esta en esa franja y el arco no lo saca de ella
        claramente hacia adentro."""
        limit = p['dojo_radius'] - p['guard_margin']
        r0 = math.hypot(x, y)
        dt = p['guard_lookahead'] / steps
        r = r0
        for _ in range(steps):
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th += w * dt
            r = math.hypot(x, y)
            if r > limit and r > r0 + 0.002:
                return True
        return r0 > limit and r > r0 - 0.01

    def _tick(self):
        p = self.p
        now = self._now()
        st = {'reason': '', 'params': dict(p)}
        v = w = 0.0

        glove_ok = (now - self.t_rpy) < p['glove_timeout'] and self.rpy is not None
        st['glove_ok'] = glove_ok
        if self.rpy is not None:
            st['roll'], st['pitch'] = self._tilt()

        if not p['enabled'] or not glove_ok or int(p['strategy']) != 1:
            self.stroke = None
            self.need_still = False
        if not p['enabled']:
            st['reason'] = 'Desarmado'
            if self.was_enabled:
                self.stop_burst = 5
        elif not glove_ok:
            st['reason'] = 'Guante sin datos'
        elif int(p['strategy']) == 1:
            v, w = self._strategy_plane(st)
        else:
            v, w = self._strategy_tilt(st)

        # Saturacion
        v = max(-p['v_max'], min(p['v_max'], v))
        w = max(-p['w_max'], min(p['w_max'], w))

        # Proteccion del borde del dojo (con la estimacion elegida). Es
        # predictiva: se simula el arco que recorreria el robot con el
        # comando (v, w) durante guard_lookahead s (retardo del PID +
        # inercia). Si ese arco lo acerca al borde por encima del limite,
        # se anula v (girar en el sitio siempre esta permitido).
        robot = self.robot.get(p['source']) or self.robot.get('odom')
        guard_hit = False
        if p['dojo_guard'] and robot is not None and v != 0.0:
            x, y, th, v_est = robot
            v_now = v_est if v_est is not None else self.v_out
            speed = max(abs(v), abs(v_now), abs(self.v_out))
            if self._arc_leaves(x, y, th, math.copysign(speed, v), w, p):
                v = 0.0
                guard_hit = True
                st['reason'] = 'Borde del dojo: solo giro'
            elif math.hypot(x, y) > p['dojo_radius'] - p['guard_margin'] - 0.05:
                v = max(-p['guard_slow'], min(p['guard_slow'], v))
                st['reason'] = 'Cerca del borde: velocidad reducida'
        st['robot_r'] = math.hypot(robot[0], robot[1]) if robot else None

        # Pasa-bajos del comando (evita escalones bruscos en el PID)
        if not p['enabled'] or not glove_ok:
            self.v_out = self.w_out = 0.0
            self.v_on = self.w_on = False
        elif guard_hit:
            self.v_out = 0.0     # sin suavizado: frenar ya
            a = min(1.0, (1.0 / p['rate_hz']) / max(p['smoothing_tau'], 1e-3))
            self.w_out += a * (w - self.w_out)
        else:
            a = min(1.0, (1.0 / p['rate_hz']) / max(p['smoothing_tau'], 1e-3))
            self.v_out += a * (v - self.v_out)
            self.w_out += a * (w - self.w_out)

        # La cola exponencial del pasa-bajos nunca llega a 0: con la mano en
        # neutro se corta para que el carro pare de verdad
        if v == 0.0 and abs(self.v_out) < 0.03:
            self.v_out = 0.0
        if w == 0.0 and abs(self.w_out) < 0.15:
            self.w_out = 0.0

        # Lazo de giro (estrategia 2): compara el giro pedido con el que mide
        # el giroscopio del carro y corrige la diferencia entre ruedas
        w_cmd = self.w_out
        moving = abs(self.v_out) > 0.02 or abs(self.w_out) > 0.05
        gz_ok = self.gz is not None and (now - self.t_gz) < 0.3
        if p['enabled'] and p['yaw_loop'] and int(p['strategy']) == 2 and moving and gz_ok:
            dt = 1.0 / p['rate_hz']
            e = self.w_out - self.gz
            lim = p['yaw_corr_max']
            self.yaw_i = max(-lim / max(p['yaw_ki'], 1e-6), min(lim / max(p['yaw_ki'], 1e-6), self.yaw_i + e * dt))
            w_cmd += max(-lim, min(lim, p['yaw_kp'] * e + p['yaw_ki'] * self.yaw_i))
        else:
            self.yaw_i = 0.0

        # Si una rueda pasa de su maximo se escalan v y w juntas (se conserva
        # la curva en vez de recortar solo una rueda)
        v_cmd = self.v_out
        wl = (v_cmd - w_cmd * WHEEL_L / 2) / WHEEL_R
        wr = (v_cmd + w_cmd * WHEEL_L / 2) / WHEEL_R
        m = max(abs(wl), abs(wr))
        if m > p['wheel_max']:
            v_cmd *= p['wheel_max'] / m
            w_cmd *= p['wheel_max'] / m

        st.update(enabled=p['enabled'], strategy=int(p['strategy']), v=self.v_out, w=self.w_out,
                  v_target=v, w_target=w, w_cmd=w_cmd, gz=self.gz)

        if p['enabled'] or self.stop_burst > 0:
            cmd = Twist()
            cmd.linear.x = float(v_cmd if p['enabled'] else 0.0)
            cmd.angular.z = float(w_cmd if p['enabled'] else 0.0)
            self.cmd_pub.publish(cmd)
            if not p['enabled']:
                self.stop_burst -= 1
        self.was_enabled = bool(p['enabled'])
        self.state_pub.publish(String(data=json.dumps(st)))


def main(args=None):
    rclpy.init(args=args)
    node = GloveTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
