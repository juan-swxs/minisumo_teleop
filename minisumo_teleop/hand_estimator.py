"""Estimacion de posicion, velocidad, aceleracion y rumbo de la mano del
operador con EKF y UKF en paralelo.

Entradas (ESP32 del guante, micro-ROS):
    glove/imu/raw       acelerometro con gravedad + giroscopio, sin filtrar
    glove/imu/filtered  quaternion de la fusion del guante, aceleracion
                        lineal (sin gravedad) y giroscopio con pasa-bajos
    glove/rpy           roll, pitch, yaw relativos a la postura neutra
    hand/reset          std_msgs/Empty: posicion y velocidad a cero

Salidas:
    hand/ekf/odom, hand/ukf/odom    nav_msgs/Odometry en el marco
                        "hand_plane": pose = posicion en el plano y rumbo,
                        twist.linear = velocidad (marco del plano)
    hand/ekf/accel, hand/ukf/accel  geometry_msgs/Vector3Stamped
    hand/stats          std_msgs/String JSON: tiempos, traza de P, NIS,
                        memoria y si se aplico ZUPT

Cambio de marco de referencia (sensor -> plano de la mano):
    1. a_cuerpo: aceleracion lineal en el marco del sensor del guante
       (raw: acelerometro - gravedad llevada al cuerpo con el quaternion)
    2. a_mundo = R(q) a_cuerpo
    3. a_rumbo = Rz(-yaw_abs) a_mundo   -> se compensa la inclinacion
       pero NO el rumbo: el rumbo psi es parte del estado y entra en el
       modelo de medicion no lineal z = Rz(psi)^T [ax, ay].
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, String

from .common import declare_params, hand_measurements, stamp_to_sec, yaw_to_quat
from .models import HAND_DEFAULTS, HandEstimatorCore

NODE_DEFAULTS = {
    'input_mode': 'filtered',   # raw | filtered
}


class HandEstimator(Node):
    def __init__(self):
        super().__init__('hand_estimator')
        self.cfg = declare_params(self, NODE_DEFAULTS)
        self.core = HandEstimatorCore(declare_params(self, {k: float(v) for k, v in HAND_DEFAULTS.items()}))
        self.add_post_set_parameters_callback(self._on_params)

        self.raw = None
        self.rel_yaw = None
        self.last_psi = 0.0

        self.create_subscription(Imu, 'glove/imu/raw', self._on_raw, qos_profile_sensor_data)
        self.create_subscription(Imu, 'glove/imu/filtered', self._on_filtered, qos_profile_sensor_data)
        self.create_subscription(Vector3, 'glove/rpy', self._on_rpy, qos_profile_sensor_data)
        self.create_subscription(Empty, 'hand/reset', self._on_reset, 10)

        self.odom_pub = {
            'EKF': self.create_publisher(Odometry, 'hand/ekf/odom', 10),
            'UKF': self.create_publisher(Odometry, 'hand/ukf/odom', 10),
        }
        self.acc_pub = {
            'EKF': self.create_publisher(Vector3Stamped, 'hand/ekf/accel', 10),
            'UKF': self.create_publisher(Vector3Stamped, 'hand/ukf/accel', 10),
        }
        self.stats_pub = self.create_publisher(String, 'hand/stats', 10)
        self.get_logger().info(f"Estimador de la mano (EKF + UKF) listo, entrada: {self.cfg['input_mode']}")

    def _on_params(self, params):
        changed = {}
        for p in params:
            if p.name in self.cfg:
                self.cfg[p.name] = p.value
            elif p.name in self.core.p:
                changed[p.name] = p.value
        if changed:
            self.core.set_params(**changed)

    def _on_raw(self, msg):
        self.raw = msg

    def _on_rpy(self, msg):
        self.rel_yaw = msg.z

    def _on_reset(self, _msg):
        self.core.reset(psi=self.last_psi)
        self.core.t_last = None
        self.get_logger().info('Estimacion de la mano reiniciada')

    def _on_filtered(self, msg):
        q = msg.orientation
        la, av = msg.linear_acceleration, msg.angular_velocity
        a_raw = g_raw = None
        if self.raw is not None:
            r = self.raw
            a_raw = (r.linear_acceleration.x, r.linear_acceleration.y, r.linear_acceleration.z)
            g_raw = (r.angular_velocity.x, r.angular_velocity.y, r.angular_velocity.z)
        meas = hand_measurements((q.x, q.y, q.z, q.w), (la.x, la.y, la.z), (av.x, av.y, av.z),
                                 a_raw, g_raw, self.rel_yaw, self.cfg['input_mode'])
        if meas is None:
            return
        a_head, psi, wz, gyro_norm = meas
        self.last_psi = psi

        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        self.core.step(t, a_head, psi, wz, gyro_norm)
        self._publish(msg.header.stamp)

    def _publish(self, stamp):
        stats = {'input_mode': self.cfg['input_mode'], 'zupt': self.core.zupt}
        for name in ('EKF', 'UKF'):
            s = self.core.state(name)
            stats[name] = {k: s[k] for k in ('us', 'trace', 'nis', 'bytes')}

            m = Odometry()
            m.header.stamp = stamp
            m.header.frame_id = 'hand_plane'
            m.child_frame_id = 'hand'
            m.pose.pose.position.x = s['x']
            m.pose.pose.position.y = s['y']
            _, _, qz, qw = yaw_to_quat(s['psi'])
            m.pose.pose.orientation.z = qz
            m.pose.pose.orientation.w = qw
            m.twist.twist.linear.x = s['vx']
            m.twist.twist.linear.y = s['vy']
            self.odom_pub[name].publish(m)

            a = Vector3Stamped()
            a.header = m.header
            a.vector.x = s['ax']
            a.vector.y = s['ay']
            self.acc_pub[name].publish(a)
        self.stats_pub.publish(String(data=json.dumps(stats)))


def main(args=None):
    rclpy.init(args=args)
    node = HandEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
