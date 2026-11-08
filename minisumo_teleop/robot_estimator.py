"""Estimacion de la posicion del robot minisumo con EKF y UKF en paralelo.

Entradas (del ESP32 del carro, micro-ROS):
    imu/raw       sensor_msgs/Imu    giroscopio z crudo (con sesgo)
    imu/filtered  sensor_msgs/Imu    giroscopio z sin sesgo + pasa-bajos
    wheel_*/filtered  std_msgs/Float32  rapidez de cada rueda (detecta
                  robot quieto para la pseudo-medicion w = 0)
    odom/twist    geometry_msgs/Twist  linear.x = v de los encoders
    cmd_vel       geometry_msgs/Twist  entrada del sistema (v, w comandadas)
    teleop/espnow geometry_msgs/Twist  entrada cuando el guante mueve al carro
                  por ESP-NOW (linear.z = 2: armado; v, w en linear.x, angular.z)
    robot/tracking  geometry_msgs/PoseStamped  (opcional) sistema de
                  tracking externo: posicion absoluta en el dojo
    robot/reset   std_msgs/Empty     vuelve el estado a (0, 0, 0)

Salidas:
    robot/ekf/odom, robot/ukf/odom   nav_msgs/Odometry (marco "dojo")
    robot/stats   std_msgs/String    JSON con tiempos de computo, traza
                  de P, NIS y memoria de cada filtro
    dojo/markers  visualization_msgs/MarkerArray  dojo para RViz
    TF dojo -> base_link (filtro elegido en tf_source)

Parametro input_mode: "raw" usa el giroscopio crudo y estima su sesgo;
"filtered" usa el giroscopio ya filtrado en el ESP32 (sin sesgo).
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from .common import LATCHED_QOS, declare_params, odom_cov6, quat_to_rpy, yaw_to_quat
from .models import ROBOT_DEFAULTS, RobotEstimatorCore

NODE_DEFAULTS = {
    'rate_hz': 50.0,
    'input_mode': 'raw',          # raw | filtered
    'publish_tf': True,
    'tf_source': 'ekf',           # ekf | ukf
    'dojo_radius': 2.5,           # m, dojo de 5 m de diametro
    'sensor_timeout': 0.5,        # s sin datos del carro: se congela
}


class RobotEstimator(Node):
    def __init__(self):
        super().__init__('robot_estimator')
        self.cfg = declare_params(self, NODE_DEFAULTS)
        kf_params = declare_params(self, {k: float(v) for k, v in ROBOT_DEFAULTS.items() if k != 'gyro_has_bias'})
        self.core = RobotEstimatorCore(kf_params)
        self._apply_mode()
        self.add_post_set_parameters_callback(self._on_params)

        self.last_sensor_t = None

        self.create_subscription(Imu, 'imu/raw', self._on_imu_raw, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/filtered', self._on_imu_filt, qos_profile_sensor_data)
        self.create_subscription(Twist, 'odom/twist', self._on_twist, qos_profile_sensor_data)
        self.wheel_l = 0.0
        self.create_subscription(Float32, 'wheel_left/filtered', self._on_wl, qos_profile_sensor_data)
        self.create_subscription(Float32, 'wheel_right/filtered', self._on_wr, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(Twist, 'teleop/espnow', self._on_espnow, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'robot/tracking', self._on_tracking, 10)
        self.create_subscription(Empty, 'robot/reset', self._on_reset, 10)

        self.pub = {
            'EKF': self.create_publisher(Odometry, 'robot/ekf/odom', 10),
            'UKF': self.create_publisher(Odometry, 'robot/ukf/odom', 10),
        }
        self.stats_pub = self.create_publisher(String, 'robot/stats', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'dojo/markers', LATCHED_QOS)
        self.tf = TransformBroadcaster(self)

        self._publish_dojo()
        self.create_timer(1.0 / self.cfg['rate_hz'], self._tick)
        self.get_logger().info(f"Estimador del robot (EKF + UKF) listo, entrada del giroscopio: {self.cfg['input_mode']}")

    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _apply_mode(self):
        self.core.p['gyro_has_bias'] = 1.0 if self.cfg['input_mode'] == 'raw' else 0.0

    def _on_params(self, params):
        changed = {}
        for p in params:
            if p.name in self.cfg:
                self.cfg[p.name] = p.value
            elif p.name in self.core.p:
                changed[p.name] = p.value
        if changed:
            self.core.set_params(**changed)
        self._apply_mode()

    def _on_imu_raw(self, msg):
        self.last_sensor_t = self._now()
        if self.cfg['input_mode'] == 'raw':
            self.core.add_gyro(msg.angular_velocity.z)

    def _on_imu_filt(self, msg):
        self.last_sensor_t = self._now()
        if self.cfg['input_mode'] != 'raw':
            self.core.add_gyro(msg.angular_velocity.z)

    def _on_twist(self, msg):
        self.last_sensor_t = self._now()
        self.core.add_speed(msg.linear.x)

    def _on_wl(self, msg):
        self.wheel_l = msg.data

    def _on_wr(self, msg):
        # el ESP32 publica izquierda y luego derecha en el mismo ciclo
        self.core.add_wheels(self.wheel_l, msg.data)

    def _on_cmd(self, msg):
        self.core.set_cmd(self._now(), msg.linear.x, msg.angular.z)

    def _on_espnow(self, msg):
        if int(msg.linear.z) == 2:   # el guante armado esta conduciendo
            self.core.set_cmd(self._now(), msg.linear.x, msg.angular.z)

    def _on_tracking(self, msg):
        q = msg.pose.orientation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.core.add_pose(msg.pose.position.x, msg.pose.position.y, yaw)

    def _on_reset(self, _msg):
        self.core.reset()
        self.core.t_last = None
        self.get_logger().info('Estimacion del robot reiniciada en (0, 0, 0)')

    # ------------------------------------------------------------------
    def _tick(self):
        t = self._now()
        if self.last_sensor_t is None or t - self.last_sensor_t > self.cfg['sensor_timeout']:
            # Sin datos del carro no se predice: la posicion no debe
            # derivar mientras el robot esta desconectado.
            self.core.t_last = None
            return
        self.core.step(t)

        stamp = self.get_clock().now().to_msg()
        stats = {'input_mode': self.cfg['input_mode'], 'still': self.core.still}
        for name, pub in self.pub.items():
            s = self.core.state(name)
            stats[name] = {k: s[k] for k in ('us', 'trace', 'nis', 'bytes', 'bias')}
            m = Odometry()
            m.header.stamp = stamp
            m.header.frame_id = 'dojo'
            m.child_frame_id = 'base_link'
            m.pose.pose.position.x = s['x']
            m.pose.pose.position.y = s['y']
            qx, qy, qz, qw = yaw_to_quat(s['th'])
            m.pose.pose.orientation.z = qz
            m.pose.pose.orientation.w = qw
            m.pose.covariance = odom_cov6(s['cov'])
            m.twist.twist.linear.x = s['v']
            m.twist.twist.angular.z = s['w']
            pub.publish(m)

            if self.cfg['publish_tf'] and name.lower() == self.cfg['tf_source']:
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = 'dojo'
                tf.child_frame_id = 'base_link'
                tf.transform.translation.x = s['x']
                tf.transform.translation.y = s['y']
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                self.tf.sendTransform(tf)

        self.stats_pub.publish(String(data=json.dumps(stats)))

    def _publish_dojo(self):
        """Dojo de minisumo para RViz: disco negro, borde blanco y lineas
        de salida (shikiri)."""
        R = float(self.cfg['dojo_radius'])
        arr = MarkerArray()

        def mk(mid, mtype, sx, sy, sz, rgba, z=0.0, x=0.0, y=0.0):
            m = Marker()
            m.header.frame_id = 'dojo'
            m.ns = 'dojo'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            return m

        arr.markers.append(mk(0, Marker.CYLINDER, 2 * R, 2 * R, 0.02, (0.92, 0.92, 0.92, 1.0), z=-0.012))
        arr.markers.append(mk(1, Marker.CYLINDER, 2 * (R - 0.05), 2 * (R - 0.05), 0.022, (0.05, 0.05, 0.06, 1.0), z=-0.011))
        for i, xx in enumerate((0.05, -0.05)):
            arr.markers.append(mk(2 + i, Marker.CUBE, 0.012, 0.10, 0.023, (0.55, 0.35, 0.2, 1.0), z=-0.01, x=xx))
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RobotEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
