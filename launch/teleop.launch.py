"""Lanza todo el sistema de tele-operacion del PC.

    ros2 launch minisumo_teleop teleop.launch.py
    ros2 launch minisumo_teleop teleop.launch.py agent:=true rviz:=true port:=8090
    ros2 launch minisumo_teleop teleop.launch.py sim:=true      # sin hardware

agent:=true tambien arranca el agente micro-ROS en Docker (UDP 8888) para que se
conecten el ESP32 del guante y el del carro.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('minisumo_teleop')
    params = os.path.join(share, 'config', 'params.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'teleop.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8090', description='Puerto del tablero web'),
        DeclareLaunchArgument('rviz', default_value='false', description='Abrir RViz2 con el dojo'),
        DeclareLaunchArgument('agent', default_value='false', description='Arrancar el agente micro-ROS (udp4 8888)'),
        DeclareLaunchArgument('sim', default_value='false', description='Simular guante y carro (sin hardware)'),
        DeclareLaunchArgument('hand_motion', default_value='tilt', description='Simulacion: tilt | plane | idle'),

        # Mismo comando que usas a mano (agente en Docker)
        ExecuteProcess(
            cmd=['docker', 'run', '--rm', '--net=host', '-e', 'FASTDDS_BUILTIN_TRANSPORTS=UDPv4',
                 'microros/micro-ros-agent:rolling', 'udp4', '--port', '8888', '--discovery', '7400'],
            output='screen', condition=IfCondition(LaunchConfiguration('agent'))),

        Node(package='minisumo_teleop', executable='fake_hardware', name='fake_hardware',
             parameters=[{'hand_motion': LaunchConfiguration('hand_motion')}],
             condition=IfCondition(LaunchConfiguration('sim')), output='screen'),

        Node(package='minisumo_teleop', executable='hand_estimator', name='hand_estimator',
             parameters=[params], output='screen'),
        Node(package='minisumo_teleop', executable='robot_estimator', name='robot_estimator',
             parameters=[params], output='screen'),
        Node(package='minisumo_teleop', executable='glove_teleop', name='glove_teleop',
             parameters=[params], output='screen'),
        Node(package='minisumo_teleop', executable='dashboard', name='teleop_dashboard',
             parameters=[{'port': LaunchConfiguration('port')}], output='screen'),

        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz')), output='log'),
    ])
