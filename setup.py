import os
from glob import glob

from setuptools import setup

package_name = 'minisumo_teleop'


def tree(src, dst):
    """Instala una carpeta completa (la app web) conservando subcarpetas."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join(dst, os.path.relpath(root, src)),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ] + tree('web', 'share/' + package_name + '/web'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minisumo_teleop',
    maintainer_email='noreply@example.com',
    description='Tele-operacion del minisumo con guante IMU, EKF/UKF y tablero web',
    license='MIT',
    entry_points={
        'console_scripts': [
            'glove_teleop = minisumo_teleop.glove_teleop:main',
            'hand_estimator = minisumo_teleop.hand_estimator:main',
            'robot_estimator = minisumo_teleop.robot_estimator:main',
            'dashboard = minisumo_teleop.dashboard:main',
            'offline_eval = minisumo_teleop.offline_eval:main',
            'fake_hardware = minisumo_teleop.fake_hardware:main',
        ],
    },
)
