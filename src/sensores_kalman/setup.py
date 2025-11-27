from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'sensores_kalman'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),   # AGREGAR ESTO
         glob('rviz/*.rviz')),                           # AGREGAR ESTO
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abrahammh19',
    maintainer_email='abrahammh19@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'lidar_kalman_node = sensores_kalman.lidar_kalman_node:main',
        # 'imu_kalman_node = sensores_kalman.imu_kalman_node:main',
        # 'pose_final_qcar = sensores_kalman.pose_final_qcar:main',
        # 'imu_kalman_all_node = sensores_kalman.imu_kalman_all_node:main',
        # 'qcar_pose = sensores_kalman.qcar_pose:main',
        # 'fusion_kalman_node = sensores_kalman.fusion_kalman_node:main',
        # 'kalman_all_obstacle_node = sensores_kalman.kalman_all_obstacle_node:main',
        # 'ekf_plot_node_qcar = sensores_kalman.ekf_plot_node_qcar:main',
        # 'ekf_fusion_node_qcar = sensores_kalman.ekf_fusion_node_qcar:main',
        # 'pose_ekf_qcar = sensores_kalman.pose_ekf_qcar:main',
        'pose_ekf_qcar_2 = sensores_kalman.pose_ekf_qcar_2:main',
        # 'pose_calculator = sensores_kalman.pose_calculator:main',
        # 'pose_calculator_2 = sensores_kalman.pose_calculator_2:main',
        # 'trajectory_qcar = sensores_kalman.trajectory_qcar:main',
        # 'fusion_kalman_all_node = sensores_kalman.fusion_kalman_all_node:main',
        # 'imu_external = sensores_kalman.imu_external:main',
        # 'trajectory_qcar_2 = sensores_kalman.trajectory_qcar_2:main',
        # 'qcar_ekf_sensorfusion = sensores_kalman.qcar_ekf_sensorfusion:main',
        # 'qcar_ekf_vizualizer = sensores_kalman.qcar_ekf_vizualizer:main',
        # 'ekf_fusion_node_sinlidar = sensores_kalman.ekf_fusion_node_sinlidar:main'  
        'qcar_lidar_alert = sensores_kalman.qcar_lidar_alert:main',
        'qcar_lidar_alert_2 = sensores_kalman.qcar_lidar_alert_2:main'
    ],
},
)



