from setuptools import find_packages, setup

package_name = 'sensores_kalman'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'imu_kalman_node = sensores_kalman.imu_kalman_node:main',
        'lidar_listener = sensores_kalman.lidar_listener_node:main',
        'imu_kalman_all_node = sensores_kalman.imu_kalman_all_node:main',
        'velocity_kalman_node = sensores_kalman.velocity_kalman_node:main',
        'fusion_kalman_node = sensores_kalman.fusion_kalman_node:main',
    ],
},
)

