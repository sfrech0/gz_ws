from setuptools import find_packages, setup
from glob import glob

package_name = 'water_drones'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sfrech',
    maintainer_email='sfrech@ethz.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'csv_publisher = water_drones.csv_publisher:main',
            # 'drone_state_updater = water_drones.drone_state_updater:main',
            # 'drone_client = water_drones.drone_client:main',
            # 'test_script = water_drones.test:main',
            # 'pose_to_gz = water_drones.pose_to_gz:main',
            'motor_controller = water_drones.motor_controller:main',
            'imu_pose_to_odometry = water_drones.imu_pose_to_odometry:main',
            'timed_pursuit_node = water_drones.timed_pursuit_node:main',
            'exit_plan = scripts.exit_plan:main'
        ],
    },
)
