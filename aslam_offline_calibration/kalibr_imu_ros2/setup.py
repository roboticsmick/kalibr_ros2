from setuptools import setup, find_packages

package_name = 'kalibr_imu_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='IMU-camera calibration for ROS2, ported from ETH Kalibr',
    license='New BSD',
    entry_points={
        'console_scripts': [],
    },
)
