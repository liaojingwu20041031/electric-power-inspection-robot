from glob import glob
from setuptools import setup

package_name = 'ylhb_mobile_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YLHB',
    maintainer_email='dev@example.com',
    description='HTTP/WebSocket bridge between mobile APP and ROS2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mobile_bridge_server = ylhb_mobile_bridge.mobile_bridge_server:main',
            'patrol_executor_node = ylhb_mobile_bridge.patrol_executor_node:main',
        ],
    },
)
