from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'legwheel_rgbd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_rgbd_publisher_node = legwheel_rgbd.fake_rgbd_publisher_node:main',
            'depth_visualizer_node = legwheel_rgbd.depth_visualizer_node:main',
            'sensorstream_bridge_node = legwheel_rgbd.sensorstream_bridge_node:main',
        ],
    },
)
