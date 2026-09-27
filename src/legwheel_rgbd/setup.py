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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='LegWheel launch and configuration for the Orbbec Gemini 336.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
