import os

from setuptools import find_packages, setup

package_name = 'legwheel_rgbd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Keep installed resources explicit. A wildcard can match broken,
        # package-local symlinks left in colcon's build directory after a source
        # file is deleted, causing setuptools to try copying that deleted file.
        (os.path.join('share', package_name, 'config'), [
            'config/gemini_336.yaml',
        ]),
        (os.path.join('share', package_name, 'launch'), [
            'launch/gemini_336.launch.py',
        ]),
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
