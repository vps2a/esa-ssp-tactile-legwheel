from setuptools import find_packages, setup

package_name = 'legwheel_experiments'

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
    maintainer='parallels',
    maintainer_email='f.szczebak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_experiment = legwheel_experiments.cli:main',
            'state_machine_test = legwheel_experiments.cli:sample_transition_run', #TODO: delete this later
            'state_machine_abort_test = legwheel_experiments.cli:sample_abort_run', #TODO: delete this later
            'create_run = legwheel_experiments.cli:create_new_run_directory', #TODO: delete this later
            'experiment_runner_node = legwheel_experiments.experiment_runner_node:main',
        ],
    },
)
