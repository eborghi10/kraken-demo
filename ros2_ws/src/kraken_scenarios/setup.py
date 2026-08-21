import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'kraken_scenarios'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emiliano Borghi',
    maintainer_email='emiliano.borghi@example.com',
    description='Fault scenarios, localisation scoring and the launch test suite.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scorer = kraken_scenarios.scorer:main',
            'scenario_runner = kraken_scenarios.scenario_runner:main',
            'sim_admin = kraken_scenarios.sim_admin:main',
            'sweep = kraken_scenarios.sweep:main',
        ],
    },
)
