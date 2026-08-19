import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'kraken_faults'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emiliano Borghi',
    maintainer_email='emiliano.borghi@example.com',
    description='Runtime sensor fault injection for localisation robustness testing.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'fault_injector = kraken_faults.fault_injector:main',
        ],
    },
)
