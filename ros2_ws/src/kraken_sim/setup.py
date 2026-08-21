from setuptools import find_packages, setup

package_name = 'kraken_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emiliano Borghi',
    maintainer_email='emiliano.borghi@example.com',
    description='Headless kinematic simulator used in place of O3DE for automated '
                'scenarios, and the bridge that adapts Twist commands to the O3DE '
                "robot's Ackermann control interface.",
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'headless_sim = kraken_sim.headless_sim:main',
            'ackermann_bridge = kraken_sim.ackermann_bridge:main',
            'odom_tf = kraken_sim.odom_tf:main',
        ],
    },
)
