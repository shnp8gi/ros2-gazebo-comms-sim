from setuptools import setup, find_packages

package_name = 'comms_sim_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='user@example.com',
    description='ROS 2 Communication Simulation Package for Gazebo Harmonic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comms_node = comms_sim_pkg.comms_node:main',
            'ugv_controller_node = comms_sim_pkg.ugv_controller_node:main',
        ],
    },
)
