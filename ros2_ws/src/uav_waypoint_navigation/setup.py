from setuptools import find_packages, setup

package_name = 'uav_waypoint_navigation'

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
    maintainer='ayodeji',
    maintainer_email='abioyeayo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'uav_waypoint_navigation = uav_waypoint_navigation.uav_waypoint_navigation:main',
        	'uav_waypoint_processor_marl = uav_waypoint_navigation.uav_waypoint_processor_marl:main',
        ],
    },
)
