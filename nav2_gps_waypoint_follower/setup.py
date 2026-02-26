from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_gps_waypoint_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps'),
         glob('models/turtlebot_waffle_gps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='Demo package for following GPS waypoints with nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = nav2_gps_waypoint_follower.logged_waypoint_follower:main',
            'interactive_waypoint_follower = nav2_gps_waypoint_follower.interactive_waypoint_follower:main',
            'gps_waypoint_logger = nav2_gps_waypoint_follower.gps_waypoint_logger:main'
        ],
    },
)
