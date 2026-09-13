from setuptools import setup
import os
from glob import glob

package_name = 'autodrive_roboracer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'racing_algo'],
    py_modules=['localize', 'store'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Launch files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # RViz configuration files
        (os.path.join('share', package_name, 'config'),
         ['amcl_params.yaml', 'f1tenth_online_async.yaml',
          'mpc_params.yaml', 'pp_backup_params.yaml']),
        (os.path.join('share', package_name, 'maps'),
         ['practice_track.yaml', 'practice_track.pgm']),
        (os.path.join('share', package_name, 'maps', 'track-20260731'),
         ['maps/track-20260731/map.yaml',
          'maps/track-20260731/map.pgm',
          'maps/track-20260731/meta.json']),
        (os.path.join('share', package_name, 'racelines',
                      'track-20260731-line'),
         ['racelines/track-20260731-line/waypoints.csv',
          'racelines/track-20260731-line/meta.json']),
        (os.path.join('share', package_name),
         ['practice_waypoints.csv', 'comp_waypoints.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinmay Vilas Samak' 'Tanmay Vilas Samak',
    maintainer_email='csamak@clemson.edu' 'tsamak@clemson.edu',
    description='AutoDRIVE Ecosystem ROS 2 Package for RoboRacer',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autodrive_bridge = autodrive_roboracer.autodrive_bridge:main', # AutoDRIVE ROS 2 bridge
            'teleop_keyboard = autodrive_roboracer.teleop_keyboard:main', # Teleoperation with keyboard
            'mpc_node = racing_algo.mpc_node:main',
            'pp_backup_node = racing_algo.pp_backup_node:main',
            'amcl_initialpose = racing_algo.amcl_initialpose_node:main',
        ],
    },
)
