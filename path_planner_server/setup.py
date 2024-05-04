from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_planner_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'nav_to_pose = path_planner_server.nav_to_pose_action_client:main',
        'transform = path_planner_server.transform_distance_service:main',
        'walk = path_planner_server.walk_farthest:main',
        'rectangle_finder = path_planner_server.outer_rectangle_finder:main'
        ],
    },
)
