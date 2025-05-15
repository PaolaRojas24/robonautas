from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_closed_loop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paoro',
    maintainer_email='paoro@todo.todo',
    description='Puzzlebot Closed Loop Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_generator = puzzlebot_closed_loop.path_generator:main',
            'controller = puzzlebot_closed_loop.controller:main',
            'opencv_act1 = puzzlebot_closed_loop.opencv_act1:main',
            'rqt_image_view = puzzlebot_closed_loop.rqt_image_view:main', 
            'color_detection = puzzlebot_closed_loop.color_detection:main',
            'colorpicker = puzzlebot_closed_loop.colorpicker:main',
            'prueba = puzzlebot_closed_loop.prueba:main',
            'prueba_base = puzzlebot_closed_loop.prueba_base:main',
            'puzzlebot_odometry = puzzlebot_closed_loop.puzzlebot_odometry:main',

        ],
    },
)
