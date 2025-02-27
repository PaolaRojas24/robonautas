import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'motor_control_argos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danielc',
    maintainer_email='danielc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_sys = motor_control_argos.motor_sys:main',
            'set_point = motor_control_argos.set_point:main',
            'controller = motor_control_argos.controller:main',
        ],
    },
)
