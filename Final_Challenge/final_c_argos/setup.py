import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'final_c_argos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
         (os.path.join('share', package_name, 'config'),glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paoro',
    maintainer_email='paoro@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input = final_c_argos.input:main',
        ],
    }
)
