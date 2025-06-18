import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'argos_final'  

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
    maintainer='Paola Rojas',
    maintainer_email='a01737136@tec.mx',
    description='Line Follower',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_image_view = argos_final.rqt_image_view:main',
            'controllerf = argos_final.controllerf:main',
            'line_follower = argos_final.line_follower:main',
            'camara_node = argos_final.camara_node:main',
            'detection = argos_final.detection:main',
        ],
    },
)

