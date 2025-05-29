from setuptools import find_packages, setup

package_name = 'line_argos'  

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
    maintainer='Paola Rojas',
    maintainer_email='a01737136@tec.mx',
    description='Line Follower',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_image_view = line_argos.rqt_image_view:main', 
            'color_detection = line_argos.color_detection:main',
            'controllerf = line_argos.controllerf:main',
            'line_follower = line_argos.line_follower:main',
            'line_follower_p = line_argos.line_follower_p:main',
            'line_follower_d = line_argos.line_follower_d:main',
            'prueba = line_argos.prueba:main',
        ],
    },
)
