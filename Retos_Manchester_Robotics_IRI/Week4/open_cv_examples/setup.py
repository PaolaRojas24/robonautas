from setuptools import find_packages, setup

package_name = 'open_cv_examples'

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
    maintainer='Mario Martinez',
    maintainer_email='mario.mtz@manchester-robotics.com',
    description='OpenCV Examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_generator = open_cv_examples.path_generator:main',
            'controller = open_cv_examples.controller:main',
            'opencv_act1 = open_cv_examples.opencv_act1:main',
            'rqt_image_view = open_cv_examples.rqt_image_view:main', 
            'color_detection = open_cv_examples.color_detection:main',
            'colorpicker = open_cv_examples.colorpicker:main',
            'prueba = open_cv_examples.prueba:main',
            'prueba_base = open_cv_examples.prueba_base:main',
            'puzzlebot_odometry = open_cv_examples.puzzlebot_odometry:main',
        ],
    },
)
