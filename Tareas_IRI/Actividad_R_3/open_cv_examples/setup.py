from setuptools import find_packages, setup

package_name = 'open_cv_examples'  # Asegúrate de usar el nombre correcto de tu paquete

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
    maintainer='Tu Nombre',
    maintainer_email='tu.email@dominio.com',
    description='OpenCV Examples para ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencv_act1 = open_cv_examples.opencv_act1:main',
            'rqt_image_view = open_cv_examples.rqt_image_view:main', 
            'color_detection = open_cv_examples.color_detection:main',
            'colorpicker = open_cv_examples.colorpicker:main',
        ],
    },
)
