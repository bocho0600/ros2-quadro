from setuptools import setup
import os

package_name = 'actuators_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # this must match the folder inside src
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # marker file
        ('share/' + package_name, ['package.xml']),  # package.xml explicitly installed
    ],
    install_requires=['setuptools', 'pyqt5'],
    zip_safe=True,
    maintainer='kelvin_2204',
    description='ROS 2 node using QDialog with PyQt5',
    entry_points={
        'console_scripts': [
            'dialog_node = actuators_calibration.dialog_node:main',
        ],
    },
)
