from setuptools import setup

package_name = 'actuators_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # this must match inner folder name
    install_requires=['setuptools', 'pyqt5'],
    zip_safe=True,
    maintainer='kelvin_2204',
    description='ROS 2 node using QDialog with PyQt5',
    entry_points={
        'console_scripts': [
            'dialog_node = actuators_calibration.dialog_node:main',  # module path matches folder
        ],
    },
)
