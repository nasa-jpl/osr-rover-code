import os
from glob import glob
from setuptools import setup

package_name = 'osr_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Achille',
    maintainer_email='achille.verheye@gmail.com',
    description='core control nodes for the OSR',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover = osr_control.rover:main',
            'roboclaw_wrapper = osr_control.roboclaw_wrapper:main',
            'servo_control = osr_control.servo_control:main',
            'ina260 = osr_control.ina_260_pub:main',
            'joy_extras = osr_control.joy_extras:main'
        ],
    },
)
