import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'osr_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Achllle',
    maintainer_email='achille.verheye@gmail.com',
    description='OSR robot description URDF',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = osr_description.state_publisher:main'
        ],
    },
)
