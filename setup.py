from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'action_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dghwang',
    maintainer_email='daegyu@telelian.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_control_node = action_control.action_control_node:main',
            'intersection_check_node = action_control.intersection_check_node:main',
        ],
    },
)
