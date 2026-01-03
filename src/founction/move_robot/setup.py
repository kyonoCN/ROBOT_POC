from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'move_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Virtual robot base movement control GUI for wheeled humanoid robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot_gui = move_robot.move_robot_gui:main',
        ],
    },
)
