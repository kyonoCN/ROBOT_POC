from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'workspace'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Enhanced workspace obstacle management GUI for MoveIt planning scene',
    license='MIT',
    entry_points={
        'console_scripts': [
            'workspace_gui = workspace.workspace_gui:main',
            'test_attach_detach = workspace.test_attach_detach:main',
        ],
    },
)
