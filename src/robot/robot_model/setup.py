from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'meshes', 'AR5_P_R'), glob('meshes/AR5_P_R/*.stl')),
        (os.path.join('share', package_name, 'meshes', 'AR5_P_L'), glob('meshes/AR5_P_L/*.stl')),
        (os.path.join('share', package_name, 'meshes', 'CTM_2F_110'), glob('meshes/CTM_2F_110/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nerv',
    maintainer_email='Arinffy@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
