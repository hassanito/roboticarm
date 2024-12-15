from setuptools import setup
from glob import glob
import os

package_name = 'eezybotarm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # URDF files
        (os.path.join('share', package_name, 'description'),
         glob(os.path.join('description', '*.urdf'))),
        # Mesh files
        (os.path.join('share', package_name, 'meshes', 'ebamk2'),
         glob(os.path.join('meshes', 'ebamk2', '*.STL'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='EEZYbotARM MK2 ROS2 Control Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = eezybotarm_control.arm_controller:main',
            'arm_test_patterns = eezybotarm_control.arm_test_patterns:main',
        ],
    },
)