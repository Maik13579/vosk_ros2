from setuptools import setup
import os 
from glob import glob

package_name = 'vosk_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'grammar'), glob(os.path.join('grammar', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maik Knof',
    maintainer_email='Maik.Knof@gmx.de',
    description='ROS2 wrapper for Vosk',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vosk_ros2_node = vosk_ros2.vosk_ros2_node:main'
        ],
    },
)
