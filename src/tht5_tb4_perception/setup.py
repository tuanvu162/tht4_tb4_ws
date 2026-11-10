from setuptools import find_packages, setup
from glob import glob

package_name = 'tht5_tb4_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuanvu',
    maintainer_email='tuanvu16205@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_navigation = tht5_tb4_perception.waypoint_navigation:main',
            'yolo_detector = tht5_tb4_perception.yolo_detector:main',
        ],
    },
)
