from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


setup(
    name='tht7_tb4',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tht7_tb4']),
        ('share/tht7_tb4', ['package.xml', 'README.md', 'requirements-voice.txt']),
        *[('share/tht7_tb4/' + directory,
           [path for path in glob(directory + '/*') if Path(path).is_file()])
          for directory in ('config', 'launch', 'rviz', 'results')],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuanvu',
    maintainer_email='tuanvu16205@gmail.com',
    description='Semantic map segmentation and Vietnamese voice navigation for TurtleBot4.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'segment_map = tht7_tb4.segmentation:main',
        'region_markers = tht7_tb4.region_markers:main',
        'voice_navigation = tht7_tb4.voice_navigation:main',
        'speech_input = tht7_tb4.speech_input:main',
        'send_command = tht7_tb4.speech_input:text_main',
    ]},
)
