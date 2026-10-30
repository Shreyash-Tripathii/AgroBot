import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agrobot_depth_fusion'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shreyash-tripathi',
    maintainer_email='shreyash5600@gmail.com',
    description='Sparse-to-Dense Scale Alignment pipeline using Depth Anything V2 and 2D LiDAR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_fusion_node = agrobot_depth_fusion.depth_fusion_node:main',
        ],
    },
)
