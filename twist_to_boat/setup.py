from setuptools import setup
from glob import glob
import os

package_name = 'twist_to_boat'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atticus',
    maintainer_email='atticusrussell@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_position_node = twist_to_boat.twist_to_position_node:main',
        ],
    },
)
