from setuptools import setup

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
    maintainer='Atticus Russell',
    maintainer_email='atticusrussell@gmail.com',
    description='converts twist messages to appropriate types for boat rudder and propeller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_boat = twist_to_boat.twist_to_boat:main'
        ],
    },
)
