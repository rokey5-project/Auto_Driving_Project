from setuptools import find_packages, setup
from glob import glob

package_name = 'patrol_robot_b'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rlarkqals@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'patrol_node = patrol_robot_b.patrol_node:main',
            'detect_person_node = patrol_robot_b.detect_person_node:main',
            'buzzer_node = patrol_robot_b.buzzer_node:main',
        ],
    },
)
