import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'apf_reactive_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Ensures launch and config files are installed
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='song-iar',
    maintainer_email='song-iar@todo.todo',
    description='APF Reactive Control Package',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'object_position = apf_reactive_control.object_position:main',
            'motion_planner = apf_reactive_control.motion_planner:main',
            'execute_dodging = apf_reactive_control.execute_dodging:main',
            'controller = apf_reactive_control.controller:main', 
        ],
    },
)