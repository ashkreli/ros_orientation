from setuptools import setup
import os
from glob import glob

package_name = 'multisim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', 'models', 'turtlebot3_burger', '*.config'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', 'models', 'turtlebot3_burger', '*.sdf'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', 'models', 'turtlebot3_burger', '*.dae'), recursive=True)),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'), recursive=True)),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arba Shkreli',
    maintainer_email='ashkreli@college.harvard.edu',
    description='Multi-Robot Reference Tracking via Potential Function Gradient Descent',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle = multisim.turtle:main',
            'turtles = multisim.turtles:main',
        ],
    },
)
