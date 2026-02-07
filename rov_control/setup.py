import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rov_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # --- NEW: THIS LINE REGISTERS THE LAUNCH FILE ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shashwat',
    maintainer_email='iamshashwatfr@gmail.com',
    description='ROV Control Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Link your python scripts here
            'thruster_controller = rov_control.thruster_controller:main',
            'serial_bridge = rov_control.serial_bridge:main', # <--- ADD THIS
        ],
    },
)
