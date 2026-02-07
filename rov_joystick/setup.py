from setuptools import find_packages, setup

package_name = 'rov_joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shashwat',
    maintainer_email='iamshashwatfr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_node = rov_joystick.joy_node:main',
            'keyboard_teleop = rov_joystick.keyboard_teleop:main', 
        ],
    },
)
