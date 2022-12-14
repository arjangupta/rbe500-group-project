from setuptools import setup

package_name = 'velocity_kinematics'

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
    maintainer='arjan',
    maintainer_email='agupta11@wpi.edu',
    description='ROS Node for Velocity Kinematics Calculations',
    license='Property of WPI',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'veloctiy_kinematics = veloctiy_kinematics.scara_veloctiy_kinematics:main'
        ],
    },
)
