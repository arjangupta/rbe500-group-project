from setuptools import setup

package_name = 'inv_kin_serv'

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
    maintainer='Arjan Gupta',
    maintainer_email='agupta11@wpi.edu',
    description='Inverse kinematics ROS portion for Group Assignment 1',
    license='Property of WPI',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = inv_kin_serv.inverse_kin_service:main'
        ],
    },
)
