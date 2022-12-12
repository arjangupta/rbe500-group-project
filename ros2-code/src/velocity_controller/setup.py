from setuptools import setup

package_name = 'velocity_controller'

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
    description='Velocity controller for SCARA',
    license='Property of WPI',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_controller = scara_velocity_controller.scara_velocity_controller:main'
        ],
    },
)
