from setuptools import setup

package_name = 'group_project'

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
    maintainer='Joshua Gross',
    maintainer_email='jjgross@wpi.edu',
    description='This will complete the first part of the group project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calculate = group_project.calc_kinematics:main'
        ],
    },
)
