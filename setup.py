from setuptools import find_packages, setup

package_name = 'ros2_set_C_from_A_via_B'

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
    maintainer='Ignacio Montesino',
    maintainer_email='monte.igna@gmail.com',
    description='Example for calling a ROS2 sercive wwhithin another callback',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
