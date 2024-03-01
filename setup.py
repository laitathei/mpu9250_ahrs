from setuptools import find_packages, setup

package_name = 'mpu9250_madgwick'

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
    maintainer='laitathei',
    maintainer_email='k66115704@gmail.com',
    description='MPU9250 driver with Madgwick filter',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_node = mpu9250_madgwick.ros2_node:main'
        ],
    },
)
