from setuptools import setup

package_name = 'mpu6050'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'imu_publisher_node = mpu6050.imu_publisher_node:main',
           'imu_offset_corrector = mpu6050.imu_offset_corrector:main',
        ],
    },
)
