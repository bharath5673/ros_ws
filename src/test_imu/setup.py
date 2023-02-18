from setuptools import setup

package_name = 'test_imu'

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
    maintainer='bharath',
    maintainer_email='s.bharath.2000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = test_imu.imu_publisher:main',
            'imu_subscriber = test_imu.imu_subscriberr:main',
            'simple_imu = test_imu.imu_simple_pub_sub:main',
            'picoW_mpu6050 = test_imu.imu_MPU6050_picoW:main',
            'esp32_mpu6050 = test_imu.imu_MPU6050_esp32:main'
        ],
    },
)
