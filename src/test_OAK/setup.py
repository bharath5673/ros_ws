from setuptools import setup

package_name = 'test_OAK'

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
            'OAK_1_publisher = test_OAK.oak1_pub:main',
            'OAK_D_publisher = test_OAK.oakD_pub:main',
            'OAK_subscriber = test_OAK.oak_sub:main',
        ],
    },
)
