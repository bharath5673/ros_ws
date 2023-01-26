from setuptools import setup

package_name = 'test_opencv'

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
    maintainer_email='bharath@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = test_opencv.webcam_pub:main',
            'img_subscriber = test_opencv.webcam_sub:main',
            'run_test = test_opencv.run_cv_test:main'
        ],
    },
)
