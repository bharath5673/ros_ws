from setuptools import setup

package_name = 'test_yolov5'

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
            'yolov5n_demo = test_yolov5.test_yolov5n:main',
            'yolov5s_demo = test_yolov5.test_yolov5s:main'
        ],
    },
)
