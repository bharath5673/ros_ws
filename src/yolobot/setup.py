from setuptools import setup

package_name = 'yolobot'

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
            'yolov5_demo = yolobot.yolobot_recognition.demo:main',
            'yolov5_gazebo = yolobot.yolobot_recognition.yolobot_recognition:main'

        ],
    },
)
