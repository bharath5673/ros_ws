from setuptools import setup

package_name = 'test_mediapipe'

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
            'holistic_demo = test_mediapipe.holistic_detector:main',
            'hands_demo = test_mediapipe.hands_detector:main',
            'pose_demo = test_mediapipe.pose_detetcor:main',
            'facemesh_demo = test_mediapipe.face_mesh_detector:main'
        ],
    },
)
