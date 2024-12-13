from setuptools import find_packages, setup

package_name = 'py_demo_gtsam'

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
    maintainer='root',
    maintainer_email='thibault.schweitzer@estaca.eu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_motion = py_demo_gtsam.2_robot_motion:main',
            'robot_localization = py_demo_gtsam.3_robot_localization:main',
            'PoseSLAM = py_demo_gtsam.4_PoseSLAM:main',
            'Landmark_SLAM = py_demo_gtsam.5_Landmark_SLAM:main',
        ],
    },
)
