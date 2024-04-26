from setuptools import find_packages, setup

package_name = 'uwb_slam'

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
    maintainer='arms',
    maintainer_email='arms@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_LM = uwb_slam.test_LM:main',
            'test_rot = uwb_slam.test_rotate:main',
            'init_ancs = uwb_slam.initialize_ancs:main',
            'run_SLAM = uwb_slam.all_uwb_SLAM:main'
        ],
    },
)
