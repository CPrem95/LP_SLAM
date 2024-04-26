from setuptools import find_packages, setup

package_name = 'lp_slam'

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
            'lp_node = lp_slam.lp_node:main',
            # 'kdf_node = ekf_slam.kdf:main',
            # 'gs_ekf_node = ekf_slam.ekf_gs_node:main'
        ],
    },
)
