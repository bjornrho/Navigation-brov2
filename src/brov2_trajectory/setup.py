from setuptools import setup

package_name = 'brov2_trajectory'

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
    maintainer='Bjoernar Hogstad',
    maintainer_email='bjornar-rh@hotmail.com',
    description='Trajectory publisher for predefined trajectories.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brov2_trajectory_exe = brov2_trajectory.trajectory_publisher_node_main:main',
        ],
    },
)
