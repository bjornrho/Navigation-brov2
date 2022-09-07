from setuptools import setup

package_name = 'brov2_qekf'

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
    description='State estimation through Quaternion based Extended Kalman Filter.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brov2_qekf_exe = brov2_qekf.state_estimate_subscribe_and_publish_node_main:main',
        ],
    },
)
