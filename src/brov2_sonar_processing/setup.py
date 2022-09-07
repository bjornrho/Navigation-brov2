from setuptools import setup

package_name = 'brov2_sonar_processing'

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
    description='Sonar processing pipeline performing echo intensity correction, blind zone removal, slant range correction, and geometric correction.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brov2_sonar_processing_exe = brov2_sonar_processing.sonar_processing_node_main:main',
        ],
    },
)
