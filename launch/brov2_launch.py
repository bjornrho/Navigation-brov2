import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        #launch.actions.ExecuteProcess(
        #    cmd=['ros2', 'bag', 'record', '-a'],
        #    output='screen'
        #),
        Node(
            package='brov2_barometer',
            namespace='barometer',
            executable='barometer_publisher',
            name='sensors'
        ),
        Node(
            package='brov2_dvl',
            namespace='dvl',
            executable='DVLPublisher',
            name='sensors'
        ),
        Node(
            package='bno055_sensor',
            namespace='bno055',
            executable='bno055_sensor_node',
            name='sensors'
        )#,
        #Node(
        #    package='brov2_sonar',
        #    namespace='sonar',
        #    executable='SonarPublisher',
        #    name='sensors'
        #)
    ])