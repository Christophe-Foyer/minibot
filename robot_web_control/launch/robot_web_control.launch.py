from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/Vzense/color/image_raw',
            description='Input image topic to subscribe to'
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='5000',
            description='Web interface port'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Command velocity topic name'
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='1.0',
            description='Maximum linear speed in m/s'
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='2.0',
            description='Maximum angular speed in rad/s'
        ),
        DeclareLaunchArgument(
            'flip_image',
            default_value=True,
            description='Flip the camera image'
        ),

        Node(
            package='robot_web_control',
            executable='robot_web_control',
            name='robot_web_control_node',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'web_port': LaunchConfiguration('web_port'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            }],
            output='screen',
        )
    ])