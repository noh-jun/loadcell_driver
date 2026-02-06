from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_filename = LaunchConfiguration('param_filename')

    param_file = PathJoinSubstitution([
        FindPackageShare('loadcell_driver'),
        'config',
        param_filename,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_filename',
            default_value='loadcell_driver_1.yaml',
            description='Parameter YAML file name under share/loadcell_driver/config/'
        ),

        Node(
            package='loadcell_driver',
            executable='loadcell_driver_node',
            name='loadcell_driver_node_1',
            output='screen',
            parameters=[param_file],
        )
    ])
