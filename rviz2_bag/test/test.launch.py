from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
import os.path

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),
        Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            arguments=[
                '--frame-id', 'base_link',
                '--child-frame-id', 'velodyne_top',
            ],
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.join(get_package_share_directory('rviz2_bag'), 'test', 'test.rviz'),
            ],
            ros_arguments=[
                '--log-level', LaunchConfiguration("log_level"),
            ],
            on_exit=Shutdown(),
        )
    ])
