import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, SetRemap, Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dr_spaam_ros_path = get_package_share_directory('dr_spaam_ros')

    declare_dr_spaam_ros_file_cmd = DeclareLaunchArgument(
        'dr_spaam_ros_params_file',
        default_value=os.path.join(dr_spaam_ros_path,
                                'config', 'dr_spaam_ros.yaml'),
        description='Full path to the ROS2 parameters file to use for the dr_spaam_ros node')
    
    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/stretch1/scan')

    dr_spaam_node = LaunchDescription([
        Node(
        parameters=[
            LaunchConfiguration('dr_spaam_ros_params_file'),
            {"scan_topic": LaunchConfiguration("scan_topic")},
        ],
        package='dr_spaam_ros',
        executable='dr_spaam_ros',
        name='dr_spaam_ros',
        output='screen'
        )
    ])

    return LaunchDescription([
      declare_dr_spaam_ros_file_cmd,
      declare_scan_topic_cmd,
      dr_spaam_node,
    ])