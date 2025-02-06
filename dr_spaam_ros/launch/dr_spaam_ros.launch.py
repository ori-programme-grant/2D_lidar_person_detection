# <!-- <launch>
#   <node pkg="dr_spaam_ros" type="node.py" name="dr_spaam_ros" output="screen">
#     <rosparam command="load" file="$(find dr_spaam_ros)/config/dr_spaam_ros.yaml"/>
#     <rosparam command="load" file="$(find dr_spaam_ros)/config/topics.yaml"/>
#   </node>
# </launch> -->
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

    # declare_dr_spaam_topics_file_cmd = DeclareLaunchArgument(
    #     'dr_spaam_topics_params_file',
    #     default_value=os.path.join(dr_spaam_ros_path,
    #                             'config', 'topics.yaml'),
    #     description='Full path to the ROS2 parameters file to use for the dr_spaam_ros node')

    dr_spaam_node = LaunchDescription([
        Node(
        parameters=[
            LaunchConfiguration('dr_spaam_ros_params_file'),
        ],
        package='dr_spaam_ros',
        executable='dr_spaam_ros',
        name='dr_spaam_ros',
        output='screen'
        )
    ])

    return LaunchDescription([
      declare_dr_spaam_ros_file_cmd,
      dr_spaam_node,
    ])