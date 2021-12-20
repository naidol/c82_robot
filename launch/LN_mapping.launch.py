# C82_ROBOT
# Author: LOGAN NAIDOO (DEC 2021)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable 
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MY_NEO_ROBOT = os.environ['MY_ROBOT'] # Set the environment variable in bashrc
#MY_NEO_ROBOT = 'mpo_500'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    param_file_name = 'mapping.yaml'
    print(param_file_name)
    param_dir = LaunchConfiguration(
        'parameters',
        default=os.path.join(
            get_package_share_directory('c82_robot'),
            'configs/',
            param_file_name))
    print(param_dir)
    #slam_launch_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')
    print(param_dir)

    return LaunchDescription([
        DeclareLaunchArgument(
            'parameters',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='slam_toolbox', executable='sync_slam_toolbox_node', output='screen',
            name='slam_toolbox', parameters = [param_dir])
    ])
