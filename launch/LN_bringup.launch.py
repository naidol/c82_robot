# C82_ROBOT
# Author: LOGAN NAIDOO (DEC 2021)

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from pathlib import Path

#MY_NEO_ROBOT = os.environ['MY_ROBOT'] # Set the environment variable in bashrc
MY_NEO_ROBOT = 'c82realrobot'
MY_NEO_ENVIRONMENT = os.environ['MY_ROBOT_ENVIRONMENT'] # Set the environment variable in bashrc


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('c82_robot'), 'urdf/', MY_NEO_ROBOT+'.urdf')
    #ekf_config_path = os.path.join(get_package_share_directory('c82_robot'), 'configs/', 'ekf.yaml')
    ekf_config_path = PathJoinSubstitution([FindPackageShare("c82_robot"), "configs", "ekf.yaml"])
    rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'),'rviz','nav2_default_view.rviz')
    
    
    start_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf]
    )

    start_joint_state_pub = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdf]
    )

    rplidar = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    uros_teensy = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        #arguments=['serial', '--dev', LaunchConfiguration("base_serial_port")]
        arguments=['serial','--dev','/dev/ttyACM0']
    )

    robot_localisation = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path
        ],
        remappings=[("odometry/filtered", "odom")]
    )

    rviz =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            #parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
    )


    return LaunchDescription([  start_robot_state_pub,
                                start_joint_state_pub,
                                uros_teensy,
                                robot_localisation,
                                rplidar, 
                                rviz]
    )