import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('finalproject')
    default_model_path = os.path.join(pkg_share, 'description/diff_drive_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/navigation.rviz')
    world_path=os.path.join(pkg_share, 'worlds/my_world.sdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
    )

    joy_node = Node(
    package='joy',
    executable='joy_node',
    name='joy_node',
    parameters=[{'dev': '/dev/input/js0'}]
    )

    teleop_twist_joy_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='teleop_twist_joy',
    parameters=[os.path.join(pkg_share, 'config', 'teleop.yaml')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    
 
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'bot', 
                   '-topic', 'robot_description', 
                   '-x', '-7.0', 
                   '-y', '-3.5',
                   '-z', '0.2',
                   '-Y', '1.57'
                  ],
        output='screen'
    )
    

    return LaunchDescription([
    DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
    DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
    DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
    ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
    joint_state_publisher_node,
    robot_state_publisher_node,
    spawn_entity,
    rviz_node,
    joy_node,
    teleop_twist_joy_node
])


    