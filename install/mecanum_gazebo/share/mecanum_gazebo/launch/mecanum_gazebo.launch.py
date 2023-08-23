import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True
    # joy_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    # )

    # world_path = PathJoinSubstitution(
    #     [FindPackageShare("mecanum_gazebo"), "worlds", "playground.world"]
    # )
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    world_path = os.path.join(pkg_share, world_file_path)
    empty_world_path = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "empty_wo.world"]
    )
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('mecanum_description'), 'launch', 'mecanum_description.launch.py']
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name='world', 
        #     default_value=world_path,
        #     description='Gazebo world'
        # ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "mecanum_robot"]
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'true',
            }.items()
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(joy_launch_path),
        # )
    ])
