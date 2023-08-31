import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_rviz = "mecanum_description.rviz"
    # folder of pkg, files name
    pkg_mecanum_description = FindPackageShare('mecanum_description').find('mecanum_description')
    pkg_mecanum_gazebo = FindPackageShare('mecanum_gazebo').find('mecanum_gazebo')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    #pkg_cafe_gazebo = FindPackageShare('my_box_bot_gazebo')
    # Set path
    robot_description_path = os.path.join(pkg_mecanum_description,'launch','mecanum_description.launch.py')
    empty_world_launch_path = os.path.join(pkg_gazebo_ros,'launch','gazebo.launch.py')
    cafe_world_path = os.path.join(pkg_mecanum_gazebo, 'worlds', 'cafe.world')
    rviz_config_path = os.path.join(pkg_mecanum_description, "rviz", robot_rviz)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),
        # Start Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        ),
        # Start Gazebo client    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_joints': 'true',
                'rviz': 'false',
            }.items()
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "mecanum_robot"]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        )
        
    ])
