import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
def generate_launch_description():
    # folder of pkg, files name
    pkg_mecanum_descrtiption = FindPackageShare('mecanum_description').find('mecanum_description')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    # Set path
    world_file_name = 'turtlebot3_houses/' + TURTLEBOT3_MODEL + '.model'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)
    robot_descriptuon_path = os.path.join(pkg_mecanum_descrtiption,'launch','mecanum_description.launch.py')
    empty_world_launch_path = os.path.join(pkg_gazebo_ros,'launch','gazebo.launch.py')


    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            name='headless',
            default_value='False',
            description='Whether to execute gzclient'),

    
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "mecanum_robot"]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(robot_descriptuon_path),
        #     launch_arguments={
        #         'use_sim_time': LaunchConfiguration('use_sim_time'),
        #         'publish_joints': 'true',
        #     }.items()
        # ),

        
    ])
