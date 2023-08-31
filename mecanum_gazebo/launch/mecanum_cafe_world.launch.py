import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    robot_rviz = "mecanum_map.rviz"
    # Set the path to the SDF model files.
    # folder of pkg, files name
    pkg_mecanum_description = FindPackageShare('mecanum_description').find('mecanum_description')
    pkg_mecanum_gazebo = FindPackageShare('mecanum_gazebo').find('mecanum_gazebo')
    pkg_mecanum_slam = FindPackageShare('mecanum_slam').find('mecanum_slam')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    #pkg_cafe_gazebo = FindPackageShare('my_box_bot_gazebo')
    # Set path
    robot_description_path = os.path.join(pkg_mecanum_description,'launch','mecanum_description.launch.py')
    empty_world_launch_path = os.path.join(pkg_gazebo_ros,'launch','gazebo.launch.py')
    cafe_world_path = os.path.join(pkg_mecanum_gazebo, 'worlds', 'cafe.world')
    rviz_config_path = os.path.join(pkg_mecanum_slam, "rviz", robot_rviz)

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name='use_sim_time', 
        #     default_value='true',
        #     description='Use simulation time'
        # ),
         DeclareLaunchArgument(
          name = 'world',
          default_value= cafe_world_path,
          description='SDF world file'),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(empty_world_launch_path)
        # ),   
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
        #     output='screen'
        # ),
        # Start Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items()),
        # Start Gazebo client    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', LaunchConfiguration('use_sim_time')],
        #     output='screen'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time', default='true'),
                'publish_joints': 'false',
                'rviz': 'false',
            }.items()
        ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner.py",
        #     arguments=["diff_cont"],
        # ),
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
