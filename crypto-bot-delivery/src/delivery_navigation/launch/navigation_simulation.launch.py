import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch.conditions # Ensure this import is present

def generate_launch_description():
    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_simulation'), 'worlds', 'simple_world.world'
        ]),
        description='Full path to the Gazebo world file to load'
    )

    # Argument for the map file
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_navigation'), 'maps', 'my_map.yaml' # Default map
            # User should replace 'my_map.yaml' with their actual saved map file
            # Or provide full path if it's outside the package.
            # For POC, assume 'my_map.yaml' exists in 'delivery_navigation/maps/'
        ]),
        description='Full path to the map file to load (e.g., my_map.yaml)'
    )

    # Parameter file specifically for AMCL navigation
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_navigation'), 'config', 'nav2_amcl_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file for Nav2 AMCL'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'
            # Consider creating a custom RViz config for navigation later
            # FindPackageShare('delivery_navigation'), 'rviz', 'navigation_view.rviz'
        ]),
        description='Full path to the RViz configuration file'
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    initial_pose_x_arg = DeclareLaunchArgument('initial_pose_x', default_value='0.0', description='Initial X pose for the robot')
    initial_pose_y_arg = DeclareLaunchArgument('initial_pose_y', default_value='0.0', description='Initial Y pose for the robot')
    initial_pose_yaw_arg = DeclareLaunchArgument('initial_pose_yaw', default_value='0.0', description='Initial Yaw pose for the robot')


    # --- Get Launch Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    start_rviz = LaunchConfiguration('start_rviz')
    initial_pose_x = LaunchConfiguration('initial_pose_x') # Not directly used in this version but declared
    initial_pose_y = LaunchConfiguration('initial_pose_y') # Not directly used in this version but declared
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw') # Not directly used in this version but declared


    # --- Include Gazebo Simulation Launch ---
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('delivery_simulation'), 'launch', 'spawn_robot_in_gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            # Initial pose for spawning the robot in Gazebo can be passed here if spawn_robot_in_gazebo.launch.py supports it
            # For now, spawn_robot_in_gazebo.launch.py has a fixed spawn pose.
            # If that launch file is updated to accept initial_pose_x/y/yaw, they can be passed:
            # 'initial_pose_x': initial_pose_x,
            # 'initial_pose_y': initial_pose_y,
            # 'initial_pose_yaw': initial_pose_yaw,
        }.items()
    )

    # --- Include Nav2 Bringup Launch (Configured for AMCL via params_file) ---
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'map': map_file, 
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'True',
            # For AMCL, initial pose can be set via RViz2 "2D Pose Estimate"
            # or directly in amcl_params.yaml, or by publishing to /initialpose
            # The initial_pose_x/y/yaw arguments are not directly consumed by nav2_bringup_launch.py
            # in a standard way without modification or specific nodes to handle them.
        }.items()
    )

    # --- RViz2 Node ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=launch.conditions.IfCondition(start_rviz) # Corrected IfCondition usage
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        map_file_arg,
        params_file_arg,
        autostart_arg,
        rviz_config_file_arg,
        start_rviz_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,

        gazebo_sim_launch,
        nav2_bringup_launch,
        rviz_node,
    ])
