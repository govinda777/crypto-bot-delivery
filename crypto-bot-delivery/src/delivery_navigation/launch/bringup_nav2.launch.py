import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the share directory for the delivery_navigation package
    delivery_navigation_share_dir = get_package_share_directory("delivery_navigation")

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",  # Important for Gazebo
        description="Use simulation (Gazebo) clock if true",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            delivery_navigation_share_dir, "config", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    # --- Get Launch Configurations ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    # --- Nav2 Bringup ---
    # This is the preferred way if your Nav2 installation supports it well.
    # It encapsulates the launching of all Nav2 lifecycle nodes.
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "map": "",  # Path to map yaml file, if not using SLAM or map_server in params
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": "True",  # Recommended for performance
            # 'use_respawn': 'False', # Can be true if you want nodes to respawn
            # 'namespace': '', # Optional namespace
            # 'log_level': 'info', # Optional log level
        }.items(),
    )

    # --- Alternative: Manual Lifecycle Manager Nodes (If Nav2Bringup is problematic or for more control) ---
    # This section would be used if you are not using the nav2_bringup_launch Include.
    # For this POC, we'll primarily aim for nav2_bringup_launch, but keep this as a reference.
    #
    # lifecycle_nodes_localization = ['map_server', 'amcl'] # Or 'slam_toolbox'
    # lifecycle_nodes_navigation = [
    #     'controller_server',
    #     'planner_server',
    #     'behavior_server',
    #     'bt_navigator',
    #     # 'waypoint_follower',
    #     # 'velocity_smoother'
    # ]
    #
    # localization_manager_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_localization',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time},
    #                 {'autostart': autostart},
    #                 {'node_names': lifecycle_nodes_localization},
    #                 params_file] # Pass the full params_file here
    # )
    #
    # navigation_manager_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_navigation',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time},
    #                 {'autostart': autostart},
    #                 {'node_names': lifecycle_nodes_navigation},
    #                 params_file] # Pass the full params_file here
    # )

    return LaunchDescription(
        [
            use_sim_time_arg,
            params_file_arg,
            autostart_arg,
            nav2_bringup_launch,
            # If using manual nodes, uncomment these and comment out nav2_bringup_launch
            # localization_manager_node,
            # navigation_manager_node
        ]
    )
