import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch.conditions  # Added for IfCondition


def generate_launch_description():
    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution(
            [FindPackageShare("delivery_simulation"), "worlds", "simple_world.world"]
        ),
        description="Full path to the Gazebo world file to load",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("delivery_navigation"), "config", "nav2_params.yaml"]
        ),
        description="Full path to the ROS2 parameters file for Nav2",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz",
                # Consider creating a custom RViz config for SLAM later
                # FindPackageShare('delivery_navigation'), 'rviz', 'slam_view.rviz'
            ]
        ),
        description="Full path to the RViz configuration file",
    )

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz", default_value="true", description="Whether to start RViz"
    )

    start_teleop_arg = DeclareLaunchArgument(
        "start_teleop",
        default_value="true",
        description="Whether to start teleop_twist_keyboard for manual control",
    )

    # --- Get Launch Configurations ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    start_rviz = LaunchConfiguration("start_rviz")
    start_teleop = LaunchConfiguration("start_teleop")

    # --- Include Gazebo Simulation Launch ---
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("delivery_simulation"),
                    "launch",
                    "spawn_robot_in_gazebo.launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time, "world": world}.items(),
    )

    # --- Include Nav2 Bringup Launch (Configured for SLAM via params_file) ---
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("delivery_navigation"),
                    "launch",
                    "bringup_nav2.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            # 'map': '' # Ensure map is empty for SLAM if bringup_nav2 needs it explicitly
        }.items(),
    )

    # --- RViz2 Node ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=launch.conditions.IfCondition(start_rviz),
    )

    # --- Teleop Twist Keyboard Node ---
    teleop_twist_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",  # Launch in a new xterm window
        condition=launch.conditions.IfCondition(start_teleop),
        parameters=[
            {"use_sim_time": use_sim_time}
        ],  # Though teleop usually doesn't care about sim time
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            world_arg,
            params_file_arg,
            autostart_arg,
            rviz_config_file_arg,
            start_rviz_arg,
            start_teleop_arg,
            gazebo_sim_launch,
            nav2_bringup_launch,
            rviz_node,
            teleop_twist_keyboard_node,
        ]
    )
