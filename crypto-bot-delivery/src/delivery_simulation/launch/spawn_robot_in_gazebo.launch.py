import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

    # --- Get Launch Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # --- Paths ---
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_delivery_simulation = get_package_share_directory('delivery_simulation')

    # --- Gazebo ---
    # Start Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'pause': 'false'}.items()
        # 'pause': 'true' can be useful for debugging, allows stepping through simulation
    )

    # Start Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # --- Robot Model ---
    # Process the URDF file (xacro)
    robot_urdf_path = os.path.join(pkg_delivery_simulation, 'models', 'delivery_bot.urdf.xacro')
    robot_description = Command(['xacro ', robot_urdf_path])

    # Robot State Publisher
    # Publishes TF transforms for the robot based on joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
        # arguments=[robot_urdf_path] # Not needed if using robot_description parameter
    )

    # Spawn Robot Entity in Gazebo
    # Takes the processed URDF and spawns it in Gazebo.
    # The 'robot_description' topic is typically used by gazebo_ros_factory.
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description', # Use the robot_description topic
            '-entity', 'delivery_bot',     # Name of the entity in Gazebo
            '-x', '0.0',                   # Initial X pose
            '-y', '0.0',                   # Initial Y pose
            '-z', '0.1',                   # Initial Z pose (slightly above ground)
            '-Y', '0.0',                   # Initial Yaw
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Joint State Publisher (Optional, often not needed if Gazebo plugins publish joint states) ---
    # For a diff drive robot where wheel joints are controlled by a Gazebo plugin that also
    # publishes their states (as part of /odom or /tf), this might not be strictly necessary
    # for visualization if the diff drive plugin correctly updates TFs for wheels.
    # However, if you have other joints or want a GUI, uncomment this.
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     # condition=IfCondition(LaunchConfiguration('gui')) # If you add a 'gui' launch arg
    # )
    # joint_state_publisher_gui_node = Node(
    #    package='joint_state_publisher_gui',
    #    executable='joint_state_publisher_gui',
    #    name='joint_state_publisher_gui',
    #    parameters=[{'use_sim_time': use_sim_time}],
    #    condition=IfCondition(LaunchConfiguration('gui'))
    # )


    return LaunchDescription([
        use_sim_time_arg,
        world_arg,

        gzserver_cmd,
        gzclient_cmd,

        robot_state_publisher_node,
        spawn_entity_node,
        # joint_state_publisher_node, # Uncomment if needed
    ])
