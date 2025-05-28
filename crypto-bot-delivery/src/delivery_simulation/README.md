# Delivery Simulation Package (`delivery_simulation`)

This package provides the Gazebo simulation environment for the Crypto-Bot-Delivery robot.

## Overview

The `delivery_simulation` package includes:
*   **Robot Model (URDF/Xacro):** The `delivery_bot.urdf.xacro` file defining the robot's physical structure, sensors (LiDAR), and actuators (differential drive). It includes Gazebo plugin configurations.
*   **Gazebo World Files:** Files like `simple_world.world` that define the environment in which the robot operates, including ground, obstacles, and lighting.
*   **Launch Files:** ROS2 launch files (e.g., `spawn_robot_in_gazebo.launch.py`) to start the Gazebo simulation environment and spawn the robot model into it.

## Key Functionalities

*   **Robot Simulation:** Simulates the `delivery_bot` in a 3D environment.
*   **Sensor Simulation:** Simulates sensor data, particularly:
    *   LaserScan data from the LiDAR sensor (published on `/scan`).
    *   Odometry data from the differential drive plugin (published on `/odom` and as TF transforms).
*   **World Environment:** Provides static environments with obstacles for testing navigation and perception.
*   **ROS2 Integration:** Interfaces with ROS2 through Gazebo plugins, allowing ROS2 nodes (like Nav2) to interact with the simulated robot and environment.

## Simulation Components (Conceptual Diagram)

The following diagram illustrates the key components of the Gazebo simulation setup:

```mermaid
graph TD
    subgraph ROS2Ecosystem [ROS2 Ecosystem]
        RobotStatePublisher[robot_state_publisher]
        Nav2Stack[Nav2 Stack & Other ROS2 Nodes]
        Rviz[RViz2]
    end

    subgraph GazeboSimulation [Gazebo Simulation Environment]
        GazeboServer[Gazebo Server (gzserver)]
        
        subgraph SimulatedWorld [Simulated World]
            WorldFile[simple_world.world (SDF)]
            GroundPlane[Ground Plane]
            StaticObstacles[Static Obstacles]
        end

        subgraph SimulatedRobot [Simulated Robot: delivery_bot]
            URDF[delivery_bot.urdf.xacro]
            BaseLink[base_link (Chassis)]
            Wheels[Wheels (Left/Right)]
            LidarLink[lidar_link (Sensor)]
            
            subgraph GazeboPlugins [Gazebo Plugins in URDF]
                DiffDrivePlugin[DiffDrive (libgazebo_ros_diff_drive)]
                LidarPlugin[LiDAR (libgazebo_ros_ray_sensor)]
            end
        end
    end

    URDF -- Parsed by xacro --> ProcessedURDFDescription
    ProcessedURDFDescription -- Loaded by --> RobotStatePublisher
    ProcessedURDFDescription -- Spawned by gazebo_ros_factory --> GazeboServer

    GazeboServer -- Loads --> WorldFile
    WorldFile --> GroundPlane
    WorldFile --> StaticObstacles
    
    GazeboServer -- Simulates --> BaseLink
    BaseLink --> Wheels
    BaseLink --> LidarLink
    
    DiffDrivePlugin -- Publishes --> OdometryTopic[/odom]
    DiffDrivePlugin -- Publishes --> OdomTF[TF (odom -> base_link)]
    DiffDrivePlugin -- Subscribes --> CmdVelTopic[/cmd_vel]
    
    LidarPlugin -- Publishes --> LaserScanTopic[/scan]
    LidarPlugin -- Associated with --> LidarLink

    OdometryTopic --> Nav2Stack
    LaserScanTopic --> Nav2Stack
    OdomTF --> Nav2Stack
    CmdVelTopic <-- Nav2Stack

    RobotStatePublisher -- Publishes --> RobotTF[TF (base_link -> other links)]
    RobotTF --> Nav2Stack
    RobotTF --> Rviz
    LaserScanTopic --> Rviz
    OdometryTopic --> Rviz
    
    %% Styling
    classDef rosnode fill:#ddeeff,stroke:#333;
    classDef gazebo fill:#fca,stroke:#333;
    classDef urdf fill:#ccffee,stroke:#333;
    classDef plugin fill:#ddccff,stroke:#333;

    class RobotStatePublisher,Nav2Stack,Rviz rosnode;
    class GazeboServer,WorldFile,GroundPlane,StaticObstacles gazebo;
    class URDF,BaseLink,Wheels,LidarLink urdf;
    class DiffDrivePlugin,LidarPlugin plugin;
```
This diagram shows how the URDF defines the robot and its plugins, how Gazebo loads the world and robot, and how data (like sensor topics and TF) flows between Gazebo plugins and the broader ROS2 ecosystem.

## Usage

Refer to the launch files in the `launch/` directory (e.g., `spawn_robot_in_gazebo.launch.py`) and the POC documentation in `../../POC/poc_ros2_navigation/README.md` for instructions on how to run the simulation.
```
