# Delivery Navigation Package (`delivery_navigation`)

This package is responsible for the autonomous navigation capabilities of the Crypto-Bot-Delivery robot using the ROS2 Navigation Stack (Nav2).

## Overview

The `delivery_navigation` package includes:
*   Configuration files for Nav2 (including parameters for costmaps, planners, controllers, behavior servers, SLAM Toolbox, and AMCL).
*   Launch files to bring up the Nav2 stack in different modes (e.g., for SLAM/mapping, and for navigation with a pre-existing map using AMCL).
*   (Future) Custom Python or C++ nodes for specific navigation tasks or behaviors if needed.

## Key Functionalities

*   **Mapping:** Provides configurations and launch files to use SLAM Toolbox for creating maps of unknown environments.
*   **Localization:** Configures AMCL (Adaptive Monte Carlo Localization) to enable the robot to determine its position within a known map.
*   **Path Planning:** Utilizes Nav2 planners (e.g., NavFn, SmacPlanner) to compute optimal paths to navigation goals.
*   **Obstacle Avoidance:** Employs Nav2 controllers (e.g., DWB, TEB, RPP) and costmaps to navigate safely while avoiding obstacles detected by sensors.
*   **Behavior Management:** Leverages Nav2's behavior server and behavior trees for complex navigation logic and recovery behaviors.

## Navigation System Components (Conceptual Diagram)

The following diagram illustrates the high-level components and data flow within the Nav2 stack as configured for this package (actual nodes may vary based on specific launch/params):

```mermaid
graph TD
    subgraph ApplicationLayer [Application Layer]
        GoalSource[External Goal Source (e.g., API, RViz)]
    end

    subgraph Nav2Stack [ROS2 Nav2 Stack]
        BTNav[BT Navigator Server (bt_navigator)]
        PlannerServer[Planner Server (planner_server)]
        ControllerServer[Controller Server (controller_server)]
        BehaviorServer[Behavior Server (behavior_server)]
        SmootherServer[Velocity Smoother Server (velocity_smoother) - Optional]
        WaypointFollower[Waypoint Follower Server - Optional]
    end

    subgraph Localization [Localization System]
        SLAMToolbox[SLAM Toolbox (during mapping)]
        AMCL[AMCL (during navigation)]
        MapServer[Map Server (serves map for AMCL)]
    end

    subgraph Costmaps [Costmap Generation]
        GlobalCostmap[Global Costmap]
        LocalCostmap[Local Costmap]
    end

    subgraph RobotInterface [Robot Hardware/Simulation Interface]
        RobotSensors[Sensors (LiDAR /scan, Odometry /odom)]
        RobotActuators[Actuators (/cmd_vel)]
    end

    GoalSource -- NavGoal (Action) --> BTNav
    BTNav -- ComputePathToPose (Action) --> PlannerServer
    BTNav -- FollowPath (Action) --> ControllerServer
    BTNav -- Recovery Actions --> BehaviorServer
    
    PlannerServer -- GlobalPlan --> GlobalCostmap
    PlannerServer -- Path --> BTNav
    
    ControllerServer -- ControlCmd (/cmd_vel) --> RobotActuators
    ControllerServer -- LocalPlan --> LocalCostmap
    
    BehaviorServer -- ControlCmd / Recovery Directives --> ControllerServer
    BehaviorServer -- Recovery Directives --> PlannerServer

    RobotSensors -- /scan, /odom, TF --> Localization
    RobotSensors -- /scan --> GlobalCostmap
    RobotSensors -- /scan --> LocalCostmap
    
    Localization -- /map, /tf (map to odom) --> GlobalCostmap
    Localization -- /map, /tf --> PlannerServer
    Localization -- /tf (odom to base_link) --> LocalCostmap
    Localization -- /tf --> ControllerServer
    Localization -- CurrentPose --> BTNav

    MapServer -- /map --> AMCL
    MapServer -- /map --> GlobalCostmap

    %% Styling (optional)
    classDef nav2core fill:#ddeeff,stroke:#333;
    classDef localization fill:#eebbaa,stroke:#333;
    classDef costmap fill:#ddefdd,stroke:#333;
    classDef robotif fill:#f0f0f0,stroke:#333;

    class BTNav,PlannerServer,ControllerServer,BehaviorServer,SmootherServer,WaypointFollower nav2core;
    class SLAMToolbox,AMCL,MapServer localization;
    class GlobalCostmap,LocalCostmap costmap;
    class RobotSensors,RobotActuators robotif;
```
This diagram provides a conceptual overview. The exact nodes and their interactions are defined in the Nav2 parameter files (e.g., `nav2_params.yaml`, `nav2_amcl_params.yaml`) and brought up by the launch files in the `launch/` directory.

## Usage

Refer to the launch files in the `launch/` directory and the POC documentation in `../../POC/poc_ros2_navigation/README.md` for instructions on running SLAM and navigation demonstrations.
```
