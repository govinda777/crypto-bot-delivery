# POC 2: ROS2 Navigation Stack - SLAM Demonstration

## 1. Overview

This Proof of Concept (POC) demonstrates the map building (SLAM - Simultaneous Localization and Mapping) capabilities of the ROS2 Navigation Stack (Nav2). It utilizes the `slam_toolbox` package for generating a map of an unknown environment simulated in Gazebo.

The key ROS2 packages involved in this demonstration are:
-   **`delivery_simulation`**: Contains the Gazebo world, the robot model (URDF), and the launch file to start the simulation environment.
-   **`delivery_navigation`**: Contains the Nav2 configuration (including SLAM Toolbox settings), and the main launch file to bring up the SLAM process along with the simulation.

## 2. Prerequisites

Before running this POC, ensure you have the following installed and configured:

*   **ROS2 Humble Hawksbill:**
    *   A full desktop installation is recommended: `sudo apt install ros-humble-desktop`
    *   Ensure Nav2 related packages are installed: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
    *   Gazebo (usually included with `ros-humble-desktop`): `sudo apt install ros-humble-gazebo-*`
    *   SLAM Toolbox: `sudo apt install ros-humble-slam-toolbox`
    *   RViz2 (included with `ros-humble-desktop`): `sudo apt install ros-humble-rviz2`
    *   Teleop Twist Keyboard: `sudo apt install ros-humble-teleop-twist-keyboard`
*   **Colcon Build Tools:**
    *   `sudo apt install python3-colcon-common-extensions`
*   **Xterm:**
    *   `sudo apt install xterm` (Used by `teleop_twist_keyboard` in the launch file to open in a new window).
*   **ROS2 Workspace:**
    *   Create a ROS2 workspace if you don't have one, e.g., `mkdir -p ~/ros2_ws/src`.
    *   Clone or copy the `crypto-bot-delivery` project into this `src` directory (e.g., `~/ros2_ws/src/crypto-bot-delivery`).

## 3. Build Instructions

1.  **Navigate to your ROS2 workspace root:**
    ```bash
    cd ~/ros2_ws 
    ```
    (Adjust the path if your workspace is located elsewhere).

2.  **Source ROS2 Humble setup file:**
    ```bash
    source /opt/ros/humble/setup.bash
    ```

3.  **Build the necessary packages:**
    It's recommended to build only the packages required for this POC to save time.
    ```bash
    colcon build --packages-select delivery_simulation delivery_navigation
    ```
    Alternatively, if these are the only packages in your workspace or you want to build everything:
    ```bash
    # colcon build
    ```

4.  **Source your local workspace's setup file:**
    After the build completes successfully, source the overlay workspace.
    ```bash
    source install/setup.bash
    ```
    **Important:** You'll need to do this in every new terminal you open to use the packages from this workspace.

## 4. Running the SLAM Simulation

1.  Ensure your ROS2 environment and local workspace are sourced as described above.
2.  Execute the main SLAM simulation launch file:
    ```bash
    ros2 launch delivery_navigation slam_simulation.launch.py
    ```

3.  **Expected Outcome:**
    *   A **Gazebo** window will open, loading the `simple_world.world` and spawning the `delivery_bot` model at the origin.
    *   An **RViz2** window will open, configured with a default Nav2 view. You should see:
        *   The robot model.
        *   Laser scan data from the robot's LiDAR.
        *   The map (`/map` topic) being built by SLAM Toolbox.
        *   TF transforms.
    *   A new **xterm terminal window** will pop up, running the `teleop_twist_keyboard` node.

## 5. Building the Map

1.  **Focus the xterm window** that is running `teleop_twist_keyboard`.
2.  Use the keyboard commands displayed in the xterm window to drive the robot around the Gazebo environment. Common commands:
    *   `i`: Move forward
    *   `,`: Move backward
    *   `j`: Turn left
    *   `l`: Turn right
    *   `k`: Stop
    *   (Refer to the xterm window for the full list of commands and speed controls).
3.  As you drive the robot, observe the RViz2 window. You will see the `/map` topic update as SLAM Toolbox processes the LiDAR scans and robot odometry to build a representation of the environment.
4.  Carefully explore all accessible areas of the `simple_world.world` to ensure a complete map is generated. Drive along walls and around obstacles.

## 6. Saving the Map

Once you are satisfied with the completeness and accuracy of the map built in RViz2:

1.  **Open a new terminal.**
2.  **Source your ROS2 Humble setup and your local workspace setup:**
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash 
    ```
    (Adjust `~/ros2_ws` if needed).

3.  **Use the `nav2_map_saver_cli` to save the map.** This tool subscribes to the `/map` topic and saves it to files.
    It's a good idea to create a dedicated directory for your maps, for example, within the `crypto-bot-delivery` project or your workspace.
    ```bash
    mkdir -p ~/ros2_ws/src/crypto-bot-delivery/maps 
    cd ~/ros2_ws/src/crypto-bot-delivery/maps
    ```
    Then run the map saver command:
    ```bash
    ros2 run nav2_map_saver_cli map_saver_cli -f my_slam_map --ros-args -p use_sim_time:=true
    ```
    *   `-f my_slam_map`: This specifies the output filename base. It will create `my_slam_map.yaml` (metadata) and `my_slam_map.pgm` (image data).
    *   `--ros-args -p use_sim_time:=true`: This is crucial because the simulation is running with `use_sim_time:=true`. The map saver needs to be aware of this to correctly receive the map.

4.  **Alternative: SLAM Toolbox Service Call (Optional)**
    SLAM Toolbox also provides a service to save the map directly. This can be useful for programmatic saving.
    ```bash
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_slam_map_service'}}"
    ```
    This will save the map files (e.g., `my_slam_map_service.yaml` and `my_slam_map_service.pgm`) in the directory where the `slam_toolbox` node is effectively running from (usually `~/.ros/` or the directory where you launched the main simulation if not specified otherwise by SLAM Toolbox). Using `map_saver_cli` often gives more direct control over the output location.

The map files (`.yaml` and `.pgm`) are now saved and can be used for localization in the next stage of the POC.

## Demonstrating Localization (AMCL) and Navigation

This part of the POC demonstrates using a previously saved map for localization with AMCL and navigating the robot to a specified goal.

**Prerequisites:**
*   A map must have been previously generated and saved (e.g., as `my_map.yaml` and `my_map.pgm`) as described in the "Saving the Map" section.
*   For this guide, we'll assume you saved your map as `my_map.yaml` and `my_map.pgm` and placed them into a new directory: `crypto-bot-delivery/src/delivery_navigation/maps/`. Create this `maps` directory if it doesn't exist and move your map files there.
    *   *(Note: The `navigation_simulation.launch.py` file defaults to looking for `my_map.yaml` in `install/delivery_navigation/share/delivery_navigation/maps/` after building. Ensure your `setup.py` for `delivery_navigation` is configured to install this `maps` directory if you place it within the package source. A simpler approach for quick testing is to provide an absolute path or a path relative to your workspace root when launching, e.g., `map:=/path/to/your/my_map.yaml`)*

**1. Update `setup.py` for `delivery_navigation` (If storing map in package):**

If you decide to store your map files inside `delivery_navigation/maps/` and want `colcon build` to install them, you need to update `crypto-bot-delivery/src/delivery_navigation/setup.py`.

Add the `maps` directory to the `data_files` list. For example:

```python
# (Inside setup.py of delivery_navigation)
# ... other imports ...
import os
from glob import glob

package_name = 'delivery_navigation'

setup(
    # ... other setup arguments ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')), # <--- Add this line
        # (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # If you add rviz files
    ],
    # ...
)
```
After adding this, rebuild the workspace: `colcon build --packages-select delivery_navigation && source install/setup.bash`.

**2. Running the Navigation Simulation:**

*   Ensure your ROS2 workspace is sourced: `source install/setup.bash` (from workspace root).
*   Command to launch:
    `ros2 launch delivery_navigation navigation_simulation.launch.py map:=<path_to_your_map_file.yaml>`
    *   Replace `<path_to_your_map_file.yaml>` with the actual path to your map file (e.g., `$(pwd)/my_map.yaml` if it's in your current directory, or the path from the `setup.py` installation like `$(ros2 pkg prefix delivery_navigation)/share/delivery_navigation/maps/my_map.yaml`).
    *   If you placed `my_map.yaml` in `crypto-bot-delivery/src/delivery_navigation/maps/` and updated `setup.py`, after building, you can use:
        `ros2 launch delivery_navigation navigation_simulation.launch.py map:=$(ros2 pkg prefix delivery_navigation)/share/delivery_navigation/maps/my_map.yaml`
    *   Or, if you used the default name `my_map.yaml` and placed it in the default path expected by the launch file (after installation), you might just run:
        `ros2 launch delivery_navigation navigation_simulation.launch.py`

*   **Expected Outcome:**
    *   Gazebo window opens with the `simple_world.world` and the `delivery_bot`.
    *   RViz2 window opens. The saved map should be displayed.
    *   The robot model should appear in RViz2. AMCL particles might be widely dispersed initially.

**3. Initializing Robot Pose (If Necessary):**

*   In RViz2, if the robot's estimated pose (shown by AMCL particles or the robot model on the map) is incorrect or too dispersed, you need to initialize it.
*   Use the "2D Pose Estimate" tool in the RViz2 toolbar.
*   Click and drag on the map at the robot's approximate current location in Gazebo, with the arrow pointing in its current orientation.
*   AMCL particles should converge around this pose. Repeat if necessary.

**4. Sending a Navigation Goal:**

*   In RViz2, use the "Nav2 Goal" tool from the toolbar.
*   Click on a valid location on the map and drag to set the desired orientation.
*   Release the mouse button to send the goal to Nav2.

**5. Observing Navigation:**

*   The Nav2 stack will plan a path (visible in RViz2).
*   The robot in Gazebo should start moving along the planned path, avoiding obstacles.
*   Monitor the robot's progress in both Gazebo and RViz2.
*   You should see TF updates, costmap updates, and the robot's actual movement.

This completes the demonstration of localization and navigation using a pre-built map.

## Testing the ROS2 Navigation Stack

Testing a complex system like the ROS2 Navigation Stack integrated with Gazebo involves multiple levels, from individual node unit tests (if applicable) to full simulation-based integration tests.

**Unit Testing:**

The current Python code within the `delivery_navigation` and `delivery_simulation` packages primarily consists of ROS2 launch files and configuration files (YAML, URDF, world files). These types of files declare and configure ROS nodes and simulation environments rather than containing complex, separable Python logic. As such, traditional Python unit tests for standalone functions or classes are not extensively applicable to the current codebase of these specific packages. Future development of custom Python nodes within these packages would warrant dedicated unit tests using frameworks like `pytest`.

**Integration and Behavior Testing (Conceptual BDD Scenarios):**

For higher-level behavior and integration testing, Behavior-Driven Development (BDD) can be used to define expected system behaviors. The following scenarios are defined conceptually in `src/delivery_navigation/tests/features/robot_navigation_poc.feature`. They outline key navigation capabilities:

```gherkin
Feature: Robot Navigation POC Scenarios
  As a system integrator,
  I want to define conceptual scenarios for robot navigation
  to outline expected behaviors in simulation.

  Background:
    Given the robot simulation is running with a known map
    And the Nav2 stack is active and localized

  Scenario: Robot navigates to a valid goal
    Given the robot is at an initial pose (e.g., 0,0,0)
    When a navigation goal (e.g., 5,5,0) is sent to Nav2
    Then Nav2 should plan a path to the goal
    And the robot should start moving towards the goal
    And the robot should eventually reach the goal location (or close proximity)
    And Nav2 should report success

  Scenario: Robot encounters an unexpected obstacle
    Given the robot is at an initial pose
    And there is an unexpected dynamic obstacle placed on its planned path
    When a navigation goal is sent that requires passing the obstacle's location
    Then Nav2 should initially plan a path
    And the robot should start moving
    And when the robot detects the unexpected obstacle
    Then Nav2 should attempt to re-plan or engage recovery behaviors
    And the robot should either find an alternative path or stop safely if no path is found

  Scenario: Invalid goal is sent
    Given the robot is at an initial pose
    When an invalid navigation goal (e.g., inside a known obstacle on the map) is sent
    Then Nav2 should fail to find a valid path
    And Nav2 should report failure or rejection of the goal
```

**Notes on Implementing ROS2/Gazebo BDD Tests:**

Implementing full automated step definitions for these BDD scenarios in a ROS2/Gazebo environment is an advanced task that typically requires:
*   **ROS2 Testing Frameworks:** Utilizing tools like `launch_testing` or `rostest` (from ROS1, with ROS2 equivalents) to manage the lifecycle of nodes and simulations within tests.
*   **Test-Specific Nodes:** Creating Python or C++ nodes that can interact with the simulation (e.g., publish goals, subscribe to robot pose, check Nav2 status, spawn/move obstacles in Gazebo).
*   **Gazebo Test Tools:** Using Gazebo services or plugins to control aspects of the simulation, query world state, or inject events.
*   **Mocking and Simulation Control:** Carefully managing the simulation state and potentially mocking parts of the Nav2 stack or sensor data for specific test conditions.

While full implementation of these step definitions is beyond the scope of this initial POC's automated generation, these Gherkin scenarios provide a valuable blueprint for future testing efforts. The manual testing steps outlined in the previous sections for SLAM and AMCL navigation serve as a practical way to verify these behaviors for this POC.

## Navigation Processes Flow (Activity Diagram)

The following diagram outlines the two main operational flows for the ROS2 navigation stack as demonstrated in this POC: map building (SLAM) and autonomous navigation using a pre-built map (AMCL).

```mermaid
graph TD
    A[Start] --> B{Choose Mode};

    subgraph SLAM (Map Building) Process
        B -- SLAM Mode --> C[Launch SLAM Simulation (slam_simulation.launch.py)];
        C --> D[Gazebo: Robot in Unknown World];
        D --> E[RViz: Visualize Robot & Sensor Data];
        E --> F[Teleoperate Robot to Explore Environment];
        F --> G[SLAM Toolbox: Process Sensor Data & Odometry];
        G --> H[RViz: Live Map Update];
        H -- Exploration Complete? --> I{Satisfied with Map?};
        I -- Yes --> J[Save Map (nav2_map_saver_cli)];
        J --> K[Map Saved (.yaml & .pgm)];
        K --> EndSLAM[SLAM Process Complete];
        I -- No --> F;
    end

    subgraph AMCL (Autonomous Navigation) Process
        B -- Navigation Mode --> L[Ensure Map is Saved (e.g., my_map.yaml)];
        L --> M[Launch Navigation Simulation (navigation_simulation.launch.py with map argument)];
        M --> N[Gazebo: Robot in Known World (from map)];
        N --> O[RViz: Display Saved Map & Robot Model];
        O --> P[AMCL: Localize Robot on Map];
        P -- Initial Pose Correct? --> Q{Pose Accurate?};
        Q -- No --> R[RViz: Set Initial Pose ("2D Pose Estimate")];
        R --> P;
        Q -- Yes --> S[RViz: Send Navigation Goal ("Nav2 Goal")];
        S --> T[Nav2: Plan Path to Goal];
        T --> U[Robot: Autonomously Follows Path];
        U --> V[Nav2: Monitor Progress & Avoid Obstacles];
        V -- Goal Reached? --> W{Goal Achieved?};
        W -- Yes --> X[Nav2: Report Success];
        X --> EndNav[Navigation Process Complete];
        W -- No / Stuck --> Y[Nav2: Recovery Behaviors / Re-planning];
        Y -- Path Found? --> U;
        Y -- No Path / Failure --> Z[Nav2: Report Failure];
        Z --> EndNav;
    end
    
```

## 7. Next Steps (Preview)

The map (`my_slam_map.yaml` and `my_slam_map.pgm`) created in this SLAM demonstration will be used in the next phase of the ROS2 Navigation POC. This next phase will focus on:
*   **Localization:** Using AMCL (Adaptive Monte Carlo Localization) to enable the robot to determine its position within the previously saved map.
*   **Path Planning and Navigation:** Sending navigation goals to Nav2 to make the robot autonomously navigate the known map while avoiding obstacles.
