# NAOqi Navigation for ROS 2

This ROS 2 package acts as a bridge to the navigation and motion functionalities of SoftBank Robotics' Pepper and NAO robots. It exposes functionalities from NAOqi's `ALMotion` and `ALNavigation` modules as ROS 2 services, enabling control of the robot's movement and exploration capabilities within a ROS 2 environment.

## Features

*   **Relative Motion**: Command the robot to move to a position relative to its current location (`moveTo`).
*   **Map-based Navigation**: Command the robot to navigate to a specific coordinate in a pre-loaded map (`navigateTo`).
*   **Autonomous Exploration**: Start and stop the robot's autonomous exploration of its environment.

## Dependencies

*   `rclpy`
*   `std_srvs`
*   `naoqi_utilities_msgs`: Contains the custom message and service definitions for `MoveTo` and `Explore`.

## How to Run the Node

To start the node, you must provide the IP address and port of the robot.

```bash
ros2 run naoqi_navigation naoqi_navigation_node --ros-args -p ip:=<robot_ip> -p port:=<robot_port>
```

For example:
```bash
ros2 run naoqi_navigation naoqi_navigation_node --ros-args -p ip:=192.168.1.101 -p port:=9559
```

## ROS 2 API

All services are exposed under the node's namespace (`/naoqi_navigation_node/` by default).

### Services

*   **`~/move_to`** ([naoqi_utilities_msgs/srv/MoveTo](naoqi_utilities_msgs/srv/MoveTo.srv))  
    Moves the robot to a target position relative to its current pose using `ALMotion`. The target is defined by `(x, y, theta)`.

*   **`~/navigate_to`** ([naoqi_utilities_msgs/srv/MoveTo](naoqi_utilities_msgs/srv/MoveTo.srv))  
    Navigates the robot to a specific coordinate `(x, y)` in the map previously loaded by `ALNavigation`. The `theta` component is ignored.

*   **`~/start_exploring`** ([naoqi_utilities_msgs/srv/Explore](naoqi_utilities_msgs/srv/Explore.srv))  
    Starts the autonomous exploration of the environment within a given `radius` (in meters).

*   **`~/stop_exploring`** ([std_srvs/srv/Trigger](https://docs.ros2.org/foxy/api/std_srvs/srv/Trigger.html))  
    Stops the autonomous exploration currently in progress.

## Usage Example

To move the robot 0.5 meters forward:

```bash
ros2 service call /naoqi_navigation_node/move_to naoqi_utilities_msgs/srv/MoveTo "{x_coordinate: 0.5, y_coordinate: 0.0, theta_coordinate: 0.0}"
```

To make the robot navigate to the point (1.0, 2.0) on its map:

```bash
ros2 service call /naoqi_navigation_node/navigate_to naoqi_utilities_msgs/srv/MoveTo "{x_coordinate: 1.0, y_coordinate: 2.0}"
```

To start exploring a 3-meter radius area:

```bash
ros2 service call /naoqi_navigation_node/start_exploring naoqi_utilities_msgs/srv/Explore "{radius: 3.0}"
```

To stop exploration:

```bash
ros2 service call /naoqi_navigation_node/stop_exploring std_srvs/srv/Trigger "{}"
```