# tf_switch_node

[![GitHub](https://img.shields.io/badge/GitHub-iDifferent--studio%2Ftf__switch__node-blue)](https://github.com/iDifferent-studio/tf_switch_node)

**Language:** English | [日本語](README_ja.md)

## Overview
The `tf_switch_node` is a ROS 2 package designed to switch the source of `map->odom` transformation between EKF (Extended Kalman Filter) and AMCL (Adaptive Monte Carlo Localization) using a state machine approach. The node monitors AMCL pose covariance and GPS fix status to determine which localization method should publish the `map->odom` transform through well-defined state transitions, ensuring reliable localization performance in different environments.

## Features
- **State Machine-based Transform Switching**: Uses a deterministic state machine to decide whether EKF or AMCL should publish the `map->odom` transformation
- **AMCL Pose Monitoring**: Continuously monitors AMCL pose covariance to assess localization quality
- **GPS Status Integration**: Evaluates GPS fix status from the `/fix` topic to determine GPS reliability
- **Dual Localization Support**: Seamlessly switches between EKF-based and AMCL-based localization methods
- **Deterministic Decision Making**: Makes switching decisions based on predefined state machine logic and sensor data thresholds
- **ROS 2 Native**: Built specifically for ROS 2 with modern C++ practices

## Installation
To build and install the `tf_switch_node` package, follow these steps:

1. **Create a ROS 2 workspace:**
   ```bash
   mkdir -p ~/tf_switch_ws/src
   cd ~/tf_switch_ws/src
   ```

2. **Clone the repository:**
   ```bash
   git clone https://github.com/iDifferent-studio/tf_switch_node.git
   cd ~/tf_switch_ws
   ```

3. **Install dependencies:**
   Make sure you have all the necessary dependencies installed. You can use the following command:
   ```bash
   rosdep install -i --from-path src --rosdistro humble -y
   ```

4. **Build the package:**
   Use the following command to build the package:
   ```bash
   colcon build --packages-select tf_switch_node
   ```

5. **Source the setup file:**
   After building, source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage
To run the `tf_switch_node`, use the following command:
```bash
ros2 run tf_switch_node tf_switch_node
```

## Parameters
The node can be configured with the following parameters:

### Frame Configuration
- **`map_frame_id`** (string, default: "map")
  - Frame ID for the map coordinate frame

- **`odom_frame_id`** (string, default: "odom")
  - Frame ID for the odometry coordinate frame

- **`base_link_frame_id`** (string, default: "base_link")
  - Frame ID for the robot base coordinate frame

### Node Configuration
- **`amcl_node_name`** (string, default: "amcl")
  - Name of the AMCL node for parameter control

- **`publish_tf_param_name`** (string, default: "tf_broadcast")
  - Parameter name to control AMCL's transform broadcasting

### State Machine Parameters
- **`covariance_threshold`** (double, default: 1.5)
  - Threshold for AMCL pose covariance to determine localization quality
  - Higher values indicate poor localization quality
  - When exceeded, may trigger state transitions

### Topic Subscriptions
The node subscribes to the following topics:
- **`/fix`** - GPS NavSatFix messages for GNSS status
- **`/amcl_pose`** - AMCL pose with covariance for localization quality assessment
- **`/odometry/global`** - Global odometry from EKF for map->odom transform calculation

### Topic Publications
The node publishes to the following topics:
- **`/initialpose`** - Initial pose for AMCL when transitioning from EKF to AMCL
- **`/tf`** - Transform from map to odom frame when in EKF mode

### State Machine Logic
The node operates with three states:
- **DEAD_RECKON**: Neither GPS nor AMCL are reliable
- **RTK_BAD**: GPS is not available but AMCL localization is valid
- **RTK_GOOD**: GPS RTK fix is available and reliable

Example parameter file (`tf_switch_params.yaml`):
```yaml
tf_switch_node:
  ros__parameters:
    map_frame_id: "map"
    odom_frame_id: "odom"
    base_link_frame_id: "base_link"
    amcl_node_name: "amcl"
    publish_tf_param_name: "tf_broadcast"
    covariance_threshold: 1.5
```

To run with custom parameters:
```bash
ros2 run tf_switch_node tf_switch_node --ros-args --params-file tf_switch_params.yaml
```

## Contributing
Contributions are welcome! Please feel free to submit a pull request or open an issue for any suggestions or improvements.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

For more information, visit the [GitHub repository](https://github.com/iDifferent-studio/tf_switch_node).

## License
This project is licensed under the Apache-2.0 License. See the LICENSE file for more details.

## Dependencies
This package depends on the following ROS 2 packages:
- `rclcpp` - ROS 2 C++ client library
- `tf2_ros` - TF2 ROS bindings
- `tf2_geometry_msgs` - TF2 geometry message conversions
- `geometry_msgs` - Geometry message definitions
- `sensor_msgs` - Sensor message definitions
- `nav_msgs` - Navigation message definitions

## Version
Current version: 0.1.0