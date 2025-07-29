# tf_switch_node

## Overview
The `tf_switch_node` is a ROS 2 package designed to manage and switch between different transformations in a robotic system. It provides a simple interface for subscribing to transformation data and publishing the necessary outputs.

## Features
- Initializes a ROS 2 node for handling transformations.
- Subscribes to relevant topics to receive transformation data.
- Publishes transformation data based on the logic defined in the node.

## Installation
To build and install the `tf_switch_node` package, follow these steps:

1. **Clone the repository:**
   ```bash
   git clone <repository_url>
   cd tf_switch_node
   ```

2. **Install dependencies:**
   Make sure you have all the necessary dependencies installed. You can use the following command:
   ```bash
   rosdep install -i --from-path src --rosdistro <your_ros_distro> -y
   ```

3. **Build the package:**
   Use the following command to build the package:
   ```bash
   colcon build --packages-select tf_switch_node
   ```

4. **Source the setup file:**
   After building, source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage
To run the `tf_switch_node`, use the following command:
```bash
ros2 run tf_switch_node tf_switch_node
```

## Contributing
Contributions are welcome! Please feel free to submit a pull request or open an issue for any suggestions or improvements.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.