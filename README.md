# ROS 2 Action Server/Client Example

[![ROS 2](https://img.shields.io/badge/ROS-2-Jade.svg)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

A ROS 2 package demonstrating an action server-client implementation with dynamic goal cancellation. The client subscribes to a topic and sends new goals while aborting previous ones when new messages arrive at high frequency.

## Features

- 🚀 Action server with 5-second execution timer
- ⚡ Client with topic-triggered goal updates
- 🔄 Automatic cancellation of pending goals
- 🛠️ Thread-safe goal handling
- 📦 C++ implementation for optimal performance

## Prerequisites

- ROS 2 Jazzy (or current LTS distro)
- C++17 compatible compiler
- colcon build system
- rclcpp, rclcpp_action packages

## Installation

1. Clone the repository:
```bash
git clone https://github.com/bzyfuzy/ros2_action_example.git
cd ros2_action_example
```

2. Build the package:
```bash
colcon build --symlink-install --packages-select ros2_action_example
source install/setup.bash
```

## Usage
Start Action Server
```bash
ros2 run ros2_action_example action_server
```

Start Action Client
```bash
ros2 run ros2_action_example action_client
```

Trigger Goals
```bash
ros2 topic pub /goal_topic std_msgs/msg/Empty -r 2
```
## Monitoring
View action status:
```bash
ros2 action list
ros2 action info /example_action
```
## Package Structure
```
ros2_action_example/
├── action/
│   └── Example.action       # Action interface definition
├── src/
│   ├── action_server.cpp    # Action server implementation
│   └── action_client.cpp    # Action client implementation
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Customization

### Modify Action Interface
Edit action/Example.action:
```.action
# Goal definition
int32 target
---
# Result definition
bool success
---
# Feedback definition
int32 progress
```

### Change Execution Time
Modify the server's execution duration in action_server.cpp:
```.cpp
// Change 5.0 to desired duration (seconds)
while ((this->now() - start_time).seconds() < 5.0) {
```

### Debugging

Check node outputs:
```bash
ros2 topic echo /rosout
```

List active nodes:
```bash
ros2 node list
```

Visualize system:
```bash
rqt_graph
```

### ROS Graph
```
                +----------------+
                | goal_publisher |
                +----------------+
                        |
                        v
                +----------------+
                | /goal_topic    |
                +----------------+
                        |
                        v
                +----------------+
                |  action_client |
                +----------------+
                    ⇅     ⇅
             +---------------------+
             | /example_action     |
             +---------------------+
                    ⇅     ⇅
                +---------------+
                | action_server |
                +---------------+
```

## License
Apache License 2.0 - See LICENSE for details.

## Contributing
Pull requests welcome! Please follow ROS 2 development guidelines.

---

**Maintainer**: BzY*FuZy <bzy.fuzy@gmail.com>  
**ROS 2 Documentation**: [Actions Tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)


