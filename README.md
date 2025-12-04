(The file `/home/viki/ros2_ws/src/toy_robot_simulator/README.md` exists, but is empty)
# Toy Robot Simulator

Lightweight ROS 2 package that implements a simple toy robot controller node and a small command client for driving the robot. Intended for learning ROS 2 pub/sub basics and for small unit/integration tests.

**Contents**
- `toy_robot_simulator/robot_node.py`: `RobotController` node, subscribes to `robot_command` and publishes `robot_report`.
- `toy_robot_simulator/command_client.py`: example client that sends commands to the robot and can receive reports.
- `launch/robot_launch.py`: launch file for starting the robot node.
- `tests/test_robot_controller.py`: unit tests for the controller logic.

**Requirements**
- ROS 2 (tested with a recent distribution)
- Python 3.8+

**Install / Build**
1. From your ROS 2 workspace root (where `src/` lives):

```bash
colcon build
source install/setup.bash
```

**Run (manual)**
- Launch the robot node in one terminal:

```bash
ros2 launch toy_robot_simulator robot_launch.py
```

- In another terminal run the command client to send a sample command sequence:

```bash
ros2 run toy_robot_simulator command_client
```

You should see the client log a `Robot Report: X,Y,F` message after the `REPORT` command.

**Run tests**

```bash
colcon test --packages-select toy_robot_simulator
colcon test-result --verbose
```

The tests exercise the controller logic and verify that `robot_report` messages are published. Tests use an in-process `rclpy` node and spin briefly to allow subscription callbacks to process.

**Notes**
- The package uses `rclpy` publishers/subscriptions; subscription callbacks run only while the node is being spun (`rclpy.spin` / `rclpy.spin_once`).
- Tests set a reliable QoS for the `robot_report` subscription to make delivery deterministic in CI.

If you'd like a longer-lived example client, automatic test helpers, or additional examples (e.g., interactive CLI), tell me which you'd prefer and I can add them.
