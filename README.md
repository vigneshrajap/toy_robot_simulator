# Toy Robot Simulator

Lightweight ROS 2 package that implements a simple toy robot controller node and a small command client for driving the robot. Also includes a unit test and shell script to test the functionality following the constraints. 

**Contents**
- `toy_robot_simulator/robot_node.py`: `RobotController` node, subscribes to `robot_command` and publishes `robot_report`.
- `toy_robot_simulator/command_client.py`: example client that sends commands to the robot and can receive reports.
- `scripts/sample_test.sh`: shell script that quickly publishes to the `robot_command` topic.
- `launch/robot_launch.py`: launch file for starting the robot node.
- `tests/test_robot_controller.py`: unit tests for the controller logic.

**Requirements**
- ROS 2 (tested with a recent distribution)
- Python 3.8+

**Install / Build**
1. Make a ROS 2 workspace and build the repo locally:
```
mkdir -p ~/ros2_ws/src
git clone https://github.com/vigneshrajap/toy_robot_simulator.git
colcon build --symlink-install
source install/setup.bash
```
2. To test the "toy robot simulator" functionality via client node, do the following

**Run (manual)**

Launch the robot node in one terminal:

```bash
source install/setup.bash
ros2 launch toy_robot_simulator robot_launch.py
```

In another terminal run the command client to send a sample command sequence:

```bash
source install/setup.bash
ros2 run toy_robot_simulator command_client
```

You should see the client log a `Robot Report: X,Y,F` message after the `REPORT` command.

Another way to send commands directly to robot via command line will be using `ros2 topic pub`. To do that, run the shell script to send a sample command sequence:

```bash
~/ros2_ws/src/toy_robot_simulator/scripts/sample_test.sh
```

**Run tests**

```bash
colcon test --packages-select toy_robot_simulator
colcon test-result --verbose
```

The tests exercise the controller logic and verify that `robot_report` messages are published. Tests use an in-process `rclpy` node and spin briefly to allow subscription callbacks to process.

**Notes**
- The package uses `rclpy` publishers/subscriptions; subscription callbacks run only while the node is being spun (`rclpy.spin` / `rclpy.spin_once`).
- Tests set a reliable QoS for the `robot_report` subscription to make delivery deterministic in CI.