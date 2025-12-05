import unittest
from toy_robot_simulator.robot_node import RobotController
from std_msgs.msg import String
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class TestRobotController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RobotController()

    def publish_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.node.command_callback(msg)

    def test_place(self):
        # Test valid place
        self.publish_cmd("PLACE 1,2,NORTH")
        self.assertEqual((self.node.robot.x, self.node.robot.y, self.node.robot.f), (1,2,"NORTH"))
        # Test invalid place
        self.publish_cmd("PLACE 6,2,EAST")
        self.assertEqual((self.node.robot.x, self.node.robot.y, self.node.robot.f), (1,2,"NORTH")) 

    def test_move_north(self):
        # Test moving north
        self.publish_cmd("PLACE 0,0,NORTH")
        self.publish_cmd("MOVE")
        self.assertEqual((self.node.robot.x, self.node.robot.y), (0,1))

    def test_move_blocked(self):
        self.publish_cmd("PLACE 0,4,NORTH")
        self.publish_cmd("MOVE")
        self.assertEqual((self.node.robot.x, self.node.robot.y), (0,4))

    def test_rotate(self):
        self.publish_cmd("PLACE 0,0,NORTH")
        self.publish_cmd("LEFT")
        self.assertEqual(self.node.robot.f, "WEST")
        self.publish_cmd("PLACE 0,0,NORTH")
        self.publish_cmd("RIGHT")
        self.assertEqual(self.node.robot.f, "EAST")

    def test_report_output(self):
        self.publish_cmd("PLACE 2,3,EAST")
        # Capture published data
        last_report = []
        def listener(msg):
            last_report.append(msg.data)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sub = self.node.create_subscription(String, 'robot_report', listener, qos)

        # publish REPORT and spin the node so the subscription callback runs
        self.publish_cmd("REPORT")
        timeout = 1.0
        start = time.time()
        while not last_report and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertIn("2,3,EAST", last_report)