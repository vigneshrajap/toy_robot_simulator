import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class CommandClient(Node):
    def __init__(self):
        super().__init__('robot_command_client')
        self.pub = self.create_publisher(String, 'robot_command', 10)
        self.subscription = self.create_subscription(
            String, 'robot_report', self.report_callback, 10)

    def report_callback(self, msg):
        self.get_logger().info(f"Robot Report: {msg.data}")

    def send(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        time.sleep(0.2)

def main():
    rclpy.init()
    c = CommandClient()
    cmds = ["PLACE 1,2,EAST","MOVE","MOVE","LEFT","MOVE","REPORT"]
    for x in cmds: c.send(x)
    # keep the client running so it continues to receive `robot_report`
    try:
        rclpy.spin(c)
    except KeyboardInterrupt:
        pass
    finally:
        c.destroy_node()
        rclpy.shutdown()
