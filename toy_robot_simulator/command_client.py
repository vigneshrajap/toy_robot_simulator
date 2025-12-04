import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class CommandClient(Node):
    def __init__(self):
        super().__init__('robot_command_client')
        self.pub = self.create_publisher(String, 'robot_command', 10)

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
    time.sleep(0.5)
    rclpy.shutdown()
