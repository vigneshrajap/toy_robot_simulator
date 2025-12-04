import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DIRECTIONS = ["NORTH", "EAST", "SOUTH", "WEST"]

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.subscription = self.create_subscription(
            String, 'robot_command', self.command_callback, 10)

        # **Create report publisher immediately**
        self.report_pub = self.create_publisher(String, 'robot_report', 10)
        
        self.x = None
        self.y = None
        self.f = None
        self.table_size = 5

    def is_placed(self):
        return self.x is not None

    def valid_position(self, x, y):
        return 0 <= x < self.table_size and 0 <= y < self.table_size

    def place(self, x, y, f):
        if self.valid_position(x, y) and f in DIRECTIONS:
            self.x, self.y, self.f = x, y, f
        self.get_logger().info(f"PLACE: {self.x},{self.y},{self.f}")

    def move(self):
        if not self.is_placed(): return
        dx = dy = 0
        if self.f == "NORTH": dy = 1
        if self.f == "SOUTH": dy = -1
        if self.f == "EAST": dx = 1
        if self.f == "WEST": dx = -1
        nx, ny = self.x + dx, self.y + dy
        if self.valid_position(nx, ny):
            self.x, self.y = nx, ny
        self.get_logger().info(f"MOVE: {self.x},{self.y},{self.f}")

    def rotate(self, dir):
        if not self.is_placed(): return
        i = DIRECTIONS.index(self.f)
        if dir == "LEFT": self.f = DIRECTIONS[(i-1)%4]
        if dir == "RIGHT": self.f = DIRECTIONS[(i+1)%4]
        self.get_logger().info(f"ROTATE: {self.x},{self.y},{self.f}")

    def report(self):
        if self.is_placed():
            msg = String()
            msg.data = f"{self.x},{self.y},{self.f}"
            self.report_pub.publish(msg)
            self.get_logger().info(f"REPORT: {msg.data}")

    def command_callback(self, msg):
        cmd = msg.data.strip()
        self.get_logger().info(f"Command is: {cmd}")
        if cmd.startswith("PLACE"):
            _, args = cmd.split()
            x,y,f = args.split(",")
            self.place(int(x), int(y), f)
            return
        if not self.is_placed(): return
        if cmd=="MOVE": self.move()
        if cmd=="LEFT": self.rotate("LEFT")
        if cmd=="RIGHT": self.rotate("RIGHT")
        if cmd=="REPORT": self.report()

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()
