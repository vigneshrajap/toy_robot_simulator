import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dataclasses import dataclass

DIRECTIONS = ["NORTH", "EAST", "SOUTH", "WEST"]


@dataclass
class RobotState:
    x: int | None = None
    y: int | None = None
    f: str | None = None
    table_size: int = 5

    def is_placed(self) -> bool:
        return self.x is not None

    def valid_position(self, x: int, y: int) -> bool:
        return 0 <= x < self.table_size and 0 <= y < self.table_size

    def place(self, x: int, y: int, f: str) -> bool:
        if self.valid_position(x, y) and f in DIRECTIONS:
            self.x, self.y, self.f = x, y, f
            return True
        return False

    def next_coords(self) -> tuple[int, int] | None:
        if not self.is_placed():
            return None
        # Set increment based on facing direction
        mapping = {"NORTH": (0, 1), "SOUTH": (0, -1), "EAST": (1, 0), "WEST": (-1, 0)}
        dx, dy = mapping.get(self.f, (0, 0))
        return self.x + dx, self.y + dy

    def move(self) -> bool:
        nxt = self.next_coords()
        if nxt is None:
            return False
        nx, ny = nxt
        if self.valid_position(nx, ny):
            self.x, self.y = nx, ny
            return True
        return False

    def rotate(self, dir: str) -> bool:
        if not self.is_placed():
            return False
        i = DIRECTIONS.index(self.f)
        # Rotate left or right based on current direction
        if dir == "LEFT":
            self.f = DIRECTIONS[(i - 1) % 4]
        elif dir == "RIGHT":
            self.f = DIRECTIONS[(i + 1) % 4]
        else:
            return False
        return True

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.subscription = self.create_subscription(
            String, "robot_command", self.command_callback, 10
        )

        # Create report publisher immediately
        self.report_pub = self.create_publisher(String, "robot_report", 10)

        # robot state (struct-like)
        self.robot = RobotState()

    def place(self, x: int, y: int, f: str):
        ok = self.robot.place(x, y, f)
        if ok:
            self.get_logger().info(f"PLACE: {self.robot.x},{self.robot.y},{self.robot.f}")
        else:
            self.get_logger().warn(f"Invalid Place Position: {x},{y},{f}, Robot not placed.")

    def move(self):
        if not self.robot.is_placed():
            return
        ok = self.robot.move()
        if ok:
            self.get_logger().info(f"MOVE: {self.robot.x},{self.robot.y},{self.robot.f}")
        else:
            self.get_logger().warn(f"MOVE BLOCKED: {self.robot.x},{self.robot.y},{self.robot.f}, cannot move further in direction {self.robot.f}")

    def rotate(self, dir: str):
        if not self.robot.is_placed():
            return
        ok = self.robot.rotate(dir)
        if ok:
            self.get_logger().info(f"ROTATE: {self.robot.x},{self.robot.y},{self.robot.f}")

    def report(self):
        if self.robot.is_placed():
            msg = String()
            msg.data = f"{self.robot.x},{self.robot.y},{self.robot.f}"
            self.report_pub.publish(msg)
            self.get_logger().info(f"REPORT: {msg.data}")

    def command_callback(self, msg: String):
        cmd = msg.data.strip()
        if cmd.startswith("PLACE"):
            try:
                _, args = cmd.split()
                x, y, f = args.split(",")
                self.place(int(x), int(y), f)
            except Exception:
                self.get_logger().warn(f"Invalid PLACE command: {cmd}")
            return
        if not self.robot.is_placed():
            self.get_logger().warn(f"Robot has to be PLACED before {cmd}")
            return
        if cmd == "MOVE":
            self.move()
        elif cmd == "LEFT" or cmd == "RIGHT":
            self.rotate(cmd)
        elif cmd == "REPORT":
            self.report()

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
