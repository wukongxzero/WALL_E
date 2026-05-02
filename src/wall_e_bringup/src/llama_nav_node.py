#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

WAYPOINTS = {
    "a":    (1.0,  0.0),
    "b":    (2.0,  1.0),
    "c":    (0.0,  2.0),
    "home": (0.0,  0.0),
    "door": (3.0,  0.0),
}

class LlamaNavNode(Node):
    def __init__(self):
        super().__init__('llama_nav_node')
        self.goal_pub_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_sub_ = self.create_subscription(
            String, '/nav_command', self.cmd_callback, 10)
        self.get_logger().info(f"Nav node ready. Waypoints: {list(WAYPOINTS.keys())}")

    def cmd_callback(self, msg):
        self.parse_command(msg.data.strip().lower())

    def parse_command(self, cmd):
        if cmd == 'home':
            self.send_goal(0.0, 0.0)
        elif cmd == 'stop':
            self.get_logger().info("Stop requested")
            self.send_goal(0.0, 0.0, cancel=True)
        elif cmd.startswith('go '):
            name = cmd[3:].strip()
            if name in WAYPOINTS:
                x, y = WAYPOINTS[name]
                self.send_goal(x, y)
            else:
                self.get_logger().warn(f"Unknown waypoint: {name}")
        elif cmd.startswith('forward '):
            try:
                dist = float(cmd[8:].strip())
                self.send_goal(dist, 0.0)
            except ValueError:
                self.get_logger().warn(f"Bad distance: {cmd}")
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    def send_goal(self, x, y, cancel=False):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        if not cancel:
            self.get_logger().info(f"Navigating to ({x:.2f}, {y:.2f})")
        self.goal_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LlamaNavNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()