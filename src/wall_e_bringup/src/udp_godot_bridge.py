#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import socket
import struct

GODOT_IP   = "127.0.0.1"
GODOT_PORT = 8881

class UdpGodotBridge(Node):
    def __init__(self):
        super().__init__('udp_godot_bridge')

        self.sock_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.pitch_       = 0.0
        self.roll_        = 0.0
        self.drive_left_  = 127
        self.drive_right_ = 127

        self.create_subscription(Float32, '/wall_e/pitch',
            lambda m: setattr(self, 'pitch_', m.data), 10)
        self.create_subscription(Float32, '/wall_e/roll',
            lambda m: setattr(self, 'roll_', m.data), 10)
        self.create_subscription(Twist, '/cmd_vel',
            self.cmd_callback, 10)

        self.create_wall_timer(0.05, self.send_packet)
        self.get_logger().info(f"UDP Godot bridge → {GODOT_IP}:{GODOT_PORT}")

    def cmd_callback(self, msg):
        throttle = int(msg.linear.x  * 124)
        steering = int(msg.angular.z * 124)
        self.drive_right_ = 127 - max(-124, min(124, throttle))
        self.drive_left_  = 127 - max(-124, min(124, steering))

    def send_packet(self):
        euler_x = 0
        euler_y = int(self.pitch_)
        euler_z = int(self.roll_)
        buf = struct.pack('<BBhhh',
            self.drive_left_,
            self.drive_right_,
            euler_x,
            euler_y,
            euler_z
        )
        self.sock_.sendto(buf, (GODOT_IP, GODOT_PORT))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(UdpGodotBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
