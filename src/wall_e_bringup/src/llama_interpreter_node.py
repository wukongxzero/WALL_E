#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import json
import ollama

SYSTEM_PROMPT = """
You are a robot navigation and status assistant.
You will receive the current robot state as context before each command.
Respond with ONLY a JSON object — no explanation, no markdown.

Navigation commands:
"go to the door" → {"action": "navigate", "command": "go door"}
"go home" → {"action": "navigate", "command": "home"}
"move forward 2 meters" → {"action": "navigate", "command": "forward 2.0"}
"stop" → {"action": "navigate", "command": "stop"}

Dynamic object navigation (uses YOLO):
"go to the chair" → {"action": "find", "target": "chair"}
"go to the person" → {"action": "find", "target": "person"}
"navigate to the bottle" → {"action": "find", "target": "bottle"}
"follow the person" → {"action": "find", "target": "person"}

Mode commands:
"switch to manual" → {"action": "mode", "command": "manual"}
"go autonomous" → {"action": "mode", "command": "autonomous"}
"emergency stop" → {"action": "estop"}

Status queries:
"where is the robot" → {"action": "status", "response": "Robot is at x=<x>, y=<y>"}
"what mode" → {"action": "status", "response": "Currently in <mode> mode"}
"is the platform level" → {"action": "status", "response": "Pitch is <pitch> degrees, roll is <roll> degrees"}
"status report" → {"action": "status", "response": "<full summary>"}
"""

class LlamaInterpreterNode(Node):
    def __init__(self):
        super().__init__('llama_interpreter_node')

        self.cmd_pub_    = self.create_publisher(String, '/nav_command', 10)
        self.yolo_pub_   = self.create_publisher(String, '/yolo_target', 10)
        self.toggle_pub_ = self.create_publisher(String, '/state_toggle_cmd', 10)
        self.estop_pub_  = self.create_publisher(String, '/emergency_stop_cmd', 10)

        self.robot_x_     = 0.0
        self.robot_y_     = 0.0
        self.pitch_       = 0.0
        self.roll_        = 0.0
        self.mode_        = "MANUAL"
        self.cmd_linear_  = 0.0
        self.cmd_angular_ = 0.0

        self.create_subscription(Odometry, '/odom',
            self.odom_callback, 10)
        self.create_subscription(Float32, '/wall_e/pitch',
            lambda m: setattr(self, 'pitch_', m.data), 10)
        self.create_subscription(Float32, '/wall_e/roll',
            lambda m: setattr(self, 'roll_', m.data), 10)
        self.create_subscription(String, '/wall_e/state',
            lambda m: setattr(self, 'mode_', m.data), 10)
        self.create_subscription(Twist, '/cmd_vel',
            self.cmd_callback, 10)

        self.ollama_client_ = ollama.Client(host='http://localhost:11434')

        self.get_logger().info("LLaMA interpreter ready — type commands below")
        self.thread_ = threading.Thread(target=self.input_loop, daemon=True)
        self.thread_.start()

    def odom_callback(self, msg):
        self.robot_x_ = msg.pose.pose.position.x
        self.robot_y_ = msg.pose.pose.position.y

    def cmd_callback(self, msg):
        self.cmd_linear_  = msg.linear.x
        self.cmd_angular_ = msg.angular.z

    def build_context(self):
        return f"""
Current robot state:
- Position: x={self.robot_x_:.2f}m, y={self.robot_y_:.2f}m
- Mode: {self.mode_}
- Platform pitch: {self.pitch_:.1f} degrees
- Platform roll: {self.roll_:.1f} degrees
- Current velocity: linear={self.cmd_linear_:.2f} m/s, angular={self.cmd_angular_:.2f} rad/s
"""

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Say: ").strip()
                if not user_input:
                    continue
                self.process(user_input)
            except EOFError:
                break

    def process(self, text):
        try:
            context = self.build_context()

            response = self.ollama_client_.chat(
                model='llama3.2:3b',
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user',   'content': f"{context}\nUser command: {text}"}
                ]
            )

            raw = response['message']['content'].strip()
            self.get_logger().info(f"LLaMA: {raw}")

            parsed = json.loads(raw)
            action = parsed.get('action', '')

            if action == 'navigate':
                msg = String()
                msg.data = parsed.get('command', '')
                self.cmd_pub_.publish(msg)
                self.get_logger().info(f"Nav: {msg.data}")

            elif action == 'find':
                target = parsed.get('target', '')
                msg = String()
                msg.data = target
                self.yolo_pub_.publish(msg)
                self.get_logger().info(f"YOLO target: {target}")

            elif action == 'mode':
                cmd = parsed.get('command', '')
                if cmd == 'manual' and self.mode_ != 'MANUAL':
                    msg = String()
                    msg.data = 'toggle'
                    self.toggle_pub_.publish(msg)
                elif cmd == 'autonomous' and self.mode_ != 'AUTONOMOUS':
                    msg = String()
                    msg.data = 'toggle'
                    self.toggle_pub_.publish(msg)

            elif action == 'estop':
                msg = String()
                msg.data = 'stop'
                self.estop_pub_.publish(msg)
                self.get_logger().info("Emergency stop!")

            elif action == 'status':
                response_text = parsed.get('response', '')
                self.get_logger().info(f"Status: {response_text}")
                print(f"\nRobot: {response_text}\n")

            else:
                self.get_logger().warn(f"Unknown action: {action}")

        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON from LLaMA: {raw}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LlamaInterpreterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()