#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
import ollama

SYSTEM_PROMPT = """
You are a robot navigation assistant.
The robot has these waypoints: door, home, a, b, c.
Given a natural language command respond with ONLY a JSON object.
No explanation, no markdown, just raw JSON.

Examples:
"go to the door" → {"command": "go", "target": "door"}
"take me home" → {"command": "home"}
"move forward 2 meters" → {"command": "forward", "distance": 2.0}
"stop" → {"command": "stop"}
"go to position a" → {"command": "go", "target": "a"}
"""

class LlamaInterpreterNode(Node):
    def __init__(self):
        super().__init__('llama_interpreter_node')
        self.cmd_pub_ = self.create_publisher(String, '/nav_command', 10)
        self.get_logger().info("LLaMA interpreter ready — type commands below")
        self.thread_ = threading.Thread(target=self.input_loop, daemon=True)
        self.thread_.start()

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
            self.get_logger().info(f"Sending to LLaMA: '{text}'")
            response = ollama.chat(
                model='llama3.2:3b',
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user',   'content': text}
                ]
            )
            raw = response['message']['content'].strip()
            self.get_logger().info(f"LLaMA response: {raw}")
            parsed = json.loads(raw)
            cmd = parsed.get('command', '')

            if cmd == 'go':
                nav_cmd = f"go {parsed.get('target', '')}"
            elif cmd == 'forward':
                nav_cmd = f"forward {parsed.get('distance', 1.0)}"
            elif cmd == 'home':
                nav_cmd = 'home'
            elif cmd == 'stop':
                nav_cmd = 'stop'
            else:
                self.get_logger().warn(f"Unknown command: {cmd}")
                return

            msg = String()
            msg.data = nav_cmd
            self.cmd_pub_.publish(msg)
            self.get_logger().info(f"Published: {nav_cmd}")

        except json.JSONDecodeError:
            self.get_logger().warn(f"LLaMA returned invalid JSON: {raw}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LlamaInterpreterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()