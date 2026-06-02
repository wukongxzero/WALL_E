#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import threading
import math
import json
import ollama

# Named location map — fill in real coordinates after first mapping run
NAMED_GOALS = {
    'door':    (3.0,  0.0, 0.0),   # (x, y, yaw_deg)
    'home':    (0.0,  0.0, 0.0),
}

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

        self.yolo_pub_   = self.create_publisher(String, '/yolo_target', 10)
        self.toggle_pub_ = self.create_publisher(String, '/state_toggle_cmd', 10)
        self.estop_pub_  = self.create_publisher(String, '/emergency_stop_cmd', 10)

        self.robot_x_     = 0.0
        self.robot_y_     = 0.0
        self.robot_yaw_   = 0.0
        self.pitch_       = 0.0
        self.roll_        = 0.0
        self.mode_        = "MANUAL"
        self.cmd_linear_  = 0.0
        self.cmd_angular_ = 0.0

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/wall_e/pitch',
            lambda m: setattr(self, 'pitch_', m.data), 10)
        self.create_subscription(Float32, '/wall_e/roll',
            lambda m: setattr(self, 'roll_', m.data), 10)
        self.create_subscription(String, '/wall_e/state',
            lambda m: setattr(self, 'mode_', m.data), 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.nav_client_  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None

        # process() runs in a background thread — post goals here, send from spin thread
        self._pending_goal = None
        self._goal_lock    = threading.Lock()
        self.create_timer(0.1, self._goal_dispatch_timer)

        self.ollama_client_ = ollama.Client(host='http://localhost:11434')

        self.get_logger().info("LLaMA interpreter ready — type commands below")
        self.thread_ = threading.Thread(target=self.input_loop, daemon=True)
        self.thread_.start()

    # ── SUBSCRIBERS ────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.robot_x_ = msg.pose.pose.position.x
        self.robot_y_ = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw_ = math.atan2(siny, cosy)

    def cmd_callback(self, msg):
        self.cmd_linear_  = msg.linear.x
        self.cmd_angular_ = msg.angular.z

    # ── CONTEXT ────────────────────────────────────────────────────────────────

    def build_context(self):
        return (
            f"Current robot state:\n"
            f"- Position: x={self.robot_x_:.2f}m, y={self.robot_y_:.2f}m\n"
            f"- Mode: {self.mode_}\n"
            f"- Platform pitch: {self.pitch_:.1f} degrees\n"
            f"- Platform roll:  {self.roll_:.1f} degrees\n"
            f"- Current velocity: linear={self.cmd_linear_:.2f} m/s, "
            f"angular={self.cmd_angular_:.2f} rad/s\n"
        )

    # ── INPUT LOOP (background thread) ─────────────────────────────────────────

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Say: ").strip()
                if user_input:
                    self.process(user_input)
            except EOFError:
                break

    def process(self, text):
        try:
            response = self.ollama_client_.chat(
                model='llama3.2:3b',
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user',   'content': f"{self.build_context()}\nUser command: {text}"}
                ]
            )

            raw = response['message']['content'].strip()
            self.get_logger().info(f"LLaMA: {raw}")
            parsed = json.loads(raw)
            action = parsed.get('action', '')

            if action == 'navigate':
                self._handle_navigate(parsed.get('command', ''))

            elif action == 'find':
                target = parsed.get('target', '')
                msg = String(); msg.data = target
                self.yolo_pub_.publish(msg)
                self.get_logger().info(f"YOLO target: {target}")

            elif action == 'mode':
                cmd = parsed.get('command', '')
                if (cmd == 'manual' and self.mode_ != 'MANUAL') or \
                   (cmd == 'autonomous' and self.mode_ != 'AUTONOMOUS'):
                    msg = String(); msg.data = 'toggle'
                    self.toggle_pub_.publish(msg)

            elif action == 'estop':
                self._cancel_goal()
                msg = String(); msg.data = 'stop'
                self.estop_pub_.publish(msg)
                self.get_logger().info("Emergency stop!")

            elif action == 'status':
                text = parsed.get('response', '')
                self.get_logger().info(f"Status: {text}")
                print(f"\nRobot: {text}\n")

            else:
                self.get_logger().warn(f"Unknown action: {action}")

        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON from LLaMA: {raw}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    # ── NAVIGATION ─────────────────────────────────────────────────────────────

    def _handle_navigate(self, command: str):
        command = command.strip().lower()

        if command == 'stop':
            self._cancel_goal()
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0

        if command.startswith('forward '):
            try:
                dist = float(command.split()[1])
            except (IndexError, ValueError):
                self.get_logger().warn(f"Bad forward command: {command}")
                return
            pose.header.frame_id = 'base_footprint'
            pose.pose.position.x = dist

        elif command.startswith('go '):
            name = command[3:].strip()
            if name not in NAMED_GOALS:
                self.get_logger().warn(f"Unknown location: '{name}'")
                return
            x, y, yaw_deg = NAMED_GOALS[name]
            pose.pose.position.x = x
            pose.pose.position.y = y
            yaw = math.radians(yaw_deg)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)

        elif command in NAMED_GOALS:
            x, y, yaw_deg = NAMED_GOALS[command]
            pose.pose.position.x = x
            pose.pose.position.y = y
            yaw = math.radians(yaw_deg)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)

        else:
            self.get_logger().warn(f"Unrecognised navigate command: '{command}'")
            return

        # Post to spin thread via lock — timer dispatches it
        with self._goal_lock:
            self._pending_goal = pose

    def _goal_dispatch_timer(self):
        with self._goal_lock:
            pose = self._pending_goal
            self._pending_goal = None
        if pose is None:
            return
        self._send_nav_goal(pose)

    def _send_nav_goal(self, pose: PoseStamped):
        if not self.nav_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("NavigateToPose server not available")
            return

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"Nav goal → ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) "
            f"frame={pose.header.frame_id}")

        future = self.nav_client_.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            self.get_logger().info("Active goal cancelled")

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn("Nav goal rejected")
            self._goal_handle = None
            return
        self.get_logger().info("Nav goal accepted")
        future = self._goal_handle.get_result_async()
        future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {dist:.2f} m",
                               throttle_duration_sec=2.0)

    def _result_callback(self, future):
        self._goal_handle = None
        status = future.result().status
        if status == 4:
            self.get_logger().info("Goal reached.")
        else:
            self.get_logger().warn(f"Goal failed with status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = LlamaInterpreterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
