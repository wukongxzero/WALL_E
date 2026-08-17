#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import tf2_ros
import rclpy.duration

STOP_SHORT_M = 0.5   # stop this far in front of the detected object
GOAL_COOLDOWN = 3.0  # seconds before sending a new goal


class YoloNavNode(Node):
    def __init__(self):
        super().__init__('yolo_nav_node')

        self.bridge_ = CvBridge()
        self.model_  = YOLO('yolov8n.pt')

        self.fx_ = self.fy_ = self.cx_ = self.cy_ = None
        self.target_class_ = None
        self.latest_depth_ = None

        self.tf_buffer_   = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        self.nav_client_ = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle    = None   # track active goal so we can cancel
        self._last_goal_time = 0.0

        self.create_subscription(Image,
            '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image,
            '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo,
            '/camera/color/camera_info', self.camera_info_callback, 10)
        self.create_subscription(String,
            '/yolo_target', self.target_callback, 10)

        self.get_logger().info("YOLO nav node ready")

    # ── CALLBACKS ──────────────────────────────────────────────────────────────

    def camera_info_callback(self, msg):
        if self.fx_ is None:
            self.fx_ = msg.k[0]
            self.fy_ = msg.k[4]
            self.cx_ = msg.k[2]
            self.cy_ = msg.k[5]
            self.get_logger().info(f"Camera intrinsics: fx={self.fx_:.1f}")

    def depth_callback(self, msg):
        self.latest_depth_ = self.bridge_.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    def target_callback(self, msg):
        self.target_class_ = msg.data.lower()
        self.get_logger().info(f"YOLO target: {self.target_class_}")

    def image_callback(self, msg):
        if self.fx_ is None or self.latest_depth_ is None:
            return
        if self.target_class_ is None:
            return

        # Throttle: don't spam goals faster than GOAL_COOLDOWN
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_goal_time < GOAL_COOLDOWN:
            return

        cv_image = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        results  = self.model_(cv_image, verbose=False)

        for result in results:
            for box in result.boxes:
                cls_name = self.model_.names[int(box.cls)]
                if cls_name != self.target_class_:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                if not (0 <= cy < self.latest_depth_.shape[0] and
                        0 <= cx < self.latest_depth_.shape[1]):
                    continue

                depth = float(self.latest_depth_[cy, cx]) / 1000.0
                if depth < 0.3 or depth > 5.0:
                    continue

                x_cam = (cx - self.cx_) * depth / self.fx_
                y_cam = (cy - self.cy_) * depth / self.fy_
                z_cam = depth

                try:
                    goal_cam = PoseStamped()
                    goal_cam.header.stamp    = self.get_clock().now().to_msg()
                    goal_cam.header.frame_id = 'camera_color_optical_frame'
                    goal_cam.pose.position.x = z_cam
                    goal_cam.pose.position.y = -x_cam
                    goal_cam.pose.position.z = -y_cam
                    goal_cam.pose.orientation.w = 1.0

                    goal_map = self.tf_buffer_.transform(
                        goal_cam, 'map',
                        timeout=rclpy.duration.Duration(seconds=0.1))

                    # Stop short of the object
                    goal_map.pose.position.x -= STOP_SHORT_M

                    self._send_nav_goal(goal_map, cls_name)
                    self._last_goal_time = now

                except Exception as e:
                    self.get_logger().warn(f"TF error: {e}")
                break

    # ── NAVIGATION ─────────────────────────────────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped, label: str):
        if not self.nav_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("NavigateToPose action server not available")
            return

        # Cancel any active goal before sending a new one
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending nav goal for '{label}' → "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")

        send_future = self.nav_client_.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn("Nav goal rejected")
            self._goal_handle = None
            return
        self.get_logger().info("Nav goal accepted")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {dist:.2f} m", throttle_duration_sec=2.0)

    def _result_callback(self, future):
        self._goal_handle = None
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info("Goal reached.")
        else:
            self.get_logger().warn(f"Goal failed with status: {status}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(YoloNavNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
