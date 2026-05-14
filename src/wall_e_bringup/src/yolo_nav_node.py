#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import tf2_ros
import rclpy.duration

class YoloNavNode(Node):
    def __init__(self):
        super().__init__('yolo_nav_node')

        self.bridge_ = CvBridge()
        self.model_  = YOLO('yolov8n.pt')

        self.fx_ = None
        self.fy_ = None
        self.cx_ = None
        self.cy_ = None

        self.target_class_ = None
        self.latest_depth_ = None

        self.tf_buffer_   = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        self.create_subscription(Image,
            '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image,
            '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo,
            '/camera/color/camera_info', self.camera_info_callback, 10)
        self.create_subscription(String,
            '/yolo_target', self.target_callback, 10)

        self.goal_pub_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info("YOLO nav node ready")

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

                if (0 <= cy < self.latest_depth_.shape[0] and
                    0 <= cx < self.latest_depth_.shape[1]):

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

                        goal_map.pose.position.x -= 0.5

                        self.goal_pub_.publish(goal_map)
                        self.get_logger().info(
                            f"Navigating to {cls_name} at "
                            f"({goal_map.pose.position.x:.2f}, "
                            f"{goal_map.pose.position.y:.2f})")

                    except Exception as e:
                        self.get_logger().warn(f"TF error: {e}")
                    break

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(YoloNavNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()