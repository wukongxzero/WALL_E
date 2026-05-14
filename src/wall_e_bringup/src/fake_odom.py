#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.odom_pub_ = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.x_ = 0.0
        self.y_ = 0.0
        self.create_timer(0.05, self.publish)
        self.get_logger().info("Fake odometry publishing at 20Hz")

    def publish(self):
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[35] = 0.01
        self.odom_pub_.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x_
        tf.transform.translation.y = self.y_
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster_.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FakeOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()