#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include<chrono>
#include <cstdint>

#define CMD_LEN     8    // TankStatus packet Jetson → Mega
#define ODOM_LEN    20   // Odometry packet   Mega → Jetson
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD        B115200
#define TRACK_WIDTH 0.32f

class MegaNode : public rclcpp::Node {
public:
    MegaNode() : Node("mega_node") {

        fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", SERIAL_PORT);
            throw std::runtime_error("Serial open failed");
        }
        configure_serial(fd_);
        RCLCPP_INFO(get_logger(), "Mega connected on %s", SERIAL_PORT);

        // Subscribe to cmd_vel → send drive commands to Mega
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MegaNode::cmd_callback, this, std::placeholders::_1));

        // Publish odometry from Mega encoder data
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // TF broadcaster odom → base_footprint
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Watchdog — stop motors if no cmd_vel for 0.5s
        last_cmd_ = now();
        watchdog_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MegaNode::watchdog, this));

        // Read serial in background thread
        read_thread_ = std::thread(&MegaNode::read_loop, this);
        read_thread_.detach();

        RCLCPP_INFO(get_logger(), "Mega node ready");
    }

    ~MegaNode() {
    running_ = false;       // stop thread first
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    send_packet(127, 127);  // then stop motors
    close(fd_);             // then close
    }

private:
    // ── CMD_VEL → DRIVE PACKET ──
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = now();

        int throttle = static_cast<int>(msg->linear.x  * 124.0);
        int steering = static_cast<int>(msg->angular.z * 124.0);
        throttle = std::max(-124, std::min(124, throttle));
        steering = std::max(-124, std::min(124, steering));

        // Encode unsigned 0-255 (127 = stop)
        uint8_t drive_right = 127 - throttle;
        uint8_t drive_left  = 127 - steering;

        send_packet(drive_left, drive_right);
    }

    void watchdog() {
        double elapsed = (now() - last_cmd_).seconds();
        if (elapsed > 0.5)
            send_packet(127, 127);
    }

    void send_packet(uint8_t drive_left, uint8_t drive_right,
                     int16_t euler_x = 0, int16_t euler_y = 0,
                     int16_t euler_z = 90) {
        uint8_t buf[CMD_LEN];
        memset(buf, 0, CMD_LEN);
        buf[0] = drive_left;
        buf[1] = drive_right;
        memcpy(buf + 2, &euler_x, 2);
        memcpy(buf + 4, &euler_y, 2);
        memcpy(buf + 6, &euler_z, 2);
        write(fd_, buf, CMD_LEN);
    }

    // ── READ ODOMETRY FROM MEGA ──
    void read_loop() {
        uint8_t buf[256];
        uint8_t idx = 0;

        while (running_ && rclcpp::ok()) {
            uint8_t c;
            if (read(fd_, &c, 1) > 0) {
                buf[idx++] = c;

                // Sync on 0xAA 0xBB header
                if (idx >= 2) {
                    if (buf[idx-2] == 0xAA && buf[idx-1] == 0xBB) {
                        buf[0] = 0xAA;
                        buf[1] = 0xBB;
                        idx = 2;
                        }
                }

                if (idx >= ODOM_LEN) {
                    if (buf[0] == 0xAA && buf[1] == 0xBB) {
                        parse_odom(buf);
                    }
                    idx = 0;
                }
            }
        }
    }

    void parse_odom(uint8_t* buf) {
        float x, y, theta, lvel, rvel;
        memcpy(&x,     buf + 2,  4);
        memcpy(&y,     buf + 6,  4);
        memcpy(&theta, buf + 10, 4);
        memcpy(&lvel,  buf + 14, 4);
        memcpy(&rvel,  buf + 18, 4);

        auto stamp = get_clock()->now();

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        // Publish /odom
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp    = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_footprint";

        odom.pose.pose.position.x    = x;
        odom.pose.pose.position.y    = y;
        odom.pose.pose.position.z    = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        float linear  = (rvel + lvel) / 2.0f;
        float angular = (rvel - lvel) / TRACK_WIDTH;

        odom.twist.twist.linear.x  = linear;
        odom.twist.twist.angular.z = angular;

        odom.pose.covariance[0]  = 0.01;
        odom.pose.covariance[7]  = 0.01;
        odom.pose.covariance[35] = 0.01;

        odom_pub_->publish(odom);

        // Broadcast TF odom → base_footprint
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_footprint";

        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x    = q.x();
        tf.transform.rotation.y    = q.y();
        tf.transform.rotation.z    = q.z();
        tf.transform.rotation.w    = q.w();

        tf_broadcaster_->sendTransform(tf);
    }

    void configure_serial(int fd) {
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        tcgetattr(fd, &tty);
        cfsetispeed(&tty, BAUD);
        cfsetospeed(&tty, BAUD);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;
        tcsetattr(fd, TCSANOW, &tty);
    }

    int fd_;
    bool running_ = true;
    rclcpp::Time last_cmd_;
    std::thread read_thread_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MegaNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}