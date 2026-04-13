#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <cstring>
#include <cstdint>

#define PACKET_LEN  16
#define BT_PORT     "/dev/rfcomm0"
#define BAUD        B9600

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {

        fd_ = open(BT_PORT, O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", BT_PORT);
            throw std::runtime_error("BT open failed");
        }
        configure_serial(fd_);
        RCLCPP_INFO(get_logger(), "Parallax controller connected on %s", BT_PORT);

        joy_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/joy_cmd_vel", 10);
        toggle_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/state_toggle", 10);
        estop_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/emergency_stop", 10);

        read_thread_ = std::thread(&ControllerNode::read_loop, this);
        read_thread_.detach();

        RCLCPP_INFO(get_logger(), "Controller node ready");
    }

    ~ControllerNode() {
        running_ = false;
        close(fd_);
    }

private:
    void read_loop() {
        uint8_t buf[PACKET_LEN];
        uint8_t idx = 0;
        while (running_ && rclcpp::ok()) {
            uint8_t c;
            if (read(fd_, &c, 1) > 0) {
                buf[idx++] = c;
                if (idx >= PACKET_LEN) {
                    parse_packet(buf);
                    idx = 0;
                }
            }
        }
    }

    void parse_packet(uint8_t* buf) {
        // Same TankStatus encoding
        // buf[0] = driveLeft  (127 = stop)
        // buf[1] = driveRight (127 = stop)
        int steering = -((int)buf[0] - 127);
        int throttle = -((int)buf[1] - 127);

        steering = std::max(-124, std::min(124, steering));
        throttle = std::max(-124, std::min(124, throttle));

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x  = (throttle / 124.0f) * MAX_LINEAR;
        twist.angular.z = (steering / 124.0f) * MAX_ANGULAR;
        joy_pub_->publish(twist);

        // eulerZ at byte 8 — default 90
        // Parallax sends 0 as toggle signal
        int8_t euler_z;
        memcpy(&euler_z, buf + 8, 1);
        if (euler_z == 0 && !prev_toggle_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            toggle_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "State toggle triggered");
        }
        prev_toggle_ = (euler_z == 0);
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

    static constexpr float MAX_LINEAR  = 0.5f;
    static constexpr float MAX_ANGULAR = 1.0f;

    int fd_;
    bool running_     = true;
    bool prev_toggle_ = false;
    std::thread read_thread_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr toggle_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
