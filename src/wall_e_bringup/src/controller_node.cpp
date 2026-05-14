#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <algorithm> // for std::max std::min 

#define PACKET_LEN  8
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
        RCLCPP_INFO(get_logger(), "Parallax connected on %s", BT_PORT);

        joy_pub_    = create_publisher<geometry_msgs::msg::Twist>(
            "/joy_cmd_vel", 10);
        toggle_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/state_toggle", 10);
        estop_pub_  = create_publisher<std_msgs::msg::Bool>(
            "/emergency_stop", 10);

        // Subscribe to pitch from Uno → send back to Parallax LCD
        pitch_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/wall_e/pitch", 10,
            std::bind(&ControllerNode::pitch_callback, this,
                      std::placeholders::_1));

        state_sub_ = create_subscription<std_msgs::msg::String>(
            "/wall_e/state",10,
            std::bind(&ControllerNode::state_callback, this,
                        std::placeholders::_1));
        

        read_thread_ = std::thread(&ControllerNode::read_loop, this);
        read_thread_.detach();

        RCLCPP_INFO(get_logger(), "Controller node ready");
    }

    ~ControllerNode() {
        running_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    last_drive_left_  = buf[0];
    last_drive_right_ = buf[1];

    int steering = -((int)buf[0] - 127);
    int throttle = -((int)buf[1] - 127);

    steering = std::max(-124, std::min(124, steering));
    throttle = std::max(-124, std::min(124, throttle));

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x  = (throttle / 124.0f) * MAX_LINEAR;
    twist.angular.z = (steering / 124.0f) * MAX_ANGULAR;
    joy_pub_->publish(twist);

    // changeFlag at byte 7
    uint8_t change_flag = buf[7];

    // Toggle MANUAL ↔ AUTONOMOUS
    if (change_flag == 1 && !prev_toggle_) {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        toggle_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "State toggle triggered");
    }

    // Emergency stop
    if (change_flag == 2 && !prev_estop_) {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        estop_pub_->publish(msg);
        RCLCPP_WARN(get_logger(), "Emergency stop triggered");
        }

    prev_toggle_ = (change_flag == 1);
    prev_estop_  = (change_flag == 2);
    }

    void pitch_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_pitch_ = msg->data;
        send_telemetry_to_lcd();
    }
    void state_callback(const std_msgs::msg::String::SharedPtr msg){
        if (msg->data == "MANUAL") current_state_ = 0;
        else if (msg->data == "AUTONOMOUS") current_state_ = 1;
        else if (msg->data == "IDLE") current_state_ = 2;

    }

    void send_telemetry_to_lcd() {
        uint8_t buf[PACKET_LEN];
        memset(buf, 0, PACKET_LEN);

        buf[0] = last_drive_left_;
        buf[1] = last_drive_right_;

        int16_t euler_x = 0;
        memcpy(buf + 2, &euler_x, 2);

        int16_t pitch_short = (int16_t)current_pitch_;
        memcpy(buf + 4, &pitch_short, 2);

        int16_t euler_z = 90;
        memcpy(buf + 6, &euler_z, 2);

        // changeFlag byte 7 — send current state to LCD
        // 0=MANUAL 1=AUTONOMOUS 2=IDLE
        buf[7] = current_state_;

        write(fd_, buf, PACKET_LEN);
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
    bool running_         = true;
    bool prev_toggle_     = false;
    uint8_t current_state_ = 0;  // 0=MANUAL 1=AUTONOMOUS 2=IDLE
    bool prev_estop_       = false;
    float current_pitch_  = 0.0f;
    uint8_t last_drive_left_  = 127;
    uint8_t last_drive_right_ = 127;

    std::thread read_thread_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr toggle_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}