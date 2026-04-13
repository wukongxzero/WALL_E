#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define PACKET_LEN 16
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD B115200

class MegaBridge : public rclcpp::Node {
public:
    MegaBridge() : Node("mega_bridge") {
        // Open serial port
        fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", SERIAL_PORT);
            throw std::runtime_error("Serial open failed");
        }
        configure_serial(fd_);
        RCLCPP_INFO(get_logger(), "Mega connected on %s", SERIAL_PORT);

        // Subscribe to cmd_vel
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MegaBridge::cmd_callback, this, std::placeholders::_1));

        // Watchdog timer - stop motors if no cmd_vel for 0.5s
        last_cmd_ = now();
        watchdog_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MegaBridge::watchdog, this));

        RCLCPP_INFO(get_logger(), "Mega bridge ready");
    }

    ~MegaBridge() {
        send_packet(127, 127); // stop on shutdown
        close(fd_);
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = now();

        // Scale to ±124
        int throttle = static_cast<int>(msg->linear.x  * 124.0);
        int steering = static_cast<int>(msg->angular.z * 124.0);
        throttle = std::max(-124, std::min(124, throttle));
        steering = std::max(-124, std::min(124, steering));

        // Encode to unsigned 0-255 (127 = stop)
        // Mega decodes: throttle = -((driveRight - 127))
        uint8_t drive_right = 127 - throttle;
        uint8_t drive_left  = 127 - steering;

        send_packet(drive_left, drive_right);
    }

    void watchdog() {
        double elapsed = (now() - last_cmd_).seconds();
        if (elapsed > 0.5) {
            send_packet(127, 127); // 127 = stop
        }
    }

    void send_packet(uint8_t drive_left, uint8_t drive_right,
                     int16_t euler_x = 0, int16_t euler_y = 0,
                     int8_t  euler_z = 90) {
        uint8_t buf[PACKET_LEN];
        memset(buf, 0, PACKET_LEN);

        buf[0] = drive_left;
        buf[1] = drive_right;
        // bytes 2-3 padding
        memcpy(buf + 4, &euler_x, sizeof(euler_x));
        memcpy(buf + 6, &euler_y, sizeof(euler_y));
        memcpy(buf + 8, &euler_z, sizeof(euler_z));

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

    int fd_;
    rclcpp::Time last_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MegaBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
