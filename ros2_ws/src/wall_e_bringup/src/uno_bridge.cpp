#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <string>
#include <chrono>

#define SERIAL_PORT "/dev/ttyUSB1"
#define BAUD B115200

class UnoBridge : public rclcpp::Node {
public:
    UnoBridge() : Node("uno_bridge") {
        fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", SERIAL_PORT);
            throw std::runtime_error("Serial open failed");
        }
        configure_serial(fd_);
        RCLCPP_INFO(get_logger(), "Uno connected on %s", SERIAL_PORT);
        roll_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/wall_e/roll", 10);
        pitch_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/wall_e/pitch", 10);

        read_thread_ = std::thread(&UnoBridge::read_loop, this);
        read_thread_.detach();

        RCLCPP_INFO(get_logger(), "Uno bridge ready");
    }

    ~UnoBridge() {
        running_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        close(fd_);
    }

private:
    void read_loop() {
        std::string line;
        char c;
        while (running_ && rclcpp::ok()) {
            if (read(fd_, &c, 1) > 0) {
                if (c == '\n') {
                    parse_line(line);
                    line.clear();
                } else if (c != '\r') {
                    line += c;
                }
            }
        }
    }

    void parse_line(const std::string &line) {
    if (line.rfind("P:", 0) != 0) return;
    auto comma = line.find(",R:");
    try {
        float pitch = std::stof(line.substr(2, comma - 2));
        auto pmsg = std_msgs::msg::Float32();
        pmsg.data = pitch;
        pitch_pub_->publish(pmsg);

        if (comma != std::string::npos) {
            float roll = std::stof(line.substr(comma + 3));
            auto rmsg = std_msgs::msg::Float32();
            rmsg.data = roll;
            roll_pub_->publish(rmsg);
        }
    } catch (...) {
        RCLCPP_WARN(get_logger(), "Bad line: %s", line.c_str());
    }
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
    std::thread read_thread_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnoBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}