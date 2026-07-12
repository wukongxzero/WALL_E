#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>


enum class State { MANUAL, AUTONOMOUS, IDLE };


constexpr double CAMERA_TIMEOUT_S  = 2.0;
constexpr double LOCALIZATION_TIMEOUT_S = 3.0;

class StateMachine : public rclcpp::Node {
public:
    StateMachine() : Node("state_machine"), state_(State::MANUAL) {

        joy_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/joy_cmd_vel", 10,
            std::bind(&StateMachine::joy_callback, this, std::placeholders::_1));

        nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/nav_cmd_vel", 10,
            std::bind(&StateMachine::nav_callback, this, std::placeholders::_1));

        toggle_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/state_toggle", 10,
            std::bind(&StateMachine::toggle_callback, this, std::placeholders::_1));

        estop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", 10,
            std::bind(&StateMachine::estop_callback, this, std::placeholders::_1));

        serial_fault_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/wall_e/serial_fault", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && state_ == State::AUTONOMOUS)
                    go_idle("SERIAL TIMEOUT");
            });

        // Safety watchdog subscriptions
        camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/realsense2_camera/color/image_raw", 1,
            [this](const sensor_msgs::msg::Image::SharedPtr) {
                last_camera_ = now();
            });

        rtabmap_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 1,
            [this](const nav_msgs::msg::Odometry::SharedPtr) {
                last_localization_ = now();
            });

        cmd_pub_   = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/wall_e/state", 10);

        last_camera_       = now();
        last_localization_ = now();

        state_timer_ = create_wall_timer(std::chrono::seconds(1),
            std::bind(&StateMachine::publish_state, this));

        safety_timer_ = create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&StateMachine::safety_watchdog, this));

        RCLCPP_INFO(get_logger(), "State machine ready — MANUAL mode");
    }

private:
    void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (state_ == State::MANUAL)
            cmd_pub_->publish(*msg);
    }

    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (state_ == State::AUTONOMOUS)
            cmd_pub_->publish(*msg);
    }

    void toggle_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        if (state_ == State::MANUAL) {
            state_ = State::AUTONOMOUS;
            RCLCPP_INFO(get_logger(), "→ AUTONOMOUS");
        } else if (state_ == State::AUTONOMOUS) {
            state_ = State::MANUAL;
            RCLCPP_INFO(get_logger(), "→ MANUAL");
        } else if (state_ == State::IDLE) {
            state_ = State::MANUAL;
            RCLCPP_INFO(get_logger(), "→ MANUAL (recovered from IDLE)");
        }
        publish_state();
    }

    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            go_idle("EMERGENCY STOP");
        }
    }

    void safety_watchdog() {
        if (state_ != State::AUTONOMOUS) return;

        double camera_elapsed = (now() - last_camera_).seconds();
        double loc_elapsed    = (now() - last_localization_).seconds();

        if (camera_elapsed > CAMERA_TIMEOUT_S) {
            RCLCPP_ERROR(get_logger(),
                "RealSense dropout — no image for %.1fs", camera_elapsed);
            go_idle("SENSOR DROPOUT");
        }

        if (loc_elapsed > LOCALIZATION_TIMEOUT_S) {
            RCLCPP_ERROR(get_logger(),
                "Localization lost — no RTAB-Map odom for %.1fs", loc_elapsed);
            go_idle("LOCALIZATION FAILURE");
        }
    }

    void go_idle(const std::string& reason) {
        state_ = State::IDLE;
        cmd_pub_->publish(geometry_msgs::msg::Twist());  // zero velocity
        RCLCPP_WARN(get_logger(), "→ IDLE (%s)", reason.c_str());
        publish_state();
    }

    void publish_state() {
        auto msg = std_msgs::msg::String();
        switch (state_) {
            case State::MANUAL:     msg.data = "MANUAL";     
            break;
            
            case State::AUTONOMOUS: msg.data = "AUTONOMOUS"; 
            break;
            case State::IDLE:
                msg.data = "IDLE";
                cmd_pub_->publish(geometry_msgs::msg::Twist());
                break;
        }
        state_pub_->publish(msg);
    }

    State state_;

    rclcpp::Time last_camera_;
    rclcpp::Time last_localization_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_sub_, nav_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_sub_, estop_sub_, serial_fault_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rtabmap_odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateMachine>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
