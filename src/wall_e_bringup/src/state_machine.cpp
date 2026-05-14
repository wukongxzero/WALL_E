#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

enum class State { MANUAL, AUTONOMOUS, IDLE };

class StateMachine : public rclcpp::Node {
public:
    StateMachine() : Node("state_machine"), state_(State::MANUAL) {
        // Inputs
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

        // Outputs
        cmd_pub_   = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/wall_e/state", 10);

        // Publish state at 1Hz for LCD
        state_timer_ = create_wall_timer(std::chrono::seconds(1),
            std::bind(&StateMachine::publish_state, this));
        

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
            // Toggle from IDLE always goes back to MANUAL
            state_ = State::MANUAL;
            RCLCPP_INFO(get_logger(), "→ MANUAL (recovered from IDLE)");
        }
        publish_state();
    }

    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            state_ = State::IDLE;
            cmd_pub_->publish(geometry_msgs::msg::Twist());
            RCLCPP_WARN(get_logger(), "EMERGENCY STOP → IDLE");
            publish_state();
        }
    }

    void publish_state() {
        auto msg = std_msgs::msg::String();
        switch (state_) {
            case State::MANUAL:     msg.data = "MANUAL";     break;
            case State::AUTONOMOUS: msg.data = "AUTONOMOUS"; break;
            case State::IDLE:       
            msg.data = "IDLE";
            cmd_pub_->publish(geometry_msgs::msg::Twist());
            break;
        }
        state_pub_->publish(msg);
    }

    State state_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_sub_, nav_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_sub_, estop_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateMachine>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
