#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Joystick : public rclcpp::Node {
public:
    Joystick() : Node("joystick") {
        joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&Joystick::joy_callback, this, std::placeholders::_1)
        );
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "gerakkan joystick sekarang!!!");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr message) {
        auto move = geometry_msgs::msg::Twist();

        move.linear.x = message->axes[1];  
        move.angular.z = message->axes[0]; 

        publisher->publish(move);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Joystick>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
