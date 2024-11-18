#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h" 
#include "tf2/utils.h"
#include <cmath>

class moveTarget : public rclcpp::Node
{
    public:
    moveTarget() : Node("move_target") {
        subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&moveTarget::pergerakanRobot, this, std::placeholders::_1));

        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        target_x = 5.0;
        target_y = 3.0;
    }

    geometry_msgs::msg::Twist move;

        void pergerakanRobot(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double posisi_robot_x = msg->pose.pose.position.x;
        double posisi_robot_y = msg->pose.pose.position.y;

        double distance = std::sqrt(std::pow(target_x - posisi_robot_x, 2) + std::pow(target_y - posisi_robot_y, 2));

        double target_sudut = std::atan2(target_y - posisi_robot_y, target_x - posisi_robot_x);

        double sudut_sekarang = tf2::getYaw(msg->pose.pose.orientation);

        double selisih_sudut = target_sudut - sudut_sekarang;

        move.linear.x = 0.5 * distance;
        move.angular.z = 1.5 * selisih_sudut;

        if (move.linear.x > 1.0) move.linear.x = 1.0;
        if (move.angular.z > 1.0) move.angular.z = 1.0;

        publisher->publish(move);

        if (distance < 0.1) {
        move.linear.x = 0.0;
        move.angular.z = 0.0;
        publisher->publish(move);
        RCLCPP_INFO(this->get_logger(), "ROBOTT WES NYAMPEEEEEEEE.");
        rclcpp::shutdown();

        }
    }

    private:

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        double target_x, target_y;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<moveTarget>());
    rclcpp::shutdown();
}