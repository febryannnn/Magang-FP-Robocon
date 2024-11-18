#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h" 
#include "tf2/utils.h"
#include <cmath>
#include <vector>

class kellLapangan: public rclcpp::Node
{
public:
    kellLapangan() : Node("kell_lapangan") {
        subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&kellLapangan::pergerakanRobot, this, std::placeholders::_1));
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        targets = {{-7.0, 2.0}, {5.5, 3.0}, {6.5, -1.2},{-6.0, -3.0}};

        this->declare_parameter("kp_linear", 1.0);
        this->declare_parameter("ki_linear", 0.0);
        this->declare_parameter("kd_linear", 0.0);
        this->declare_parameter("kp_angular", 1.0);
        this->declare_parameter("ki_angular", 0.0);
        this->declare_parameter("kd_angular", 0.0);

        this->get_parameter("kp_linear", kp_linear);
        this->get_parameter("ki_linear", ki_linear);
        this->get_parameter("kd_linear", kd_linear);
        this->get_parameter("kp_angular", kp_angular);
        this->get_parameter("ki_angular", ki_angular);
        this->get_parameter("kd_angular", kd_angular);
    }

    void pergerakanRobot(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (current_target >= targets.size()) {
            RCLCPP_INFO(this->get_logger(), "semua titik sudah didatangi!!");
            rclcpp::shutdown();
            return;
        }

        double target_x = targets[current_target].first;
        double target_y = targets[current_target].second;
        double posisi_robot_x = msg->pose.pose.position.x;
        double posisi_robot_y = msg->pose.pose.position.y;

        double distance = std::sqrt(std::pow(target_x - posisi_robot_x, 2) + std::pow(target_y - posisi_robot_y, 2));
        double target_sudut = std::atan2(target_y - posisi_robot_y, target_x - posisi_robot_x);
        double sudut_sekarang = tf2::getYaw(msg->pose.pose.orientation);
        double selisih_sudut = target_sudut - sudut_sekarang;

        double linear_move = kp_linear * distance + ki_linear * integral_linear + kd_linear * (distance - previous_distance_error);
        integral_linear += distance;
        previous_distance_error = distance;

        double angular_move = kp_angular * selisih_sudut + ki_angular * integral_angular + kd_angular * (selisih_sudut - previous_angle_error);
        integral_angular += selisih_sudut;
        previous_angle_error = selisih_sudut;

        geometry_msgs::msg::Twist move;
        move.linear.x = std::min(linear_move, 1.0);
        move.angular.z = std::min(angular_move, 1.0);

        publisher->publish(move);

        if (distance < 0.1) {
            RCLCPP_INFO(this->get_logger(), "Target ke-%zu", current_target + 1);

            move.linear.x = 0.0;
            move.angular.z = 0.0;
            publisher->publish(move);

            integral_linear = 0.0;
            integral_angular = 0.0;
            previous_distance_error = 0.0;
            previous_angle_error = 0.0;

            rclcpp::sleep_for(std::chrono::milliseconds(500));

            current_target++;
        }
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    std::vector<std::pair<double, double>> targets;
    size_t current_target = 0;
    double kp_linear, ki_linear, kd_linear;
    double kp_angular, ki_angular, kd_angular;
    double integral_linear = 0.0, integral_angular= 0.0;
    double previous_distance_error = 0.0, previous_angle_error = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kellLapangan>());
    rclcpp::shutdown();
}
