#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <math.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class KellLapangan : public rclcpp::Node

{
    public:
    KellLapangan() : Node("kell_lapangan") {
    robotPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Mulai mengelilingi lapangan!!!");

    }
    
    geometry_msgs::msg::Twist message;

    void moveRobot(double horizontal, double vertical, double degree, double durasi) {
        message.linear.x = horizontal;
        message.linear.y = vertical;
        message.angular.z = degree;
        robotPublisher -> publish(message);
        std::this_thread::sleep_for(std::chrono::duration<double>(durasi));
        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.angular.z = 0.0;
        robotPublisher -> publish(message);
    }

    void rotate (double degree, double durasi) {
        message.angular.z = degree * M_PI / 180.0;
        robotPublisher -> publish(message);
        std::this_thread::sleep_for(std::chrono::duration<double>(durasi));
        message.angular.z = 0.0;
        robotPublisher -> publish(message);
    }

    void gaskan() {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(2.5, 0.0, 0.0, 1.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rotate(-0.1, 0.5);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        rotate(-90.0, 1.0);

        std::this_thread::sleep_for(std::chrono::seconds(1));
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(5.0, 0.0, 0.0, 1.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        moveRobot(1.0, 0.0, 0.0, 1.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        //rotate(0.2, 0.5);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        rotate(-90.0, 1.0);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(2.3, 0.0, 0.0, 1.0);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        rotate(90.0, 1.0);

        std::this_thread::sleep_for(std::chrono::seconds(1));
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(5.0, 0.0, 0.0, 1.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        moveRobot(1.0, 0.0, 0.0, 1.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void gaskan_2() {
        rotate (-90.0, 1.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rotate (0.2, 1.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(5.0, 0.0, 0.0, 1.0);
        moveRobot(2.5, 0.0, 0.0, 1.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robotPublisher;
};

int main(int argc,char *argv[]) 
{
    rclcpp::init(argc, argv); 
    auto node1= std::make_shared<KellLapangan>();

        node1 -> gaskan();

    auto node2 = std::make_shared<KellLapangan>();

        node2 -> gaskan();
    
    rclcpp::shutdown();
    return 0;
        
}
