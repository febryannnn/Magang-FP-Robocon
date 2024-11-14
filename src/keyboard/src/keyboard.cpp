#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

class Keyboard : public rclcpp::Node {
public:
    Keyboard() : Node("keyboard") {
        keyPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "\n\tw: Ketik 'w' untuk maju,\n\
        s: Ketik 's' untuk mundur,\n\
        a: Ketik 'a' untuk belok kiri,\n\
        d: Ketik 'w' untuk belok kanan,\n\
        space: tekan spasi untuk berhenti,\n\
        q: QUIT!!!");
    }

    char inputKeyboard() {
        char buf = 0;
        struct termios old = {};
        if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0) perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");
        return buf;
    }

    void run() {
        char ketik;
        while (rclcpp::ok()) {
            ketik = inputKeyboard();

            if (ketik == 'w') {
                gerak.linear.x += 0.5;
            } else if (ketik == 's') {
                gerak.linear.x -= 0.5;
            } else if (ketik == 'a') {
                gerak.angular.z += 0.5;
            } else if (ketik == 'd') {
                gerak.angular.z -= 0.5;
            } else if (ketik == ' ') {
                gerak.linear.x = 0;
                gerak.angular.z = 0;
            } else if (ketik == 'q') {
                break;
            }

            RCLCPP_INFO(this->get_logger(), "linear: %.2f, angular: %.2f", gerak.linear.x, gerak.angular.z);
            keyPub->publish(gerak);
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr keyPub;
    geometry_msgs::msg::Twist gerak;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Keyboard>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
