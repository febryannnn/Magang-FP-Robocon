#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class BallFollower : public rclcpp::Node {
public:
    BallFollower() : Node("ball_follower") {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&BallFollower::imageCallback, this, std::placeholders::_1)
        );
        
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Konversi pesan gambar ke format OpenCV
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Deteksi bola berdasarkan warna (misal, warna oranye)
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(85, 100, 100), cv::Scalar(95, 255, 255), mask);

        // Temukan kontur bola
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Temukan kontur terbesar (diasumsikan sebagai bola)
            auto max_contour = std::max_element(
                contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                    return cv::contourArea(c1) < cv::contourArea(c2);
                }
            );

            cv::Moments moments = cv::moments(*max_contour);
            double ball_x = moments.m10 / moments.m00;

            // Tentukan kecepatan berdasarkan posisi bola
            geometry_msgs::msg::Twist cmd_vel_msg;
            int frame_center_x = frame.cols / 2;

            // Jika bola ada di kiri layar, putar ke kiri; jika di kanan, putar ke kanan
            if (ball_x < frame_center_x - 50) {
                cmd_vel_msg.angular.z = 0.3;
                cmd_vel_msg.linear.x = 0.5;
            } else if (ball_x > frame_center_x + 50) {
                cmd_vel_msg.angular.z = -0.3;
                cmd_vel_msg.linear.x = 0.5;
            } else {
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_msg.linear.x = 0.5;  // Maju ke arah bola
            }

            // Publikasikan pesan kecepatan
            cmd_vel_publisher_->publish(cmd_vel_msg);
        } else {
            // Jika tidak ada bola yang terdeteksi, berhenti
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_publisher_->publish(cmd_vel_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallFollower>());
    rclcpp::shutdown();
    return 0;
}
