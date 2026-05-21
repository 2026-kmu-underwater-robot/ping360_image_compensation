#ifndef PING360_IMAGE_COMPENSATION__PING360_IMAGE_COMPENSATION_H_
#define PING360_IMAGE_COMPENSATION__PING360_IMAGE_COMPENSATION_H_

#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ping360_sonar_msgs/msg/sonar_echo.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace ping360_image_compensation
{

    class Ping360ImageCompensation : public rclcpp::Node
    {
    public:
        explicit Ping360ImageCompensation(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    private:
        struct Pose2D
        {
            rclcpp::Time stamp;
            double x{};
            double y{};
            double yaw{};
        };

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        std::optional<Pose2D> findOdomPose(const rclcpp::Time &stamp);
        double yawFromOdometry(const nav_msgs::msg::Odometry &odom);
        void configureImage();
        void echoCallback(const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg);
        void publishImage();

        std::string echo_topic_;
        std::string odom_topic_;
        std::string output_topic_;
        std::string odom_frame_;
        int image_size_{};
        int sync_tolerance_ms_{};
        int image_rate_ms_{};
        double image_resolution_{};
        double sonar_mount_x_{};
        double sonar_mount_y_{};
        double sonar_mount_yaw_{};
        double intensity_scale_{};
        double angle_wrap_threshold_{};
        bool reset_on_angle_wrap_{};

        image_transport::Publisher image_pub_;
        sensor_msgs::msg::Image image_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr echo_sub_;
        rclcpp::TimerBase::SharedPtr image_timer_;
        std::deque<Pose2D> odom_cache_;
        std::optional<Pose2D> reference_pose_;
        std::optional<double> previous_angle_;
        int64_t last_odom_delta_ms_{-1};
    };
}  // namespace ping360_image_compensation

#endif  // PING360_IMAGE_COMPENSATION__PING360_IMAGE_COMPENSATION_H_
