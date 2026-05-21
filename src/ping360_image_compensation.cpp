#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ping360_sonar_msgs/msg/sonar_echo.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ping360_image_compensation.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <string>

namespace ping360_image_compensation
{

    Ping360ImageCompensation::Ping360ImageCompensation(const rclcpp::NodeOptions &options) : 
    Node("ping360_image_compensation", options)
    {
        // 토픽 이름, 이미지 크기/해상도, 소나 장착 위치를 launch 파일이나 CLI에서 조정할 수 있게 한다.
        echo_topic_ = declare_parameter<std::string>("echo_topic", "/ping360/scan_echo");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/sim/odom");
        output_topic_ = declare_parameter<std::string>("output_topic", "/ping360/scan_image_odom");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        image_size_ = declare_parameter<int>("odom_image_size", 500);
        image_resolution_ = declare_parameter<double>("odom_image_resolution", 0.02);
        sync_tolerance_ms_ = declare_parameter<int>("odom_sync_tolerance_ms", 5000);
        sonar_mount_x_ = declare_parameter<double>("sonar_mount_x", 0.0);
        sonar_mount_y_ = declare_parameter<double>("sonar_mount_y", 0.0);
        sonar_mount_yaw_ = declare_parameter<double>("sonar_mount_yaw", 0.0);
        intensity_scale_ = declare_parameter<double>("intensity_scale", 4.0);
        image_rate_ms_ = declare_parameter<int>("image_rate", 100);
        reset_on_angle_wrap_ = declare_parameter<bool>("reset_on_angle_wrap", true);
        angle_wrap_threshold_ = declare_parameter<double>("angle_wrap_threshold", M_PI);

        configureImage();

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
        image_pub_ = image_transport::create_publisher(this, output_topic_);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, qos,
        std::bind(&Ping360ImageCompensation::odomCallback, this, std::placeholders::_1));
        echo_sub_ = create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
        echo_topic_, qos,
        std::bind(&Ping360ImageCompensation::echoCallback, this, std::placeholders::_1));

        image_timer_ = create_wall_timer(
        std::chrono::milliseconds(image_rate_ms_),
        std::bind(&Ping360ImageCompensation::publishImage, this));
    }
    double Ping360ImageCompensation::yawFromOdometry(const nav_msgs::msg::Odometry &odom)
    {
        const auto &q = odom.pose.pose.orientation;
        const auto siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const auto cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
    void Ping360ImageCompensation::configureImage()
    {
        // odom 기준 원점을 이미지 중앙에 두기 위해 정사각형 이미지로 만들고,
        // 중앙 인덱스가 흔들리지 않도록 크기를 최소 2 이상의 짝수로 보정한다.
        if(image_size_ < 2)
        image_size_ = 2;
        if(image_size_ % 2 != 0)
        image_size_ += 1;

        image_.header.set__frame_id(odom_frame_);
        image_.set__encoding("mono8");
        image_.set__is_bigendian(0);
        image_.height = image_.width = image_.step = image_size_;
        image_.data.assign(image_size_ * image_size_, 0);
    }

    void Ping360ImageCompensation::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(odom_cache_.empty())
        {
        RCLCPP_INFO(get_logger(), "Received first odom message from %s", odom_topic_.c_str());
        }

        odom_cache_.push_back(Pose2D{rclcpp::Time(msg->header.stamp),
                                    msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    yawFromOdometry(*msg)});

        const auto newest = odom_cache_.back().stamp;
        // echo 메시지 timestamp와 맞출 수 있을 만큼 최근 odom은 유지하되,
        // 오래된 pose가 계속 쌓여 메모리가 증가하지 않도록 30초보다 오래된 항목은 제거한다.
        while(odom_cache_.size() > 2 &&
            (newest - odom_cache_.front().stamp) > rclcpp::Duration::from_seconds(30.0))
        {
        odom_cache_.pop_front();
        }
    }

    std::optional<Ping360ImageCompensation::Pose2D> Ping360ImageCompensation::findOdomPose(const rclcpp::Time &stamp)
    {
        last_odom_delta_ms_ = -1;
        if(odom_cache_.empty())
        return std::nullopt;

        // scan_echo와 odom은 서로 다른 토픽으로 들어오므로 timestamp가 정확히 같지 않을 수 있다.
        // 캐시에 저장된 odom 중 echo timestamp와 시간 차이가 가장 작은 pose를 선택한다.
        const Pose2D *best_pose{nullptr};
        auto best_delta{std::numeric_limits<int64_t>::max()};
        for(const auto &pose: odom_cache_)
        {
        const auto delta = std::llabs((pose.stamp - stamp).nanoseconds());
        if(delta < best_delta)
        {
            best_delta = delta;
            best_pose = &pose;
        }
        }
        last_odom_delta_ms_ = best_delta / 1000000;

        if(best_pose == nullptr ||
        best_delta > static_cast<int64_t>(sync_tolerance_ms_) * 1000000)
        {
        return std::nullopt;
        }
        return *best_pose;
    }

    void Ping360ImageCompensation::echoCallback(const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg)
    {
        const rclcpp::Time echo_stamp{msg->header.stamp};
        const auto stamp = echo_stamp.nanoseconds() == 0 ? now() : echo_stamp;
        const auto pose = findOdomPose(stamp);
        if(!pose)
        {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "No /sim/odom pose within %d ms of scan_echo stamp "
                            "(closest: %ld ms, cached poses: %zu)",
                            sync_tolerance_ms_, last_odom_delta_ms_, odom_cache_.size());
        return;
        }

        // Ping360은 한 바퀴를 스캔하며 angle이 증가하다가 다음 sweep에서 다시 작은 값으로 돌아간다.
        // angle wrap이 감지되면 이전 sweep의 누적 이미지를 지우고 새 기준 pose부터 다시 그린다.
        if(reset_on_angle_wrap_ && previous_angle_ &&
        msg->angle + angle_wrap_threshold_ < *previous_angle_)
        {
        reference_pose_.reset();
        std::fill(image_.data.begin(), image_.data.end(), 0);
        }
        previous_angle_ = msg->angle;

        if(!reference_pose_)
        {
        reference_pose_ = pose;
        std::fill(image_.data.begin(), image_.data.end(), 0);
        }

        if(msg->intensities.empty() || msg->range == 0 || image_resolution_ <= 0)
        {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Skipping scan_echo because intensity/range/image resolution is invalid");
        return;
        }

        // 현재 차량 pose를 기준 pose(reference_pose_) 좌표계로 변환한다.
        // 이렇게 하면 차량이 움직여도 각 echo 샘플을 첫 scan 기준의 odom 이미지에 누적할 수 있다.
        const auto dx = pose->x - reference_pose_->x;
        const auto dy = pose->y - reference_pose_->y;
        const auto c0 = std::cos(reference_pose_->yaw);
        const auto s0 = std::sin(reference_pose_->yaw);
        const auto rel_x = c0*dx + s0*dy;
        const auto rel_y = -s0*dx + c0*dy;
        const auto rel_yaw = pose->yaw - reference_pose_->yaw;

        // 반복문 안에서 모든 샘플마다 다시 계산하지 않도록,
        // 소나 장착 각도, 차량 상대 yaw, 현재 beam angle의 삼각함수를 미리 계산한다.
        const auto cm = std::cos(sonar_mount_yaw_);
        const auto sm = std::sin(sonar_mount_yaw_);
        const auto cr = std::cos(rel_yaw);
        const auto sr = std::sin(rel_yaw);
        const auto ct = std::cos(msg->angle);
        const auto st = std::sin(msg->angle);
        const auto half_size = image_size_/2;
        const auto sample_count = static_cast<double>(msg->intensities.size());

        for(size_t index = 0; index < msg->intensities.size(); ++index)
        {
        const auto raw_intensity = msg->intensities[index];
        if(raw_intensity == 0)
            continue;
        const auto intensity = static_cast<uint8_t>(
            std::min(255.0, raw_intensity * intensity_scale_));

        const auto range = (static_cast<double>(index) + 1.0) * msg->range / sample_count;
        const auto sonar_x = range * ct;
        const auto sonar_y = range * st;
        // 샘플 위치를 단계적으로 변환한다.
        // 1) sonar frame의 beam 좌표 -> 2) 차량 base 좌표 -> 3) 기준 odom 좌표.
        const auto base_x = sonar_mount_x_ + cm*sonar_x - sm*sonar_y;
        const auto base_y = sonar_mount_y_ + sm*sonar_x + cm*sonar_y;
        const auto corrected_x = rel_x + cr*base_x - sr*base_y;
        const auto corrected_y = rel_y + sr*base_x + cr*base_y;
        // 이미지 중앙을 odom 원점으로 사용한다.
        // corrected_x는 이미지의 세로 방향, corrected_y는 가로 방향 픽셀 이동량으로 매핑한다.
        const auto pixel_x = half_size - static_cast<int>(std::round(corrected_y/image_resolution_));
        const auto pixel_y = half_size - static_cast<int>(std::round(corrected_x/image_resolution_));

        if(pixel_x < 0 || pixel_x >= image_size_ ||
            pixel_y < 0 || pixel_y >= image_size_)
        {
            continue;
        }

        auto &pixel = image_.data[pixel_x + image_.step*pixel_y];
        // 여러 echo 샘플이 같은 픽셀에 들어오면 가장 강한 반사값을 남긴다.
        pixel = std::max<uint8_t>(pixel, intensity);
        }
    }

    void Ping360ImageCompensation::publishImage()
    {
        image_.header.set__stamp(now());
        image_pub_.publish(image_);
    }
}
