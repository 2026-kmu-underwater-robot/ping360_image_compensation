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
#include <cstdlib>
#include <deque>
#include <functional>
#include <iterator>
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
        output_frame_ = declare_parameter<std::string>("output_frame", "ping360_image_stabilized");
        image_size_ = declare_parameter<int>("odom_image_size", 500);
        image_resolution_ = declare_parameter<double>("odom_image_resolution", 0.02);
        sync_tolerance_ms_ = declare_parameter<int>("odom_sync_tolerance_ms", 300);
        use_echo_header_stamp_ = declare_parameter<bool>("use_echo_header_stamp", true);
        use_odom_header_stamp_ = declare_parameter<bool>("use_odom_header_stamp", true);
        sonar_mount_x_ = declare_parameter<double>("sonar_mount_x", 0.0);
        sonar_mount_y_ = declare_parameter<double>("sonar_mount_y", 0.0);
        sonar_mount_yaw_ = declare_parameter<double>("sonar_mount_yaw", 0.0);
        intensity_scale_ = declare_parameter<double>("intensity_scale", 4.0);
        image_rate_ms_ = declare_parameter<int>("image_rate", 100);
        reset_on_angle_wrap_ = declare_parameter<bool>("reset_on_angle_wrap", true);
        angle_wrap_threshold_ = declare_parameter<double>("angle_wrap_threshold", M_PI);
        angle_epsilon_ = declare_parameter<double>("angle_epsilon", 1e-4);
        sweep_completion_margin_ = declare_parameter<double>("sweep_completion_margin", 0.25);
        odom_cache_duration_sec_ = declare_parameter<double>("odom_cache_duration_sec", 5.0);

        configureImage();

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        image_pub_ = image_transport::create_publisher(this, output_topic_);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, qos,
        std::bind(&Ping360ImageCompensation::odomCallback, this, std::placeholders::_1));
        echo_sub_ = create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
        echo_topic_, qos,
        std::bind(&Ping360ImageCompensation::echoCallback, this, std::placeholders::_1));

        image_timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(1, image_rate_ms_)),
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
        // 보정 결과의 기준 frame 원점을 이미지 중앙에 두기 위해 정사각형 이미지로 만들고,
        // 중앙 인덱스가 흔들리지 않도록 크기를 최소 2 이상의 짝수로 보정한다.
        if(image_size_ < 2)
        image_size_ = 2;
        if(image_size_ % 2 != 0)
        image_size_ += 1;

        image_.header.set__frame_id(output_frame_);
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

        const auto header_stamp = rclcpp::Time(
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            get_clock()->get_clock_type());
        const auto stamp = (use_odom_header_stamp_ && header_stamp.nanoseconds() != 0) ?
            header_stamp : now();
        const Pose2D timed_pose{stamp,
                                msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                yawFromOdometry(*msg)};

        auto insert_pos = odom_cache_.end();
        while(insert_pos != odom_cache_.begin() && timed_pose.stamp < std::prev(insert_pos)->stamp)
        {
        --insert_pos;
        }
        odom_cache_.insert(insert_pos, timed_pose);
        pruneOdomCache();
    }

    std::optional<Ping360ImageCompensation::Pose2D> Ping360ImageCompensation::interpolateOdomPose(const rclcpp::Time &stamp)
    {
        last_odom_delta_ms_ = -1;
        if(odom_cache_.empty())
        return std::nullopt;

        const auto tolerance_ns = static_cast<int64_t>(std::max(0, sync_tolerance_ms_)) * 1000000;
        auto delta_ns = [&stamp](const Pose2D &pose)
        {
        return std::llabs((pose.stamp - stamp).nanoseconds());
        };

        if(odom_cache_.size() == 1)
        {
        const auto only_delta = delta_ns(odom_cache_.front());
        last_odom_delta_ms_ = only_delta / 1000000;
        if(only_delta <= tolerance_ns)
            return odom_cache_.front();
        return std::nullopt;
        }

        if(stamp <= odom_cache_.front().stamp)
        {
        const auto front_delta = delta_ns(odom_cache_.front());
        last_odom_delta_ms_ = front_delta / 1000000;
        if(front_delta <= tolerance_ns)
            return odom_cache_.front();
        return std::nullopt;
        }

        if(stamp >= odom_cache_.back().stamp)
        {
        const auto back_delta = delta_ns(odom_cache_.back());
        last_odom_delta_ms_ = back_delta / 1000000;
        if(back_delta <= tolerance_ns)
            return odom_cache_.back();
        return std::nullopt;
        }

        for(std::size_t index = 1; index < odom_cache_.size(); ++index)
        {
        const auto &previous = odom_cache_[index - 1];
        const auto &next = odom_cache_[index];

        if(stamp == previous.stamp)
        {
            last_odom_delta_ms_ = 0;
            return previous;
        }
        if(stamp == next.stamp)
        {
            last_odom_delta_ms_ = 0;
            return next;
        }
        if(stamp < next.stamp)
        {
            const auto previous_delta = delta_ns(previous);
            const auto next_delta = delta_ns(next);
            const auto best_delta = std::min(previous_delta, next_delta);
            last_odom_delta_ms_ = best_delta / 1000000;
            if(best_delta > tolerance_ns)
            return std::nullopt;

            const auto interval_ns = (next.stamp - previous.stamp).nanoseconds();
            if(interval_ns <= 0)
            return previous;
            const auto alpha =
            static_cast<double>((stamp - previous.stamp).nanoseconds()) /
            static_cast<double>(interval_ns);
            return interpolatePose(previous, next, alpha, stamp);
        }
        }

        return std::nullopt;
    }

    Ping360ImageCompensation::Pose2D Ping360ImageCompensation::interpolatePose(
        const Pose2D &start,
        const Pose2D &end,
        double alpha,
        const rclcpp::Time &stamp) const
    {
        const auto clamped_alpha = std::clamp(alpha, 0.0, 1.0);
        return Pose2D{stamp,
                    start.x + (end.x - start.x) * clamped_alpha,
                    start.y + (end.y - start.y) * clamped_alpha,
                    start.yaw + normalizeAngleDelta(end.yaw - start.yaw) * clamped_alpha};
    }

    double Ping360ImageCompensation::normalizeAngleDelta(double delta) const
    {
        while(delta > M_PI)
        delta -= 2.0 * M_PI;
        while(delta < -M_PI)
        delta += 2.0 * M_PI;
        return delta;
    }

    void Ping360ImageCompensation::pruneOdomCache()
    {
        if(odom_cache_duration_sec_ <= 0.0 || odom_cache_.empty())
        return;

        const auto newest = odom_cache_.back().stamp;
        while(odom_cache_.size() > 2 &&
            (newest - odom_cache_.front().stamp) >
            rclcpp::Duration::from_seconds(odom_cache_duration_sec_))
        {
        odom_cache_.pop_front();
        }
    }

    void Ping360ImageCompensation::echoCallback(const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg)
    {
        const rclcpp::Time echo_stamp{
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            get_clock()->get_clock_type()};
        const auto stamp = (use_echo_header_stamp_ && echo_stamp.nanoseconds() != 0) ?
            echo_stamp : now();
        const auto pose = interpolateOdomPose(stamp);
        if(!pose)
        {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "No odom pose within %d ms of scan_echo stamp "
                            "(closest: %ld ms, cached poses: %zu)",
                            sync_tolerance_ms_, last_odom_delta_ms_, odom_cache_.size());
        return;
        }

        // Ping360은 sweep을 만드는 동안 각도가 변한다. wrap뿐 아니라 스캔 방향이 바뀌는
        // 왕복 스캔도 새 sweep으로 보아 기준 pose와 이미지를 새로 잡는다.
        if(reset_on_angle_wrap_ && previous_angle_ && detectSweepBoundary(msg->angle))
        {
        resetSweep(*pose, stamp);
        }
        previous_angle_ = msg->angle;

        if(!reference_pose_)
        {
        resetSweep(*pose, stamp);
        }

        if(msg->intensities.empty() || msg->range == 0 || image_resolution_ <= 0)
        {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Skipping scan_echo because intensity/range/image resolution is invalid");
        return;
        }
        last_echo_stamp_ = stamp;

        // 현재 차량 pose를 기준 pose(reference_pose_) 좌표계로 변환한다.
        // 이렇게 하면 차량이 움직여도 각 echo 샘플을 sweep 시작 기준 이미지에 누적할 수 있다.
        const auto dx = pose->x - reference_pose_->x;
        const auto dy = pose->y - reference_pose_->y;
        const auto c0 = std::cos(reference_pose_->yaw);
        const auto s0 = std::sin(reference_pose_->yaw);
        const auto rel_x = c0*dx + s0*dy;
        const auto rel_y = -s0*dx + c0*dy;
        const auto rel_yaw = normalizeAngleDelta(pose->yaw - reference_pose_->yaw);

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
        // 이미지 중앙을 output frame 원점으로 사용한다.
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

    bool Ping360ImageCompensation::detectSweepBoundary(double angle)
    {
        if(!previous_angle_)
        return false;

        const auto raw_delta = angle - *previous_angle_;
        const auto wrapped_forward =
        angle + angle_wrap_threshold_ < *previous_angle_;
        const auto delta = normalizeAngleDelta(raw_delta);
        if(std::abs(delta) < angle_epsilon_)
        return false;

        const auto current_direction = (delta > 0.0) ? 1 : -1;
        if(scan_direction_ == 0)
        {
        scan_direction_ = current_direction;
        accumulated_sweep_angle_ = std::abs(delta);
        return false;
        }

        if(current_direction != scan_direction_)
        {
        scan_direction_ = current_direction;
        accumulated_sweep_angle_ = 0.0;
        return true;
        }

        accumulated_sweep_angle_ += std::abs(delta);
        if(wrapped_forward ||
        accumulated_sweep_angle_ >=
            (2.0 * M_PI - std::max(sweep_completion_margin_, 2.0 * angle_epsilon_)))
        {
        accumulated_sweep_angle_ = 0.0;
        return true;
        }

        return false;
    }

    void Ping360ImageCompensation::resetSweep(const Pose2D &reference_pose, const rclcpp::Time &stamp)
    {
        reference_pose_ = reference_pose;
        std::fill(image_.data.begin(), image_.data.end(), 0);
        image_.header.set__frame_id(output_frame_);
        publishOutputFrameTransform(reference_pose, stamp);
    }

    void Ping360ImageCompensation::publishOutputFrameTransform(
        const Pose2D &pose,
        const rclcpp::Time &stamp)
    {
        if(!tf_broadcaster_)
        return;

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = output_frame_;
        transform.transform.translation.x = pose.x;
        transform.transform.translation.y = pose.y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = std::sin(pose.yaw * 0.5);
        transform.transform.rotation.w = std::cos(pose.yaw * 0.5);
        tf_broadcaster_->sendTransform(transform);
    }

    void Ping360ImageCompensation::publishImage()
    {
        const auto stamp = last_echo_stamp_.nanoseconds() == 0 ? now() : last_echo_stamp_;
        image_.header.set__stamp(stamp);
        image_.header.set__frame_id(output_frame_);
        if(reference_pose_)
        publishOutputFrameTransform(*reference_pose_, stamp);
        image_pub_.publish(image_);
    }
}
