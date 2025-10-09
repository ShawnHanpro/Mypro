#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <cmath>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct ImuData {
    uint64_t           timestamp_ns;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d    angular_velocity;
    Eigen::Vector3d    linear_acceleration;
};

struct OdomData {
    uint64_t           timestamp_ns;
    Eigen::Vector3d    position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d    linear_velocity;
    Eigen::Vector3d    angular_velocity;
};

class LidarCalibrationNode : public rclcpp::Node {
public:
    LidarCalibrationNode() : Node("lidar_calibration_node") {
        RCLCPP_INFO(this->get_logger(), "Lidar Calibration Node has been started.");
        // init
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarCalibrationNode::laser_callback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&LidarCalibrationNode::odom_callback, this, _1));
        // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     "/imu", 10, std::bind(&LidarCalibrationNode::imu_callback, this,
        //     _1));

        corrected_scan_pub_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>("/corrected_scan", 10);

        corrected_point_cloud_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_point_cloud", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
        std::lock_guard<std::mutex> lock(laser_mutex_);
        laser_buffer_.push_back(laser_msg);
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_queue_.push_back(odom_msg);
            if (odom_queue_.size() > 1000) {
                odom_queue_.pop_front();
            }
        }

        ProcessData();
    }
    
    void ProcessData() {
        std::lock_guard<std::mutex> laser_lock(laser_mutex_);
        if (laser_buffer_.empty()) {
            return;
        }

        // Copy odom queue to avoid holding the lock for a long time
        std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_vector;
        {
            std::lock_guard<std::mutex> odom_lock(odom_mutex_);
            if (odom_queue_.empty()) return;
            odom_vector = odom_queue_;
        }

        // Process as many scans as we have data for
        while (!laser_buffer_.empty()) {
            const auto& laser_msg = laser_buffer_.front();
            
            // Calculate the end time of the laser scan
            double scan_duration_ns = laser_msg->time_increment > 0 ? 
                                      laser_msg->time_increment * laser_msg->ranges.size() * 1e9 : 
                                      166.6 * 1e6;
            uint64_t laser_start_ns = laser_msg->header.stamp.sec * 1e9 + laser_msg->header.stamp.nanosec;
            uint64_t laser_end_ns = laser_start_ns + static_cast<uint64_t>(scan_duration_ns);
            uint64_t last_odom_ns = odom_vector.back()->header.stamp.sec * 1e9 + odom_vector.back()->header.stamp.nanosec;

            // If the last odom message is older than the end of the scan, we can't process yet.
            if (last_odom_ns < laser_end_ns) {
                // RCLCPP_INFO(this->get_logger(), "Waiting for odom data. Laser end: %lu, last odom: %lu", laser_end_ns, last_odom_ns);
                break;
            }

            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped =
                    tf_buffer_->lookupTransform("base_link", laser_msg->header.frame_id,
                                                laser_msg->header.stamp, tf2::durationFromSec(0.1));
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s. Discarding scan.", ex.what());
                laser_buffer_.pop_front();
                continue;
            }
            
            std::vector<OdomData> odom_data_vector;
            for (auto odom : odom_vector) {
                OdomData odom_data;
                odom_data.timestamp_ns = odom->header.stamp.sec * 1e9 + odom->header.stamp.nanosec;
                odom_data.position << odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z;
                odom_data.orientation = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
                odom_data.linear_velocity << odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;
                odom_data.angular_velocity << odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z;
                odom_data_vector.push_back(odom_data);
            }

            sensor_msgs::msg::LaserScan corrected_laser = UndistortLaserFrame(laser_msg, odom_data_vector);
            
            corrected_scan_pub_->publish(corrected_laser);

            laser_buffer_.pop_front();
        }
    }

    // void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    //     RCLCPP_INFO(this->get_logger(), "Received IMU message.");
    //     // todo...
    // }

    template <typename T>
    size_t FindClosestTimestampIndex(uint64_t              target_timestamp,
                                    const std::vector<T>& data_vec) {
        if (data_vec.empty()) {
            return -1;
        }

        auto it = std::lower_bound(
            data_vec.begin(), data_vec.end(), target_timestamp,
            [](const T& data, uint64_t ts) { return data.timestamp_ns < ts; });

        if (it == data_vec.end()) {
            return data_vec.size() - 1;
        }
        if (it == data_vec.begin()) {
            return 0;
        }

        size_t idx = std::distance(data_vec.begin(), it);
        if ((target_timestamp - (it - 1)->timestamp_ns) <
            (it->timestamp_ns - target_timestamp)) {
            return idx - 1;
        } else {
            return idx;
        }
    }

    Eigen::Quaterniond slerp(const Eigen::Quaterniond& q1,
                            const Eigen::Quaterniond& q2, double t) {
        return q1.slerp(t, q2);
    }

    Eigen::Vector3d lerp(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
                        double t) {
        return v1 + t * (v2 - v1);
    }

    // Interpolate odom data
    OdomData InterpolateOdom(uint64_t                     target_timestamp,
                            const std::vector<OdomData>& odom_data_vec) {
        if (odom_data_vec.empty()) {
            return {};
        }
        size_t idx = FindClosestTimestampIndex(target_timestamp, odom_data_vec);
        
        if (idx == 0 && target_timestamp < odom_data_vec[idx].timestamp_ns) {
            return odom_data_vec.front();
        }
        if (idx == odom_data_vec.size() - 1 && target_timestamp > odom_data_vec[idx].timestamp_ns) {
            return odom_data_vec.back();
        }
        
        OdomData odom1, odom2;
        if (odom_data_vec[idx].timestamp_ns <= target_timestamp) {
            odom1 = odom_data_vec[idx];
            odom2 = odom_data_vec[idx+1];
        } else {
            odom1 = odom_data_vec[idx-1];
            odom2 = odom_data_vec[idx];
        }

        if (odom1.timestamp_ns == odom2.timestamp_ns) {
            return odom1;
        }

        double t = static_cast<double>(target_timestamp - odom1.timestamp_ns) /
                (odom2.timestamp_ns - odom1.timestamp_ns);

        OdomData interpolated_odom;
        interpolated_odom.timestamp_ns = target_timestamp;
        interpolated_odom.position = lerp(odom1.position, odom2.position, t);
        interpolated_odom.orientation = slerp(odom1.orientation, odom2.orientation, t);
        interpolated_odom.linear_velocity = lerp(odom1.linear_velocity, odom2.linear_velocity, t);
        interpolated_odom.angular_velocity = lerp(odom1.angular_velocity, odom2.angular_velocity, t);

        return interpolated_odom;
    }

    // Interpolation distortion correction
    sensor_msgs::msg::LaserScan UndistortLaserFrame(const sensor_msgs::msg::LaserScan::SharedPtr& laser_frame,
                            const std::vector<OdomData>& odom_data_vec) {
        if (laser_frame->ranges.empty() || odom_data_vec.empty()) {
            return *laser_frame;
        }

        sensor_msgs::msg::LaserScan laser;
        laser.header = laser_frame->header;
        laser.angle_min = laser_frame->angle_min;
        laser.angle_max = laser_frame->angle_max;
        laser.angle_increment = laser_frame->angle_increment;
        laser.time_increment = laser_frame->time_increment;
        laser.scan_time = laser_frame->scan_time;
        laser.range_min = laser_frame->range_min;
        laser.range_max = laser_frame->range_max;
        laser.intensities = laser_frame->intensities;
        
        size_t scan_size = laser_frame->ranges.size();
        laser.ranges.assign(scan_size, std::numeric_limits<float>::infinity());

        uint64_t laser_start_ns = static_cast<uint64_t>(laser.header.stamp.sec) * 1000000000ULL + laser.header.stamp.nanosec;
        OdomData odom_start = InterpolateOdom(laser_start_ns, odom_data_vec);

        double scan_duration_ns = laser_frame->time_increment > 0 ? 
                    laser_frame->time_increment * laser_frame->ranges.size() * 1e9 : 
                    166.6 * 1e6;
        laser.header.stamp = rclcpp::Time(laser_start_ns + scan_duration_ns/2, RCL_ROS_TIME);

        // Eigen::Affine3d T_world_to_robot_start = Eigen::Translation3d(odom_start.position) * Eigen::Quaterniond(odom_start.orientation);
        Eigen::Affine3d T_world_to_robot_start(odom_start.orientation);
        T_world_to_robot_start.pretranslate(odom_start.position);
        Eigen::Affine3d T_robot_start_to_world = T_world_to_robot_start.inverse();

        for (size_t i = 0; i < scan_size; ++i) {

            float range = laser_frame->ranges[i];
            Eigen::Vector3d raw_point;
            if (std::isfinite(range) && range > laser.range_min && range < laser.range_max) {
                float angle = laser.angle_min + i * laser.angle_increment;
                raw_point = Eigen::Vector3d(range * std::cos(angle), range * std::sin(angle), 0.0);
            } else {
                raw_point = Eigen::Vector3d::Zero();
            }

            if (raw_point.isZero()){
                continue;
            }

            uint64_t point_timestamp_ns = laser_start_ns + static_cast<uint64_t>(laser.time_increment * 1e9 * i);
            OdomData odom_point = InterpolateOdom(point_timestamp_ns, odom_data_vec);

            // Eigen::Affine3d T_world_to_robot_point = Eigen::Translation3d(odom_point.position) * Eigen::Quaterniond(odom_point.orientation);
            Eigen::Affine3d T_world_to_point(odom_point.orientation);
            T_world_to_point.pretranslate(odom_point.position);
            
            Eigen::Affine3d T_robot_start_to_point = T_robot_start_to_world * T_world_to_point;
            Eigen::Vector3d corrected_point = T_robot_start_to_point.inverse() * raw_point;
            
            float corrected_angle = atan2(corrected_point.y(), corrected_point.x());
            if (corrected_angle < 0) {
                corrected_angle += 2.0 * M_PI;
            }
            float corrected_range = hypot(corrected_point.x(), corrected_point.y());
            int index =  static_cast<int>((corrected_angle - laser.angle_min) / laser.angle_increment);
            if (index >= 0 && index < laser.ranges.size()) {
                if (corrected_range < laser.ranges[index]) {
                    laser.ranges[index] = corrected_range;
                }
            }
        }

        return laser;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    corrected_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  corrected_point_cloud_pub_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::mutex odom_mutex_;
    std::mutex laser_mutex_;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_queue_;
    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> laser_buffer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCalibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}