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

// 定义IMU数据结构
struct ImuData {
    uint64_t           timestamp_ns;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d    angular_velocity;
    Eigen::Vector3d    linear_acceleration;
};

// 定义Odometry数据结构
struct OdomData {
    uint64_t           timestamp_ns;
    Eigen::Vector3d    position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d    linear_velocity;
    Eigen::Vector3d    angular_velocity;
};

// 定义Laser帧数据结构
struct LaserFrame {
    uint64_t timestamp_ns;
    double   time_increment;  // Time increment between measurements
    double   angle_min;
    double   angle_max;
    double   angle_increment;
    double   range_min;
    double   range_max;
    int      points_count;
    // std::vector<cv::Point2f>     points;         // For 2D display initially
    std::vector<Eigen::Vector3d> raw_points_3d;  // Raw 3D points
    std::vector<Eigen::Vector3d> corrected_points_3d;  // Corrected 3D points
};

class LidarCalibrationNode : public rclcpp::Node {
public:
    LidarCalibrationNode() : Node("lidar_calibration_node") {
        RCLCPP_INFO(this->get_logger(), "Lidar Calibration Node has been started.");
        // 初始化订阅者、发布者等
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarCalibrationNode::laser_callback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&LidarCalibrationNode::odom_callback, this, _1));
        // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     "/imu", 10, std::bind(&LidarCalibrationNode::imu_callback, this,
        //     _1));

        point_cloud_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);

        corrected_scan_pub_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>("/corrected_scan", 10);

        // 初始化 TF2 buffer和listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
        // 获取odom到laser的变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped =
                tf_buffer_->lookupTransform("base_link", laser_msg->header.frame_id,
                                            laser_msg->header.stamp, tf2::durationFromSec(0.1));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        // 获取odom数据
        std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_vector;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_vector = odom_queue_;
        }

        LaserFrame laser_frame;

        laser_frame.timestamp_ns = laser_msg->header.stamp.sec * 1e9 + laser_msg->header.stamp.nanosec;
        laser_frame.time_increment = laser_msg->time_increment;
        laser_frame.angle_min = laser_msg->angle_min;
        laser_frame.angle_max = laser_msg->angle_max;
        laser_frame.angle_increment = laser_msg->angle_increment;
        laser_frame.range_min = laser_msg->range_min;
        laser_frame.range_max = laser_msg->range_max;
        laser_frame.points_count = laser_msg->ranges.size();

        for (size_t i = 0; i < laser_msg->ranges.size(); ++i) {
            double range = laser_msg->ranges[i];
            if (std::isfinite(range) && range > laser_msg->range_min &&
                range < laser_msg->range_max) {
                double angle = laser_msg->angle_min + i * laser_msg->angle_increment;
                Eigen::Vector3d point(range * std::cos(angle), range * std::sin(angle), 0.0);
                laser_frame.raw_points_3d.push_back(point);
            } else {
                // 无效点
                laser_frame.raw_points_3d.push_back(Eigen::Vector3d::Zero());
            }
        }

        std::vector<OdomData> odom_data_vector;
        for (auto odom : odom_vector) {
            OdomData odom_data;
            odom_data.timestamp_ns = odom->header.stamp.sec * 1e9 + odom->header.stamp.nanosec;
            odom_data.position.x() = odom->pose.pose.position.x;
            odom_data.position.y() = odom->pose.pose.position.y;
            odom_data.position.z() = odom->pose.pose.position.z;
            odom_data.orientation.w() = odom->pose.pose.orientation.w;
            odom_data.orientation.x() = odom->pose.pose.orientation.x;
            odom_data.orientation.y() = odom->pose.pose.orientation.y;
            odom_data.orientation.z() = odom->pose.pose.orientation.z;
            odom_data.linear_velocity.x() = odom->twist.twist.linear.x;
            odom_data.linear_velocity.y() = odom->twist.twist.linear.y;
            odom_data.linear_velocity.z() = odom->twist.twist.linear.z;
            odom_data.angular_velocity.x() = odom->twist.twist.angular.x;
            odom_data.angular_velocity.y() = odom->twist.twist.angular.y;
            odom_data.angular_velocity.z() = odom->twist.twist.angular.z;

            // 插值odom数据
            // if (odom_data.timestamp_ns <= laser_frame.timestamp_ns) {
            odom_data_vector.push_back(odom_data);
            // }
        }

        // 检查odom数据是否足够覆盖整个激光扫描时间
        double scan_duration_ns = laser_frame.time_increment > 0 ? laser_frame.time_increment * laser_frame.points_count * 1e9 : 166.6 * 1e6;
        uint64_t laser_start = laser_frame.timestamp_ns;
        uint64_t laser_end = laser_frame.timestamp_ns + static_cast<uint64_t>(scan_duration_ns);

        RCLCPP_WARN(this->get_logger(), "Laser start: %lu, laser end: %lu, last odom: %lu",
                    laser_start,
                    laser_end,
                    odom_data_vector.back().timestamp_ns);

        // 1. 将收到的激光帧放入缓冲区
        // {
        //     std::lock_guard<std::mutex> lock(laser_mutex_); // 假设你添加了一个laser_mutex_
        //     laser_buffer_.push_back(laser_msg);
        // }

        UndistortLaserFrame(laser_frame, odom_data_vector);

        // 创建PointCloud2消息
        auto point_cloud2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        point_cloud2_msg->header.stamp = laser_msg->header.stamp;
        point_cloud2_msg->header.frame_id = "base_link";
        point_cloud2_msg->height = 1;
        point_cloud2_msg->width = laser_msg->ranges.size();
        point_cloud2_msg->is_dense = false;

        // 设置PointCloud2的字段
        sensor_msgs::PointCloud2Modifier modifier(*point_cloud2_msg);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                      sensor_msgs::msg::PointField::FLOAT32);

        // 迭代器
        sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud2_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud2_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud2_msg, "z");

        /*
        rclcpp::Time start_time, end_time;
        start_time = laser_msg->header.stamp;
        int laser_size = laser_msg->ranges.size();
        end_time =
            start_time + rclcpp::Duration::from_seconds(laser_size * laser_msg->time_increment);

        std::vector<double> angles, ranges;
        for (int i = 0; i < laser_size; ++i) {
            double range = laser_msg->ranges[i];

            // 判断是否为无效点
            if (std::isfinite(range) && range > laser_msg->range_min &&
                range < laser_msg->range_max) {
                double angle = laser_msg->angle_min + i * laser_msg->angle_increment;

                // 计算每个点的采样时间
                rclcpp::Time point_stamp =
                    start_time + rclcpp::Duration::from_seconds(i * laser_msg->time_increment);

                // 插值odom数据
                auto interp_odom = InterpolateOdometry(odom_vector, point_stamp);
                if (!interp_odom) {
                    RCLCPP_WARN(this->get_logger(), "No valid odometry data for time: %f",
                                point_stamp.seconds());
                    *iter_x = nanf("");
                    *iter_y = nanf("");
                    *iter_z = nanf("");
                    ++iter_x;
                    ++iter_y;
                    ++iter_z;
                    continue;
                }

                // 激光在base_laser坐标系下的点
                Eigen::Vector3d raw_point(range * std::cos(angle), range * std::sin(angle), 0.0);

                // 插值后的odom位姿
                Eigen::Vector3d    odom_position(interp_odom->pose.pose.position.x,
                                                 interp_odom->pose.pose.position.y,
                                                 interp_odom->pose.pose.position.z);
                Eigen::Quaterniond odom_orientation(
                    interp_odom->pose.pose.orientation.w, interp_odom->pose.pose.orientation.x,
                    interp_odom->pose.pose.orientation.y, interp_odom->pose.pose.orientation.z);

                // 将激光点从base_laser坐标系转换到odom坐标系
                Eigen::Vector3d corrected_point = odom_orientation * raw_point + odom_position;

                // geometry_msgs::msg::PointStamped point_from_laser;
                // point_from_laser.header.frame_id =
                // laser_msg->header.frame_id; point_from_laser.header.stamp =
                // laser_msg->header.stamp; point_from_laser.point.x = range *
                // std::cos(angle); point_from_laser.point.y = range *
                // std::sin(angle); point_from_laser.point.z = 0.2;

                // // 转换坐标系
                // geometry_msgs::msg::PointStamped point_to_odom;
                // tf2::doTransform(point_from_laser, point_to_odom,
                //                  transform_stamped);

                // 填充PointCloud2消息
                *iter_x = corrected_point.x();
                *iter_y = corrected_point.y();
                *iter_z = 0.2;
            } else {
                // 如果是NaN或无效点，填充为0
                *iter_x = nanf("");
                *iter_y = nanf("");
                *iter_z = nanf("");
            }

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
        */
        // Pointcloud2Laserscan(point_cloud2_msg, laser_msg);

        for (int i = 0; i < laser_frame.corrected_points_3d.size(); ++i) {
            *iter_x = laser_frame.corrected_points_3d[i].x();
            *iter_y = laser_frame.corrected_points_3d[i].y();
            *iter_z = 0.4;

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        point_cloud_pub_->publish(*point_cloud2_msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_queue_.push_back(odom_msg);
        if (odom_queue_.size() > 1000) {
            odom_queue_.pop_front();
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received IMU message.");
        // 具体处理逻辑...
    }

    // 查找最近时间戳
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
        // Compare with the previous element to find the closest
        if ((target_timestamp - (it - 1)->timestamp_ns) <
            (it->timestamp_ns - target_timestamp)) {
            return idx - 1;
        } else {
            return idx;
        }
    }

    // Slerp for Quaternion (for orientation interpolation)
    Eigen::Quaterniond slerp(const Eigen::Quaterniond& q1,
                            const Eigen::Quaterniond& q2, double t) {
        return q1.slerp(t, q2);
    }

    // Linear interpolation for Vector3d (for position/velocity)
    Eigen::Vector3d lerp(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
                        double t) {
        return v1 + t * (v2 - v1);
    }

    // 插值imu数据
    ImuData interpolateImu(uint64_t                    target_timestamp,
                        const std::vector<ImuData>& imu_data_vec) {
        size_t idx = FindClosestTimestampIndex(target_timestamp, imu_data_vec);

        if (idx == -1) {  // No IMU data
            return {};
        }

        if (imu_data_vec.size() == 1 ||
            imu_data_vec[idx].timestamp_ns == target_timestamp) {
            return imu_data_vec[idx];
        }

        ImuData imu1 = imu_data_vec[idx];
        ImuData imu2;

        if (imu1.timestamp_ns < target_timestamp && idx + 1 < imu_data_vec.size()) {
            imu2 = imu_data_vec[idx + 1];
        } else if (imu1.timestamp_ns > target_timestamp && idx > 0) {
            imu2 = imu_data_vec[idx - 1];
            imu1 = imu_data_vec[idx];  // Swap to ensure imu1 is earlier
        } else {
            return imu_data_vec[idx];  // Should not happen if idx is valid and not
                                    // exact match
        }

        if (imu1.timestamp_ns == imu2.timestamp_ns) {  // Avoid division by zero
            return imu1;
        }

        double t = static_cast<double>(target_timestamp - imu1.timestamp_ns) /
                (imu2.timestamp_ns - imu1.timestamp_ns);

        ImuData interpolated_imu;
        interpolated_imu.timestamp_ns = target_timestamp;
        interpolated_imu.orientation = slerp(imu1.orientation, imu2.orientation, t);
        interpolated_imu.angular_velocity =
            lerp(imu1.angular_velocity, imu2.angular_velocity, t);
        interpolated_imu.linear_acceleration =
            lerp(imu1.linear_acceleration, imu2.linear_acceleration, t);

        return interpolated_imu;
    }

    // 插值odom数据
    OdomData InterpolateOdom(uint64_t                     target_timestamp,
                            const std::vector<OdomData>& odom_data_vec) {
        size_t idx = FindClosestTimestampIndex(target_timestamp, odom_data_vec);

        if (idx == -1) {
            return {};
        }

        if (odom_data_vec.size() == 1 ||
            odom_data_vec[idx].timestamp_ns == target_timestamp) {
            return odom_data_vec[idx];
        }

        OdomData odom1 = odom_data_vec[idx];
        OdomData odom2;

        if (odom1.timestamp_ns < target_timestamp &&
            idx + 1 < odom_data_vec.size()) {
            odom2 = odom_data_vec[idx + 1];
        } else if (odom1.timestamp_ns > target_timestamp && idx > 0) {
            odom2 = odom_data_vec[idx - 1];
            odom1 = odom_data_vec[idx];  // Swap to ensure odom1 is earlier
        } else {
            return odom_data_vec[idx];  // Should not happen if idx is valid and not
                                        // exact match
        }

        if (odom1.timestamp_ns == odom2.timestamp_ns) {  // Avoid division by zero
            return odom1;
        }

        double t = static_cast<double>(target_timestamp - odom1.timestamp_ns) /
                (odom2.timestamp_ns - odom1.timestamp_ns);

        OdomData interpolated_odom;
        interpolated_odom.timestamp_ns = target_timestamp;
        interpolated_odom.position = lerp(odom1.position, odom2.position, t);
        interpolated_odom.orientation =
            slerp(odom1.orientation, odom2.orientation, t);
        interpolated_odom.linear_velocity =
            lerp(odom1.linear_velocity, odom2.linear_velocity, t);
        interpolated_odom.angular_velocity =
            lerp(odom1.angular_velocity, odom2.angular_velocity, t);

        return interpolated_odom;
    }

    // 插值去畸变
    void UndistortLaserFrame(LaserFrame&                  laser_frame,
                            const std::vector<OdomData>& odom_data_vec) {
        if (laser_frame.raw_points_3d.empty()) {
            return;
        }

        OdomData odom_start =
            InterpolateOdom(laser_frame.timestamp_ns, odom_data_vec);
        if (odom_start.timestamp_ns == 0) {
            std::cerr << "Warning: Could not interpolate Odom for laser frame "
                    << laser_frame.timestamp_ns << std::endl;
            laser_frame.corrected_points_3d =
                laser_frame.raw_points_3d;
            return;
        }

        // Create transformation from world to start pose
        Eigen::Matrix3d R_start = odom_start.orientation.toRotationMatrix();
        Eigen::Vector3d t_start = odom_start.position;
        Eigen::Affine3d T_world_to_robot_start = Eigen::Affine3d::Identity();
        T_world_to_robot_start.translate(t_start);
        T_world_to_robot_start.rotate(R_start);
        Eigen::Affine3d T_robot_start_to_world = T_world_to_robot_start.inverse();

        laser_frame.corrected_points_3d.clear();
        laser_frame.corrected_points_3d.reserve(laser_frame.raw_points_3d.size());

        // 对于不同频率的激光需要使用time_increment变量来计算采集一帧数据的时间
        double scan_duration_ns = 166.666667 * 1000 * 1000;
        if (laser_frame.time_increment > 0) {
            scan_duration_ns = static_cast<double>(laser_frame.points_count) *
                               laser_frame.time_increment * 1e9;
        } else {
            RCLCPP_WARN(this->get_logger(), "Laser scan time_increment is zero or negative, using default duration.");
        }

        double time_per_point_ns =
            scan_duration_ns / laser_frame.raw_points_3d.size();

        for (size_t i = 0; i < laser_frame.raw_points_3d.size(); ++i) {
            Eigen::Vector3d raw_point = laser_frame.raw_points_3d[i];

            // Estimate timestamp for this point
            uint64_t point_timestamp_ns =
                laser_frame.timestamp_ns +
                static_cast<uint64_t>(i * time_per_point_ns);

            // Interpolate Odometry at the point's timestamp
            OdomData odom_point_pose =
                InterpolateOdom(point_timestamp_ns, odom_data_vec);

            // If interpolation failed for some reason, use the start pose or skip
            if (odom_point_pose.timestamp_ns == 0) {
                laser_frame.corrected_points_3d.push_back(raw_point);
                continue;
            }

            // Transformation from laser_start_pose to point_pose_in_world
            Eigen::Matrix3d R_point =
                odom_point_pose.orientation.toRotationMatrix();
            Eigen::Vector3d t_point = odom_point_pose.position;
            Eigen::Affine3d T_world_to_point = Eigen::Affine3d::Identity();
            T_world_to_point.translate(t_point);
            T_world_to_point.rotate(R_point);

            Eigen::Affine3d T_robot_start_to_point = T_robot_start_to_world * T_world_to_point;

            // Apply the inverse transform to correct the point
            Eigen::Vector3d corrected_point =
                T_robot_start_to_point.inverse() * raw_point;
            laser_frame.corrected_points_3d.push_back(corrected_point);
        }
    }

    void Pointcloud2Laserscan(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
                            const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg) {
        float angle_min = scan_msg->angle_min;
        float angle_max = scan_msg->angle_max;
        float angle_increment = scan_msg->angle_increment;
        float range_min = scan_msg->range_min;
        float range_max = scan_msg->range_max;
        int num_ranges = static_cast<int>((angle_max - angle_min) / angle_increment) + 1;

        // 初始化ranges为inf
        std::vector<float> ranges(num_ranges, std::numeric_limits<float>::infinity());

        int valid_points = 0;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // 只处理接近平面的点
            if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::abs(z) > 0.2) continue;

            float range = std::sqrt(x * x + y * y);
            if (range < range_min || range > range_max) continue;

            float angle = std::atan2(y, x);
            int bin = static_cast<int>((angle - angle_min) / angle_increment);
            if (bin < 0 || bin >= num_ranges) continue;

            if (range < ranges[bin]) {
                ranges[bin] = range;
            }
        }

        // 填充scan_msg
        scan_msg->ranges = ranges;
        // scan_msg->header = scan_msg->header; // 时间戳和frame_id用点云的

        // 发布
        corrected_scan_pub_->publish(*scan_msg);
    }

    nav_msgs::msg::Odometry::SharedPtr InterpolateOdometry(
        const std::deque<nav_msgs::msg::Odometry::SharedPtr>& odom_vector,
        const rclcpp::Time&                                   target_time) {
        if (odom_vector.size() < 2) return nullptr;

        // 早于第一帧
        if (target_time <= odom_vector.front()->header.stamp) {
            return odom_vector.front();
        }

        // 晚于最后一帧
        if (target_time >= odom_vector.back()->header.stamp) {
            return odom_vector.back();
        }

        for (size_t i = 1; i < odom_vector.size(); ++i) {
            rclcpp::Time t0 = odom_vector[i - 1]->header.stamp;
            rclcpp::Time t1 = odom_vector[i]->header.stamp;
            if (t0 <= target_time && t1 >= target_time) {
                double t = (target_time - t0).seconds() / (t1 - t0).seconds();

                // 线性插值position
                geometry_msgs::msg::Point p0 = odom_vector[i - 1]->pose.pose.position;
                geometry_msgs::msg::Point p1 = odom_vector[i]->pose.pose.position;
                geometry_msgs::msg::Point p;
                p.x = p0.x + t * (p1.x - p0.x);
                p.y = p0.y + t * (p1.y - p0.y);
                p.z = p0.z + t * (p1.z - p0.z);

                // 四元数插值 orientation
                // auto& q0 = odom_vector[i-1]->pose.pose.orientation;
                // auto& q1 = odom_vector[i]->pose.pose.orientation;
                // Eigen::Quaterniond eq0(q0.w, q0.x, q0.y, q0.z);
                // Eigen::Quaterniond eq1(q1.w, q1.x, q1.y, q1.z);
                // Eigen::Quaterniond eq = eq0.slerp(t, eq1);
                // geometry_msgs::msg::Quaternion q;
                // q.x = eq.x(); q.y = eq.y(); q.z = eq.z(); q.w = eq.w();

                // 线性插值orientation（简单插值，不考虑四元数插值）
                geometry_msgs::msg::Quaternion q0 = odom_vector[i - 1]->pose.pose.orientation;
                geometry_msgs::msg::Quaternion q1 = odom_vector[i]->pose.pose.orientation;
                geometry_msgs::msg::Quaternion q;
                q.x = q0.x + t * (q1.x - q0.x);
                q.y = q0.y + t * (q1.y - q0.y);
                q.z = q0.z + t * (q1.z - q0.z);
                q.w = q0.w + t * (q1.w - q0.w);

                // 构造插值后的odom消息
                auto interp_odom = std::make_shared<nav_msgs::msg::Odometry>();
                interp_odom->header.stamp = target_time;
                interp_odom->header.frame_id = odom_vector[i]->header.frame_id;
                interp_odom->pose.pose.position = p;
                interp_odom->pose.pose.orientation = q;
                return interp_odom;
            }
        }

        // 如果没有找到合适的插值点，返回nullptr
        return nullptr;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    corrected_scan_pub_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex                                     odom_mutex_;  // 用于保护odom数据的互斥锁
    std::mutex                                     laser_mutex_; // 用于保护激光数据的互斥锁
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_queue_;  // 用于存储最新的odom数据
    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> laser_buffer_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCalibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"