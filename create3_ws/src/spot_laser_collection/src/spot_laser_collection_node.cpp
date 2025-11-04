#include <cmath>
#include <iostream>
#include <vector>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class VirtualLaser : public rclcpp::Node {
    struct Pose2D;
public:
    VirtualLaser() : Node("virtual_laser") {

        // 声明参数并加载默认值
        this->declare_parameter<bool>("enable_point_interpolation_compensation", false);
        this->get_parameter("enable_point_interpolation_compensation", enable_point_interpolation_compensation_);
        this->declare_parameter<bool>("save_point_laser_points", false);
        this->get_parameter("save_point_laser_points", save_point_laser_points_);
        this->declare_parameter<bool>("publish_cloud", true);
        this->get_parameter("publish_cloud", publish_cloud_);
        this->declare_parameter<bool>("publish_scan", true);
        this->get_parameter("publish_scan", publish_scan_);
        if (enable_point_interpolation_compensation_) RCLCPP_INFO(this->get_logger(), "enable_point_interpolation_compensation = \033[1;32m[true]\033[0m");
        else RCLCPP_INFO(this->get_logger(), "enable_point_interpolation_compensation = \033[1;31m[false]\033[0m");
        if (save_point_laser_points_) RCLCPP_INFO(this->get_logger(), "save_point_laser_points = \033[1;32m[true]\033[0m");
        else RCLCPP_INFO(this->get_logger(), "save_point_laser_points = \033[1;31m[false]\033[0m");
        if (publish_cloud_) RCLCPP_INFO(this->get_logger(), "publish_cloud = \033[1;32m[true]\033[0m");
        else RCLCPP_INFO(this->get_logger(), "publish_cloud = \033[1;31m[false]\033[0m");
        if (publish_scan_) RCLCPP_INFO(this->get_logger(), "publish_scan = \033[1;32m[true]\033[0m");
        else RCLCPP_INFO(this->get_logger(), "publish_scan = \033[1;31m[false]\033[0m");

        imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        point_laser_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 指定每个回调属于哪个组
        rclcpp::SubscriptionOptions imu_opts;
        imu_opts.callback_group = imu_group_;

        rclcpp::SubscriptionOptions point_laser_opts;
        point_laser_opts.callback_group = point_laser_group_;

        point_laser_sub_ = this->create_subscription<sensor_msgs::msg::Range>("point_laser", 10, std::bind(&VirtualLaser::range_callback, this, _1), point_laser_opts);
        scan_laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&VirtualLaser::scan_callback, this, _1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS(), std::bind(&VirtualLaser::imu_callback, this, _1), imu_opts);
        // wheel_vels_sub_ = this->create_subscription<irobot_create_msgs::msg::WheelVels>("wheel_vels", rclcpp::SensorDataQoS(),
        //                                                                                 std::bind(&VirtualLaser::wheel_vels_callback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(),
                                                                       std::bind(&VirtualLaser::odom_callback, this, _1));
        cal_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_vels_odom", rclcpp::SensorDataQoS());
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("combined_point_laser_cloud", 10);
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("combined_point_laser_scan", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), std::chrono::seconds(5));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(this->get_logger(), "start spot laser");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 找第一个有效的点（range 有效且非 inf）
        int idx = -1;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (std::isfinite(msg->ranges[i]) &&
                msg->ranges[i] >= msg->range_min &&
                msg->ranges[i] <= msg->range_max) {
                idx = static_cast<int>(i);
                break;
            }
        }

        if (idx == -1) {
            RCLCPP_WARN(this->get_logger(), "No valid laser point found");
            return;
        }

        scan_range_ = msg->ranges[idx];

#if 0
        float range = msg->ranges[idx];
        float angle = msg->angle_min + idx * msg->angle_increment;

        // Laser 坐标系下的第一个点坐标（假设激光在平面上，z=0）
        geometry_msgs::msg::PointStamped point_in_laser;
        point_in_laser.header.frame_id = msg->header.frame_id;
        point_in_laser.header.stamp = msg->header.stamp;
        point_in_laser.point.x = range * std::cos(angle);
        point_in_laser.point.y = range * std::sin(angle);
        point_in_laser.point.z = 0.0;

        try {
            // 查找 TF：odom <- laser_frame
            geometry_msgs::msg::TransformStamped tf_laser_to_odom =
                tf_buffer_->lookupTransform("odom", msg->header.frame_id, rclcpp::Time(0));

            // 执行变换
            geometry_msgs::msg::PointStamped point_in_odom;
            tf2::doTransform(point_in_laser, point_in_odom, tf_laser_to_odom);

            RCLCPP_INFO(this->get_logger(), "Range=%.3f  ->  Odom: (%.3f, %.3f)  Yaw=%.3f",
                        range,
                        point_in_odom.point.x,
                        point_in_odom.point.y,
                        angle);
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
#endif
#if 0

        float x0, y0;
        {
            std::lock_guard<std::mutex> lock2(odom_mutex_);
            x0 = odom_x_;
            y0 = odom_y_;
        }

        float yaw;
        {
            std::lock_guard<std::mutex> lock(yaw_mutex_);
            yaw = yaw_;
        }
        float point_in_laser_x = range * std::cos(angle);
        float point_in_laser_y = range * std::sin(angle);
        float x_odom = x0 + cos(yaw) * point_in_laser_x - sin(yaw) * point_in_laser_y;
        float y_odom = y0 + sin(yaw) * point_in_laser_x + cos(yaw) * point_in_laser_y;
        RCLCPP_INFO(this->get_logger(), "cal odom: %f, %f", x_odom, y_odom);
#endif
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        if (yaw < 0) {
            yaw = 2 * M_PI + yaw;
        }

        {
            std::lock_guard<std::mutex> lock(yaw_mutex_);
            yaw_ = yaw;
            angular_z_ = msg->angular_velocity.z;
        }
        
        // RCLCPP_INFO(this->get_logger(), "imu angular_z_: %f", angular_z_);
    }

    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg) {

        // debug cal point in odom
        if (std::isnan(msg->range) || msg->range <= msg->min_range || msg->range >= msg->max_range) {
            RCLCPP_WARN(this->get_logger(), "Invalid range: %.3f", msg->range);
            return;
        }

        float yaw, angular_z;
        {
            std::lock_guard<std::mutex> lock(yaw_mutex_);
            yaw = yaw_;
            angular_z = angular_z_;
        }
        float x0, y0;
        {
            std::lock_guard<std::mutex> lock2(odom_mutex_);
            x0 = odom_x_;
            y0 = odom_y_;
        }

        // 点在laser下的坐标
        float theta = 0.0;
        float point_in_laser_x = msg->range * std::cos(theta);
        float point_in_laser_y = msg->range * std::sin(theta);

        // laser->base安装位置偏移计算
        // float delta_range = scan_range_ - msg->range;
        float delta_range = 0.15;
        float laser_dyaw = 0.01745329;
        float laser_dx = delta_range;
        float laser_dy = 0.0;
        float cl = std::cos(laser_dyaw);
        float sl = std::sin(laser_dyaw);
        float x_base = laser_dx + (cl * point_in_laser_x - sl * point_in_laser_y);
        float y_base = laser_dy + (sl * point_in_laser_x + cl * point_in_laser_y);
        Pose2D temp_point(x_base, y_base);

        // base->odom
        float co = std::cos(yaw);
        float so = std::sin(yaw);
        float x_odom = x0 + (co * x_base - so * y_base);
        float y_odom = y0 + (so * x_base + co * y_base);
        Pose2D point(x_odom, y_odom);

        // RCLCPP_INFO(this->get_logger(), "delta_range:%.3f, x0:%.3f, y0:%.3f", delta_range, x0, y0);
        // RCLCPP_INFO(this->get_logger(), "point laser x:%.3f y:%.3f", msg->range * std::cos(yaw), msg->range * std::sin(yaw));
        // RCLCPP_INFO(this->get_logger(), "range=%.3f  ->  odom: (%.3f, %.3f)  yaw=%.3f",
        //     msg->range, x_odom, y_odom, yaw);

        // main
        static float last_yaw = 0.0f;

        if (!collecting_) {
            collecting_ = true;
            last_yaw = yaw;
            points_.clear();
            RCLCPP_INFO(this->get_logger(), "start collecting range data");
        }

        // 需要根据tf获取的两个角度之间进行差值处理
        float delta_yaw = std::fabs(yaw - last_yaw);
        static std::vector<Pose2D> temp_points;

        if (enable_point_interpolation_compensation_) {
            if (delta_yaw == 0 && std::fabs(angular_z) > 0.01) {
                temp_points.push_back(temp_point);
            }
        }
        // RCLCPP_INFO(this->get_logger(), "range yaw: %f", yaw);
        // RCLCPP_INFO(this->get_logger(), "delta_yaw: %f", delta_yaw);
        // RCLCPP_INFO(this->get_logger(), "temp_points: %ld", temp_points.size());
        // RCLCPP_INFO(this->get_logger(), "angular_z: %f", angular_z);

        if (delta_yaw >= angle_step_rad_) {
            if (!temp_points.empty()) {
                float step_angel = delta_yaw / temp_points.size();
                for (size_t i = 0; i < temp_points.size(); ++i) {

                    // base->odom
                    float co_temp = std::cos(last_yaw + step_angel * (i+1));
                    float so_temp = std::sin(last_yaw + step_angel * (i+1));
                    float x_odom_temp = x0 + (co_temp * temp_points[i].x - so_temp * temp_points[i].y);
                    float y_odom_temp = y0 + (so_temp * temp_points[i].x + co_temp * temp_points[i].y);
                    points_.push_back({x_odom_temp, y_odom_temp});
                }

                temp_points.clear();
            }
            points_.push_back(point);
        }

        float diff = yaw - last_yaw;
        if (std::fabs(diff) > M_PI && diff < 0) {
            diff = 2 * M_PI + diff;
        } else if (std::fabs(diff) > M_PI && diff > 0) {
            diff = 2 * M_PI - diff;
        }

        yaw_continue_ += diff;

        if (!collecting_) return;

        if (std::fabs(yaw_continue_) >= 2 * M_PI) {
            if(publish_cloud_) PublishCloud(msg);
            if(publish_scan_) PublishScan(msg);
            if(save_point_laser_points_) SavePointsToTxt(points_, "/home/shan2/Mypro/create3_ws/src/spot_laser_collection/data/point_laser.txt");
            RCLCPP_INFO(this->get_logger(), "point size: \033[1;32m%ld\033[0m", points_.size());
            points_.clear();
            collecting_ = false;
            yaw_continue_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "publish full scan");
        }

        last_yaw = yaw;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        float yaw = GetYawFromOdom(msg);
        if (yaw < 0) {
            yaw = 2 * M_PI + yaw;
        }
        {
            std::lock_guard<std::mutex> lock2(odom_mutex_);
            odom_x_ = msg->pose.pose.position.x;
            odom_y_ = msg->pose.pose.position.y;
        }
        // RCLCPP_INFO(this->get_logger(), "odom yaw: %f", yaw);
    }

    void wheel_vels_callback(const irobot_create_msgs::msg::WheelVels::SharedPtr msg) {
        float wheel_base = 0.235;  // m
        float wheel_R = 0.036;     // m

        rclcpp::Time current_time = now();
        float        dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        float v_l = msg->velocity_left * wheel_R;
        float v_r = msg->velocity_right * wheel_R;

        float v = (v_l + v_r) / 2.0;
        float angular = (v_r - v_l) / wheel_base;

        float delta_x, delta_y;
        // float delta_theta;

        if (std::fabs(angular) < 1e-6) {
            delta_x = v * cos(yaw_) * dt;
            delta_y = v * sin(yaw_) * dt;
        } else {
            delta_x = (v / angular) * (sin(yaw_ + angular * dt) - sin(yaw_));
            delta_y = (v / angular) * (-cos(yaw_ + angular * dt) + cos(yaw_));
        }
        // delta_theta = angular * dt;

        x_ += delta_x;
        y_ += delta_y;
        // theta_ += delta_theta;

        // RCLCPP_INFO(this->get_logger(), "cal odom: %f, %f, %f", x_, y_, yaw_);

        // pub cal odom
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "wheel_vels_odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
        odom.pose.pose.orientation.w = cos(yaw_ / 2.0);
        cal_odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg->header.stamp;
        tf.header.frame_id = "wheel_vels_odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    void PublishCloud(const sensor_msgs::msg::Range::SharedPtr msg) {
        if (points_.empty()) return;

        sensor_msgs::msg::PointCloud2 cloud;
        // 该时间需要考虑使用哪种
        cloud.header.stamp = msg->header.stamp;
        cloud.header.frame_id = "odom";
        cloud.height = 1;
        cloud.width = points_.size();
        cloud.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto &p : points_) {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = 0;

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        cloud_pub_->publish(cloud);
    }

    void PublishScan(const sensor_msgs::msg::Range::SharedPtr msg) {
        if (points_.empty()) {
            RCLCPP_WARN(this->get_logger(), "points_ a empty, skipping LaserScan publish.");
            return;
        }

        geometry_msgs::msg::TransformStamped tf_odom_to_combined;
        try {
            tf_odom_to_combined = tf_buffer_->lookupTransform(
                "combined_scan",  // target frame
                "odom",           // source frame
                msg->header.stamp, 
                rclcpp::Duration::from_seconds(0.2));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }
        tf2::Transform tf;
        tf2::fromMsg(tf_odom_to_combined.transform, tf);
        std::vector<Pose2D> transformed_points;
        transformed_points.reserve(points_.size());

        for (const auto &p : points_) {
            tf2::Vector3 v(p.x, p.y, 0.0);
            tf2::Vector3 v_t = tf * v;  // 应用 odom->combined_scan 变换

            Pose2D p_out;
            p_out.x = v_t.x();
            p_out.y = v_t.y();
            transformed_points.push_back(p_out);
        }



        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = msg->header.stamp; // 使用传入消息的时间戳
        scan.header.frame_id = "combined_scan"; 
        
        scan.angle_min = -M_PI;
        scan.angle_max = M_PI;
        // scan.angle_increment = 2*M_PI / (float)num_readings;
        scan.angle_increment = M_PI / 360.0; // 0.5deg
        scan.time_increment = 0.0;
        scan.scan_time = 0.0;
        scan.range_min = 0.03;
        scan.range_max = 10.0;
        // scan.ranges.resize(num_readings, std::numeric_limits<float>::infinity());
        int num_readings = (scan.angle_max - scan.angle_min) / scan.angle_increment;
        scan.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

        // for (const auto &p : points_) {
        for (const auto &p : transformed_points) {
            float range = std::sqrt(p.x * p.x + p.y * p.y);
            float angle = std::atan2(p.y, p.x);

            if (range < scan.range_min || range > scan.range_max  || 
                angle < scan.angle_min || angle > scan.angle_max) {
                continue;
            }

            int index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);

            if (index >= 0 && index < num_readings) {
                if (range < scan.ranges[index]) {
                    scan.ranges[index] = range;
                }
            }
        }

        scan_pub_->publish(scan);
    }

    float GetYawFromOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto  q = msg->pose.pose.orientation;
        float siny = 2.0 * (q.w * q.z + q.x * q.y);
        float cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny, cosy);
    }

    void SavePointsToTxt(const std::vector<Pose2D>& points, const std::string& file_path) {
        if (points.empty()) {
            std::cerr << "[WARN] SavePointsToTxt: empty points vector, skip saving.\n";
            return;
        }
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        std::ofstream ofs(file_path, std::ios::app);
        if (!ofs.is_open()) {
            std::cerr << "Failed to open file: " << file_path << std::endl;
            return;
        }

        ofs << "---" << "\n";
        ofs << "timestamp: " << now_ms << "\n";
        // 设置浮点型精度
        ofs << std::fixed << std::setprecision(6);

        for (const auto& p : points) {
            ofs << p.x << " " << p.y << "\n";
        }

        ofs.close();
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr            point_laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr        scan_laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::WheelVels>::SharedPtr wheel_vels_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr              imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               cal_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr           scan_pub_;

    rclcpp::CallbackGroup::SharedPtr imu_group_;
    rclcpp::CallbackGroup::SharedPtr point_laser_group_;

    geometry_msgs::msg::TransformStamped initial_odom_transform_;

    std::unique_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool         collecting_ = false;
    float        yaw_continue_ = 0.0f;
    rclcpp::Time last_time_ = this->now();
    float        x_ = 0.0f;
    float        y_ = 0.0f;
    float        yaw_ = 0.0f;
    float        angular_z_ = 0.0f;

    float odom_x_ = 0.0f;
    float odom_y_ = 0.0f;
    float scan_range_ = 0.0f;

    const float angle_step_deg_ = 0.5f;
    const float angle_step_rad_ = angle_step_deg_ * M_PI / 180.0;

    std::mutex yaw_mutex_;
    std::mutex odom_mutex_;

    struct Pose2D {
        Pose2D():x(0), y(0) {}
        Pose2D(float x_, float y_):x(x_), y(y_) {}
        float x;
        float y;
    };
    std::vector<Pose2D> points_;

    bool enable_point_interpolation_compensation_;
    bool save_point_laser_points_;
    bool publish_cloud_;
    bool publish_scan_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VirtualLaser>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}