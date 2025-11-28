#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <fstream>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

// 定义状态机的不同状态
enum class State { SEARCHING, DRIVING_TO_WALL, ROTATING_LEFT, WALL_FOLLOWING, BUMP };

struct Color {
    static constexpr const char* RED     = "\033[0;31m";
    static constexpr const char* GREEN   = "\033[0;32m";
    static constexpr const char* YELLOW  = "\033[0;33m";
    static constexpr const char* BLUE    = "\033[0;34m";
    static constexpr const char* MAGENTA = "\033[0;35m";
    static constexpr const char* CYAN    = "\033[0;36m";
    static constexpr const char* WHITE   = "\033[0;37m";

    static constexpr const char* RED_BOLD     = "\033[1;31m";
    static constexpr const char* GREEN_BOLD   = "\033[1;32m";
    static constexpr const char* YELLOW_BOLD  = "\033[1;33m";
    static constexpr const char* BLUE_BOLD    = "\033[1;34m";
    static constexpr const char* MAGENTA_BOLD = "\033[1;35m";
    static constexpr const char* CYAN_BOLD    = "\033[1;36m";
    static constexpr const char* WHITE_BOLD   = "\033[1;37m";

    static constexpr const char* RESET   = "\033[0m";
};

class Point2D {
public:
    float x;
    float y;

    Point2D() : x(0), y(0) {}
    Point2D(float x_, float y_) : x(x_), y(y_) {}

    Point2D operator+(const Point2D& other) const { return Point2D(x + other.x, y + other.y); }

    Point2D& operator+=(const Point2D& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
};

class Point3D {
public:
    float x;
    float y;
    float z;

    Point3D() : x(0), y(0), z(0) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Point3D operator+(const Point3D& other) const { return Point3D(x + other.x, y + other.y, z + other.z); }

    Point3D& operator+=(const Point3D& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
};

struct Frame {
    std::string              timestamp;
    std::vector<Point3D> xyz_points;
};

/**
 * @brief 线激光3D点云数据
 * @param point_type -1=初始化 0=黑色区域gap 1=真实gap
 */
class LineLaserPoints {
public:
    /**
     * @brief -1=初始化 0=黑色区域gap 1=真实gap
     */
    int points_type;
    std::vector<Point3D> ground_points;
    std::vector<Point3D> wall_points;

    LineLaserPoints() : points_type(-1) {}
    LineLaserPoints(int points_type_, const std::vector<Point3D>& ground_points_, const std::vector<Point3D>& wall_points_)
        : points_type(points_type_), ground_points(ground_points_), wall_points(wall_points_) {}

    void Clear() {
        points_type = -1;
        ground_points.clear();
        wall_points.clear();
    }

    void AddGroundPoint(const Point3D& p) {
        ground_points.push_back(p);
    }

    void AddWallPoint(const Point3D& p) {
        wall_points.push_back(p);
    }
};

class EdgeFollower : public rclcpp::Node {
public:
    EdgeFollower() : Node("edge_follower_node") {
        // 加载参数
        this->declare_parameter<float>("wheel_base", 235.0);
        this->get_parameter("wheel_base", wheel_base_);
        this->declare_parameter<int>("window_size", 5);
        this->get_parameter("window_size", window_size_);
        this->declare_parameter<float>("sigma", 1.0);
        this->get_parameter("sigma", sigma_);
        this->declare_parameter<float>("wall_distance", 0.25);
        this->get_parameter("wall_distance", wall_distance_);
        this->declare_parameter<float>("following_wall_linear", 0.05);
        this->get_parameter("following_wall_linear", following_wall_linear_);
        this->declare_parameter<bool>("state_loop", true);
        this->get_parameter("state_loop", state_loop_);
        this->declare_parameter<bool>("pub_nearby_point", false);
        this->get_parameter("pub_nearby_point", pub_nearby_point_);
        this->declare_parameter<bool>("right_angle_difference_method", true);
        this->get_parameter("right_angle_difference_method", right_angle_difference_method_);
        this->declare_parameter<bool>("linear_fitting_method", false);
        this->get_parameter("linear_fitting_method", linear_fitting_method_);
        this->declare_parameter<bool>("interpolate_gap_points", true);
        this->get_parameter("interpolate_gap_points", interpolate_gap_points_); 
        this->declare_parameter<bool>("use_moving_average_filter", true);
        this->get_parameter("use_moving_average_filter", use_moving_average_filter_); 
        this->declare_parameter<bool>("save_3Dpoints_to_txt", false);
        this->get_parameter("save_3Dpoints_to_txt", save_3Dpoints_to_txt_); 
        this->declare_parameter<bool>("line_laser_process", true);
        this->get_parameter("line_laser_process", line_laser_process_); 

        RCLCPP_INFO(this->get_logger(), "wheel_base = %s%.5f%s", Color::GREEN_BOLD, wheel_base_, Color::RESET);
        RCLCPP_INFO(this->get_logger(), "window_size = %s%d%s", Color::GREEN_BOLD, window_size_, Color::RESET);
        RCLCPP_INFO(this->get_logger(), "sigma = %s%.5f%s", Color::GREEN_BOLD, sigma_, Color::RESET);
        RCLCPP_INFO(this->get_logger(), "wall_distance = %s%.5f%s", Color::GREEN_BOLD, wall_distance_, Color::RESET);
        RCLCPP_INFO(this->get_logger(), "following_wall_linear = %s%.5f%s", Color::GREEN_BOLD, following_wall_linear_, Color::RESET);
        if (state_loop_) RCLCPP_INFO(this->get_logger(), "state_loop = %strue%s", Color::GREEN_BOLD, Color::RESET);
        else RCLCPP_INFO(this->get_logger(), "state_loop = %sfalse%s", Color::RED_BOLD, Color::RESET);
        if (pub_nearby_point_) RCLCPP_INFO(this->get_logger(), "pub_nearby_point = %strue%s", Color::GREEN_BOLD, Color::RESET);
        else RCLCPP_INFO(this->get_logger(), "pub_nearby_point = %sfalse%s", Color::RED_BOLD, Color::RESET);
        if (right_angle_difference_method_) RCLCPP_INFO(this->get_logger(), "right_angle_difference_method = %strue%s", Color::GREEN_BOLD, Color::RESET);
        else RCLCPP_INFO(this->get_logger(), "right_angle_difference_method = %sfalse%s", Color::RED_BOLD, Color::RESET);
        if (linear_fitting_method_) RCLCPP_INFO(this->get_logger(), "linear_fitting_method = %strue%s", Color::GREEN_BOLD, Color::RESET);
        else RCLCPP_INFO(this->get_logger(), "linear_fitting_method = %sfalse%s", Color::RED_BOLD, Color::RESET);
        if (interpolate_gap_points_) RCLCPP_INFO(this->get_logger(), "interpolate_gap_points = %strue%s", Color::GREEN_BOLD, Color::RESET);
        else RCLCPP_INFO(this->get_logger(), "interpolate_gap_points = %sfalse%s", Color::RED_BOLD, Color::RESET);
        if (use_moving_average_filter_) RCLCPP_INFO(this->get_logger(), "use_moving_average_filter = %strue%s", Color::GREEN_BOLD, Color::RESET);
        else RCLCPP_INFO(this->get_logger(), "use_moving_average_filter = %sfalse%s", Color::RED_BOLD, Color::RESET);
        if (save_3Dpoints_to_txt_) RCLCPP_INFO(this->get_logger(), "save_3Dpoints_to_txt = %strue%s", Color::GREEN_BOLD, Color::RESET);
        else RCLCPP_INFO(this->get_logger(), "save_3Dpoints_to_txt = %sfalse%s", Color::RED_BOLD, Color::RESET);
        if (line_laser_process_) RCLCPP_INFO(this->get_logger(), "line_laser_process = %strue%s", Color::GREEN_BOLD, Color::RESET);
        else RCLCPP_INFO(this->get_logger(), "line_laser_process = %sfalse%s", Color::RED_BOLD, Color::RESET);

        scan_laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&EdgeFollower::scan_callback, this, _1));
        line_laser_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>("/line_scan", 10, std::bind(&EdgeFollower::line_laser_callback, this, _1));
        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>("/hazard_detection", rclcpp::SensorDataQoS(),
                                                                                                std::bind(&EdgeFollower::hazard_callback, this, _1));
        // wheel_sub_ = this->create_subscription<irobot_create_msgs::msg::WheelVels>("/wheel_vels", rclcpp::SensorDataQoS(),
        //                                                                            std::bind(&EdgeFollower::wheel_vels_callback, this, _1));
        // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS(),
        //                                                             std::bind(&EdgeFollower::ImuCallback, this, std::placeholders::_1));

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        wall_clouds_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("wall_point_clouds", 10);
        gap_wall_clouds_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("gap_wall_point_clouds", 10);
        ground_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_point_clouds", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        // 发布 Odometry
        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);
        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if (state_loop_) control_loop_timer_ = this->create_wall_timer(50ms, std::bind(&EdgeFollower::control_loop, this));
        if (line_laser_process_) line_laser_process_timer_ = this->create_wall_timer(50ms, std::bind(&EdgeFollower::line_laser_process, this));

        odom_last_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Wall follower node has been started.");
    }

private:
    void line_laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        /*
            线激光0°线位于x轴方向，向y轴正方向旋转的角度为﹢，反方向为﹣
        */

        if (msg->ranges.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "have no data!");
            return;
        }

        // 多线程访问需要加锁
        line_laser_ = *msg;

        if (use_moving_average_filter_) {
            std::vector<float> ranges_out(line_laser_.ranges.size());
            int window_size = 9;
            ranges_out.resize(line_laser_.ranges.size());
            int N = static_cast<int>(line_laser_.ranges.size());
            int half = window_size / 2;
    
            for (int i = 0; i < N; ++i) {
                float sum = 0.0f;
                int count = 0;
                for (int k = -half; k <= half; ++k) {
                    int idx = i + k;
                    if (idx >= 0 && idx < N && line_laser_.ranges[idx] > 0.01f) {
                        sum += line_laser_.ranges[idx];
                        count++;
                    }
                }
                if (count > 0)
                    ranges_out[i] = sum / count;
                else
                    ranges_out[i] = line_laser_.ranges[i];
            }
            line_laser_.ranges = ranges_out;
        }

// test show points
#if 0
        sensor_msgs::msg::LaserScan scan = *msg;
        float start_angle = 0.0 * M_PI / 180.0;
        float end_angle = 45.0 * M_PI / 180.0;
        size_t start = find_index_for_angle(msg, start_angle);
        size_t end = find_index_for_angle(msg, end_angle);

        std::fill(scan.ranges.begin() + start, scan.ranges.begin() + end, std::numeric_limits<float>::quiet_NaN());
        sensor_msgs::msg::PointCloud2 laser_cloud;
        sensor_msgs::msg::PointCloud2 odom_cloud;
        try {
            // laserscan -> pointcloud in laser
            projector_.transformLaserScanToPointCloud(scan.header.frame_id, scan, laser_cloud, *tf_buffer_);
            // in odom
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(
                    "odom", msg->header.frame_id, msg->header.stamp,
                    rclcpp::Duration::from_seconds(0.2));
            tf2::doTransform(laser_cloud, odom_cloud,
                             transform_stamped);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "trans failed:%s", ex.what());
        }

        // pub
        wall_clouds_pub_->publish(odom_cloud);
#endif

        // save 3D points to txt
        if (save_3Dpoints_to_txt_) {
            std::vector<Point3D> points_in_laser = ResetPointsOrder(line_laser_);

            SaveLaserPoints3DToTxt(points_in_laser, "/home/shan2/Mypro/create3_ws/src/edge_following/data/line_points_in_laser.txt");
    
            std::vector<Point3D> points_in_odom;
            try {
                // 获取 line_laser 坐标系相对于 odom 的变换
                geometry_msgs::msg::TransformStamped transformStamped =
                    tf_buffer_->lookupTransform("odom", line_laser_.header.frame_id, tf2::TimePointZero);
    
                // 遍历 points_in_laser 数据
                for (size_t i = 0; i < points_in_laser.size(); ++i) {
    
                    // point in laser
                    geometry_msgs::msg::PointStamped point_in_laser;
                    point_in_laser.header = line_laser_.header;
                    point_in_laser.point.x = points_in_laser[i].x;
                    point_in_laser.point.y = points_in_laser[i].y;
                    point_in_laser.point.z = points_in_laser[i].z;
                    // 转换到 odom 坐标系
                    geometry_msgs::msg::PointStamped point_in_odom;
                    tf2::doTransform(point_in_laser, point_in_odom, transformStamped);
    
                    // 保存到 Point3D
                    Point3D p;
                    p.x = point_in_odom.point.x;
                    p.y = point_in_odom.point.y;
                    p.z = point_in_odom.point.z;
                    points_in_odom.push_back(p);
                }
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform %s to odom: %s", line_laser_.header.frame_id.c_str(), ex.what());
            }
    
            SaveOdomPoints3DToTxt(points_in_odom, "/home/shan2/Mypro/create3_ws/src/edge_following/data/line_points_in_odom.txt");
        }
    }

    void hazard_callback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg) {
        if (msg->detections.empty()) {
            // RCLCPP_INFO(this->get_logger(), "No hazards detected.");
            return;
        }

        for (const auto& detection : msg->detections) {
            if (detection.type == 0) {
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Hazard type: %d, frame: %s", detection.type, detection.header.frame_id.c_str());

            // 例如根据 type 判断
            switch (detection.type) {
                case irobot_create_msgs::msg::HazardDetection::BUMP:
                    RCLCPP_WARN(this->get_logger(), "⚠️ Bump detected!");
                    hazard_frame_id_ = detection.header.frame_id;
                    hazard_type_ = detection.type;
                    align_start_time_ = this->get_clock()->now();
                    current_state_ = State::BUMP;
                    break;
                case irobot_create_msgs::msg::HazardDetection::CLIFF:
                    RCLCPP_WARN(this->get_logger(), "⚠️ Cliff detected!");
                    break;
                case irobot_create_msgs::msg::HazardDetection::STALL:
                    RCLCPP_WARN(this->get_logger(), "⚠️ Stall detected!");
                    break;
                default:
                    RCLCPP_INFO(this->get_logger(), "Other hazard.");
                    break;
            }
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->ranges.size() < 10) {
            std::cout << "no data" << std::endl;
            return;
        }
        // 多线程访问需要加锁
        {
            std::lock_guard<std::mutex> lock(scan_lock_);
            scan_laser_ = *msg;
        }


        size_t minus_5_deg = get_index_from_angle(*msg, 355 * M_PI / 180);  // -5
        size_t positive_5_deg = get_index_from_angle(*msg, 5 * M_PI / 180);  // 5
        size_t p_360_deg = get_index_from_angle(*msg, 360 * M_PI / 180);   // 360

        // 范围内平均距离
        float front_dis_0_left = avg_distance_cal(msg, 0, positive_5_deg);
        float front_dis_0_right = avg_distance_cal(msg, minus_5_deg, p_360_deg);
        scan_front_dis_ = (front_dis_0_left + front_dis_0_right) / 2;

        // 发布最近点
        if (pub_nearby_point_) PublishCylinder(target_point_.x, target_point_.y, 0.1, 0.03, 0.03, 0.3, 0.0, 1.0, 0.0, 1.0, 0);

// test pub points
#if 0
        sensor_msgs::msg::LaserScan scan = *msg;
        float start_angle = 270.0 * M_PI / 180.0;
        float end_angle = 360.0 * M_PI / 180.0;
        size_t start = find_index_for_angle(msg, start_angle);
        size_t end = find_index_for_angle(msg, end_angle);

        std::fill(scan.ranges.begin() + start, scan.ranges.begin() + end, std::numeric_limits<float>::quiet_NaN());
        sensor_msgs::msg::PointCloud2 laser_cloud;
        sensor_msgs::msg::PointCloud2 odom_cloud;
        try {
            // laserscan -> pointcloud in laser
            projector_.transformLaserScanToPointCloud(scan.header.frame_id, scan, laser_cloud, *tf_buffer_);
            // in odom
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(
                    "odom", msg->header.frame_id, msg->header.stamp,
                    rclcpp::Duration::from_seconds(0.2));
            tf2::doTransform(laser_cloud, odom_cloud,
                             transform_stamped);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "trans failed:%s", ex.what());
        }

        // pub
        wall_clouds_pub_->publish(odom_cloud);
#endif

#if 0
        // -45 to -135
        float right_min_dist = std::numeric_limits<double>::infinity();
        int right_min_index = -1;
        size_t start_index = get_index_from_angle(msg, 3.93); // -135
        size_t end_index = get_index_from_angle(msg, 5.5); // -45

        for (size_t i = start_index; i <= end_index; ++i) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < right_min_dist && msg->ranges[i] > 0.1) {
                right_min_dist = msg->ranges[i];
                right_min_index = i;
            }
        }

        if (right_min_index != -1) {
            float target_angle = msg->angle_min + right_min_index * msg->angle_increment;

            // 激光雷达坐标系坐标点
            Point2D laser_point = Point2D(right_min_dist * cos(target_angle), right_min_dist * sin(target_angle));

            // pointstamped类型存储数据
            geometry_msgs::msg::PointStamped right_in_laser;
            right_in_laser.header = msg->header;
            right_in_laser.header.stamp = rclcpp::Time(0);

            right_in_laser.point.x = laser_point.x;
            right_in_laser.point.y = laser_point.y;
            right_in_laser.point.z = 0.0;

            geometry_msgs::msg::PointStamped right_in_odom;
            // 转换到odom
            try {
                right_in_odom = tf_buffer_->transform(right_in_laser, "odom");
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "right target：%s", ex.what());
            }

            Point2D right_min_point;
            right_min_point.x = right_in_odom.point.x;
            right_min_point.y = right_in_odom.point.y;

            PublishCylinder(right_min_point.x, right_min_point.y, 0.1, 0.03, 0.03, 0.3, 0.0, 1.0, 0.0, 1.0, 0);
        }
#endif
// pointclouds
#if 0
        sensor_msgs::msg::LaserScan scan = *msg;
        std::fill(scan.ranges.begin() + start_index, scan.ranges.begin() + end_index, std::numeric_limits<float>::quiet_NaN());
        sensor_msgs::msg::PointCloud2 laser_cloud;
        sensor_msgs::msg::PointCloud2 odom_cloud;
        try {
            // laserscan -> pointcloud in laser
            projector_.transformLaserScanToPointCloud(scan.header.frame_id, scan, laser_cloud, *tf_buffer_);
            // in odom
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(
                    "odom", msg->header.frame_id, msg->header.stamp,
                    rclcpp::Duration::from_seconds(0.2));
            tf2::doTransform(laser_cloud, odom_cloud,
                             transform_stamped);
            // pub
            wall_clouds_pub_->publish(odom_cloud);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "point clouds trans failed:%s", ex.what());
        }
#endif
// 显示机身位置
#if 0
        // 机身坐标
        geometry_msgs::msg::PointStamped base_point;
        base_point.header.frame_id = "base_link";
        base_point.header.stamp = rclcpp::Time(0);
        base_point.point.x = 0.0;
        base_point.point.y = 0.0;
        base_point.point.z = 0.0;
        // odom下机身坐标
        geometry_msgs::msg::PointStamped base_in_odom;
        try {
            base_in_odom = tf_buffer_->transform(base_point, "odom");
            PublishCylinder(base_in_odom.point.x, base_in_odom.point.y, 0.1, 0.4, 0.4, 0.1, 1.0, 0.0, 0.0, 1.0, 1);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "无法转换base_link坐标点:%s", ex.what());
        }
#endif
    }

    void wheel_vels_callback(const irobot_create_msgs::msg::WheelVels::SharedPtr msg) {
        // 左右轮速度，单位应为 m/s
        float v_left = msg->velocity_left;
        float v_right = msg->velocity_right;
        
        // 差速驱动运动学模型计算
        // 线速度 (Vx)
        linear_x_ = (v_right + v_left) / 2.0;
        
        // 角速度 (Vyaw)
        angular_z_ = (v_right - v_left) / wheel_base_;
    }

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 从 IMU 四元数提取 yaw
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        imu_yaw_ = yaw;
    }

    void publish_odom() {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - odom_last_time_).seconds();

        if (dt <= 0.0) {
            return;
        }

        float yaw = imu_yaw_;

        // ----------------- 里程计积分 (欧拉积分) -----------------
        x_ += linear_x_ * std::cos(yaw) * dt;
        y_ += linear_x_ * std::sin(yaw) * dt;

        // ----------------- Odometry 消息创建 -----------------
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "wheel_odom";
        odom_msg.child_frame_id = "base_link_filtered"; 

        // 位姿 (Pose)
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        
        // 姿态 (Yaw -> Quaternion)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // 速度 (Twist)
        odom_msg.twist.twist.linear.x = linear_x_;
        odom_msg.twist.twist.angular.z = angular_z_;

        // 发布 Odometry
        odom_pub_->publish(odom_msg);

        // 发布tf
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = current_time;
        tf_msg.header.frame_id = "wheel_odom";
        tf_msg.child_frame_id = "base_link_filtered";
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(tf_msg);
        
        odom_last_time_ = current_time;
    }

    void control_loop() {
        switch (current_state_) {
            case State::SEARCHING:
                handle_searching();
                break;
            case State::DRIVING_TO_WALL:
                handle_driving_to_wall();
                break;
            case State::ROTATING_LEFT:
                handle_rotating_left();
                break;
            case State::WALL_FOLLOWING: {
                handle_wall_following();
                break;
            }
            case State::BUMP: {
                handle_bump();
                break;
            }
        }
    }

    // 状态1: 寻找最近的点
    void handle_searching() {
        std::cout << "search" << std::endl;
        sensor_msgs::msg::LaserScan scan;
        {
            std::lock_guard<std::mutex> lock(scan_lock_);
            scan = scan_laser_;
        }

        if (scan.ranges.size() < 10) {
            std::cout << "no scan data" << std::endl;
            return;
        }

        // scan最近点
        if (!has_min_dist_) {
            int   min_index = -1;
            float min_dist = std::numeric_limits<float>::infinity();

            for (size_t i = 0; i < scan.ranges.size(); ++i) {
                if (scan.ranges[i] < min_dist && scan.ranges[i] > 0.2) {
                    min_dist = scan.ranges[i];
                    min_index = i;
                }
            }

            if (min_index != -1) {
                has_min_dist_ = true;
                float target_angle = scan.angle_min + min_index * scan.angle_increment;

                // 激光雷达坐标系坐标点
                Point2D                          laser_point = Point2D(min_dist * cos(target_angle), min_dist * sin(target_angle));
                geometry_msgs::msg::PointStamped point_in_laser;
                // pointstamped类型存储数据
                point_in_laser.header = scan.header;
                point_in_laser.header.stamp = rclcpp::Time(0);

                point_in_laser.point.x = laser_point.x;
                point_in_laser.point.y = laser_point.y;
                point_in_laser.point.z = 0.0;

                geometry_msgs::msg::PointStamped point_in_odom;
                // 转换到odom
                try {
                    point_in_odom = tf_buffer_->transform(point_in_laser, "odom");
                    target_point_.x = point_in_odom.point.x;
                    target_point_.y = point_in_odom.point.y;

                    current_state_ = State::DRIVING_TO_WALL;
                } catch (const tf2::TransformException& ex) {
                    RCLCPP_WARN(this->get_logger(), "无法从激光坐标转换到里程计坐标：%s", ex.what());
                    has_min_dist_ = false;
                }
            }
        }
    }

    // 状态2: 转向并移动到墙前
    void handle_driving_to_wall() {
        std::cout << "driving" << std::endl;
        geometry_msgs::msg::TransformStamped transform;

        geometry_msgs::msg::PointStamped target_in_odom;
        target_in_odom.header.frame_id = "odom";
        target_in_odom.header.stamp = rclcpp::Time(0);
        target_in_odom.point.x = target_point_.x;
        target_in_odom.point.y = target_point_.y;
        target_in_odom.point.z = 0.0;

        geometry_msgs::msg::PointStamped target_in_base;

        try {
            // 转换到 base_link
            target_in_base = tf_buffer_->transform(target_in_odom, "base_link");

            // 计算相对机器人前进方向的角度
            double target_yaw = std::atan2(target_in_base.point.y, target_in_base.point.x);

            geometry_msgs::msg::Twist twist_msg;

            // 转向最近的点
            if (std::fabs(target_yaw) > 0.04 && !turn_step_final_) {
                twist_msg.angular.z = (target_yaw > 0) ? 0.3 : -0.3;
                RCLCPP_INFO(this->get_logger(), "Rotating. Target yaw: %.2f", target_yaw);
            } else if (scan_front_dis_ > 0.22) {
                twist_msg.linear.x = 0.1;
                turn_step_final_ = true;
                RCLCPP_INFO(this->get_logger(), "Moving forward. Distance to wall: %f", scan_front_dis_);
            } else {
                twist_msg.linear.x = 0;
                twist_msg.angular.z = 0;
                RCLCPP_INFO(this->get_logger(), "Reached wall. Preparing to rotate left.");
                current_state_ = State::ROTATING_LEFT;

                rotation_start_time_ = this->get_clock()->now();
            }

            // 处理碰撞1.0
            // if (hazard_type_ == 1) {
            //     handle_bump();
            // } else {
            //     cmd_publisher_->publish(twist_msg);
            // }

            // 2.0
            cmd_publisher_->publish(twist_msg);

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "tran base_link tf err:%s", ex.what());
        }
    }

    // 状态3: 左转90度
    void handle_rotating_left() {
        std::cout << "turn" << std::endl;
        geometry_msgs::msg::Twist twist_msg;
        // 假设角速度为0.5 rad/s，旋转90度(PI/2)大约需要3.14秒
        double rotation_duration = (M_PI / 2) / 0.5;

        if ((this->get_clock()->now() - rotation_start_time_).seconds() < rotation_duration) {
            twist_msg.angular.z = 0.5;  // 向左转
            RCLCPP_INFO(this->get_logger(), "Executing 90-degree left turn.");
        } else {
            twist_msg.angular.z = 0;
            RCLCPP_INFO(this->get_logger(), "Rotation complete. Starting wall following.");
            current_state_ = State::WALL_FOLLOWING;
        }
        cmd_publisher_->publish(twist_msg);
    }

    // 状态4: 沿边算法
    void handle_wall_following() {
        std::cout << "following" << std::endl;
        if (scan_laser_.ranges.size() < 10) {
            std::cout << "no data" << std::endl;
            return;
        }

        float kp_distance = 10.0f;
        float kp_angle = 2.0f;

        float right_angle_rad = 270.0 * M_PI / 180.0;
        float scan_range_rad = 60.0 * M_PI / 180.0;

        size_t start_index = find_index_for_angle(scan_laser_, right_angle_rad - scan_range_rad / 2.0);
        size_t end_index = find_index_for_angle(scan_laser_, right_angle_rad + scan_range_rad / 2.0);

        // 保存右侧数据点到数组
        std::vector<Point2D> points;

        // 寻找此范围内的最近点
        float  min_dist = std::numeric_limits<float>::max();
        size_t min_index = 0;
        for (size_t i = start_index; i <= end_index; ++i) {
            float angle = scan_laser_.angle_min + i * scan_laser_.angle_increment;
            float r = scan_laser_.ranges[i];
            // 确保索引在有效范围内
            if (i >= scan_laser_.ranges.size()) continue;

            if (std::isfinite(scan_laser_.ranges[i]) && scan_laser_.ranges[i] < min_dist && scan_laser_.ranges[i] > 0) {
                min_dist = scan_laser_.ranges[i];
                min_index = i;
            }
            points.emplace_back(r * cos(angle), r * sin(angle));
        }

        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();

        // 1.0 robot正右方角度与-90°角度差值法
        if (right_angle_difference_method_) {
            if (min_dist == std::numeric_limits<float>::max()) {
                // 如果右侧没有检测到任何东西，让机器人旋转寻找墙壁
                twist_msg->linear.x = 0.0;
                twist_msg->angular.z = -0.3;
            } else {
                // 距离误差: (目标距离 - 当前距离)
                float distance_error = wall_distance_ - min_dist;
    
                // 通过看最近的点偏离了-90度多远来计算角度误差。
                float closest_point_angle = scan_laser_.angle_min + scan_laser_.angle_increment * min_index;
                float angle_error = right_angle_rad - closest_point_angle;
    
                // 通过控制距墙距离和机器人朝向来判断旋转方向
                float angular_z = kp_distance * distance_error - kp_angle * angle_error;
    
                twist_msg->linear.x = following_wall_linear_;
                twist_msg->angular.z = angular_z;
    
                RCLCPP_INFO(this->get_logger(), "Dist: %.2f, DistErr: %.2f, AngleErr: %.2f, AngularZ: %.2f", min_dist, distance_error, angle_error,
                            angular_z);
            }
        }

        // 2.0 直线拟合法
        if (linear_fitting_method_) {
            float sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
            for (auto &p : points) {
                sum_x += p.x;
                sum_y += p.y;
                sum_xx += p.x * p.x;
                sum_xy += p.x * p.y;
            }
            float n = points.size();
            float k = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
            float theta_wall = atan(k);
    
            float distance_to_wall = 0;
            for (auto &p : points)
                distance_to_wall += fabs(p.y - k * p.x) / sqrt(1 + k*k);
            distance_to_wall /= points.size();
            float error_distance = wall_distance_ - distance_to_wall;
            float error_angle = theta_wall;
            float angular_z = kp_angle * error_angle + kp_distance * error_distance;
    
            twist_msg->linear.x = following_wall_linear_;
            twist_msg->angular.z = angular_z;
        }

        if (scan_front_dis_ < 0.20 && scan_front_dis_ > 0) {
            RCLCPP_INFO(this->get_logger(), "scan front dis: %f", scan_front_dis_);
            rotation_start_time_ = this->get_clock()->now();
            current_state_ = State::ROTATING_LEFT;
            return;
        }

        cmd_publisher_->publish(std::move(twist_msg));
    }

    void line_laser_process() {
        /*
            -查找gap，判断并补偿；分离地面点
                1.根据数据点之间的距离值判断gap位置；
                2.判断是黑色导致的gap还是真实gap；
                3.补偿黑色区域gap；
                4.分离地面点；
                5.区分gap数据并返回；
        */
        sensor_msgs::msg::LaserScan line_laser;
        // 多线程访问需要加锁
        {
            line_laser = line_laser_;
        }
        if (line_laser.ranges.size() < 10) {
            std::cout << "find gap line_laser is empty" << std::endl;
            return;
        }

        // 重置laser顺序
        std::vector<Point3D> order_points = ResetPointsOrder(line_laser);
        std::pair<int, std::vector<Point3D>> pair_out = FindGapFunction(order_points);

        std::vector<Point3D> points_in_laser = pair_out.second;

        if (points_in_laser.size() != 0) {
            // 分割地面点
            Eigen::MatrixXf np_end_points(3, 2);
            // 首端点
            np_end_points(0, 0) = points_in_laser.front().x;
            np_end_points(1, 0) = points_in_laser.front().y;
            np_end_points(2, 0) = 0; // index
            // 末端点
            np_end_points(0, 1) = points_in_laser.back().x;
            np_end_points(1, 1) = points_in_laser.back().y;
            np_end_points(2, 1) = points_in_laser.size() - 1;
            
            Eigen::MatrixXf lines = IepfFunction(points_in_laser, np_end_points);

            // 解析地面点
            int ground_point_index = -1;
            bool has_ground_points = false;
            float min_k = std::numeric_limits<float>::max();
            for (int i = 0; i < lines.cols() - 1; ++i) {
                float k = (lines(1, i + 1) - lines(1, i)) / (lines(0, i + 1) - lines(0, i));
    
                if (std::fabs(k) > 1) continue;
                if (k < min_k) {
                    min_k = k;
                    ground_point_index = i;
                    has_ground_points = true;
                }
            }

            std::vector<Point3D> ground_points;
            if (has_ground_points) {
                ground_points.resize((int)(lines(2, ground_point_index + 1) - lines(2, ground_point_index) + 1));
                std::copy(points_in_laser.begin() + (int)(lines(2, ground_point_index)), points_in_laser.begin() + (int)(lines(2, ground_point_index + 1)) + 1,
                        ground_points.begin());
            }

            std::vector<Point3D> wall_points = symmetricDifference(points_in_laser, ground_points);

            if (ground_points.size() != 0) {
                std::vector<Point3D> odom_ground_points = PointsLaser2Odom(line_laser, ground_points);
                sensor_msgs::msg::PointCloud2 ground_point_cloud = Point3DToPointCloud(line_laser, odom_ground_points);
                // 发布地面历史点云数据
                pub_ground_points(ground_point_cloud);
            }
            
            std::vector<Point3D> odom_wall_points = PointsLaser2Odom(line_laser, wall_points);
            sensor_msgs::msg::PointCloud2 wall_point_cloud = Point3DToPointCloud(line_laser, odom_wall_points);
    
            // 发布墙体历史点云数据
            if (pair_out.first == 0) pub_wall_points(wall_point_cloud);
            else if (pair_out.first == 1) pub_gap_wall_points(wall_point_cloud);
            else return;
        }
    // }
    }

    /**
     * @brief 查找gap点，判断并进行补偿；分割地面点
     * 
     * @details 根据点之间距离判断gap点，识别黑色区域进行补偿；分割地面点
     * 
     * @param scan: 线激光数据
     * @return 返回点云数据类型，包含地面点与墙体点；区分真实gap数据与黑色区域gap数据
     * 
     * @note null
     * @warning warning1 该函数处理0°以下的激光数据，线激光的安装位置x轴为侧前方，0°以下为地面位置，安装坐标系不同需要修改
     * @warning warning2 0°线正方向为地面点方向，使用最后一个gap点对，其他场景需要重新设计
     * @warning warning3 未考虑地面沟壑场景
     * @warning warning4 未考虑地面上方gap场景
     * 
     */
    std::pair<int, std::vector<Point3D>> FindGapFunction(const std::vector<Point3D>& points) {
        int key = -1;
        std::vector<Point3D> points_in_laser = points;
        std::pair<int, std::vector<Point3D>> pair_out(key, points_in_laser);

        float  gap_dis = 0.01f;
        size_t gap_num = 0;
        std::vector<std::pair<Point3D, Point3D>> gaps_sides_points;

        if (!points.empty()) {
            float max_dis = 0;
            for (size_t i = 0; i < points.size() - 1; ++i) {
                Point3D p1 = points[i];
                Point3D p2 = points[i + 1];

                float dis = Distance3D(p1, p2);
                if (dis > max_dis) max_dis = dis;

                if (dis > gap_dis) {
                    ++gap_num;
                    std::pair<Point3D, Point3D> gap_sides_points(p1, p2);
                    gaps_sides_points.emplace_back(gap_sides_points);
                }
            }

            Point3D gap_first_point;
            Point3D gap_second_point;
            if (gap_num > 0) {
                gap_first_point = gaps_sides_points.back().first;
                gap_second_point = gaps_sides_points.back().second;
                max_dis = Distance3D(gap_first_point, gap_second_point);
                
                if(max_dis > gap_dis) {
                    if (std::fabs(gap_first_point.x - gap_second_point.x) > 0.01) {
                        std::cout << "real gap" << std::endl;
                        key = 1;
                    } else {
                        // 补偿gap处的点云数据
                        std::vector<Point3D> inter_points_in_laser;
                        inter_points_in_laser = InterpolatePoints(gap_first_point, gap_second_point, 0.001);

                        // 找 gap_first_point 在原始点云中的位置
                        auto it = std::find_if(points_in_laser.begin(), points_in_laser.end(), [&](const Point3D& p) {
                            return (std::fabs(p.x - gap_first_point.x) < 1e-6 && std::fabs(p.y - gap_first_point.y) < 1e-6 && std::fabs(p.z - gap_first_point.z) < 1e-6);
                        });

                        if (it != points_in_laser.end()) {
                            points_in_laser.insert(it + 1, inter_points_in_laser.begin(), inter_points_in_laser.end());
                        } else {
                            // 找不到 gap_first_point, 则插在最后
                            points_in_laser.insert(points_in_laser.end(), inter_points_in_laser.begin(), inter_points_in_laser.end());
                        }

                        // points_in_laser.insert(points_in_laser.end(), inter_points_in_laser.begin(), inter_points_in_laser.end());
                        key = 0;
                    }
                } else {
                    std::cout << "normal wall" << std::endl;
                }
            } else {
                key = 0;
            }
        } else {
            key = 0;
            return pair_out;
        }

        pair_out.first = key;
        pair_out.second = points_in_laser;

        return pair_out;
    }

    // 输出Ax + By + C = 0
    Eigen::MatrixXf IepfFunction(const std::vector<Point3D>& points, Eigen::MatrixXf& np_end_points, float dis_threshold = 0.02) {

        bool has_new_break = false;
        int insert_pos = -1;
        Eigen::Vector3f break_point;

        // 遍历每一对端点 -By = Ax + C, B = -1
        for (int i = 0; i < np_end_points.cols() - 1; ++i) {
            float max_dis_point_to_line = 0.0f; // 最大距离初始化为0
            int break_point_index = -1; // 断点索引初始化-1

            // 计算斜率A
            float var_A = (np_end_points(1, i + 1) - np_end_points(1, i)) / (np_end_points(0, i + 1) - np_end_points(0, i));
            float var_B = -1.0f;
            float var_C = np_end_points(1, i) - var_A * np_end_points(0, i);

            // 遍历端点之间的点
            int start_index = static_cast<int>(np_end_points(2, i));
            int end_insdex = static_cast<int>(np_end_points(2, i + 1));
            // std::cout << "np end points: " << np_end_points.cols() << std::endl;
            for (int j = start_index + 1; j < end_insdex; ++j) {
                // 跳过首尾点
                // if (j == 0 || j == np_end_points(2, i)) continue;
                // 计算点到直线的距离
                // float dis_point_to_line = std::fabs((var_A * points[j].y + var_B * points[j].z + var_C) / (std::sqrt(var_A * var_A + var_B * var_B)));
                float dis_point_to_line = std::fabs((var_A * points[j].x + var_B * points[j].y + var_C) / (std::sqrt(var_A * var_A + var_B * var_B)));
                // std::cout << "dis point to line: " << dis_point_to_line << std::endl;
                // 如果距离超过阈值，检查是否为最大值
                if (dis_point_to_line > dis_threshold && dis_point_to_line > max_dis_point_to_line) {
                    max_dis_point_to_line = dis_point_to_line;
                    break_point_index = j;
                }
            }

            if (break_point_index != -1) {
                has_new_break = true;
                insert_pos = i + 1;
                // break_point = Eigen::Vector3f(static_cast<float>(points[break_point_index].y), static_cast<float>(points[break_point_index].z),
                //                               static_cast<float>(break_point_index));
                break_point = Eigen::Vector3f(static_cast<float>(points[break_point_index].x), static_cast<float>(points[break_point_index].y),
                                            static_cast<float>(break_point_index));
                
                break;
            }
        }

        // 如果没有新断点，终止递归
        if (!has_new_break) return np_end_points;

        // 列
        int old_cols = np_end_points.cols();
        int new_cols = old_cols + 1;
        // 插入列必须新建矩阵
        Eigen::MatrixXf new_np_end_points(3, new_cols);

        // 复制插入点之前的值
        if (insert_pos > 0) {
            // 0行0列开始，取3行insert_pos列
            new_np_end_points.block(0, 0, 3, insert_pos) = np_end_points.block(0, 0, 3, insert_pos);
        }
        // 插入新断点
        new_np_end_points.col(insert_pos) = break_point;
        std::cout << "break_point: " << break_point.z() << std::endl;
        // 复制插入点之后的值
        if ((old_cols - insert_pos) > 0) {
            new_np_end_points.block(0, insert_pos + 1, 3, old_cols - insert_pos) = np_end_points.block(0, insert_pos, 3, old_cols - insert_pos);
        }

        np_end_points = new_np_end_points;

        np_end_points = IepfFunction(points, np_end_points, dis_threshold);

        return np_end_points;
    }

    std::vector<Point3D> InterpolatePoints(const Point3D& p1, const Point3D& p2, float step) {
        std::vector<Point3D> points;
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float dz = p2.z - p1.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < 1e-6) return {};  // 两点太近

        int num_steps = static_cast<int>(dist / step);
        points.reserve(num_steps);

        for (int i = 0; i <= num_steps; ++i) {
            float ratio = static_cast<float>(i) / num_steps;
            Point3D p;
            p.x = p1.x + ratio * dx;
            p.y = p1.y + ratio * dy;
            p.z = p1.z + ratio * dz;
            points.push_back(p);
        }

        return points;
    }

    sensor_msgs::msg::PointCloud2 Point3DToPointCloud(sensor_msgs::msg::LaserScan& laser, std::vector<Point3D>& points) {
        // 构造输出 PointCloud2
        sensor_msgs::msg::PointCloud2 point_cloud;
        point_cloud.header.stamp = laser.header.stamp;
        point_cloud.header.frame_id = "odom";
        point_cloud.height = 1;
        point_cloud.width = points.size();
        point_cloud.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(point_cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> out_x(point_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(point_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(point_cloud, "z");

        for (size_t i = 0; i < points.size(); i++, ++out_x, ++out_y, ++out_z) {
            *out_x = points[i].x;
            *out_y = points[i].y;
            *out_z = points[i].z;
        }

        return point_cloud;
    }

    std::vector<Point3D> ResetPointsOrder(sensor_msgs::msg::LaserScan& laser) {
        std::vector<Point3D> points_in_laser;
        size_t start_index = find_index_for_angle(laser, M_PI);
        size_t end_index = laser.ranges.size();
        float angle = laser.angle_min;
        
        // 按顺序保存数据点
        angle += laser.angle_increment * start_index;
        for (size_t i = start_index; i < end_index; ++i, angle += laser.angle_increment) {
            float range = laser.ranges[i];
            if (std::isnan(range) || std::isinf(range)) continue;  // 跳过无效值

            Point3D point_in_laser;
            point_in_laser.x = range * cos(angle);
            point_in_laser.y = range * sin(angle);
            point_in_laser.z = 0.0;

            points_in_laser.emplace_back(point_in_laser);
        }
        
        angle = laser.angle_min;
        for (size_t i = 0; i < start_index; ++i, angle += laser.angle_increment) {
            float range = laser.ranges[i];
            if (std::isnan(range) || std::isinf(range)) continue;  // 跳过无效值

            Point3D point_in_laser;
            point_in_laser.x = range * cos(angle);
            point_in_laser.y = range * sin(angle);
            point_in_laser.z = 0.0;

            points_in_laser.emplace_back(point_in_laser);
        }

        return points_in_laser;
    }

    std::vector<Point3D> PointsLaser2Odom(const sensor_msgs::msg::LaserScan& line_laser, const std::vector<Point3D>& points_in_laser) {
        // 解析到odom坐标系下
        std::vector<Point3D> points_in_odom;
        try {
            // 获取 line_laser 坐标系相对于 odom 的变换
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_->lookupTransform("odom", line_laser.header.frame_id, tf2::TimePointZero);

            // 遍历 points_in_laser 数据
            for (size_t i = 0; i < points_in_laser.size(); ++i) {

                // point in laser
                geometry_msgs::msg::PointStamped point_in_laser;
                point_in_laser.header = line_laser.header;
                point_in_laser.point.x = points_in_laser[i].x;
                point_in_laser.point.y = points_in_laser[i].y;
                point_in_laser.point.z = points_in_laser[i].z;
                // 转换到 odom 坐标系
                geometry_msgs::msg::PointStamped point_in_odom;
                tf2::doTransform(point_in_laser, point_in_odom, transformStamped);

                // 保存到 Point3D
                Point3D p;
                if (point_in_odom.point.z > 0.1) continue;
                p.x = point_in_odom.point.x;
                p.y = point_in_odom.point.y;
                p.z = point_in_odom.point.z;
                points_in_odom.push_back(p);
            }
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to odom: %s", line_laser.header.frame_id.c_str(), ex.what());
        }

        return points_in_odom;
    }

    bool isSamePoint(const Point3D& a, const Point3D& b, double eps = 1e-6) {
        return (std::fabs(a.x - b.x) < eps && std::fabs(a.y - b.y) < eps && std::fabs(a.z - b.z) < eps);
    }

    std::vector<Point3D> symmetricDifference(const std::vector<Point3D>& A, const std::vector<Point3D>& B, double eps = 1e-6) {
        std::vector<Point3D> result;

        // A 中不在 B 中
        for (const auto& pa : A) {
            bool found = false;
            for (const auto& pb : B) {
                if (isSamePoint(pa, pb, eps)) {
                    found = true;
                    break;
                }
            }
            if (!found) result.push_back(pa);
        }

        // B 中不在 A 中
        for (const auto& pb : B) {
            bool found = false;
            for (const auto& pa : A) {
                if (isSamePoint(pa, pa, eps)) {
                    found = true;
                    break;
                }
            }
            if (!found) result.push_back(pb);
        }

        return result;
    }

    static inline uint64_t pack_cell_key(int32_t bx, int32_t by) {
        // 把两个 32-bit 有符号整数打包到一个 uint64_t（保留 bitwise 表示）
        return (static_cast<uint64_t>(static_cast<uint32_t>(bx)) << 32) | static_cast<uint64_t>(static_cast<uint32_t>(by));
    }

    void pub_wall_points(sensor_msgs::msg::PointCloud2& wall_points_in_odom) {
        static bool                          is_init = false;
        static sensor_msgs::msg::PointCloud2 wall_point_clouds;

        if (!is_init) {
            // 如果是第一帧，直接用它来初始化累积点云
            wall_point_clouds = wall_points_in_odom;
            is_init = true;
        } else {
            // 检查点云结构是否一致
            if (wall_point_clouds.fields != wall_points_in_odom.fields || wall_point_clouds.point_step != wall_points_in_odom.point_step) {
                RCLCPP_ERROR(this->get_logger(), "Wall Point cloud fields or point_step do not match. Cannot merge.");
                return;
            }

            // 获取旧数据的大小
            size_t old_data_size = wall_point_clouds.data.size();
            // 获取新数据的大小
            size_t new_data_size = wall_points_in_odom.data.size();

            // 调整累积点云数据区的大小以容纳新数据
            wall_point_clouds.data.resize(old_data_size + new_data_size);

            // 将新数据拷贝到累积点云数据的末尾
            std::copy(wall_points_in_odom.data.begin(), wall_points_in_odom.data.end(), wall_point_clouds.data.begin() + old_data_size);

            // 更新元数据
            wall_point_clouds.width += wall_points_in_odom.width;
            wall_point_clouds.row_step = wall_point_clouds.width * wall_point_clouds.point_step;
        }

        // 更新时间戳并发布
        wall_point_clouds.header.stamp = this->get_clock()->now();
        wall_clouds_pub_->publish(wall_point_clouds);
    }

    void pub_gap_wall_points(sensor_msgs::msg::PointCloud2& gap_wall_points_in_odom) {
        static bool                          is_init = false;
        static sensor_msgs::msg::PointCloud2 gap_wall_point_clouds;

        if (!is_init) {
            // 如果是第一帧，直接用它来初始化累积点云
            gap_wall_point_clouds = gap_wall_points_in_odom;
            is_init = true;
        } else {
            // 检查点云结构是否一致
            if (gap_wall_point_clouds.fields != gap_wall_points_in_odom.fields ||
                gap_wall_point_clouds.point_step != gap_wall_points_in_odom.point_step) {
                RCLCPP_ERROR(this->get_logger(),
                                "Gap Wall Point cloud fields or point_step do not match. Cannot merge.");
                return;
            }

            // 获取旧数据的大小
            size_t old_data_size = gap_wall_point_clouds.data.size();
            // 获取新数据的大小
            size_t new_data_size = gap_wall_points_in_odom.data.size();

            // 调整累积点云数据区的大小以容纳新数据
            gap_wall_point_clouds.data.resize(old_data_size + new_data_size);

            // 将新数据拷贝到累积点云数据的末尾
            std::copy(gap_wall_points_in_odom.data.begin(), gap_wall_points_in_odom.data.end(), gap_wall_point_clouds.data.begin() + old_data_size);

            // 更新元数据
            gap_wall_point_clouds.width += gap_wall_points_in_odom.width;
            gap_wall_point_clouds.row_step = gap_wall_point_clouds.width * gap_wall_point_clouds.point_step;
        }

        // 更新时间戳并发布
        gap_wall_point_clouds.header.stamp = this->get_clock()->now();
        gap_wall_clouds_pub_->publish(gap_wall_point_clouds);
    }

    void pub_ground_points(sensor_msgs::msg::PointCloud2& ground_points_in_odom) {
        static bool                          is_init = false;
        static sensor_msgs::msg::PointCloud2 ground_point_clouds;

        if (!is_init) {
            // 如果是第一帧，直接用它来初始化累积点云
            ground_point_clouds = ground_points_in_odom;
            is_init = true;
        } else {
            // 检查点云结构是否一致
            if (ground_point_clouds.fields != ground_points_in_odom.fields ||
                ground_point_clouds.point_step != ground_points_in_odom.point_step) {
                RCLCPP_ERROR(this->get_logger(),
                                "Gap Wall Point cloud fields or point_step do not match. Cannot merge.");
                return;
            }

            // 获取旧数据的大小
            size_t old_data_size = ground_point_clouds.data.size();
            // 获取新数据的大小
            size_t new_data_size = ground_points_in_odom.data.size();

            // 调整累积点云数据区的大小以容纳新数据
            ground_point_clouds.data.resize(old_data_size + new_data_size);

            // 将新数据拷贝到累积点云数据的末尾
            std::copy(ground_points_in_odom.data.begin(), ground_points_in_odom.data.end(), ground_point_clouds.data.begin() + old_data_size);

            // 更新元数据
            ground_point_clouds.width += ground_points_in_odom.width;
            ground_point_clouds.row_step = ground_point_clouds.width * ground_point_clouds.point_step;
        }

        // 更新时间戳并发布
        ground_point_clouds.header.stamp = this->get_clock()->now();
        ground_cloud_pub_->publish(ground_point_clouds);
    }

    size_t find_index_for_angle(const sensor_msgs::msg::LaserScan& laser, float angle_rad) {
        int index = static_cast<int>((angle_rad - laser.angle_min) / laser.angle_increment);
        // 确保索引不会超出范围
        if (index < 0) return 0;
        if (static_cast<size_t>(index) >= laser.ranges.size()) return laser.ranges.size() - 1;
        return static_cast<size_t>(index);
    }

    void PublishCylinder(float x, float y, float z,  // 圆柱体中心坐标
                         float scale_x, float scale_y,
                         float scale_z,                       // 圆柱体尺寸 (x,y直径, z高度)
                         float r, float g, float b, float a,  // 颜色和透明度
                         int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";  // 你要显示在哪个坐标系下
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "my_cylinders";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置位置
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // 设置尺寸
        marker.scale.x = scale_x;  // 直径 (x方向)
        marker.scale.y = scale_y;  // 直径 (y方向)
        marker.scale.z = scale_z;  // 高度 (z方向)

        // 设置颜色
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;

        marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 表示永久

        marker_pub_->publish(marker);
    }

    void handle_bump() {
        geometry_msgs::msg::Twist twist_msg;
        float                    time_in_align = (this->get_clock()->now() - align_start_time_).seconds();

        float rotation_duration = 3.0;  // 默认旋转时间（约90度）

        // 根据碰撞位置调整旋转时间
        if (hazard_frame_id_ == "bump_left") {
            rotation_duration = 5.0;
        } else if (hazard_frame_id_ == "bump_front_left") {
            rotation_duration = 3.5;
        } else if (hazard_frame_id_ == "bump_front_center") {
            rotation_duration = 3.0;
        } else if (hazard_frame_id_ == "bump_front_right") {
            rotation_duration = 2.5;
        } else if (hazard_frame_id_ == "bump_right") {
            rotation_duration = 1.5;
        }

        float forward_duration = 0.5;
        // create3mcu有一个强制后退的动作，在这里抵消掉
        if (time_in_align < forward_duration) {
            twist_msg.linear.x = 0.08; // 前进
            twist_msg.angular.z = 0.0;
        } else {
            if (time_in_align < rotation_duration) {
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.6;  // 左转
            } else {
                RCLCPP_INFO(this->get_logger(), "对准完成。切换到 WALL_FOLLOWING 状态。");
                // stop_robot();
                hazard_type_ = 0;
                current_state_ = State::WALL_FOLLOWING;
            }
        }

        cmd_publisher_->publish(twist_msg);
    }

    void stop_robot() {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);
    }

    float Distance3D(const Point3D& p1, const Point3D& p2) {
        return std::sqrt(
            (p2.x - p1.x) * (p2.x - p1.x) +
            (p2.y - p1.y) * (p2.y - p1.y) +
            (p2.z - p1.z) * (p2.z - p1.z)
        );
    }

    size_t get_index_from_angle(const sensor_msgs::msg::LaserScan &scan, double angle_rad) {
        // 将角度转换为激光雷达数据数组中的索引
        int index = static_cast<int>((scan.angle_min + angle_rad) / scan.angle_increment);
        return std::max(0, std::min(static_cast<int>(scan.ranges.size() - 1), index));
    }

    float avg_distance_cal(const sensor_msgs::msg::LaserScan::SharedPtr scan, size_t start, size_t end) {
        float dis = -1.0f;

        std::vector<float> values(scan->ranges.begin() + start, scan->ranges.begin() + end);
        float              sum = 0.0f;
        int                count = 0;
        for (float val : values) {
            if (!std::isnan(val) && !std::isinf(val)) {
                sum += val;
                ++count;
            }
        }
        if (count == 0) return -1;
        dis = sum / count;

        return dis;
    }

    float min_distance_cal(const sensor_msgs::msg::LaserScan::SharedPtr scan, size_t start, size_t end) {
        float dis = std::numeric_limits<float>::infinity();

        std::vector<float> values(scan->ranges.begin() + start, scan->ranges.begin() + end);
        for (float val : values) {
            if (!std::isnan(val) && val < dis) {
                dis = val;
            }
        }

        return dis;
    }

    void SaveOdomPoints3DToTxt(const std::vector<Point3D>& points, const std::string& file_path) {
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
            ofs << p.x << " " << p.y << " " << p.z << "\n";
        }

        ofs.close();
    }

    void SaveLaserPoints3DToTxt(const std::vector<Point3D>& points, const std::string& file_path) {
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
            ofs << p.x << " " << p.y << " " << p.z << "\n";
        }

        ofs.close();       
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                    scan_laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                    line_laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::WheelVels>::SharedPtr             wheel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                          imu_sub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   wall_clouds_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   gap_wall_clouds_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   ground_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;

    rclcpp::Time                 rotation_start_time_;
    rclcpp::Time                 align_start_time_;
    rclcpp::Time                 odom_last_time_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr line_laser_process_timer_;

    laser_geometry::LaserProjection projector_;

    std::unique_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    Point2D                                target_point_;
    sensor_msgs::msg::LaserScan            line_laser_;
    sensor_msgs::msg::LaserScan            scan_laser_;

    std::mutex line_lock_;
    std::mutex scan_lock_;

    State              current_state_ = State::SEARCHING;
    std::string        hazard_frame_id_;
    uint8_t            hazard_type_;
    std::string        target_frame_ = "odom";
    float              scan_front_dis_ = -1.0f;
    bool               has_min_dist_ = false;
    bool               search_right_ = false;
    bool               turn_step_final_ = false;
    int                forward_compensation_count_ = 0;
    float              linear_x_ = 0.0f, angular_z_ = 0.0f;
    float              x_ = 0.0f, y_ = 0.0f, yaw_ = 0.0f;
    float              imu_yaw_ = 0.0f;

    // yaml param
    float wheel_base_;
    int   window_size_;
    float sigma_;
    float wall_distance_;
    float following_wall_linear_;
    bool  state_loop_;
    bool  pub_nearby_point_;
    bool  right_angle_difference_method_;
    bool  linear_fitting_method_;
    bool  interpolate_gap_points_;
    bool  use_moving_average_filter_;
    bool  save_3Dpoints_to_txt_;
    bool  line_laser_process_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EdgeFollower>());
    rclcpp::shutdown();
    return 0;
}