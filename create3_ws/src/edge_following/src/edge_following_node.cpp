#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

// 定义状态机的不同状态
enum class State { SEARCHING, DRIVING_TO_WALL, ROTATING_LEFT, WALL_FOLLOWING };

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

class EdgeFollower : public rclcpp::Node {
public:
    EdgeFollower() : Node("edge_follower_node") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&EdgeFollower::scan_callback, this, _1));

        line_laser_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>("/line_scan", 10, std::bind(&EdgeFollower::line_laser_callback, this, _1));

        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>("/hazard_detection", rclcpp::SensorDataQoS(),
                                                                                                std::bind(&EdgeFollower::hazard_callback, this, _1));

        // point cloud pub
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_clouds", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&EdgeFollower::control_loop, this));
        line_right_dis_ = -1.0f;
        scan_right_dis_ = -1.0f;
        has_min_dist_ = false;
        front_dist_ = -1.0f;
        scan_right_front_dis_ = -1.0f;
        scan_right_back_dis_ = -1.0f;
        scan_front_dis_ = -1.0f;

        front_obstacle_dist_ = 0.3;
        follow_distance_ = 0.2;

        RCLCPP_INFO(this->get_logger(), "Wall follower node has been started.");
    }

private:
    void line_laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 多线程访问需要加锁
        line_laser_ = msg;

        if (msg->ranges.size() < 10 || current_state_ != State::WALL_FOLLOWING) {
            line_right_dis_ = -1.0f;
            return;
        }
        // 使用线激光的中心读数作为墙距
        size_t minus_5_deg = get_index_from_angle(msg, 359.913); // -5
        size_t positive_5_deg = get_index_from_angle(msg, 0.087);   // 5

        std::vector<float> head(msg->ranges.begin(), msg->ranges.begin() + positive_5_deg);
        std::vector<float> tail(msg->ranges.begin(), msg->ranges.begin() + minus_5_deg);
        head.insert(head.end(), tail.begin(), tail.end());

        float sum = 0.0f;
        int   count = 0;
        for (float val : head) {
            if (!std::isnan(val)) {
                sum += val;
                ++count;
            }
        }

        if (count == 0) {
            // RCLCPP_WARN(this->get_logger(), "have no valid data!");
            line_right_dis_ = -1.0f;
            return;
        }

        line_right_dis_ = sum / count;

        // sensor_msgs::msg::LaserScan scan = *msg;
        // int scan_size = scan.ranges.size();
        // std::fill(scan.ranges.begin() + 5, scan.ranges.end() - 5, std::numeric_limits<float>::quiet_NaN());
        // sensor_msgs::msg::PointCloud2 laser_cloud;
        // sensor_msgs::msg::PointCloud2 odom_cloud;
        // try {
        //     // laserscan -> pointcloud in laser
        //     projector_.transformLaserScanToPointCloud(scan.header.frame_id, scan, laser_cloud, *tf_buffer_);
        //     // in odom
        //     geometry_msgs::msg::TransformStamped transform_stamped =
        //         tf_buffer_->lookupTransform(
        //             "odom", msg->header.frame_id, msg->header.stamp,
        //             rclcpp::Duration::from_seconds(0.2));
        //     tf2::doTransform(laser_cloud, odom_cloud,
        //                      transform_stamped);
        //     // pub
        //     cloud_pub_->publish(odom_cloud);
        // } catch (const tf2::TransformException &ex) {
        //     RCLCPP_WARN(this->get_logger(), "trans failed:%s", ex.what());
        // }
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
            scan_laser_ = msg;
        }

        front_dist_ = msg->ranges[0];

        if (msg->ranges.size() < 10) {
            scan_right_dis_ = -1.0f;
            return;
        }

        // scan右侧沿墙距离
        size_t minus_135_deg = get_index_from_angle(msg, 3.93);      // -135
        size_t minus_90_deg = get_index_from_angle(msg, 4.71);       // -90
        size_t minus_80_deg = get_index_from_angle(msg, 4.887);      // -80
        size_t minus_45_deg = get_index_from_angle(msg, 5.5);        // -45
        size_t minus_22_5_deg = get_index_from_angle(msg, 5.89);     // -22.5
        size_t positive_22_5_deg = get_index_from_angle(msg, 0.39);  // 22.5
        size_t positive_360_deg = get_index_from_angle(msg, 6.28);   // 360

        // 范围内平均距离
        scan_right_dis_ = avg_distance_cal(msg, minus_90_deg, minus_80_deg);
        scan_right_front_dis_ = avg_distance_cal(msg, minus_90_deg, minus_45_deg);
        scan_right_back_dis_ = avg_distance_cal(msg, minus_135_deg, minus_90_deg);
        float front_dis_0_left = avg_distance_cal(msg, 0, positive_22_5_deg);
        float front_dis_0_right = avg_distance_cal(msg, minus_22_5_deg, positive_360_deg);
        scan_front_dis_ = (front_dis_0_left + front_dis_0_right) / 2;

        // 范围内最小距离
        scan_right_min_dis_ = min_distance_cal(msg, minus_135_deg, minus_45_deg);


        // 发布最近点
        // PublishCylinder(target_point_.x, target_point_.y, 0.1, 0.03, 0.03, 0.3, 0.0, 1.0, 0.0, 1.0, 0);

// 凹轮廓线检测
#if 0
        sensor_msgs::msg::LaserScan scan = *msg;
        std::fill(scan.ranges.begin(), scan.ranges.end(), std::numeric_limits<float>::quiet_NaN());

        // 然后保留 -135° 到 -45° 的范围
        std::copy(msg->ranges.begin() + minus_135_deg, msg->ranges.begin() + minus_45_deg,
                scan.ranges.begin() + minus_135_deg);
        // 轮廓线检测
        sensor_msgs::msg::PointCloud2 laser_cloud;
        sensor_msgs::msg::PointCloud2 odom_cloud;
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            // laserscan -> pointcloud in laser
            projector_.transformLaserScanToPointCloud(scan.header.frame_id, scan, laser_cloud, *tf_buffer_);
            // in odom
            transform_stamped =
                tf_buffer_->lookupTransform("odom", scan.header.frame_id, scan.header.stamp, rclcpp::Duration::from_seconds(0.2));
            tf2::doTransform(laser_cloud, odom_cloud, transform_stamped);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "point clouds trans failed:%s", ex.what());
        }

        tf2::Transform tf;
        tf2::fromMsg(transform_stamped.transform, tf);
        std::vector<Point2D>                         points;

        float angle = scan.angle_min;
        for (auto r : scan.ranges) {
            if (r < scan.range_min || r > scan.range_max) {
                angle += scan.angle_increment;
                continue;
            }

            tf2::Vector3 p_laser(r * cos(angle), r * sin(angle), 0.0);
            tf2::Vector3 p_odom = tf * p_laser;  // 转到odom
            points.push_back({(float)p_odom.x(), (float)p_odom.y()});
            angle += scan.angle_increment;
        }

        if (points.empty()) {
            std::cout << "empty" << std::endl;
        }

        // 计算凸包作为房间轮廓
        auto hull = concaveHull(points);

        // 发布到 RViz
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "odom";
        line_strip.header.stamp = this->now();
        line_strip.ns = "room_contour";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.02;  // 线宽
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;

        for (auto& p : hull) {
            if (std::isnan(p.x) || std::isnan(p.y)) continue;
            if (std::isinf(p.x) || std::isinf(p.y)) continue;
            geometry_msgs::msg::Point pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        // 闭合轮廓
        // if (!hull.empty()) {
        //     geometry_msgs::msg::Point pt;
        //     pt.x = hull[0].x;
        //     pt.y = hull[0].y;
        //     pt.z = 0.0;
        //     line_strip.points.push_back(pt);
        // }

        marker_pub_->publish(line_strip);
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

        scan_right_dis_ = (right_min_dist == std::numeric_limits<double>::infinity()) ? -1.0 : right_min_dist;
        std::cout << "scan_right_dis_: " << scan_right_dis_ << std::endl;
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
            cloud_pub_->publish(odom_cloud);
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

    // 计算凸包 (Graham scan / Andrew 算法)
    std::vector<Point2D> TconvexHull(std::vector<Point2D>& pts) {
        if (pts.size() < 3) return pts;
        std::sort(pts.begin(), pts.end(), [](const Point2D& a, const Point2D& b) { return a.x == b.x ? a.y < b.y : a.x < b.x; });
        std::vector<Point2D> hull;
        auto cross = [](const Point2D& O, const Point2D& A, const Point2D& B) { return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x); };
        // 下凸壳
        for (auto& p : pts) {
            while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0) {
                hull.pop_back();
            }
            hull.push_back(p);
        }
        // 上凸壳
        size_t t = hull.size() + 1;
        for (int i = pts.size() - 1; i >= 0; i--) {
            auto& p = pts[i];
            while (hull.size() >= t && cross(hull[hull.size() - 2], hull.back(), p) <= 0) {
                hull.pop_back();
            }
            hull.push_back(p);
        }
        hull.pop_back();
        return hull;
    }

    // 计算两点距离
    float dist2(const Point2D& a, const Point2D& b) {
        return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
    }

    // 极角排序
    std::vector<Point2D> concaveHull(std::vector<Point2D>& pts, float k = 10) {
        if (pts.size() < 3) return pts;

        // 1. 找到最下方点作为起点
        auto start_it = std::min_element(pts.begin(), pts.end(),
                                        [](const Point2D& a, const Point2D& b) { return a.y < b.y; });
        Point2D start = *start_it;

        std::vector<Point2D> hull;
        hull.push_back(start);
        Point2D current = start;
        // Point2D previous = {start.x - 1.0f, start.y};  // 用左边点初始化前一条边方向

        pts.erase(start_it);

        while (!pts.empty()) {
            // 2. 找 k 个最近点
            std::sort(pts.begin(), pts.end(),
                    [this, &current](const Point2D& a, const Point2D& b) { return dist2(a, current) < dist2(b, current); });
            size_t n = std::min((size_t)k, pts.size());

            // 3. 选择最合适的点（保持逆时针/顺时针方向）
            bool found = false;
            for (size_t i = 0; i < n; i++) {
                Point2D candidate = pts[i];
                // 可以加简单交叉判断避免自交，这里简化
                hull.push_back(candidate);
                current = candidate;
                pts.erase(pts.begin() + i);
                found = true;
                break;
            }
            if (!found) break;
        }

        return hull;
    }

    void control_loop() {
#if 1
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
        }
        // rclcpp::Time now = this->get_clock()->now();
        // RCLCPP_INFO(this->get_logger(), "ROS time: %ld.%09ld", now.seconds(), now.nanoseconds());
#endif
    }

    // 状态1: 寻找最近的点
    void handle_searching() {
        std::cout << "search" << std::endl;
        std::lock_guard<std::mutex> lock(scan_lock_);
        if (!scan_laser_) {
            std::cout << "scan_laser_ is nullptr" << std::endl;
            return;
        }
        if (scan_laser_->ranges.size() < 10) {
            std::cout << "no data" << std::endl;
            return;
        }

        // scan最近点
        if (!has_min_dist_) {
            int   min_index = -1;
            float min_dist = std::numeric_limits<float>::infinity();
            if (!search_right_) {
                for (size_t i = 0; i < scan_laser_->ranges.size(); ++i) {
                    if (scan_laser_->ranges[i] < min_dist && scan_laser_->ranges[i] > 0) {
                        min_dist = scan_laser_->ranges[i];
                        min_index = i;
                    }
                }
            } else {
                sensor_msgs::msg::LaserScan scan = *scan_laser_;
                size_t                      minus_135_deg = get_index_from_angle(scan_laser_, 3.93);  // -135
                size_t                      minus_45_deg = get_index_from_angle(scan_laser_, 5.5);    // -45
                std::vector<float>          search_range(scan.ranges.begin() + minus_135_deg, scan.ranges.begin() + minus_45_deg);
                for (size_t i = 0; i < search_range.size(); ++i) {
                    if (search_range[i] < min_dist && search_range[i] > 0) {
                        min_dist = search_range[i];
                        min_index = i + minus_135_deg;
                    }
                }
            }

            if (min_index != -1) {
                float target_angle = scan_laser_->angle_min + min_index * scan_laser_->angle_increment;

                // 激光雷达坐标系坐标点
                Point2D                          laser_point = Point2D(min_dist * cos(target_angle), min_dist * sin(target_angle));
                geometry_msgs::msg::PointStamped point_in_laser;
                // pointstamped类型存储数据
                point_in_laser.header = scan_laser_->header;
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
                } catch (const tf2::TransformException& ex) {
                    RCLCPP_WARN(this->get_logger(), "无法从激光坐标转换到里程计坐标：%s", ex.what());
                }

                if (!search_right_) {
                    current_state_ = State::DRIVING_TO_WALL;
                } else {
                    current_state_ = State::WALL_FOLLOWING;
                    search_right_ = false;
                }
                has_min_dist_ = true;
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
            } else if (front_dist_ > 0.22) {
                twist_msg.linear.x = 0.1;
                turn_step_final_ = true;
                RCLCPP_INFO(this->get_logger(), "Moving forward. Distance to wall: %f", front_dist_);
            } else {
                twist_msg.linear.x = 0;
                twist_msg.angular.z = 0;
                RCLCPP_INFO(this->get_logger(), "Reached wall. Preparing to rotate left.");
                current_state_ = State::ROTATING_LEFT;

                rotation_start_time_ = this->get_clock()->now();
            }

            // 处理碰撞
            if (hazard_type_ == 1) {
                handle_bump();
            } else {
                publisher_->publish(twist_msg);
            }

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
        publisher_->publish(twist_msg);
    }

    // 状态4: 沿边算法
    void handle_wall_following() {
        std::cout << "following" << std::endl;

        if (!scan_laser_) {
            std::cout << "scan_laser_ is nullptr" << std::endl;
            return;
        }
        if (scan_laser_->ranges.size() < 10) {
            std::cout << "no data" << std::endl;
            return;
        }

        float wall_distance_ = 0.23f;
        float kp_distance_ = 5.0f;
        float kp_angle_ = 2.0f;
        float linear_velocity_ = 0.05f;

        float center_angle_rad = 270.0 * M_PI / 180.0;
        float scan_range_rad = 60.0 * M_PI / 180.0;

        size_t start_index = find_index_for_angle(scan_laser_, center_angle_rad - scan_range_rad / 2.0);
        size_t end_index = find_index_for_angle(scan_laser_, center_angle_rad + scan_range_rad / 2.0);

        // 寻找此范围内的最近点
        float min_dist = std::numeric_limits<float>::max();
        size_t min_index = 0;
        for (size_t i = start_index; i <= end_index; ++i) {
            // 确保索引在有效范围内
            if (i >= scan_laser_->ranges.size()) continue;
            
            if (std::isfinite(scan_laser_->ranges[i]) && scan_laser_->ranges[i] < min_dist && scan_laser_->ranges[i] > 0) {
                min_dist = scan_laser_->ranges[i];
                min_index = i;
            }
        }

        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();

        if (min_dist == std::numeric_limits<float>::max()) {
            // 如果右侧没有检测到任何东西，让机器人旋转寻找墙壁
            twist_msg->linear.x = 0.0;
            twist_msg->angular.z = -0.3;
        } else {
            // 距离误差: (目标距离 - 当前距离)
            double distance_error = wall_distance_ - min_dist;

            // 通过看最近的点偏离了-90度多远来计算角度误差。
            double closest_point_angle = scan_laser_->angle_min + scan_laser_->angle_increment * min_index;
            double angle_error = center_angle_rad - closest_point_angle;

            // PID控制器 (这里只用了P - 比例控制)
            // 如果机器人离墙太近 (distance_error > 0), 需要向左转 (angular.z > 0)
            // 如果机器人离墙太远 (distance_error < 0), 需要向右转 (angular.z < 0)
            // 如果机器人车头朝向墙 (angle_error > 0), 需要向右转 (angular.z < 0)
            // 如果机器人车头背离墙 (angle_error < 0), 需要向左转 (angular.z > 0)
            // 注意：kp_angle的符号需要调整以匹配这个逻辑
            double angular_z = kp_distance_ * distance_error - kp_angle_ * angle_error;
            
            twist_msg->linear.x = linear_velocity_;
            twist_msg->angular.z = angular_z;
            
            RCLCPP_INFO(this->get_logger(), "Dist: %.2f, DistErr: %.2f, AngleErr: %.2f, AngularZ: %.2f",
                min_dist, distance_error, angle_error, angular_z);
        }

        // 处理碰撞
        if (hazard_type_ == 1) {
            handle_bump();
            return;
        } else if (scan_front_dis_ < 0.23) {
            rotation_start_time_ = this->get_clock()->now();
            current_state_ = State::ROTATING_LEFT;
            return;
        }

        publisher_->publish(std::move(twist_msg));
    }

    size_t find_index_for_angle(const sensor_msgs::msg::LaserScan::SharedPtr msg, double angle_rad) {
        int index = static_cast<int>((angle_rad - msg->angle_min) / msg->angle_increment);
        // 确保索引不会超出范围
        if (index < 0) return 0;
        if (static_cast<size_t>(index) >= msg->ranges.size()) return msg->ranges.size() - 1;
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
        double                    time_in_align = (this->get_clock()->now() - align_start_time_).seconds();

        double rotation_duration = 3.0;  // 默认旋转时间（约90度）

        // 根据碰撞位置调整旋转时间
        if (hazard_frame_id_ == "bump_left") {
            rotation_duration = 5.0;
        } else if (hazard_frame_id_ == "bump_front_left") {
            rotation_duration = 3.0;
        } else if (hazard_frame_id_ == "bump_front_center") {
            rotation_duration = 2.5;
        } else if (hazard_frame_id_ == "bump_front_right") {
            rotation_duration = 2.0;
        } else if (hazard_frame_id_ == "bump_right") {
            rotation_duration = 1.0;
        }

        if (time_in_align < rotation_duration) {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.6;  // 左转
        } else {
            RCLCPP_INFO(this->get_logger(), "对准完成。切换到 WALL_FOLLOWING 状态。");
            // stop_robot();
            hazard_type_ = 0;
            current_state_ = State::WALL_FOLLOWING;
        }
        publisher_->publish(twist_msg);
        after_bump_ = true;
    }

    void stop_robot() {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        publisher_->publish(twist_msg);
    }

    size_t get_index_from_angle(const sensor_msgs::msg::LaserScan::SharedPtr scan, double angle_rad) {
        // 将角度转换为激光雷达数据数组中的索引
        int index = static_cast<int>((scan->angle_min + angle_rad) / scan->angle_increment);
        return std::max(0, std::min(static_cast<int>(scan->ranges.size() - 1), index));
    }

    bool is_obstacle_in_front(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (!scan) {
            return false;
        }
        // 检查前方一个狭窄的扇区（例如-15到+15度）
        size_t start_index = get_index_from_angle(scan, 6.02);  // -15 degrees
        size_t end_index = get_index_from_angle(scan, 0.26);    // +15 degrees

        std::vector<float> head(scan->ranges.begin(), scan->ranges.begin() + end_index);
        std::vector<float> tail(scan->ranges.end() - start_index, scan->ranges.end());
        head.insert(head.end(), tail.begin(), tail.end());

        for (float val : head) {
            if (!std::isnan(val) && val < front_obstacle_dist_) {
                return true;
            }
        }

        return false;
    }

    float avg_distance_cal(const sensor_msgs::msg::LaserScan::SharedPtr scan, size_t start, size_t end) {
        float dis = -1.0f;

        std::vector<float> values(scan->ranges.begin() + start, scan->ranges.begin() + end);
        float              sum = 0.0f;
        int                count = 0;
        for (float val : values) {
            if (!std::isnan(val)) {
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

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                         publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                    subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                    line_laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr                   marker_pub_;
    State                                                                           current_state_ = State::SEARCHING;
    rclcpp::Time                                                                    rotation_start_time_;
    rclcpp::Time                                                                    align_start_time_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                     cloud_pub_;
    laser_geometry::LaserProjection                                                 projector_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    Point2D target_point_;

    std::string hazard_frame_id_;
    uint8_t     hazard_type_;

    sensor_msgs::msg::LaserScan::SharedPtr line_laser_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_laser_;
    std::mutex                             line_lock_;
    std::mutex                             scan_lock_;
    float                                  line_right_dis_;
    float                                  scan_right_dis_;
    float                                  front_dist_;
    float                                  scan_right_front_dis_;
    float                                  scan_right_back_dis_;
    float                                  scan_front_dis_;

    float                                  scan_right_min_dis_ = -0.1f;

    bool  has_min_dist_;
    float front_obstacle_dist_;
    float follow_distance_;

    bool after_bump_ = false;
    bool search_right_ = false;

    bool turn_step_final_ = false;

    float target_yaw_ = -1.0f;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EdgeFollower>());
    rclcpp::shutdown();
    return 0;
}