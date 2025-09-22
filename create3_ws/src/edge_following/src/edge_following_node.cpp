#include <cmath>
#include <functional>
#include <iostream>
#include <vector>
#include <string>
#include <mutex>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "laser_geometry/laser_geometry.hpp"

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

    Point2D operator+(const Point2D &other) const { return Point2D(x + other.x, y + other.y); }

    Point2D &operator+=(const Point2D &other) {
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

        line_laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/line_scan", 10, std::bind(&EdgeFollower::line_laser_callback, this, _1));

        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>("/hazard_detection", rclcpp::SensorDataQoS(),
                                                                                                std::bind(&EdgeFollower::hazard_callback, this, _1));

        // point cloud pub
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_clouds", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&EdgeFollower::control_loop, this));
        dis_to_wall_ = -1.0f;
        has_min_dist_ = false;
        front_dist_ = -1.0f;

        RCLCPP_INFO(this->get_logger(), "Wall follower node has been started.");
    }

private:
    void line_laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->ranges.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "no line lidar data!");
            dis_to_wall_ = -1.0f;
            return;
        }
        // 使用线激光的中心读数作为墙距
        std::vector<float> head(msg->ranges.begin(), msg->ranges.begin() + 5);
        std::vector<float> tail(msg->ranges.end() - 5, msg->ranges.end());
        head.insert(head.end(), tail.begin(), tail.end());

        float sum = 0.0f;
        int count = 0;
        for (float val : head) {
            if (!std::isnan(val)) {
                sum += val;
                ++count;
            }
        }

        if (count == 0) {
            RCLCPP_WARN(this->get_logger(), "have no valid data!");
            return;
        }

        dis_to_wall_ = sum / count;

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

        for (const auto &detection : msg->detections) {
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

        if (!has_min_dist_) {
            int min_angle_index = -1;
            float min_dist = std::numeric_limits<float>::infinity();
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                if (msg->ranges[i] < min_dist && msg->ranges[i] > 0) {
                    min_dist = msg->ranges[i];
                    has_min_dist_ = true;
                    min_angle_index = i;
                }
            }

            if (min_angle_index != -1) {
                float target_angle = msg->angle_min + min_angle_index * msg->angle_increment;

                // 激光雷达坐标系坐标点
                Point2D laser_point = Point2D(min_dist * cos(target_angle), min_dist * sin(target_angle));

                // pointstamped类型存储数据
                point_in_laser_.header = msg->header;
                point_in_laser_.header.stamp = rclcpp::Time(0);

                point_in_laser_.point.x = laser_point.x;
                point_in_laser_.point.y = laser_point.y;
                point_in_laser_.point.z = 0.0;
            }
        }

        front_dist_ = msg->ranges[0];

        // 发布最近点
        PublishCylinder(target_point_.x, target_point_.y, 0.1, 0.03, 0.03, 0.3, 0.0, 1.0, 0.0, 1.0, 0);
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
        }
        // rclcpp::Time now = this->get_clock()->now();
        // RCLCPP_INFO(this->get_logger(), "ROS time: %ld.%09ld", now.seconds(), now.nanoseconds());
    }

    // 状态1: 寻找最近的点
    void handle_searching() {
        if (has_min_dist_) {
            geometry_msgs::msg::PointStamped point_in_odom;
            // 转换到odom
            try {
                point_in_odom = tf_buffer_->transform(point_in_laser_, "odom");
                target_point_.x = point_in_odom.point.x;
                target_point_.y = point_in_odom.point.y;
                current_state_ = State::DRIVING_TO_WALL;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "无法从激光坐标转换到里程计坐标：%s", ex.what());
            }
        }
    }

    // 状态2: 转向并移动到墙前
    void handle_driving_to_wall() {
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
            if (std::fabs(target_yaw) > 0.04) {
                twist_msg.angular.z = (target_yaw > 0) ? 0.3 : -0.3;
                RCLCPP_INFO(this->get_logger(), "Rotating. Target yaw: %.2f", target_yaw);
            } else if (front_dist_ > 0.4) {
                twist_msg.linear.x = 0.1;
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

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "tran base_link tf err:%s", ex.what());
        }
    }

    // 状态3: 左转90度
    void handle_rotating_left() {
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
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 0.1;

        // 如果墙壁丢失（例如外角），则右转寻找
        if (dis_to_wall_ > 0.09 || std::isinf(dis_to_wall_) || std::isnan(dis_to_wall_)) {
            RCLCPP_WARN(this->get_logger(), "墙壁丢失，正在右转寻找。");
            twist_msg.angular.z = -0.6; // 右转
            twist_msg.linear.x = 0.1;
        } else {
            // P控制器维持距离
            float error = 0.05 - dis_to_wall_;
            twist_msg.angular.z = 1.2 * error;
            RCLCPP_WARN(this->get_logger(), "调整距离");
        }
        
        RCLCPP_INFO(this->get_logger(), "循边中... 墙距: %.2f, 角速度: %.2f", dis_to_wall_, twist_msg.angular.z);

        // 处理碰撞
        if (hazard_type_ == 1) {
            handle_bump();
        } else {
            publisher_->publish(twist_msg);
        }
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
        double time_in_align = (this->get_clock()->now() - align_start_time_).seconds();
        
        double rotation_duration = 2.0; // 默认旋转时间（约90度）

        // 根据碰撞位置调整旋转时间
        if (hazard_frame_id_ == "bump_left") {
            rotation_duration = 4.5;
        } else if (hazard_frame_id_ == "bump_front_left") {
            rotation_duration = 3.5;
        } else if (hazard_frame_id_ == "bump_front_center") {
            rotation_duration = 2.5;
        } else if (hazard_frame_id_ == "bump_front_right") {
            rotation_duration = 1.5;
        } else if (hazard_frame_id_ == "bump_right") {
            rotation_duration = 0.5;
        }
        
        if (time_in_align < rotation_duration) {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.6; // 左转
        } else {
            RCLCPP_INFO(this->get_logger(), "对准完成。切换到 WALL_FOLLOWING 状态。");
            // stop_robot();
            hazard_type_ = 0;
            current_state_ = State::WALL_FOLLOWING;
        }
        publisher_->publish(twist_msg);
    }

    void stop_robot() {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        publisher_->publish(twist_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                         publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                    subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                    line_laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr                   marker_pub_;
    State                                                                           current_state_ = State::SEARCHING;
    rclcpp::Time                                                                    rotation_start_time_;
    rclcpp::Time align_start_time_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    laser_geometry::LaserProjection                              projector_;

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PointStamped point_in_laser_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    Point2D target_point_;

    std::string hazard_frame_id_;
    uint8_t hazard_type_;

    sensor_msgs::msg::LaserScan line_laser_;
    float dis_to_wall_;
    float front_dist_;
    bool has_min_dist_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EdgeFollower>());
    rclcpp::shutdown();
    return 0;
}