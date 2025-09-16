#include <cmath>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "visualization_msgs/msg/marker.hpp"

// 定义状态机的不同状态
enum class State { SEARCHING, DRIVING_TO_WALL, ROTATING_LEFT, WALL_FOLLOWING };

class Point2D {
public:
    float x;
    float y;

    Point2D() : x(0), y(0){}
    Point2D(float x_, float y_) : x(x_), y(y_) {}

    Point2D operator+(const Point2D& other) const {
        return Point2D(x + other.x, y + other.y);
    }

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

        subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&EdgeFollower::scan_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Wall follower node has been started.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        switch (current_state_) {
            case State::SEARCHING:
                handle_searching(msg);
                break;
            case State::DRIVING_TO_WALL:
                handle_driving_to_wall(msg);
                break;
            case State::ROTATING_LEFT:
                handle_rotating_left(msg);
                break;
            case State::WALL_FOLLOWING:
                handle_wall_following(msg);
                break;
        }

        // 发布圆柱体
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

    // 状态1: 寻找最近的点
    void handle_searching(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float min_dist = std::numeric_limits<float>::infinity();
        int   min_index = -1;

        if (min_index == -1) {
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                if (msg->ranges[i] < min_dist && msg->ranges[i] > 0) {
                    min_dist = msg->ranges[i];
                    min_index = i;
                }
            }
        }

        if (min_index != -1) {
            target_angle_ = msg->angle_min + min_index * msg->angle_increment;
            RCLCPP_INFO(this->get_logger(),
                        "Found nearest point at %f meters, angle %f radians. "
                        "Turning towards it.",
                        min_dist, target_angle_);

            // 激光雷达坐标系坐标点
            Point2D laser_point(min_dist * cos(target_angle_), min_dist * sin(target_angle_));

            // pointstamped类型存储数据
            geometry_msgs::msg::PointStamped point_in_laser;
            point_in_laser.header = msg->header;
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
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "无法转换坐标点：%s", ex.what());
            }

            current_state_ = State::DRIVING_TO_WALL;
        }

    }

    // 状态2: 转向并移动到墙前
    void handle_driving_to_wall(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;

        try {
            transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            double qx = transform.transform.rotation.x;
            double qy = transform.transform.rotation.y;
            double qz = transform.transform.rotation.z;
            double qw = transform.transform.rotation.w;

            // 四元数转 yaw
            double siny = 2.0 * (qw * qz + qx * qy);
            double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
            double current_yaw = std::atan2(siny, cosy);

            // 目标角度
            double target_yaw = std::atan2(target_point_.y, target_point_.x);

            // 固定角度范围
            double angle_diff = target_yaw - current_yaw;
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

            geometry_msgs::msg::Twist twist_msg;
            // 0度角向前
            float                     front_dist = msg->ranges[0];

            // 转向最近的点
            if (std::fabs(angle_diff) > 0.05) {
                twist_msg.angular.z = (angle_diff > 0) ? 0.3 : -0.3;
                RCLCPP_INFO(this->get_logger(), "Rotating. Current yaw: %.2f, Target yaw: %.2f, Diff: %.2f",
                            current_yaw, target_yaw, angle_diff);
            } else if (front_dist > 0.3) {
                twist_msg.linear.x = 0.1;
                RCLCPP_INFO(this->get_logger(), "Moving forward. Distance to wall: %f", front_dist);
            } else {
                twist_msg.linear.x = 0;
                twist_msg.angular.z = 0;
                RCLCPP_INFO(this->get_logger(), "Reached wall. Preparing to rotate left.");
                current_state_ = State::ROTATING_LEFT;

                rotation_start_time_ = this->get_clock()->now();
            }
            publisher_->publish(twist_msg);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "tf err:%s", ex.what());
        }
    }

    // 状态3: 左转90度
    void handle_rotating_left(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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

    // 状态4: 沿边算法 (一个简单的比例控制器)
    void handle_wall_following(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::Twist twist_msg;
        // 我们左转了，所以墙在右边。检查机器人右侧(3PI/2 或 -PI/2)的距离
        int right_index = (3 * M_PI / 2) / msg->angle_increment;
        if (right_index >= msg->ranges.size()) {
            right_index = msg->ranges.size() - 1;
        }
        float dist_to_wall = msg->ranges[right_index];

        // 沿墙距离的目标值
        float target_dist = 0.3;
        float error = dist_to_wall - target_dist;

        // 比例控制
        float angular_vel = -1.0 * error;  // 简单比例控制器, 负号是因为距离远了需要右转(负角速度)

        twist_msg.linear.x = 0.15;  // 保持一个较慢的前进速度
        twist_msg.angular.z = angular_vel;

        RCLCPP_INFO(this->get_logger(), "Wall following. Dist: %f, Error: %f, Angular vel: %f", dist_to_wall, error, angular_vel);
        publisher_->publish(twist_msg);
    }

    int find_min_dist_index(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float min_dist = std::numeric_limits<float>::infinity();
        int   min_index = -1;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < min_dist) {
                min_dist = msg->ranges[i];
                min_index = i;
            }
        }
        return min_index;
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

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    State                                                         current_state_ = State::SEARCHING;
    float                                                         target_angle_ = 0.0;
    rclcpp::Time                                                  rotation_start_time_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    Point2D target_point_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EdgeFollower>());
    rclcpp::shutdown();
    return 0;
}