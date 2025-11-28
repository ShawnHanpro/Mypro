/**
 * DWA Right Wall Follower for ROS 2 Galactic
 * * 功能：使用 2D 激光雷达数据，通过 DWA 算法实现右侧沿墙。
 * 约束：不可后退，右侧沿墙距离 0.25m。
 * 评分：基于预测轨迹后的 1.距离误差 2.角度误差 3.速度效益，并进行归一化处理。
 */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#define SHOW 0

// 辅助宏：角度归一化到 [-PI, PI] (虽然本例中主要用 atan2 计算，不强制需要，但保留是个好习惯)
inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

class DwaRightWallFollower : public rclcpp::Node {
public:
    DwaRightWallFollower() : Node("dwa_right_wall_follower") {
        // --- 1. 参数配置 ---

        // 机器人物理参数
        robot_diameter_ = 0.342;  // 342mm
        robot_radius_ = robot_diameter_ / 2.0;

        // 运动学约束
        max_lin_vel_ = 0.5;  // m/s
        max_ang_vel_ = 1.0;  // rad/s

        // 沿墙目标
        target_wall_dist_ = 0.25;  // 墙面距离 0.25m
        // 机器人中心到墙的目标距离 = 半径 + 间隙
        target_center_dist_ = robot_radius_ + target_wall_dist_;

        // DWA 模拟参数
        sim_time_ = 1.0;      // 向前模拟 1.5秒
        dwa_samples_v_ = 15;  // 线速度采样密度
        dwa_samples_w_ = 30;  // 角速度采样密度

        // 评分权重 (配合归一化使用)
        w_dist_ = 3.0;   // 距离保持权重 (最高优先级)
        w_head_ = 2.0;   // 平行姿态权重
        w_speed_ = 1.0;  // 速度权重

        // --- 2. ROS 通信接口 ---

        // 发布速度指令
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 订阅雷达数据 (使用 Best Effort QoS 以兼容某些雷达驱动)
        // auto qos = rclcpp::SensorDataQoS();
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DwaRightWallFollower::scanCallback, this, std::placeholders::_1));

        // 控制定时器 (20Hz = 50ms)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&DwaRightWallFollower::controlLoop, this));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("wall_point_clouds", 10);

        RCLCPP_INFO(this->get_logger(), "DWA Right Wall Follower Started.");
        RCLCPP_INFO(this->get_logger(), "Target Center Dist: %.3f m", target_center_dist_);
    }

private:
    // --- 数据结构定义 ---

    // 障碍物点信息
    struct Point {
        double x;
        double y;
        double dist;
        double angle;
    };

    // 轨迹候选者 (用于 DWA 归一化)
    struct TrajectoryCandidate {
        double v;
        double w;
        double raw_dist_cost;
        double raw_angle_cost;
        double raw_speed_cost;
        bool   valid;  // false 表示会碰撞
    };

    // --- 成员变量 ---
    double robot_diameter_;
    double robot_radius_;
    double max_lin_vel_;
    double max_ang_vel_;
    double target_wall_dist_;
    double target_center_dist_;

    double sim_time_;
    int    dwa_samples_v_;
    int    dwa_samples_w_;

    double w_dist_;
    double w_head_;
    double w_speed_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // 当前感知到的右侧最近障碍物
    std::shared_ptr<Point> right_closest_obstacle_ = nullptr;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      pub_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::TimerBase::SharedPtr                                 timer_;

    // --- 3. 雷达回调函数 ---
    // 逆时针index为0-360
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        double min_dist = std::numeric_limits<double>::max();
        double best_angle = 0.0;
        bool   found = false;

        // 定义右侧搜索范围 (弧度): -100度 到 -10度
        // -90度是正右方，稍微放宽一点范围以防雷达安装误差或墙面不平
        double search_min_angle = 210 * M_PI / 180.0;
        double search_max_angle = 330 * M_PI / 180.0;

        // 遍历雷达数据
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;

            // 范围过滤
            if (angle < search_min_angle || angle > search_max_angle) continue;

            // 有效性过滤
            double r = msg->ranges[i];
            if (r < msg->range_min || r > msg->range_max) continue;
            if (std::isinf(r) || std::isnan(r)) continue;

            // 找最近点
            if (r < min_dist) {
                min_dist = r;
                best_angle = angle;
                found = true;
            }
        }

        if (found) {
            if (!right_closest_obstacle_) right_closest_obstacle_ = std::make_shared<Point>();
            // 将极坐标转换为当前机器人坐标系 (Robot Frame) 下的 XY
            right_closest_obstacle_->dist = min_dist;
            right_closest_obstacle_->angle = best_angle;
            right_closest_obstacle_->x = min_dist * std::cos(best_angle);
            right_closest_obstacle_->y = min_dist * std::sin(best_angle);
        } else {
            right_closest_obstacle_ = nullptr;
        }

        // pub pointcloud2
#if SHOW
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = msg->header;
        cloud.height = 1;
        cloud.width = msg->ranges.size();
        cloud.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(msg->ranges.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        float angle = msg->angle_min;

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float r = msg->ranges[i];

            // 范围过滤
            if (angle < search_min_angle || angle > search_max_angle) continue;

            if (std::isfinite(r)) {
                *iter_x = r * std::cos(angle);
                *iter_y = r * std::sin(angle);
                *iter_z = 0.0f;
            } else {
                *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
            }

            ++iter_x;
            ++iter_y;
            ++iter_z;
            angle += msg->angle_increment;
        }

        cloud_pub_->publish(cloud);
#endif
    }

    // --- 4. 主控制循环 ---
    void controlLoop() {
        geometry_msgs::msg::Twist cmd;

        // 安全检查：如果没有检测到墙，停止
        if (!right_closest_obstacle_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            // 降低日志频率，避免刷屏
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No wall detected on right side.");
            pub_cmd_vel_->publish(cmd);
            return;
        }

        // 运行 DWA
        auto best_vel = computeDwa();

        cmd.linear.x = best_vel.first;
        cmd.angular.z = best_vel.second;
        std::cout << "best v: " << best_vel.first << std::endl;
        std::cout << "best w: " << best_vel.second << std::endl;
        pub_cmd_vel_->publish(cmd);
    }

    // --- 5. DWA 核心逻辑 (带归一化) ---
    std::pair<double, double> computeDwa() {
        std::vector<TrajectoryCandidate> candidates;
        candidates.reserve(dwa_samples_v_ * dwa_samples_w_);  // 预分配内存

        double total_dist_cost = 0.0;
        double total_angle_cost = 0.0;
        double total_speed_cost = 0.0;
        int    valid_count = 0;

        // 采样范围
        // 线速度: [0, 0.5] 不可后退
        double v_start = 0.0;
        double v_end = max_lin_vel_;
        double v_step = (v_end - v_start) / std::max(1, dwa_samples_v_ - 1);

        // 角速度: [-1.0, 1.0]
        double w_start = -max_ang_vel_;
        double w_end = max_ang_vel_;
        double w_step = (w_end - w_start) / std::max(1, dwa_samples_w_ - 1);

        // --- 第一步：遍历采样，计算原始 Cost ---
        for (int i = 0; i < dwa_samples_v_; ++i) {
            double v = v_start + i * v_step;

            for (int j = 0; j < dwa_samples_w_; ++j) {
                double w = w_start + j * w_step;

                TrajectoryCandidate cand;
                cand.v = v;
                cand.w = w;
                cand.valid = true;

                // 1. 轨迹推演 (计算 sim_time_ 后的位姿变化)
                double px, py, ptheta;
                // 防止除以零 (直线运动处理)
                if (std::abs(w) < 1e-4) {
                    px = v * sim_time_;
                    py = 0.0;
                    ptheta = 0.0;
                } else {
                    px = (v / w) * std::sin(w * sim_time_);
                    py = (v / w) * (1.0 - std::cos(w * sim_time_));
                    ptheta = w * sim_time_;
                }

                // 2. 将之前看到的障碍物点变换到新的预测坐标系中
                // 变换公式：
                // X_new = (X_old - px) * cos(theta) + (Y_old - py) * sin(theta)
                // Y_new = -(X_old - px) * sin(theta) + (Y_old - py) * cos(theta)

                double dx = right_closest_obstacle_->x - px;
                double dy = right_closest_obstacle_->y - py;

                double obs_new_x = dx * std::cos(ptheta) + dy * std::sin(ptheta);
                double obs_new_y = -dx * std::sin(ptheta) + dy * std::cos(ptheta);

                double dist_new = std::hypot(obs_new_x, obs_new_y);
                double angle_new = std::atan2(obs_new_y, obs_new_x);

                // 3. 碰撞检测 (硬约束)
                // 如果预测点距离障碍物太近 (小于半径 + 安全余量)
                if (dist_new < robot_radius_ + 0.05) {
                    cand.valid = false;
                    cand.raw_dist_cost = 0;
                    cand.raw_angle_cost = 0;
                    cand.raw_speed_cost = 0;
                } else {
                    // Cost 1: 距离误差
                    cand.raw_dist_cost = std::abs(dist_new - target_center_dist_);

                    // Cost 2: 角度误差 (目标是 -90度，即 -PI/2)
                    double target_angle = -M_PI / 2.0;
                    cand.raw_angle_cost = std::abs(angle_new - target_angle);

                    // Cost 3: 速度惩罚 (速度越大，Cost 越小)
                    cand.raw_speed_cost = (max_lin_vel_ - v);

                    // 累加用于归一化
                    total_dist_cost += cand.raw_dist_cost;
                    total_angle_cost += cand.raw_angle_cost;
                    total_speed_cost += cand.raw_speed_cost;
                    valid_count++;
                }

                candidates.push_back(cand);
            }
        }

        // --- 第二步：归一化并选择最佳速度 ---

        if (valid_count == 0) {
            // 所有路径都会碰撞 -> 紧急停车或原地旋转
            // 这里简单处理为停止
            std::cout << "no best motion" << std::endl;
            return {0.0, 0.0};
        }

        // 防止分母为 0
        if (total_dist_cost < 1e-6) total_dist_cost = 1.0;
        if (total_angle_cost < 1e-6) total_angle_cost = 1.0;
        if (total_speed_cost < 1e-6) total_speed_cost = 1.0;

        double min_final_score = std::numeric_limits<double>::max();
        double best_v = 0.0;
        double best_w = 0.0;

        for (const auto &cand : candidates) {
            if (!cand.valid) continue;

            // 归一化计算：(原始值 / 总和) * 权重
            double norm_dist = cand.raw_dist_cost / total_dist_cost;
            double norm_angle = cand.raw_angle_cost / total_angle_cost;
            double norm_speed = cand.raw_speed_cost / total_speed_cost;

            double score = (w_dist_ * norm_dist) + (w_head_ * norm_angle) + (w_speed_ * norm_speed);

            if (score < min_final_score) {
                min_final_score = score;
                best_v = cand.v;
                best_w = cand.w;
            }
        }

        return {best_v, best_w};
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DwaRightWallFollower>());
    rclcpp::shutdown();
    return 0;
}