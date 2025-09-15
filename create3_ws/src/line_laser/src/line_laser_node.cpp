#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

class ScanToCloudNode : public rclcpp::Node
{
public:
  ScanToCloudNode()
  : Node("scan_to_cloud_node")
  {
    target_frame_ = "odom";
    is_initialized_ = false;

    // 初始化 TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅 /scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ScanToCloudNode::scan_callback, this, std::placeholders::_1));

    // 发布累积的点云
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/accumulated_cloud", 10);

    RCLCPP_INFO(this->get_logger(), "Scan to PointCloud2 converter node (manual) has started.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    try {
      // 1. 将当前帧的 LaserScan 转换为 PointCloud2 (在 laser 坐标系下)
      sensor_msgs::msg::PointCloud2 current_cloud_ros;
      projector_.transformLaserScanToPointCloud(msg->header.frame_id, *msg, current_cloud_ros, *tf_buffer_);

      if (current_cloud_ros.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Converted point cloud is empty, skipping frame.");
        return;
      }

      // 2. 将当前帧的点云转换到目标 odom 坐标系
      sensor_msgs::msg::PointCloud2 current_cloud_transformed_ros;
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
      
      tf2::doTransform(current_cloud_ros, current_cloud_transformed_ros, transform_stamped);

      // 3. 手动合并点云
      if (!is_initialized_)
      {
        // 如果是第一帧，直接用它来初始化累积点云
        accumulated_cloud_ = current_cloud_transformed_ros;
        is_initialized_ = true;
      }
      else
      {
        // 检查点云结构是否一致
        if (accumulated_cloud_.fields != current_cloud_transformed_ros.fields ||
            accumulated_cloud_.point_step != current_cloud_transformed_ros.point_step)
        {
            RCLCPP_ERROR(this->get_logger(), "Point cloud fields or point_step do not match. Cannot merge.");
            return;
        }

        // 获取旧数据的大小
        size_t old_data_size = accumulated_cloud_.data.size();
        // 获取新数据的大小
        size_t new_data_size = current_cloud_transformed_ros.data.size();
        
        // 调整累积点云数据区的大小以容纳新数据
        accumulated_cloud_.data.resize(old_data_size + new_data_size);

        // 将新数据拷贝到累积点云数据的末尾
        std::copy(
          current_cloud_transformed_ros.data.begin(),
          current_cloud_transformed_ros.data.end(),
          accumulated_cloud_.data.begin() + old_data_size
        );
        
        // 更新元数据
        accumulated_cloud_.width += current_cloud_transformed_ros.width;
        accumulated_cloud_.row_step = accumulated_cloud_.width * accumulated_cloud_.point_step;
      }

      // 更新时间戳并发布
      accumulated_cloud_.header.stamp = this->get_clock()->now();
      cloud_pub_->publish(accumulated_cloud_);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    }
  }

  // 成员变量
  std::string target_frame_;
  bool is_initialized_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  laser_geometry::LaserProjection projector_;
  sensor_msgs::msg::PointCloud2 accumulated_cloud_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanToCloudNode>());
  rclcpp::shutdown();
  return 0;
}