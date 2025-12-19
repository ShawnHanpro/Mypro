#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 235.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

int main(int argc, char** argv) {

    // 读取图片并转化为点云

    // opencv
    cv::Mat rgb, depth;
    // rgb是8uc3彩色图像 
    // depth是16uc1的单通道图像 flag设置为-1 表示读取原始数据不做任何处理
    rgb = cv::imread("../data/rgb.png");
    depth = cv::imread("../data/depth.png", -1);

    // 点云变量
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 遍历深度图
    for (int m = 0; m < depth.rows; ++m) {
        for (int n = 0; n < depth.cols; ++n) {
            // 获取(m,n)处值
            ushort d = depth.ptr<ushort>(m)[n];
            // 判断数据是否有效
            if (d == 0) continue;

            // 计算该点的空间坐标
            pcl::PointXYZRGB p;
            // z = d / s
            // x = (u - cx) * z/fx
            // y = (v - cy) * z/fy
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            // 从rgb获取颜色
            // rgb是三通道的BGR格式图 所以按下面顺序取色
            cv::Vec3b color = rgb.at<cv::Vec3b>(m,n);
            p.b = color[0];
            p.g = color[1];
            p.r = color[2];

            cloud->points.push_back(p);
        }
    }
    // cloud参数
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    pcl::io::savePCDFile("../data/pointcloud.pcd", *cloud);

    cloud->points.clear();

    return 0;
}
