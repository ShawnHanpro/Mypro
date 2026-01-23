#include <iostream>
#include <thread>

#include "slamBase.h"

// octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

// pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>


int main(int argc, char** argv) {
    CAMERA_INTRINSIC_PARAMETERS camera = GetDefaultCamera();

    cv::Mat rgb   = cv::imread("../data/rgb.png");
    cv::Mat depth = cv::imread("../data/depth.png", -1); // 保留深度原始格式

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = image2PointCloud(rgb, depth, camera);
    // pcl::visualization::CloudViewer viewer("viewer");

    // viewer.showCloud(cloud);
    // pcl::io::savePCDFileBinary("../data/cloud.pcd", *cloud);
    // while(!viewer.wasStopped()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    octomap::Pointcloud octo_cloud;

    for (auto& p : cloud->points) {
        octo_cloud.push_back(p.x, p.y, p.z);
    }

    // 分辨率
    double resolution = 0.05;
    // 创建树
    octomap::ColorOcTree tree(resolution);
    // 原点
    octomap::point3d sensor_origin(0.0, 0.0, 0.0);

    tree.insertPointCloud(octo_cloud, sensor_origin);
    tree.updateInnerOccupancy();

    // 上色
    for (auto& p : cloud->points) {
        tree.integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }

    tree.write("../data/map.bt");   // 二进制
    tree.write("../data/map.ot");   // 全信息（含颜色）
}


