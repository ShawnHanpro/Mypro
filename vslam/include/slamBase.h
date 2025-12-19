#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

// opencv
#include <opencv2/opencv.hpp>
// #include <opencv/highgui.hpp>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 相机内参结构体
struct CAMERA_INTRINSIC_PARAMETERS {
    double cx, cy, fx, fy, scale;
};

// 函数接口
// image2PointCloud 将rgb转化为点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera);

// point2dTo3d 将单个点从图像转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);