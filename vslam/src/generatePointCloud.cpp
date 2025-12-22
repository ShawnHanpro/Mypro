#include "slamBase.h"

// 相机内参
// const double camera_factor = 1000;
// const double camera_cx = 325.5;
// const double camera_cy = 235.5;
// const double camera_fx = 518.0;
// const double camera_fy = 519.0;

int main(int argc, char** argv) {

    // 读取图片并转化为点云

    // opencv
    cv::Mat rgb, depth;
    // rgb是8uc3彩色图像 
    // depth是16uc1的单通道图像 flag设置为-1 表示读取原始数据不做任何处理
    rgb = cv::imread("../data/rgb.png");
    depth = cv::imread("../data/depth.png", -1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 235.5;
    camera.fx = 518.0;
    camera.fy = 519.0;

    cloud = image2PointCloud(rgb, depth, camera);

    // pcl::io::savePCDFile("../data/pointcloud.pcd", *cloud);

    return 0;
}
