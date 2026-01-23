#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>

// opencv
#include <opencv2/opencv.hpp>
// #include <opencv/highgui.hpp>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

// 相机内参结构体
struct CAMERA_INTRINSIC_PARAMETERS {
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME {
    int frameID; 
    cv::Mat rgb, depth; // 该帧对应的彩色图与深度图
    cv::Mat desp; // 特征描述子
    std::vector<cv::KeyPoint> kp; // 关键点
};

// pnp结果
struct RESULT_OF_PNP {
    cv::Mat rvec, tvec;
    int inliers;
};

// 函数接口
// image2PointCloud 将rgb转化为点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera);

// point2dTo3d 将单个点从图像转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);

// ORB计算特征点和特征描述子
void OrbComputerKeyPointsAndDesp(FRAME& frame);

// 计算两帧之间的变换关系
RESULT_OF_PNP EstimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera);

// 将cvmat的旋转矢量与位移矢量转换为变换矩阵
Eigen::Isometry3d CvMat2Eigen(cv::Mat& rvec, cv::Mat& trec);

// 合并点云数据
pcl::PointCloud<pcl::PointXYZRGB>::Ptr JoinPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr original, FRAME& new_frame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera);

// 参数读取类
class ParameterReader {
public:
    ParameterReader( std::string filename="../param/parameters.txt" ) {
        std::ifstream fin(filename);
        if (!fin) {
            std::cerr << "parameter file does not exist." << std::endl;
            return;
        }

        std::string line;
        while(std::getline(fin, line)) {
            if (line.empty() || line[0] == '#') continue;

            
            auto pos = line.find("=");
            if (pos == std::string::npos) continue;
            
            std::string key = line.substr( 0, pos );
            std::string value = line.substr( pos+1, line.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }

    std::string getData( std::string key ) {
        std::unordered_map<std::string, std::string>::iterator iter = data.find(key);
        if (iter == data.end()) {
            std::cerr << "Parameter name "<<key<<" not found!" << std::endl;
            return std::string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    std::unordered_map<std::string, std::string> data;

private:
    // 去掉首尾空格
    static std::string trim(const std::string& s) {
        auto start = std::find_if_not(s.begin(), s.end(), ::isspace);
        auto end   = std::find_if_not(s.rbegin(), s.rend(), ::isspace).base();
        if (start >= end) return "";
        return std::string(start, end);
    }
};

inline static CAMERA_INTRINSIC_PARAMETERS GetDefaultCamera() {
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */