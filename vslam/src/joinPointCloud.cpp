#include<iostream>
using namespace std;

#include "slamBase.h"

#include <opencv2/core/eigen.hpp>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

int main( int argc, char** argv )
{
    // 声明两个帧，FRAME结构请见include/slamBase.h
    FRAME frame1, frame2;
    
    //读取图像
    frame1.rgb = cv::imread( "../data/rgb1.png" );
    frame1.depth = cv::imread( "../data/depth1.png", -1);
    frame2.rgb = cv::imread( "../data/rgb2.png" );
    frame2.depth = cv::imread( "../data/depth2.png", -1 );

    // 提取特征并计算描述子
    cout << "extracting features" << endl;

    OrbComputerKeyPointsAndDesp(frame1);
    OrbComputerKeyPointsAndDesp(frame2);

    // 相机内参
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );

    cout << "solving pnp" << endl;
    // 求解pnp
    RESULT_OF_PNP result = EstimateMotion( frame1, frame2, camera );

    cout << result.rvec << endl << result.tvec << endl;

    // 处理result
    // 将旋转向量转化为旋转矩阵
    cv::Mat R;
    cv::Rodrigues( result.rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    cout << "translation" << endl;
    // Eigen::AngleAxisd angle(r);
    // Eigen::Translation<double,3> trans(result.tvec.at<double>(0,0), result.tvec.at<double>(0,1), result.tvec.at<double>(0,2));
    // T = angle;
    // T(0,3) = result.tvec.at<double>(0,0); 
    // T(1,3) = result.tvec.at<double>(0,1); 
    // T(2,3) = result.tvec.at<double>(0,2);

    T.linear() = r;
    T.translation() << 
        result.tvec.at<double>(0,0),
        result.tvec.at<double>(0,1),
        result.tvec.at<double>(0,2);

    // 转换点云
    cout << "converting image to clouds" << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = image2PointCloud(frame1.rgb, frame1.depth, camera);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = image2PointCloud(frame2.rgb, frame2.depth, camera);

    // 合并点云
    cout << "combining clouds" << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud( *cloud1, *output, T.matrix() );
    *output += *cloud2;
    // pcl::io::savePCDFile("data/result.pcd", *output);
    cout << "Final result saved." << endl;

    pcl::visualization::CloudViewer viewer( "viewer" );
    viewer.showCloud( output );
    while( !viewer.wasStopped() )
    {
        
    }
    return 0;
}