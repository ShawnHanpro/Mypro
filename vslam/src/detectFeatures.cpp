#include <iostream>
#include "slamBase.h"
using namespace std;

// OpenCV 特征检测模块
#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/nonfree/nonfree.hpp> // use this if you want to use SIFT or SURF
#include <opencv2/calib3d/calib3d.hpp>

int main( int argc, char** argv )
{
    // 声明并从data文件夹里读取两个rgb与深度图
    cv::Mat rgb1 = cv::imread( "../data/rgb1.png");
    cv::Mat rgb2 = cv::imread( "../data/rgb2.png");
    cv::Mat depth1 = cv::imread( "../data/depth1.png", -1);
    cv::Mat depth2 = cv::imread( "../data/depth2.png", -1);

    // 使用orb检测特征并描述特征 包含特征提取器和描述子提取器
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    vector<cv::KeyPoint> kp1, kp2;
    cv::Mat desp1, desp2;

    orb->detectAndCompute(rgb1, cv::Mat(), kp1, desp1);
    orb->detectAndCompute(rgb2, cv::Mat(), kp2, desp2);

    cout<<"Key points of two images: "<<kp1.size()<<", "<<kp2.size()<<endl;
    
    // 可视化， 显示关键点
    cv::Mat imgShow1;
    cv::Mat imgShow2;
    cv::drawKeypoints( rgb1, kp1, imgShow1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::drawKeypoints( rgb2, kp2, imgShow2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow( "keypoints1", imgShow1 );
    cv::imshow( "keypoints2", imgShow2 );
    // cv::imwrite( "../data/rgb1_keypoints.png", imgShow );
    cv::waitKey(0); //暂停等待一个按键

    // 匹配描述子

    // 暴力匹配，不推荐
    /*
        vector< cv::DMatch > matches; 
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.match( desp1, desp2, matches );
    */

    // knn匹配
    vector<vector<cv::DMatch>> knn_matches; // 汉明距离值，原理是比较二进制描述子中有多少bit不一样
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.knnMatch(desp1, desp2, knn_matches, 2);

    // 比值筛选（ratio test）
    vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.75f;
    for (const auto& m : knn_matches) {
        if (m.size() == 2 && m[0].distance < ratio_thresh * m[1].distance) {
            good_matches.push_back(m[0]);
        }
    }

    cout<<"Find total "<<good_matches.size()<<" matches."<<endl;

    // 可视化：显示匹配的特征
    cv::Mat imgMatches;
    cv::drawMatches( rgb1, kp1, rgb2, kp2, good_matches, imgMatches );
    cv::imshow( "matches", imgMatches );
    // cv::imwrite( "../data/matches.png", imgMatches );
    cv::waitKey( 0 );

    // 计算图像间的运动关系
    // 关键函数：cv::solvePnPRansac()
    // 为调用此函数准备必要的参数
    
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS C;
    C.cx = 325.5;
    C.cy = 253.5;
    C.fx = 518.0;
    C.fy = 519.0;
    C.scale = 1000.0;

    for (size_t i=0; i<good_matches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = kp1[good_matches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = depth1.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( kp2[good_matches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, C );
        pts_obj.push_back( pd );
    }

    double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    // 使用第一帧的3D点 和 第二帧中看到他们的位置 计算 相机的运动
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );

    cout << "inliers: " << inliers.rows << endl;
    cout << "R=" << rvec << endl;
    cout << "t=" << tvec << endl;

    // 画出inliers匹配 
    vector< cv::DMatch > matchesShow;
    for (size_t i=0; i<inliers.rows; i++)
    {
        matchesShow.push_back( good_matches[inliers.ptr<int>(i)[0]] );    
    }
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
    cv::imshow( "inlier matches", imgMatches );
    cv::imwrite( "./data/inliers.png", imgMatches );
    cv::waitKey( 0 );

    return 0;
}