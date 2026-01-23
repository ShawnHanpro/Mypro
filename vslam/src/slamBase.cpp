#include "slamBase.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera) {
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
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

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

    return cloud;
}

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera) {
    cv::Point3f p;
    p.z = double(point.z) / camera.scale;
    p.x = (point.x - camera.cx) * p.z / camera.fx;
    p.y = (point.y - camera.cy) * p.z / camera.fy;

    return p;
}

void OrbComputerKeyPointsAndDesp(FRAME& frame) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    std::vector<cv::KeyPoint> kp;
    cv::Mat desp;

    orb->detectAndCompute(frame.rgb, cv::Mat(), frame.kp, frame.desp);
}

RESULT_OF_PNP EstimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera) {
    ParameterReader param;
    RESULT_OF_PNP result;
    
    // knn匹配
    std::vector<std::vector<cv::DMatch>> knn_matches; // 汉明距离值，原理是比较二进制描述子中有多少bit不一样
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.knnMatch(frame1.desp, frame2.desp, knn_matches, 2);

    // 比值筛选（ratio test）
    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.75f;
    for (const auto& m : knn_matches) {
        if (m.size() == 2 && m[0].distance < ratio_thresh * m[1].distance) {
            good_matches.push_back(m[0]);
        }
    }

    // 第一个帧的三维点
    std::vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    std::vector< cv::Point2f > pts_img;

    for (size_t i=0; i<good_matches.size(); i++) {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[good_matches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[good_matches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }

    if (pts_img.size() < 6) {
        result.inliers = 0;
        return result;
    }


    // 可视化：显示匹配的特征
    // std::cout << "good matches: " << good_matches.size() << std::endl;
    // cv::Mat imgMatches;
    // cv::drawMatches( frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, good_matches, imgMatches );
    // cv::imshow( "matches", imgMatches );
    // cv::waitKey( 0 );


    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    // 使用第一帧的3D点 和 第二帧中看到他们的位置 计算 相机的运动
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers, cv::SOLVEPNP_EPNP );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}

Eigen::Isometry3d CvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec) {
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ ) 
            r(i,j) = R.at<double>(i,j);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    T.linear() = r;
    T.translation() << 
        tvec.at<double>(0,0),
        tvec.at<double>(0,1),
        tvec.at<double>(0,2);

    return T;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr JoinPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr original, FRAME& new_frame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera) {
    // 转换点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud = image2PointCloud(new_frame.rgb, new_frame.depth, camera);

    // 合并点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *new_cloud += *output;

    // 滤波降采样
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( new_cloud );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
    voxel.filter( *filter_cloud );
    return filter_cloud;

}