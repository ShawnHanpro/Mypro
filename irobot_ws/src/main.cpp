#include "baselib.h"
#include <string>
#include <fstream>
#include "hough_scan_match.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <vector>
#include <random>

#define SHOW 0
#define FILE 0
#define SHOWGRID 0
#define CORRELATIVESCANMATCHING 0
#define ORBMATCHING 0
#define FEATUREMATCHING 0
#define TESTFEATUREMATCHING 0
#define OPENCVFEATUREMATCHING 1

// check_mode: 0代表去除黑区域，1代表去除白区域; NeihborMode：0代表4邻域，1代表8邻域;
void RemoveSmallRegion(cv::Mat& src, cv::Mat& dst, int area_limit, int check_mode, int neihbor_mode) {
    dst = cv::Mat::zeros(src.size(), src.type());
    int RemoveCount = 0;  //记录除去的个数
    //记录每个像素点检验状态的标签，0代表未检查，1代表正在检查,2代表检查不合格（需要反转颜色），3代表检查合格或不需检查
    cv::Mat Pointlabel = cv::Mat::zeros(src.size(), CV_8UC1);

    if (check_mode == 1) {
        // cout<<"Mode: 去除小区域. ";
        for (int i = 0; i < src.rows; ++i) {
            uchar* iData  = src.ptr<uchar>(i);
            uchar* iLabel = Pointlabel.ptr<uchar>(i);
            for (int j = 0; j < src.cols; ++j) {
                if (iData[j] < 10) {
                    iLabel[j] = 3;
                }
            }
        }
    } else {
        // cout<<"Mode: 去除孔洞. ";
        for (int i = 0; i < src.rows; ++i) {
            uchar* iData  = src.ptr<uchar>(i);
            uchar* iLabel = Pointlabel.ptr<uchar>(i);
            for (int j = 0; j < src.cols; ++j) {
                if (iData[j] > 10) {
                    iLabel[j] = 3;
                }
            }
        }
    }

    std::vector<cv::Point2i> NeihborPos;  //记录邻域点位置
    NeihborPos.push_back(cv::Point2i(-1, 0));
    NeihborPos.push_back(cv::Point2i(1, 0));
    NeihborPos.push_back(cv::Point2i(0, -1));
    NeihborPos.push_back(cv::Point2i(0, 1));
    if (neihbor_mode == 1) {
        // cout<<"Neighbor mode: 8邻域."<<endl;
        NeihborPos.push_back(cv::Point2i(-1, -1));
        NeihborPos.push_back(cv::Point2i(-1, 1));
        NeihborPos.push_back(cv::Point2i(1, -1));
        NeihborPos.push_back(cv::Point2i(1, 1));
    }
    // else cout<<"Neighbor mode: 4邻域."<<endl;
    int NeihborCount = 4 + 4 * neihbor_mode;
    int CurrX = 0, CurrY = 0;
    //开始检测
    for (int i = 0; i < src.rows; ++i) {
        uchar* iLabel = Pointlabel.ptr<uchar>(i);
        for (int j = 0; j < src.cols; ++j) {
            if (iLabel[j] == 0) {
                //********开始该点处的检查**********
                std::vector<cv::Point2i> GrowBuffer;  //堆栈，用于存储生长点
                GrowBuffer.push_back(cv::Point2i(j, i));
                // GrowBuffer.push_back( cv::Point2i(i, j) );
                Pointlabel.at<uchar>(i, j) = 1;
                int CheckResult = 0;  //用于判断结果（是否超出大小），0为未超出，1为超出

                for (int z = 0; z < GrowBuffer.size(); z++) {
                    for (int q = 0; q < NeihborCount; q++) {  //检查四个邻域点
                        CurrX = GrowBuffer.at(z).x + NeihborPos.at(q).x;
                        CurrY = GrowBuffer.at(z).y + NeihborPos.at(q).y;
                        if (CurrX >= 0 && CurrX < src.cols && CurrY >= 0 && CurrY < src.rows) {  //防止越界
                            if (Pointlabel.at<uchar>(CurrY, CurrX) == 0) {
                                GrowBuffer.push_back(cv::Point2i(CurrX, CurrY));  //邻域点加入buffer
                                Pointlabel.at<uchar>(CurrY, CurrX) = 1;  //更新邻域点的检查标签，避免重复检查
                            }
                        }
                    }
                }

                if (GrowBuffer.size() > area_limit)
                    CheckResult = 2;  //判断结果（是否超出限定的大小），1为未超出，2为超出
                else {
                    CheckResult = 1;
                    RemoveCount++;
                }
                for (int z = 0; z < GrowBuffer.size(); z++) {  //更新Label记录
                    CurrX = GrowBuffer.at(z).x;
                    CurrY = GrowBuffer.at(z).y;
                    Pointlabel.at<uchar>(CurrY, CurrX) += CheckResult;
                }
                //********结束该点处的检查**********
            }
        }
    }

    check_mode = 255 * (1 - check_mode);
    //开始反转面积过小的区域

    // cv::imshow("output", dst);
    // cv::waitKey(0);
    int addblack = 0;
    for (int i = 0; i < src.rows; ++i) {
        uchar* iData    = src.ptr<uchar>(i);
        uchar* iDstData = dst.ptr<uchar>(i);
        uchar* iLabel   = Pointlabel.ptr<uchar>(i);
        for (int j = 0; j < src.cols; ++j) {
            if (iLabel[j] == 2) {
                iDstData[j] = check_mode;
                ++addblack;
            } else if (iLabel[j] == 3) {
                iDstData[j] = iData[j];
            }
        }
    }
    // cout<<RemoveCount<<" objects removed."<<endl;
}

//
int BfnnPoint(const std::vector<Eigen::Vector2f> &points, const Eigen::Vector2f &point) {
    return std::min_element(points.begin(), points.end(),
                            [&point](const Eigen::Vector2f &p1, const Eigen::Vector2f &p2) {
                                return (p1 - point).norm() < (p2 - point).norm();
                            }) -
           points.begin();
}

Eigen::Vector2f Normalize(std::vector<Eigen::Vector2f> &points) {
    Eigen::Vector2f mean(0.f, 0.f);
    for (const auto &p : points) {
        mean += p;
    }

    mean.x() /= points.size();
    mean.y() /= points.size();

    return mean;
}
std::pair<Eigen::Vector2f, float> ICP(const std::vector<Eigen::Vector2f> &source,
                                                      const std::vector<Eigen::Vector2f> &target,
                                                      std::vector<Eigen::Vector2f> &re_points, const int &iter_num,
                                                      const double &eps) {
    auto source_points = source;
    auto target_points = target;

    auto source_mean = Normalize(source_points);

    Eigen::Matrix2f R;
    float           res_angle = 0.f;
    Eigen::Vector2f T(0.f, 0.f);

    float error = 0.0;

    for (int i = 0; i < iter_num; ++i) {
        auto   target_mean = Normalize(target_points);
        double t_nume = 0.f, t_deno = 0.f;

        for (int j = 0; j < target_points.size(); ++j) {
            int closet_index = BfnnPoint(source_points, target_points[j]);

            // 旋转矩阵求旋转角公式
            // tnume​=j=0∑N−1​(sy,j​⋅tx,j​−sx,j​⋅ty,j​)
            t_nume += source_points[closet_index].y() * target_points[j].x() -
                      source_points[closet_index].x() * target_points[j].y();
            // tdeno​=j=0∑N−1​(sx,j​⋅tx,j​+sy,j​⋅ty,j​)
            t_deno += source_points[closet_index].x() * target_points[j].x() +
                      source_points[closet_index].y() * target_points[j].y();
        }

        double theta = std::atan2(t_nume, t_deno);
        res_angle += theta / M_PI * 180.f;

        Eigen::Matrix2f R_iter;
        R_iter << std::cos(theta), -std::sin(theta), 
                  std::sin(theta), std::cos(theta);

        Eigen::Vector2f T_iter = source_mean - R_iter * target_mean;

        for (int j = 0; j < target_points.size(); ++j) {
            target_points[j] = R_iter * target_points[j] + T_iter;
        }

        R = R_iter * R;
        T = R_iter * T + T_iter;

        // 使用角度判断是否结束
        // if (std::abs(theta / CV_PI * 180.f) < eps)
        // {
        //     break;
        // }

        // 使用距离判断是否结束
        for (int j = 0; j < target_points.size(); ++j) {
            int closet_index = BfnnPoint(source_points, target_points[j]);
            error += (source_points[closet_index] - target_points[j]).norm();
        }
        error /= target_points.size();

        if (error < eps) {
            break;
        }
    }
    re_points = target_points;

    return std::make_pair(T, res_angle);
}

//

int main (int argc, char** argv) {
    std::string png_path = "/home/shan2/brewst_files/subgrids";

    cv::Mat image_0 = cv::imread(png_path+"/grid_0.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image_1 = cv::imread(png_path+"/grid_2.png", cv::IMREAD_GRAYSCALE);
    int width_0 = image_0.cols;
    int height_0 = image_0.rows;
    int width_1 = image_1.cols;
    int height_1 = image_1.rows;
    
    if (image_0.empty() || image_1.empty()) return 0;

    Grid2D grid_0(width_0, height_0, 0.05, 0, 0);
    Grid2D grid_1(width_1, height_1, 0.05, 0, 0);
    Image2Grid(image_0, grid_0);
    Image2Grid(image_1, grid_1);

    cv::Mat result_0;
    cv::Mat result_1;
    cv::cvtColor(image_0, result_0, cv::COLOR_GRAY2BGR); 
    cv::cvtColor(image_1, result_1, cv::COLOR_GRAY2BGR); 


    std::vector<Eigen::Vector2f> target_points;
    std::vector<Eigen::Vector2f> source_points;
    Image2Points(image_0, target_points);
    Image2Points(image_1, source_points);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& p : target_points) {
        target_cloud->points.emplace_back(p.x(), p.y(), 0);
    }
    for (const auto& p : source_points) {
        source_cloud->points.emplace_back(p.x(), p.y(), 0);
    }

    // target_cloud->width = target_cloud->points.size();
    // target_cloud->height = 1;
    // target_cloud->is_dense = true;
    // source_cloud->width = source_cloud->points.size();
    // source_cloud->height = 1;
    // source_cloud->is_dense = true;

    // viewer->addPointCloud<pcl::PointXYZ>(target_cloud, "target_cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target_cloud");
    
    // viewer->addPointCloud<pcl::PointXYZ>(source_cloud, "source_cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source_cloud");

    // while(!viewer->wasStopped()) {
    //     viewer->spinOnce(10);
    // }


    // 体素滤波降低点云数量
    float                        voxel_size = 10;
    VoxelFilter                  filter(voxel_size);
    std::vector<Eigen::Vector2f> target_filtered_points = filter.filter(target_points);
    std::vector<Eigen::Vector2f> source_filtered_points = filter.filter(source_points);


    target_cloud->points.clear();
    source_cloud->points.clear();
    for (const auto& p : target_filtered_points) {
        target_cloud->points.emplace_back(p.x(), p.y(), 0);
    }
    for (const auto& p : source_filtered_points) {
        source_cloud->points.emplace_back(p.x(), p.y(), 0);
    }

    // target_cloud->width = target_cloud->points.size();
    // target_cloud->height = 1;
    // target_cloud->is_dense = true;
    // source_cloud->width = source_cloud->points.size();
    // source_cloud->height = 1;
    // source_cloud->is_dense = true;

    // viewer->addPointCloud<pcl::PointXYZ>(target_cloud, "target_cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target_cloud");
    
    // viewer->addPointCloud<pcl::PointXYZ>(source_cloud, "source_cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source_cloud");

    // while(!viewer->wasStopped()) {
    //     viewer->spinOnce(10);
    // }

    std::vector<Eigen::Vector2f> target_bfnn_points;
    std::vector<Eigen::Vector2f> source_bfnn_points;

    // debug使用的变量，保存icp计算后的点云
    std::vector<Eigen::Vector2f> re_points;
    std::cout << "size: " << source_filtered_points.size() << " " << target_filtered_points.size() << std::endl;    


    // 计算一次近邻点，使用可以计算出近邻点的数据点，避免两帧激光的质心计算误差较大
    for (int i = 0; i < target_filtered_points.size(); ++i) {
        int closet_index = BfnnPoint(source_filtered_points, target_filtered_points[i]);

        const Eigen::Vector2f &nearest_point = source_filtered_points[closet_index];
        float                  distance      = (target_filtered_points[i] - nearest_point).norm();

        if (distance < 0.5) {
            source_bfnn_points.emplace_back(nearest_point);
            target_bfnn_points.emplace_back(target_filtered_points[i]);
        }
    }

    // icp
    std::pair<Eigen::Vector2f, float> tran = ICP(source_bfnn_points, target_bfnn_points, re_points, 100, 0.1);
    float detla = std::sqrt(std::pow(tran.first.x(), 2) + std::pow(tran.first.y(), 2));
    // std::cout << "icp tran: {} {} {}", tran.first.x(), tran.first.y(), tran.second << std::endl;
    // std::cout << fmt::format("icp detla: {}", detla) << std::endl;
    // std::cout << fmt::format("icp cost time: {}ms", laser_t) << std::endl;

    Eigen::Vector2f t = tran.first;
    float theta = tran.second;

    Eigen::Matrix2f R;
    R << std::cos(theta), -std::sin(theta),
        std::sin(theta),  std::cos(theta);

    std::vector<Eigen::Vector2f> aligned_points;
    aligned_points.reserve(source_points.size());

    for(const auto& p : source_points)
    {
        Eigen::Vector2f p_new = R * p + t;
        aligned_points.push_back(p_new);
    }

    std::cout << aligned_points.size() << std::endl;

    source_cloud->clear();
    for(const auto& p : aligned_points)
    {
        source_cloud->points.emplace_back(p.x(), p.y(), 0.f);
    }

    source_cloud->width = source_cloud->points.size();
    source_cloud->height = 1;
    source_cloud->is_dense = true;

    viewer->addPointCloud<pcl::PointXYZ>(source_cloud, "source_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source_cloud");
    viewer->resetCamera();

    while(!viewer->wasStopped()) {
        viewer->spinOnce(10);
    }
    
    
    

#if OPENCVFEATUREMATCHING

    cv::Mat binary_image_0;
    cv::threshold(image_0, binary_image_0, 1, 255, cv::THRESH_BINARY_INV);  // 黑色障碍物点变成白色，其它点变为黑色
    cv::Mat binary_image_1;
    cv::threshold(image_1, binary_image_1, 1, 255, cv::THRESH_BINARY_INV);  // 黑色障碍物点变成白色，其它点变为黑色
    cv::namedWindow("binary_image_0", cv::WINDOW_NORMAL);
    cv::imshow("binary_image_0", binary_image_0);
    cv::waitKey(1);

    cv::Mat filter_binary_image_0;
    RemoveSmallRegion(binary_image_0, filter_binary_image_0, 9, 1, 1);
    cv::Mat filter_binary_image_1;
    RemoveSmallRegion(binary_image_1, filter_binary_image_1, 5, 1, 1);
    cv::namedWindow("filter_binary_image_0", cv::WINDOW_NORMAL);
    cv::imshow("filter_binary_image_0", filter_binary_image_0);
    cv::waitKey(0);

    cv::Mat morph_0;
    cv::Mat morph_1;
    // 结构元素（关键）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // 闭运算 = 膨胀 → 腐蚀
    cv::morphologyEx(filter_binary_image_0, morph_0, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(filter_binary_image_1, morph_1, cv::MORPH_CLOSE, kernel);

    // cv::namedWindow("morph_0", cv::WINDOW_NORMAL);
    // cv::imshow("morph_0", morph_0);
    // cv::waitKey(1);
    // cv::namedWindow("morph_1", cv::WINDOW_NORMAL);
    // cv::imshow("morph_1", morph_1);
    // cv::waitKey(0);

    std::vector<cv::Vec4i> lines_0;
    std::vector<cv::Vec4i> lines_1;
    cv::HoughLinesP(morph_0, lines_0, 1, CV_PI/180, 20, 20, 50);
    cv::HoughLinesP(morph_1, lines_1, 1, CV_PI/180, 20, 20, 50);
    std::cout << "line: " << lines_0.size() << std::endl;
    std::cout << "line: " << lines_1.size() << std::endl;

    // for (auto& line : lines_0) {
    //     cv::line(result_0, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,0,255), 1);
    // }
    // for (auto& line : lines_1) {
    //     cv::line(result_1, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,0,255), 1);
    // }
    
    std::vector<cv::Vec4i> merged_0;
    std::vector<cv::Vec4i> merged_1;
    mergeLines(lines_0, merged_0);
    mergeLines(lines_1, merged_1);
    std::cout << "merged_0: " << merged_0.size() << std::endl;
    std::cout << "merged_1: " << merged_1.size() << std::endl;
    
    // for (auto& line : merged_0) {
    //     cv::line(result_0, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,0,255), 1);
    // }
    // for (auto& line : merged_1) {
    //     cv::line(result_1, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,0,255), 1);
    // }

    std::vector<cv::Point2f> corner_0;
    std::vector<cv::Point2f> corner_1;
    extractCorners(merged_0, corner_0);
    extractCorners(merged_1, corner_1);
    std::cout << "intersections_0: " << corner_0.size() << std::endl;
    std::cout << "intersections_1: " << corner_1.size() << std::endl;

    for (auto& p : corner_0) {
        cv::circle(result_0, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 2, cv::Scalar(0, 0, 255), -1);
    }
    for (auto& p : corner_1) {
        cv::circle(result_1, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 2, cv::Scalar(0, 0, 255), -1);
    }
    std::cout << std::endl;

    // TriangleMatcher matcher;
    // std::vector<TriangleFeature> triangles_0;
    // matcher.extractTriangles(intersections_0, triangles_0);
    // std::vector<TriangleFeature> triangles_1;
    // matcher.extractTriangles(intersections_1, triangles_1);

    // std::cout << "triangles_0: " << triangles_0.size() << std::endl;
    // std::cout << "triangles_1: " << triangles_1.size() << std::endl;

    // MapMatcher map_matcher;
    // Eigen::Vector2f translation;
    // float rotation;

    // map_matcher.match(triangles_0, triangles_1, translation, rotation);
    // matchTriangles(triangles_0, triangles_1, translation, rotation);

    // std::cout << "rot = " << rotation << std::endl;
    // std::cout << "trans = " << translation.transpose() << std::endl;

    cv::namedWindow("result_0", cv::WINDOW_NORMAL);
    cv::imshow("result_0", result_0);
    cv::waitKey(1);
    cv::namedWindow("result_1", cv::WINDOW_NORMAL);
    cv::imshow("result_1", result_1);
    cv::waitKey(0);

    // std::vector<Submap> all_submaps;
    // Submap s0{grid_0, Pose2D{0,0,0}};
    // Submap s1{grid_1, pose};
    // all_submaps.push_back(s0);
    // all_submaps.push_back(s1);
    // cv::Mat merged = RenderSubmaps(all_submaps);
    // cv::namedWindow("merged", cv::WINDOW_NORMAL);
    // cv::imshow("merged", merged);
    // cv::waitKey(0);

#endif

#if TESTFEATUREMATCHING
    float resolution = 0.5;
    // std::vector<Point2D> laser_point_0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_0(new pcl::PointCloud<pcl::PointXYZ>);
    for (int y = 0; y < image_0.rows; ++y) {
        for (int x = 0; x < image_0.cols; ++x) {
            if (image_0.at<uchar>(y, x) > 50) continue;
            // double global_x = (x + 0.5) * resolution + 0;
            // double global_y = 0 - (y + 0.5) * resolution;
            // laser_point_0.push_back({global_x, global_y});
            point_cloud_0->points.push_back({x, y, 0});
        }
    }
    // std::vector<Point2D> laser_point_1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_point_1(new pcl::PointCloud<pcl::PointXYZ>);
    for (int y = 0; y < image_1.rows; ++y) {
        for (int x = 0; x < image_1.cols; ++x) {
            if (image_1.at<uchar>(y, x) > 50) continue;
            // double global_x = (x + 0.5) * resolution + 0;
            // double global_y = 0 - (y + 0.5) * resolution;
            // laser_point_1.push_back({global_x, global_y});
            laser_point_1->points.push_back({x, y, 0});
        }
    }

    TriangleMatcher matcher;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> wall_lines_0;

    matcher.processPointCloud(point_cloud_0, wall_lines_0);
    // matcher.processPointCloud(laser_point_1, wall_lines);

    visualizeLines(point_cloud_0, wall_lines_0);


    // show in pcl
    // {
    //     // 显示点云
    //     pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    //     viewer.addPointCloud(laser_point_0, "cloud_0");
    //     // viewer.addPointCloud(filtered_cloud_0, "cloud_1");

    //     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_0");
    //     // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud_1");
    
    //     // 添加坐标系 (长度为1)
    //     viewer.addCoordinateSystem(1.0);
    
    //     // 循环，直到用户关闭视图
    //     while (!viewer.wasStopped()) {
    //         viewer.spinOnce(100);
    //     }
    // }

    // std::vector<Submap> all_submaps;
    // Submap s0{map_grid_0, Pose2D{0,0,0}};
    // Submap s1{map_grid_1, Pose2D{result.x, result.y, result.theta}};
    // all_submaps.push_back(s0);
    // all_submaps.push_back(s1);
    // cv::Mat merged = RenderSubmaps(all_submaps);
    // cv::namedWindow("merged", cv::WINDOW_NORMAL);
    // cv::imshow("merged", merged);
    // cv::waitKey(0);


#endif

#if FEATUREMATCHING
    HSMParams params;
    params.d_theta = 0.5 * M_PI / 180.0;
    params.d_rho = 0.05;
    params.phi_max = M_PI;
    params.T_max = 1.0;
    params.n_phi = 5;
    params.n_c = 4;
    HoughScanMatch hsm(params);

    // HoughScanMatcher::Params params;
    // HoughScanMatcher matcher(params);
    // std::vector<Point2D> ref_points;
    // Grid2Point2D(grid_0, ref_points);
    // std::vector<Point2D> sen_points;
    // Grid2Point2D(grid_1, sen_points);

    cv::Mat binary_image_0;
    cv::threshold(image_0, binary_image_0, 1, 255, cv::THRESH_BINARY_INV);  // 黑色障碍物点变成白色，其它点变为黑色
    cv::Mat binary_image_1;
    cv::threshold(image_1, binary_image_1, 1, 255, cv::THRESH_BINARY_INV);  // 黑色障碍物点变成白色，其它点变为黑色
    // cv::namedWindow("binary_image_0", cv::WINDOW_NORMAL);
    // cv::imshow("binary_image_0", binary_image_0);
    // cv::waitKey(1);

    cv::Mat filter_binary_image_0;
    RemoveSmallRegion(binary_image_0, filter_binary_image_0, 5, 1, 1);
    cv::Mat filter_binary_image_1;
    RemoveSmallRegion(binary_image_1, filter_binary_image_1, 5, 1, 1);
    // cv::namedWindow("filter_binary_image_1", cv::WINDOW_NORMAL);
    // cv::imshow("filter_binary_image_1", filter_binary_image_1);
    // cv::waitKey(1);

    cv::Mat morph_0;
    cv::Mat morph_1;
    // 结构元素（关键）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // 闭运算 = 膨胀 → 腐蚀
    cv::morphologyEx(filter_binary_image_0, morph_0, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(filter_binary_image_1, morph_1, cv::MORPH_CLOSE, kernel);



    std::vector<LaserPoint> ref_points;
    std::vector<LaserPoint> sen_points;
    // Grid2LaserPoint(grid_0, ref_points);
    // Grid2LaserPoint(grid_1, sen_points);
    Image2Points(filter_binary_image_0, ref_points);
    Image2Points(filter_binary_image_1, sen_points);

    auto [theta, translation] = hsm.match(ref_points, sen_points);
    
    std::cout << "Estimated Pose: x=" << translation[0]
              << ", y=" << translation[1]
              << ", theta=" << theta << std::endl;

    std::vector<Submap> all_submaps;
    Submap s0{grid_0, Pose2D{0,0,0}};
    // Submap s1{grid_1, Pose2D{result.x, result.y, result.theta}};
    Submap s1{grid_1, Pose2D{translation[0], translation[1], theta}};
    all_submaps.push_back(s0);
    all_submaps.push_back(s1);
    cv::Mat merged = RenderSubmaps(all_submaps);
    cv::namedWindow("merged", cv::WINDOW_NORMAL);
    cv::imshow("merged", merged);
    cv::waitKey(0);
#endif

#if ORBMATCHING
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat desp1, desp2;

    orb->detectAndCompute(image_0, cv::Mat(), kp1, desp1);
    orb->detectAndCompute(image_1, cv::Mat(), kp2, desp2);

    // show
    // cv::Mat imgShow1;
    // cv::Mat imgShow2;
    // cv::drawKeypoints( image_0, kp1, imgShow1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // cv::drawKeypoints( image_1, kp2, imgShow2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // cv::namedWindow("keypoints1", cv::WINDOW_NORMAL);
    // cv::imshow( "keypoints1", imgShow1 );
    // cv::namedWindow("keypoints2", cv::WINDOW_NORMAL);
    // cv::imshow( "keypoints2", imgShow2 );
    // cv::waitKey(0);

    std::vector<std::vector<cv::DMatch>> knn_matches; // 汉明距离值，原理是比较二进制描述子中有多少bit不一样
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.knnMatch(desp1, desp2, knn_matches, 2);

    // 比值筛选（ratio test）
    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.75f;
    for (const auto& m : knn_matches) {
        if (m.size() == 2 && m[0].distance < ratio_thresh * m[1].distance) {
            good_matches.push_back(m[0]);
        }
    }

    // 可视化：显示匹配的特征
    cv::Mat imgMatches;
    cv::drawMatches( image_0, kp1, image_1, kp2, good_matches, imgMatches );
    cv::namedWindow("matches", cv::WINDOW_NORMAL);
    cv::imshow( "matches", imgMatches );
    cv::waitKey( 0 );
#endif

#if CORRELATIVESCANMATCHING

    PointCloud cloud_1;
    Image2Grid(image_0, grid_0);
    Image2Grid(image_1, grid_1);
    Grid2PointCloud(grid_1, cloud_1);

    FastCorrelativeScanMatcher2D matcher(grid_0, 4);
    Candidate result = matcher.match(cloud_1, M_PI);
    std::cout << "x: " << result.pose.x << " y: " << result.pose.y << " theta: " << result.pose.theta << " score: " << result.score << std::endl;
    
    // std::vector<Submap> all_submaps;
    // Submap s0{grid_0, Pose2D{0,0,0}};
    // Submap s1{grid_1, result.pose};
    // all_submaps.push_back(s0);
    // all_submaps.push_back(s1);
    // cv::Mat merged = RenderSubmaps(all_submaps);
    // cv::namedWindow("merged", cv::WINDOW_NORMAL);
    // cv::imshow("merged", merged);
    // cv::waitKey(0);
#endif

#if FILE
    std::ofstream file("../data/submap0.txt");
    if (!file.is_open()) {
        std::cout << "Failed to open file\n";
        return -1;
    }

    for (int i = 0; i < height_0; ++i) {
        for (int j = 0; j < width_0; ++j) {
            file << grid_0.getProbability(j,i);
        }
        file << "\n";
    }
    file.close();
#endif
    
#if SHOW
    cv::Mat show(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<Eigen::Vector2f> points_0;
    std::vector<Eigen::Vector2f> points_1;
    Image2Points(image_0, points_0);
    Image2Points(image_1, points_1);

    int c_x = width / 2;
    int c_y = height / 2;

    for (auto& p : points_0) {
        int x = static_cast<int>(c_x + p.x());
        int y = static_cast<int>(c_y - p.y());

        if (x >= 0 && x < width && y >= 0 && y < height) {
            cv::circle(show, cv::Point2d(x, y), 1, cv::Scalar(0, 0, 255), -1);
        }
    }
    for (auto& p : points_1) {
        int x = static_cast<int>(c_x + p.x());
        int y = static_cast<int>(c_y - p.y());

        if (x >= 0 && x < width && y >= 0 && y < height) {
            cv::circle(show, cv::Point2d(x, y), 1, cv::Scalar(0, 255, 0), -1);
        }
    }
    // 创建可调窗口
    cv::namedWindow("Points Image", cv::WINDOW_NORMAL);
    cv::imshow("Points Image", show);
    cv::waitKey(0);
#endif
    return 0;
}