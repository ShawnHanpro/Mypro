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

void calIntersection(int width, int height, std::vector<cv::Vec4i>& lines, std::vector<Point2D>& intersections, float min_dist = 5.0f) {

    for (size_t i = 0; i < lines.size(); i++) {
        for (size_t j = i + 1; j < lines.size(); j++) {

            // 获取直线端点
            float x1 = lines[i][0], y1 = lines[i][1], x2 = lines[i][2], y2 = lines[i][3];
            float x3 = lines[j][0], y3 = lines[j][1], x4 = lines[j][2], y4 = lines[j][3];
        
            // 计算直线的参数 (a1, b1, c1) 和 (a2, b2, c2)
            float a1 = y2 - y1;
            float b1 = x1 - x2;
            float c1 = a1 * x1 + b1 * y1;
        
            float a2 = y4 - y3;
            float b2 = x3 - x4;
            float c2 = a2 * x3 + b2 * y3;
        
            // 计算交点
            float det = a1 * b2 - a2 * b1;
            if (det != 0) {
                float x_inter = (b2 * c1 - b1 * c2) / det;
                float y_inter = (a1 * c2 - a2 * c1) / det;

                if (x_inter < 0 || x_inter >= width || y_inter < 0 || y_inter >= height) {
                    continue; // 交点不在图像范围内，跳过
                }

                Point2D new_pt(x_inter, y_inter);

                bool too_close = false;
                for (const auto& existing_pt : intersections) {
                    float dx = existing_pt.x - new_pt.x;
                    float dy = existing_pt.y - new_pt.y;
                    float dist2 = dx*dx + dy*dy;

                    if (dist2 < min_dist * min_dist) {
                        too_close = true;
                        break;
                    }
                }

                if (!too_close)
                    intersections.push_back(new_pt);
            }
        }
    }

}

// 随机生成一个颜色
cv::Scalar generateRandomColor() {
    // 随机生成 RGB 值，每个值在 0-255 范围内
    int r = rand() % 256;
    int g = rand() % 256;
    int b = rand() % 256;
    return cv::Scalar(b, g, r);  // OpenCV 使用 BGR 顺序
}

struct PoseHypothesis {
    float theta;
    Eigen::Vector2f t;
    int votes = 0;
};

float triangleSimilarity(const TriangleFeature& t1,
                         const TriangleFeature& t2)
{
    std::vector<float> l1 = {t1.l1, t1.l2, t1.l3};
    std::vector<float> l2 = {t2.l1, t2.l2, t2.l3};

    std::sort(l1.begin(), l1.end());
    std::sort(l2.begin(), l2.end());

    float edge_err =
        fabs(l1[0]-l2[0]) +
        fabs(l1[1]-l2[1]) +
        fabs(l1[2]-l2[2]);

    std::vector<float> a1 = {t1.a1, t1.a2, t1.a3};
    std::vector<float> a2 = {t2.a1, t2.a2, t2.a3};

    std::sort(a1.begin(), a1.end());
    std::sort(a2.begin(), a2.end());

    float angle_err =
        fabs(a1[0]-a2[0]) +
        fabs(a1[1]-a2[1]) +
        fabs(a1[2]-a2[2]);

    return exp(-(edge_err + 0.01f * angle_err));
}

bool estimateSE2FromTriangle(
    const TriangleFeature& t1,
    const TriangleFeature& t2,
    Eigen::Matrix2f& R,
    Eigen::Vector2f& t)
{
    std::vector<Eigen::Vector2f> src = {
        {t1.p1.x, t1.p1.y},
        {t1.p2.x, t1.p2.y},
        {t1.p3.x, t1.p3.y}
    };

    std::vector<Eigen::Vector2f> dst = {
        {t2.p1.x, t2.p1.y},
        {t2.p2.x, t2.p2.y},
        {t2.p3.x, t2.p3.y}
    };

    // --- centroid ---
    Eigen::Vector2f cs = Eigen::Vector2f::Zero();
    Eigen::Vector2f cd = Eigen::Vector2f::Zero();

    for(int i=0;i<3;i++){
        cs += src[i];
        cd += dst[i];
    }
    cs /= 3.f;
    cd /= 3.f;

    // --- covariance ---
    Eigen::Matrix2f H = Eigen::Matrix2f::Zero();

    for(int i=0;i<3;i++)
        H += (src[i]-cs)*(dst[i]-cd).transpose();

    Eigen::JacobiSVD<Eigen::Matrix2f> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    R = svd.matrixV()*svd.matrixU().transpose();

    if(R.determinant() < 0){
        Eigen::Matrix2f V = svd.matrixV();
        V.col(1) *= -1;
        R = V*svd.matrixU().transpose();
    }

    t = cd - R*cs;

    return true;
}

bool matchTriangles(
    const std::vector<TriangleFeature>& A,
    const std::vector<TriangleFeature>& B,
    Eigen::Vector2f& best_t,
    float& best_theta)
{
    std::vector<PoseHypothesis> hypotheses;

    for(const auto& ta : A)
    for(const auto& tb : B)
    {
        if(triangleSimilarity(ta,tb) < 0.8f)
            continue;

        Eigen::Matrix2f R;
        Eigen::Vector2f t;

        estimateSE2FromTriangle(ta,tb,R,t);

        PoseHypothesis h;
        h.theta = atan2(R(1,0),R(0,0));
        h.t = t;

        hypotheses.push_back(h);
    }

    if(hypotheses.empty()) return false;

    const float rot_thresh = 5.0f * M_PI/180.f;
    const float trans_thresh = 0.3f;

    for(auto& h : hypotheses)
    {
        for(const auto& other : hypotheses)
        {
            float dtheta = fabs(h.theta - other.theta);
            float dt = (h.t - other.t).norm();

            if(dtheta < rot_thresh && dt < trans_thresh)
                h.votes++;
        }
    }

    auto best = std::max_element(
        hypotheses.begin(),
        hypotheses.end(),
        [](const PoseHypothesis& a,
           const PoseHypothesis& b)
        {
            return a.votes < b.votes;
        });

    if(best->votes < 5)
        return false;

    best_theta = best->theta;
    best_t = best->t;

    return true;
}



int main (int argc, char** argv) {
    std::string png_path = "/home/shan2/brewst/subgrids";

    cv::Mat image_0 = cv::imread(png_path+"/grid_1.png", cv::IMREAD_GRAYSCALE);
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

#if OPENCVFEATUREMATCHING

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
    
    // std::vector<Point2D> intersections_0;
    // std::vector<Point2D> intersections_1;
    // calIntersection(image_0.cols, image_0.rows, lines_0, intersections_0);
    // calIntersection(image_1.cols, image_1.rows, lines_1, intersections_1);
    // std::cout << "intersections_0: " << intersections_0.size() << std::endl;
    // std::cout << "intersections_1: " << intersections_1.size() << std::endl;
    
    // 绘制交点
    // for (auto& p : intersections_0) {
    //     // std::cout << p.x << " " << p.y << " ";
    //     cv::circle(result_0, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 2, cv::Scalar(0, 0, 255), -1);
    // }
    // for (auto& p : intersections_1) {
    //     // std::cout << p.x << " " << p.y << " ";
    //     cv::circle(result_1, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 2, cv::Scalar(0, 0, 255), -1);
    // }
    // std::cout << std::endl;

    std::vector<cv::Point2f> corner_0;
    std::vector<cv::Point2f> corner_1;
    extractCorners(merged_0, corner_0);
    extractCorners(merged_1, corner_1);
    std::cout << "intersections_0: " << corner_0.size() << std::endl;
    std::cout << "intersections_1: " << corner_1.size() << std::endl;

    for (auto& p : corner_0) {
        // std::cout << p.x << " " << p.y << " ";
        cv::circle(result_0, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 2, cv::Scalar(0, 0, 255), -1);
    }
    for (auto& p : corner_1) {
        // std::cout << p.x << " " << p.y << " ";
        cv::circle(result_1, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 2, cv::Scalar(0, 0, 255), -1);
    }
    std::cout << std::endl;

    std::vector<Triangle> triangles_0 = buildTriangles(corner_0);
    std::vector<Triangle> triangles_1 = buildTriangles(corner_1);
    std::cout << "triangles 0: " << triangles_0.size() << std::endl;
    std::cout << "triangles 1: " << triangles_1.size() << std::endl;

    DrawTriangles(result_0, triangles_0);
    DrawTriangles(result_1, triangles_1);

    // TriangleMatcher matcher;
    // std::vector<TriangleFeature> triangles_0;
    // matcher.extractTriangles(intersections_0, triangles_0);
    // std::vector<TriangleFeature> triangles_1;
    // matcher.extractTriangles(intersections_1, triangles_1);

    // std::cout << "triangles_0: " << triangles_0.size() << std::endl;
    // std::cout << "triangles_1: " << triangles_1.size() << std::endl;

    // for (auto& triangle : triangles_0) {
    //     // 为每个三角形生成随机颜色
    //     cv::Scalar randomColor = generateRandomColor();
        
    //     // 绘制三角形的边
    //     cv::line(result_0, cv::Point(static_cast<int>(triangle.p1.x), static_cast<int>(triangle.p1.y)),
    //             cv::Point(static_cast<int>(triangle.p2.x), static_cast<int>(triangle.p2.y)),
    //             randomColor, 2);  // 随机颜色，线宽为2

    //     cv::line(result_0, cv::Point(static_cast<int>(triangle.p2.x), static_cast<int>(triangle.p2.y)),
    //             cv::Point(static_cast<int>(triangle.p3.x), static_cast<int>(triangle.p3.y)),
    //             randomColor, 2);  // 随机颜色，线宽为2

    //     cv::line(result_0, cv::Point(static_cast<int>(triangle.p3.x), static_cast<int>(triangle.p3.y)),
    //             cv::Point(static_cast<int>(triangle.p1.x), static_cast<int>(triangle.p1.y)),
    //             randomColor, 2);  // 随机颜色，线宽为2
    // }

    // for (auto& triangle : triangles_1) {
    //     // 为每个三角形生成随机颜色
    //     cv::Scalar randomColor = generateRandomColor();
        
    //     // 绘制三角形的边
    //     cv::line(result_1, cv::Point(static_cast<int>(triangle.p1.x), static_cast<int>(triangle.p1.y)),
    //             cv::Point(static_cast<int>(triangle.p2.x), static_cast<int>(triangle.p2.y)),
    //             randomColor, 2);  // 随机颜色，线宽为2

    //     cv::line(result_1, cv::Point(static_cast<int>(triangle.p2.x), static_cast<int>(triangle.p2.y)),
    //             cv::Point(static_cast<int>(triangle.p3.x), static_cast<int>(triangle.p3.y)),
    //             randomColor, 2);  // 随机颜色，线宽为2

    //     cv::line(result_1, cv::Point(static_cast<int>(triangle.p3.x), static_cast<int>(triangle.p3.y)),
    //             cv::Point(static_cast<int>(triangle.p1.x), static_cast<int>(triangle.p1.y)),
    //             randomColor, 2);  // 随机颜色，线宽为2
    // }

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