#pragma once

#include "baselib.h"
#include "type/laser_point.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

double computeDistance(const cv::Point2f& p1, const cv::Point2f& p2);

// 计算三角形的边长和内角
struct Triangle {
    cv::Point2f p1, p2, p3;
    double l1, l2, l3;  // 三条边的长度
    double a1, a2, a3;   // 三个角的度数

    void computeFeatures() {
        // 计算边长
        l1 = computeDistance(p1, p2);
        l2 = computeDistance(p2, p3);
        l3 = computeDistance(p3, p1);

        // 计算内角
        a1 = std::acos((l2 * l2 + l3 * l3 - l1 * l1) / (2 * l2 * l3)) * 180 / CV_PI;
        a2 = std::acos((l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 * l3)) * 180 / CV_PI;
        a3 = std::acos((l1 * l1 + l2 * l2 - l3 * l3) / (2 * l1 * l2)) * 180 / CV_PI;
    }

    // 归一化边长和角度，使得旋转、平移不敏感
    void normalize() {
        double maxLength = std::max({l1, l2, l3});
        l1 /= maxLength;
        l2 /= maxLength;
        l3 /= maxLength;

        a1 = a1 / 180.0;
        a2 = a2 / 180.0;
        a3 = a3 / 180.0;
    }

    // 判断三角形是否有效，检查边长和角度是否在合理范围内
    bool isValid(double min_edge_length = 5.0, double max_edge_length = 100.0, 
                 double min_angle = 10.0, double max_angle = 170.0) {
        // 检查边长
        if (l1 < min_edge_length || l2 < min_edge_length || l3 < min_edge_length ||
            l1 > max_edge_length || l2 > max_edge_length || l3 > max_edge_length) {
            return false;
        }

        // 检查角度
        if (a1 < min_angle || a2 < min_angle || a3 < min_angle ||
            a1 > max_angle || a2 > max_angle || a3 > max_angle) {
            return false;
        }

        return true;
    }
};

float lineAngle(const cv::Vec4i& l);
float lineLength(const cv::Vec4i& l);
float pointLineDistance(const cv::Point2f& p, const cv::Vec4i& l);
void mergeLines(std::vector<cv::Vec4i>& lines, std::vector<cv::Vec4i>& merged);
bool lineIntersection(
    const cv::Vec4i& l1,
    const cv::Vec4i& l2,
    cv::Point2f& out);

void extractCorners(std::vector<cv::Vec4i>& merged, std::vector<cv::Point2f>& corners);



// 从角点集合中为每个角点构建三角形，返回有效的三角形集合
std::vector<Triangle> buildTriangles(const std::vector<cv::Point2f>& corners, 
                                     double min_edge_length = 5.0, double max_edge_length = 100.0,
                                     double min_angle = 10.0, double max_angle = 170.0);

void DrawTriangles(
    cv::Mat& image,
    const std::vector<Triangle>& triangles);

struct HSMParams {
    double d_theta = 0.5 * M_PI / 180.0;  // theta分辨率
    double d_rho = 0.05;                  // rho分辨率
    double phi_max = M_PI;                // 旋转搜索上界
    double T_max = 1.0;                   // 平移搜索上界
    int    n_phi = 5;                     // 保留的旋转假设数
    int    n_c = 4;                       // 平移估计的方向数
};

class HoughScanMatch {
public:
    HoughScanMatch(const HSMParams &params);

    // 计算离散霍夫变换
    Eigen::MatrixXd computeDHT(const std::vector<LaserPoint> &points);

    // 计算霍夫谱
    Eigen::VectorXd computeDHS(const Eigen::MatrixXd &dht);

    // 估计旋转角度
    std::vector<double> estimatePhi(const Eigen::VectorXd &dhs_ref, const Eigen::VectorXd &dhs_sen);

    // 估计平移
    Eigen::Vector2d estimateT(const Eigen::MatrixXd &dht_ref, const Eigen::MatrixXd &dht_sen, double phi);

    // 主匹配函数 输出旋转和平移
    std::pair<double, Eigen::Vector2d> match(const std::vector<LaserPoint> &ref_points, const std::vector<LaserPoint> &sen_points);

    void initHoughSpace(double max_rho);

private:
    HSMParams           params_;
    std::vector<double> thetas_;          // 离散的theta值
    int                 n_theta_;         // theta的离散数量
    int                 dht_n_rho_;       // DHT中rho的数量
    int                 dht_rho_offset_;  // DHT中rho的偏移量

    double max_rho_;  // 全局rho范围

    double computeColCorr(const Eigen::VectorXd &col1, const Eigen::VectorXd &col2);
};

class HoughScanMatcher {
public:
    struct Params {
        int theta_bins = 360;    // 角度分辨率 (0-180度，对应360个方向)
        double rho_res = 0.05;   // 距离分辨率 (5cm)
        double max_rho = 30.0;   // 最大感知距离
        int min_points = 10;     // 最小有效点数
    };

    HoughScanMatcher(const Params& params);

    /**
     * @brief 匹配两帧扫描
     * @param ref 参考帧（通常是地图或上一帧）
     * @param sens 当前帧
     * @return 相对位姿 (sens 在 ref 坐标系下的位置)
     */
    Pose2D match(const std::vector<Point2D>& ref, const std::vector<Point2D>& sens) {
        if (ref.size() < p_.min_points || sens.size() < p_.min_points) return {0,0,0};

        // 1. 生成 DHT (Discrete Hough Transform)
        auto ref_dht = computeDHT(ref);
        auto sens_dht = computeDHT(sens);

        // 2. 生成 DHS (Hough Spectrum) 并估计旋转
        auto ref_dhs = computeDHS(ref_dht);
        auto sens_dhs = computeDHS(sens_dht);

		// auto ref_img = DHT2Image(ref_dht);
		// auto sens_img = DHT2Image(sens_dht);
		// cv::imshow("REF DHT", ref_img);
		// cv::imshow("SENS DHT", sens_img);
		// cv::waitKey(1);

        double delta_theta = estimateRotation(ref_dhs, sens_dhs);

        // 3. 补偿旋转后，估计平移 (x, y)
        return estimateTranslation(ref_dht, sens_dht, delta_theta);
    }

private:
    Params p_;
    int rho_bins_;

    // 核心：计算 DHT 矩阵 [Angle][Rho]
    std::vector<std::vector<float>> computeDHT(const std::vector<Point2D>& scan) {
        std::vector<std::vector<float>> dht(p_.theta_bins, std::vector<float>(rho_bins_, 0.0f));
        double angle_step = 2*M_PI / p_.theta_bins;

        for (const auto& pt : scan) {
            for (int i = 0; i < p_.theta_bins; ++i) {
                double theta = i * angle_step;
                double rho = pt.x * std::cos(theta) + pt.y * std::sin(theta);
                int r_idx = static_cast<int>((rho + p_.max_rho) / p_.rho_res);
                if (r_idx >= 0 && r_idx < rho_bins_) {
                    dht[i][r_idx] += 1.0f;
                }
            }
        }
        return dht;
    }

    // 计算平移无关的能谱
    std::vector<float> computeDHS(const std::vector<std::vector<float>>& dht) {
        std::vector<float> dhs(p_.theta_bins, 0.0f);
        for (int i = 0; i < p_.theta_bins; ++i) {
            for (float val : dht[i]) dhs[i] += val * val; // 能量函数
        }
        return dhs;
    }

    double estimateRotation(const std::vector<float>& ref_dhs, const std::vector<float>& sens_dhs) {
        int best_shift = 0;
        float max_corr = -1.0f;

        for (int s = 0; s < p_.theta_bins; ++s) {
            float corr = 0;
            for (int i = 0; i < p_.theta_bins; ++i) {
                corr += ref_dhs[i] * sens_dhs[(i + s) % p_.theta_bins];
            }
            if (corr > max_corr) {
                max_corr = corr;
                best_shift = s;
            }
        }
        // 返回弧度
        return best_shift * (M_PI / p_.theta_bins);
    }

    Pose2D estimateTranslation(const std::vector<std::vector<float>>& ref_dht, 
                               const std::vector<std::vector<float>>& sens_dht, 
                               double phi) {
        // 构造超定方程 A * X = B
        // 其中 A 是 [cos(theta), sin(theta)], B 是 rho 的偏移量
        double sum_cc = 0, sum_cs = 0, sum_ss = 0, sum_dc = 0, sum_ds = 0;
        
        int angle_shift = static_cast<int>(phi / (M_PI / p_.theta_bins));

        for (int i = 0; i < p_.theta_bins; ++i) {
            int ref_idx = (i + angle_shift) % p_.theta_bins;
            double theta = ref_idx * (M_PI / p_.theta_bins);

            // 通过互相关寻找在该角度下的最佳 rho 偏移
            int d_rho_idx = findBestShift(ref_dht[ref_idx], sens_dht[i]);
            double d_rho = d_rho_idx * p_.rho_res;

            // 最小二乘累加器 (Normal Equations)
            double c = std::cos(theta);
            double s = std::sin(theta);
            sum_cc += c * c;
            sum_cs += c * s;
            sum_ss += s * s;
            sum_dc += d_rho * c;
            sum_ds += d_rho * s;
        }

        // 求解 2x2 线性系统:
        // [sum_cc  sum_cs] [x] = [sum_dc]
        // [sum_cs  sum_ss] [y] = [sum_ds]
        double det = sum_cc * sum_ss - sum_cs * sum_cs;
        if (std::abs(det) < 1e-6) return {0, 0, phi};

        double x = (sum_ss * sum_dc - sum_cs * sum_ds) / det;
        double y = (sum_cc * sum_ds - sum_cs * sum_dc) / det;

        return {x, y, phi};
    }

    // 一维信号匹配：寻找使两个投影最重合的偏移量
    int findBestShift(const std::vector<float>& ref_col, const std::vector<float>& sens_col) {
        int best_s = 0;
        float max_corr = -1.0f;
        int search_range = rho_bins_ / 4; // 限制平移搜索范围

        for (int s = -search_range; s <= search_range; ++s) {
            float corr = 0;
            for (int r = 0; r < rho_bins_; ++r) {
                int sens_r = r + s;
                if (sens_r >= 0 && sens_r < rho_bins_) {
                    corr += ref_col[r] * sens_col[sens_r];
                }
            }
            if (corr > max_corr) {
                max_corr = corr;
                best_s = s;
            }
        }
        return -best_s; // 注意符号：ref = sens + shift -> shift = ref - sens
    }
};


// 三角形特征结构体
struct TriangleFeature {
    // 顶点
    Point2D p1, p2, p3;
    // 三边长度（归一化）
    float l1, l2, l3;
    // 三个内角（度）
    float a1, a2, a3;
    // 质心
    Point2D centroid;

    // 计算描述子（归一化，旋转平移不变）
    void computeDescriptor();
};

class TriangleMatcher {
public:
    // 从2D激光点云提取角点（候选顶点）
    std::vector<Point2D> extractCornerPoints(const std::vector<Point2D>& cloud, 
                                             float corner_threshold = 0.1);
    
    // 从角点构建稳定三角形特征
    void extractTriangles(const std::vector<Point2D>& corners, std::vector<TriangleFeature>& triangles,
                                                  float min_side = 50,  // 最小边长（米）
                                                  float max_side = 400,  // 最大边长
                                                  float min_angle = 10.0,// 最小内角（度）
                                                  float max_angle = 170.0);// 最大内角

    // 计算两点间距离
    static float distance(const Point2D& p1, const Point2D& p2);
    
    // 计算三角形内角（p2为顶点）
    static float computeAngle(const Point2D& p1, const Point2D& p2, const Point2D& p3);

    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& wall_lines);

private:
    // 壁角点检测步骤
    void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud);
    void extractLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& lines);
    bool isRightAngle(const Eigen::Vector2d& line1, const Eigen::Vector2d& line2);
    
};


class MapMatcher {
public:
    // 匹配两个地图的三角形特征，返回2D变换（旋转+平移）
    bool match(const std::vector<TriangleFeature>& map1,
               const std::vector<TriangleFeature>& map2,
               Eigen::Vector2f& translation,  // 输出平移
               float& rotation,               // 输出旋转（弧度）
               float match_threshold = 0.8);  // 匹配得分阈值

private:
    // 计算两个三角形特征的相似度（0-1）
    float computeSimilarity(const TriangleFeature& t1, const TriangleFeature& t2);
    
    // RANSAC剔除错误匹配，估计变换
    bool estimateTransform(const std::vector<std::pair<int, int>>& matches,
                           const std::vector<TriangleFeature>& map1,
                           const std::vector<TriangleFeature>& map2,
                           Eigen::Vector2f& translation,
                           float& rotation,
                           float inlier_threshold = 0.1);
};


