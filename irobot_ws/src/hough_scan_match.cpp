#include "hough_scan_match.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PointIndices.h>
// #include <pcl/model_coefficients.h>
#include <pcl/common/common.h>
#include <unordered_set>
#include <random>

HoughScanMatch::HoughScanMatch(const HSMParams& params) : params_(params){
    for (double theta = 0; theta < M_PI; theta += params_.d_theta) {
        thetas_.push_back(theta);
    }
    n_theta_ = thetas_.size();
}

Eigen::MatrixXd HoughScanMatch::computeDHT(const std::vector<LaserPoint>& points) {

    Eigen::MatrixXd dht = Eigen::MatrixXd::Zero(n_theta_, dht_n_rho_);

    for (const auto& p : points) {
        for (int i = 0; i < n_theta_; ++i) {
            double rho = p.x * std::cos(thetas_[i]) + p.y * std::sin(thetas_[i]);
            int rho_idx = static_cast<int>(std::round(rho / params_.d_rho)) + dht_rho_offset_;
            if (rho_idx >= 0 && rho_idx < dht_n_rho_) {
                dht(i, rho_idx) += 1.0;
            }
        }
    }

    return dht;
}

Eigen::VectorXd HoughScanMatch::computeDHS(const Eigen::MatrixXd& dht) {
    Eigen::VectorXd dhs(n_theta_);
    for (int i = 0; i < n_theta_; ++i) {
        dhs(i) = dht.row(i).squaredNorm() / (dht.row(i).sum() + 1e-6);
    }
    return dhs;
}

std::vector<double> HoughScanMatch::estimatePhi(const Eigen::VectorXd& dhs_ref, const Eigen::VectorXd& dhs_sen) {
    int phi_steps = 2 * static_cast<int>(params_.phi_max / params_.d_theta) + 1;
    Eigen::VectorXd corr(phi_steps);

    for (int i = 0; i < phi_steps; ++i) {
        double phi = -params_.phi_max + i * params_.d_theta;
        int phi_idx = static_cast<int>(std::round(phi / params_.d_theta));
        double c = 0;
        for (int j = 0; j < n_theta_; ++j) {
            int ref_j = (j - phi_idx + n_theta_) % n_theta_;
            c += dhs_ref(ref_j) * dhs_sen(j);
        }

        corr(i) = c;
    }

    std::vector<std::pair<double, double>> phi_corrs;
    for (int i = 1; i < phi_steps - 1; ++i) {
        if (corr(i) > corr(i - 1) && corr(i) > corr(i + 1)) {
            double phi = -params_.phi_max + i * params_.d_theta;
            phi_corrs.emplace_back(corr(i), phi);
        }
    }

    std::sort(phi_corrs.rbegin(), phi_corrs.rend());
    std::vector<double> phis;
    for (int i = 0; i < std::min((int)phi_corrs.size(), params_.n_phi); ++i) {
        phis.push_back(phi_corrs[i].second);
    }
    return phis;
}

Eigen::Vector2d HoughScanMatch::estimateT(const Eigen::MatrixXd& dht_ref, const Eigen::MatrixXd& dht_sen, double phi) {
    int phi_idx = std::round(phi / params_.d_theta);
    Eigen::MatrixXd dht_sen_rot = Eigen::MatrixXd::Zero(n_theta_, dht_n_rho_);

    for (int i = 0; i < n_theta_; ++i) {
        int sen_i = (i + phi_idx) % n_theta_;
        if (sen_i < 0) sen_i += n_theta_;
        dht_sen_rot.row(i) = dht_sen.row(sen_i);
    }

    Eigen::VectorXd dhs_sen = computeDHS(dht_sen_rot);
    std::vector<std::pair<double, int>> theta_energy;
    for (int i = 0; i < n_theta_; ++i) {
        theta_energy.emplace_back(dhs_sen(i), i);
    }
    std::sort(theta_energy.rbegin(), theta_energy.rend());

    std::vector<int> theta_idxs;
    for (int i = 0; i < std::min((int)theta_energy.size(), params_.n_c); ++i) {
        theta_idxs.push_back(theta_energy[i].second);
    }

    Eigen::MatrixXd A(params_.n_c, 2);
    Eigen::VectorXd b(params_.n_c);

    // for (int i = 0; i < theta_idxs.size(); ++i) {
    //     int idx = theta_idxs[i];
    //     double theta = thetas_[idx];
    //     double cos_t = cos(theta);
    //     double sin_t = sin(theta);
    //     Eigen::VectorXd col_ref = dht_ref.row(idx);
    //     Eigen::VectorXd col_sen = dht_sen_rot.row(idx);
    //     double d = computeColCorr(col_ref, col_sen);
    //     A(i, 0) = cos_t;
    //     A(i, 1) = sin_t;
    //     b(i) = d;
        
    //     // 调试输出
    //     std::cout << "[T estimation] theta_idx=" << idx << ", theta=" << theta 
    //               << ", delta_rho=" << d << std::endl;
    // }

    // // 打印 A 的维度和前几行
    // std::cout << "Matrix A dimensions: " << A.rows() << "x" << A.cols() << std::endl;
    // std::cout << "First few rows of A:\n";
    // for (int i = 0; i < std::min(3, (int)A.rows()); ++i) {
    //     std::cout << A.row(i).transpose() << std::endl;
    // }
    // // 打印 b 的前几个值，确认 Δρ 是否合理
    // std::cout << "First few values of b:\n";
    // for (int i = 0; i < std::min(3, (int)b.size()); ++i) {
    //     std::cout << b(i) << std::endl;
    // }

    Eigen::Vector2d T = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // std::cout << "Solved T: [" << T(0) << ", " << T(1) << "]" << std::endl;

    return T;
}

double HoughScanMatch::computeColCorr(const Eigen::VectorXd& col1, const Eigen::VectorXd& col2) {
    int max_shift = std::round(params_.T_max / params_.d_rho);
    int n = col1.size();
    double max_corr = -1e10;
    int best_s = 0;

    for (int s = -max_shift; s <= max_shift; ++s) {
        double corr = 0;
        int count = 0;
        for (int i = 0; i < n; ++i) {
            int j = i + s;
            if (j >= 0 && j < n) {
                corr += col1(i) * col2(j);
                count++;
            }
        }

        // 选择相关性最大的偏移
        if (corr > max_corr) {
            max_corr = corr;
            best_s = s;
        }
    }

    // 重要：如果 col2 向右移动了 s 个 bin (best_s > 0)
    // 说明 rho_sen 增大了，即 rho_sen = rho_ref + s*d_rho
    // 在方程 A*T = b 中，b 应该是 Δρ = rho_ref - rho_sen = -s*d_rho
    return -best_s * params_.d_rho;
}

// double HoughScanMatch::computeColCorr(const Eigen::VectorXd& col1, const Eigen::VectorXd& col2) {
//     int max_shift = std::round(params_.T_max / params_.d_rho);
//     int n = col1.size();
//     double max_corr = -1e10;
//     int best_s = 0;

//     for (int s = -max_shift; s <= max_shift; ++s) {
//         double corr = 0;
//         for (int i = 0; i < n; ++i) {
//             int j = i + s;
//             if (j >= 0 && j < n) {
//                 corr += col1(i) * col2(j);
//             }
//         }
//         if (corr > max_corr) {
//             max_corr = corr;
//             best_s = s;
//         }
//     }
//     return best_s * params_.d_rho;
// }

double computeGlobalMaxRho(const std::vector<LaserPoint> &a,
                           const std::vector<LaserPoint> &b) {
  double max_rho = 0;
  for (auto &p : a)
    max_rho = std::max(max_rho, hypot(p.x, p.y));
  for (auto &p : b)
    max_rho = std::max(max_rho, hypot(p.x, p.y));
  return max_rho;
}

void HoughScanMatch::initHoughSpace(double max_rho)
{
    // 保存全局 rho 范围
    max_rho_ = max_rho;

    // rho ∈ [-max_rho, max_rho]
    dht_n_rho_ =
        static_cast<int>(
            std::ceil(2.0 * max_rho_ / params_.d_rho)
        ) + 1;

    // rho=0 对应的 index
    dht_rho_offset_ = dht_n_rho_ / 2;

    // std::cout << "[HSM] Hough space initialized\n"
    //           << "  theta bins : " << n_theta_ << "\n"
    //           << "  rho bins   : " << dht_n_rho_ << "\n"
    //           << "  rho range  : ±" << max_rho_
    //           << std::endl;
}

std::pair<double, Eigen::Vector2d> HoughScanMatch::match(const std::vector<LaserPoint>& ref_points, const std::vector<LaserPoint>& sen_points) {
    double max_rho = computeGlobalMaxRho(ref_points, sen_points);
    initHoughSpace(max_rho);

    Eigen::MatrixXd dht_ref = computeDHT(ref_points);
    Eigen::MatrixXd dht_sen = computeDHT(sen_points);
    Eigen::VectorXd dhs_ref = computeDHS(dht_ref);
    Eigen::VectorXd dhs_sen = computeDHS(dht_sen);

    // std::cout << "dht_ref: " << dht_ref.rows() << " " << dht_ref.cols() << std::endl;
    // std::cout << "dhs ref: " << dhs_ref.size() << std::endl;

    std::vector<double> phis = estimatePhi(dhs_ref, dhs_sen);

    std::cout << "dht_ref: " << dht_ref.rows() << "x" << dht_ref.cols() << std::endl;
    std::cout << "dht_sen: " << dht_sen.rows() << "x" << dht_sen.cols() << std::endl;

    // 在 estimatePhi 后打印 phi 的值
    std::cout << "Estimated phi values: ";
    for (const double& phi_val : phis) {
        std::cout << phi_val << " ";
    }
    std::cout << std::endl;

    if (phis.empty()) {
        std::cerr << "no phi hypothesis found!" << std::endl;
        return {0, Eigen::Vector2d(0, 0)};
    }
    double best_phi = phis[0];

    Eigen::Vector2d best_T = estimateT(dht_ref, dht_sen, best_phi);

    return {best_phi, best_T};
}

HoughScanMatcher::HoughScanMatcher(const Params& params = Params()) : p_(params) {
    rho_bins_ = static_cast<int>(p_.max_rho * 2 / p_.rho_res);
}

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

using namespace std;

// 计算三角形描述子
void TriangleFeature::computeDescriptor() {
    TriangleMatcher matcher;

    // 计算边长
    l1 = TriangleMatcher::distance(p2, p3);
    l2 = TriangleMatcher::distance(p1, p3);
    l3 = TriangleMatcher::distance(p1, p2);
    
    // 归一化边长（旋转平移不变）
    float sum_l = l1 + l2 + l3;
    l1 /= sum_l;
    l2 /= sum_l;
    l3 /= sum_l;

    // 计算内角
    a1 = TriangleMatcher::computeAngle(p2, p1, p3);
    a2 = TriangleMatcher::computeAngle(p1, p2, p3);
    a3 = TriangleMatcher::computeAngle(p1, p3, p2);

    // 计算质心
    centroid.x = (p1.x + p2.x + p3.x) / 3.0f;
    centroid.y = (p1.y + p2.y + p3.y) / 3.0f;
}

// 提取角点（基于曲率/邻域变化）
std::vector<Point2D> TriangleMatcher::extractCornerPoints(const std::vector<Point2D>& cloud,
                                                             float corner_threshold) {
    std::vector<Point2D> corners;
    int n = cloud.size();
    if (n < 5) return corners;

    // 遍历点云，计算邻域曲率
    for (int i = 2; i < n-2; ++i) {
        const auto& p_prev = cloud[i-1];
        const auto& p_curr = cloud[i];
        const auto& p_next = cloud[i+1];

        // 计算前后向量
        Eigen::Vector2f v1(p_prev.x - p_curr.x, p_prev.y - p_curr.y);
        Eigen::Vector2f v2(p_next.x - p_curr.x, p_next.y - p_curr.y);
        v1.normalize();
        v2.normalize();

        // 计算夹角（曲率），夹角越小越可能是角点
        float angle = acos(v1.dot(v2)) * 180 / M_PI;
        if (angle < 180 - corner_threshold * 180) {
            corners.emplace_back(p_curr.x, p_curr.y);
        }
    }
    return corners;
}

// 构建稳定三角形
void TriangleMatcher::extractTriangles(const vector<Point2D>& corners, std::vector<TriangleFeature>& triangles,
                                                                   float min_side,
                                                                   float max_side,
                                                                   float min_angle,
                                                                   float max_angle) {
    
    int n = corners.size();
    if (n < 3) return ;

    // 遍历所有三元组（剪枝避免重复）
    for (int i = 0; i < n; ++i) {
        for (int j = i+1; j < n; ++j) {
            for (int k = j+1; k < n; ++k) {
                TriangleFeature tf;
                tf.p1 = corners[i];
                tf.p2 = corners[j];
                tf.p3 = corners[k];

                // 计算边长并筛选
                float l1 = distance(tf.p2, tf.p3);
                float l2 = distance(tf.p1, tf.p3);
                float l3 = distance(tf.p1, tf.p2);
                // std::cout << "Triangle sides: " << l1 << ", " << l2 << ", " << l3 << std::endl;
                if (l1 < min_side || l2 < min_side || l3 < min_side ||
                    l1 > max_side || l2 > max_side || l3 > max_side) {
                    continue;
                }

                // 计算内角并筛选
                float a1 = computeAngle(tf.p2, tf.p1, tf.p3);
                float a2 = computeAngle(tf.p1, tf.p2, tf.p3);
                float a3 = computeAngle(tf.p1, tf.p3, tf.p2);
                // std::cout << "Triangle angles: " << a1 << ", " << a2 << ", " << a3 << std::endl;
                if (a1 < min_angle || a2 < min_angle || a3 < min_angle ||
                    a1 > max_angle || a2 > max_angle || a3 > max_angle) {
                    continue;
                }

                // 计算最终描述子
                tf.computeDescriptor();
                triangles.push_back(tf);
            }
        }
    }
}

// 辅助函数：两点距离
float TriangleMatcher::distance(const Point2D& p1, const Point2D& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// 辅助函数：计算内角
float TriangleMatcher::computeAngle(const Point2D& p1, const Point2D& p2, const Point2D& p3) {
    Eigen::Vector2f v1(p1.x - p2.x, p1.y - p2.y);
    Eigen::Vector2f v2(p3.x - p2.x, p3.y - p2.y);
    v1.normalize();
    v2.normalize();
    float dot = v1.dot(v2);
    // 防止数值溢出
    if (dot < -1.0f) dot = -1.0f;
    if (dot >  1.0f) dot =  1.0f;
    return acos(dot) * 180 / M_PI;
}

void TriangleMatcher::processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& wall_lines) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filterPointCloud(cloud, filtered_cloud);
    // 提取墙壁
    extractLines(filtered_cloud, wall_lines);

    // 计算直角墙角
    // for (size_t i = 0; i < wall_lines.size(); ++i) {
    //     for (size_t j = i + 1; j < wall_lines.size(); ++j) {
    //         if (isRightAngle(wall_lines[i], wall_lines[j])) {
    //             std::cout << "Found a right angle between line " << i << " and line " << j << std::endl;
    //         }
    //     }
    // }
}

void TriangleMatcher::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f); // 设置滤波器参数
    voxel_grid.filter(*filtered_cloud);

        std::cout << "Original cloud size: " << cloud->size()
              << ", filtered size: " << filtered_cloud->size() << std::endl;
}



void TriangleMatcher::extractLines(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& lines)
{
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

    while (temp_cloud->size() > 20) {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setInputCloud(temp_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            break;
        }

        // 拟合直线的起点和方向
        Eigen::Vector2d p0(coefficients->values[0], coefficients->values[1]);
        Eigen::Vector2d direction(coefficients->values[3], coefficients->values[4]);

        // 计算端点，用于可视化（沿方向延伸 +/- scale）
        double scale = 5.0;
        Eigen::Vector2d start = p0 - direction * scale;
        Eigen::Vector2d end   = p0 + direction * scale;

        lines.push_back({start, end});

        // 移除已拟合的内点
        std::unordered_set<int> inlier_set(inliers->indices.begin(), inliers->indices.end());
        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < temp_cloud->points.size(); ++i) {
            if (inlier_set.find(i) == inlier_set.end()) {
                remaining->points.push_back(temp_cloud->points[i]);
            }
        }
        temp_cloud.swap(remaining);

        std::cout << "Remaining points: " << temp_cloud->size() << std::endl;
    }
}

// 匹配两个地图的三角形特征
bool MapMatcher::match(const vector<TriangleFeature>& map1,
                       const vector<TriangleFeature>& map2,
                       Eigen::Vector2f& translation,
                       float& rotation,
                       float match_threshold) {
    if (map1.empty() || map2.empty()) return false;

    // 1. 粗匹配：计算所有特征对的相似度
    vector<pair<int, int>> candidate_matches;
    for (int i = 0; i < map1.size(); ++i) {
        for (int j = 0; j < map2.size(); ++j) {
            float sim = computeSimilarity(map1[i], map2[j]);
            if (sim > 0.7) {  // 粗匹配阈值
                candidate_matches.emplace_back(i, j);
            }
        }
    }

    if (candidate_matches.size() < 3) return false;

    // 2. 精匹配：RANSAC估计变换
    bool success = estimateTransform(candidate_matches, map1, map2, translation, rotation);
    if (!success) return false;

    // 3. 计算匹配得分
    int inliers = 0;
    for (const auto& match : candidate_matches) {
        int i = match.first;
        int j = match.second;
        const auto& t1 = map1[i];
        const auto& t2 = map2[j];

        // 应用变换到t1质心，计算与t2质心的距离
        Eigen::Vector2f c1(t1.centroid.x, t1.centroid.y);
        Eigen::Rotation2Df rot(rotation);
        Eigen::Vector2f c1_transformed = rot * c1 + translation;
        float dist = (c1_transformed - Eigen::Vector2f(t2.centroid.x, t2.centroid.y)).norm();
        
        if (dist < 0.1) {  // 内点阈值（米）
            inliers++;
        }
    }

    float match_score = (float)inliers / candidate_matches.size();
    return match_score > match_threshold;
}

// 计算两个三角形的相似度（基于归一化边长+内角）
float MapMatcher::computeSimilarity(const TriangleFeature& t1, const TriangleFeature& t2) {
    // 边长相似度（归一化后）
    float l_sim = 1 - (fabs(t1.l1 - t2.l1) + fabs(t1.l2 - t2.l2) + fabs(t1.l3 - t2.l3)) / 3.0f;
    
    // 内角相似度（排序后匹配，避免顶点顺序影响）
    vector<float> a1 = {t1.a1, t1.a2, t1.a3};
    vector<float> a2 = {t2.a1, t2.a2, t2.a3};
    sort(a1.begin(), a1.end());
    sort(a2.begin(), a2.end());
    float angle_sim = 1 - (fabs(a1[0]-a2[0]) + fabs(a1[1]-a2[1]) + fabs(a1[2]-a2[2])) / (3*180.0f);
    
    // 加权求和（边长权重0.6，内角0.4，论文经验值）
    return 0.6 * l_sim + 0.4 * angle_sim;
}

// RANSAC估计2D刚性变换
bool MapMatcher::estimateTransform(const vector<pair<int, int>>& matches,
                                   const vector<TriangleFeature>& map1,
                                   const vector<TriangleFeature>& map2,
                                   Eigen::Vector2f& translation,
                                   float& rotation,
                                   float inlier_threshold) {
    const int max_iter = 100;
    const int min_samples = 3;
    int best_inliers = 0;
    Eigen::Vector2f best_trans;
    float best_rot = 0;

    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, matches.size()-1);

    for (int iter = 0; iter < max_iter; ++iter) {
        // 随机选3个样本
        vector<pair<int, int>> sample;
        while (sample.size() < min_samples) {
            int idx = dis(gen);
            sample.push_back(matches[idx]);
        }

        // 从3个样本估计变换
        vector<Eigen::Vector2f> src_centroids, dst_centroids;
        for (const auto& m : sample) {
            const auto& t1 = map1[m.first];
            const auto& t2 = map2[m.second];
            src_centroids.emplace_back(t1.centroid.x, t1.centroid.y);
            dst_centroids.emplace_back(t2.centroid.x, t2.centroid.y);
        }

        // 计算质心
        Eigen::Vector2f src_mean = Eigen::Vector2f::Zero();
        Eigen::Vector2f dst_mean = Eigen::Vector2f::Zero();
        for (int i = 0; i < min_samples; ++i) {
            src_mean += src_centroids[i];
            dst_mean += dst_centroids[i];
        }
        src_mean /= min_samples;
        dst_mean /= min_samples;

        // 计算旋转（SVD）
        Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
        for (int i = 0; i < min_samples; ++i) {
            Eigen::Vector2f src = src_centroids[i] - src_mean;
            Eigen::Vector2f dst = dst_centroids[i] - dst_mean;
            H += src * dst.transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2f R = svd.matrixV() * svd.matrixU().transpose();
        if (R.determinant() < 0) {
            Eigen::Matrix2f V = svd.matrixV();
            V.col(1) *= -1;
            R = V * svd.matrixU().transpose();
        }

        // 计算旋转角和平移
        float rot = atan2(R(1,0), R(0,0));
        Eigen::Vector2f trans = dst_mean - R * src_mean;

        // 统计内点
        int inliers = 0;
        for (const auto& m : matches) {
            const auto& t1 = map1[m.first];
            const auto& t2 = map2[m.second];
            Eigen::Vector2f c1(t1.centroid.x, t1.centroid.y);
            Eigen::Vector2f c1_transformed = R * c1 + trans;
            float dist = (c1_transformed - Eigen::Vector2f(t2.centroid.x, t2.centroid.y)).norm();
            if (dist < inlier_threshold) {
                inliers++;
            }
        }

        // 更新最优解
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_rot = rot;
            best_trans = trans;
        }
    }

    if (best_inliers < min_samples * 2) return false;

    rotation = best_rot;
    translation = best_trans;
    return true;
}

float lineAngle(const cv::Vec4i& l)
{
    return atan2(l[3]-l[1], l[2]-l[0]);
}

float lineLength(const cv::Vec4i& l)
{
    return hypot(l[2]-l[0], l[3]-l[1]);
}

float pointLineDistance(const cv::Point2f& p,
                               const cv::Vec4i& l)
{
    cv::Point2f a(l[0],l[1]);
    cv::Point2f b(l[2],l[3]);

    cv::Point2f v=b-a;
    cv::Point2f w=p-a;

    float area=fabs(v.x*w.y-v.y*w.x);
    return area/cv::norm(v);
}

void mergeLines(std::vector<cv::Vec4i>& lines, std::vector<cv::Vec4i>& merged) {
    for(const auto& l : lines)
    {
        bool merged_flag=false;

        for(auto& m:merged)
        {
            float da=fabs(lineAngle(l)-lineAngle(m))*180/M_PI;
            if(da>90) da=180-da;

            float d=pointLineDistance(
                    cv::Point2f(l[0],l[1]), m);

            if(da<5.0 && d<15.0)
            {
                if(lineLength(l)>lineLength(m))
                    m=l;
                merged_flag=true;
                break;
            }
        }

        if(!merged_flag)
            merged.push_back(l);
    }
}

bool lineIntersection(
    const cv::Vec4i& l1,
    const cv::Vec4i& l2,
    cv::Point2f& out)
{
    cv::Point2f p(l1[0], l1[1]);
    cv::Point2f r(l1[2]-l1[0], l1[3]-l1[1]);

    cv::Point2f q(l2[0], l2[1]);
    cv::Point2f s(l2[2]-l2[0], l2[3]-l2[1]);

    float denom = r.x*s.y - r.y*s.x;

    if (fabs(denom) < 1e-6)
        return false; // 平行

    cv::Point2f qp = q - p;

    float t = (qp.x*s.y - qp.y*s.x)/denom;

    // ⭐ 注意：不再检查 t,u 范围
    out = p + t*r;

    return true;
}

void extractCorners(std::vector<cv::Vec4i>& merged, std::vector<cv::Point2f>& corners) {

    for(size_t i=0;i<merged.size();++i)
    for(size_t j=i+1;j<merged.size();++j)
    {
        float a1=lineAngle(merged[i]);
        float a2=lineAngle(merged[j]);

        float ang=fabs(a1-a2)*180/M_PI;
        if(ang>90) ang=180-ang;

        if(ang<80 || ang>100)
            continue;

        cv::Point2f pt;
        if(lineIntersection(merged[i],merged[j],pt))
            corners.push_back(pt);
    }
}