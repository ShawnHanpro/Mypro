#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#define GROUND
// #define GAP

struct Frame {
    std::string              timestamp;
    std::vector<cv::Point3f> xyz_points;
};

class Point3D {
public:
    float x;
    float y;
    float z;

    Point3D() : x(0), y(0), z(0) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Point3D operator+(const Point3D& other) const { return Point3D(x + other.x, y + other.y, z + other.z); }

    Point3D& operator+=(const Point3D& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
};

bool isSamePoint(const Point3D& a, const Point3D& b, double eps = 1e-6) {
    return (std::fabs(a.x - b.x) < eps && std::fabs(a.y - b.y) < eps && std::fabs(a.z - b.z) < eps);
}

std::vector<Point3D> symmetricDifference(const std::vector<Point3D>& A, const std::vector<Point3D>& B, double eps = 1e-6) {
    std::vector<Point3D> result;

    // A 中不在 B 中
    for (const auto& pa : A) {
        bool found = false;
        for (const auto& pb : B) {
            if (isSamePoint(pa, pb, eps)) {
                found = true;
                break;
            }
        }
        if (!found) result.push_back(pa);
    }

    // B 中不在 A 中
    for (const auto& pb : B) {
        bool found = false;
        for (const auto& pa : A) {
            if (isSamePoint(pa, pa, eps)) {
                found = true;
                break;
            }
        }
        if (!found) result.push_back(pb);
    }

    return result;
}

void FindTheGround0(const std::vector<Point3D>& points, std::vector<Point3D>& ground, int neighbor_size = 10, float slope_thresh = 0.25f,
                    float rms_thresh = 0.03f) {
    ground.clear();
    ground.reserve(points.size());

    const int points_num = static_cast<int>(points.size());
    if (points_num == 0) return;

    // v1.0 查找最低点，作为地面位置参考
    float minz = std::numeric_limits<float>::max();
    for (auto& p : points) minz = std::min(minz, (float)p.z);

    for (int i = 0; i < points_num; ++i) {
        int start = std::max(0, i - neighbor_size);
        int end = std::min(points_num - 1, i + neighbor_size);
        int N = end - start + 1;
        if (N < 3) continue;  // 点太少无法拟合稳定直线

        // 构造 A * [a; b] = z  ， A = [ y; 1 ]
        Eigen::MatrixXf A(N, 2);
        Eigen::VectorXf zvec(N);
        for (int k = 0; k < N; ++k) {
            const Point3D& p = points[start + k];
            A(k, 0) = p.z;
            A(k, 1) = 1.0f;
            zvec(k) = p.z;
        }

        // 最小二乘求解 a,b
        // x_hat = (A^T*A)^{-1}*A^T*z
        // 正规方程是求解线性最小二乘问题的一种解析方法
        // ldlt分解直接解析线性方程比使用求逆更稳定更快，.solve()是把括号内容当做已知向量
        Eigen::Vector2f sol = (A.transpose() * A).ldlt().solve(A.transpose() * zvec);
        float           a = sol(0);  // 斜率 z = a*y + b
        // float b = sol(1);

        // 计算拟合残差 RMS
        Eigen::VectorXf z_pred = A * sol;
        Eigen::VectorXf diff = zvec - z_pred;
        float           rms = std::sqrt(diff.squaredNorm() / static_cast<float>(N));

        // std::cout << "a: " << a << std::endl;
        // std::cout << "rms: " << rms << std::endl;

        // 判断：斜率小 && 残差小 -> 地面点
        if (std::abs(a) <= slope_thresh && rms <= rms_thresh) {
            if (points[i].z > minz + 0.01f) continue;
            ground.push_back(points[i]);
        }
    }
}

// 输出Ax + By + C = 0
Eigen::MatrixXf IepfFunction(const std::vector<Point3D>& points, Eigen::MatrixXf& np_end_points, float dis_threshold = 0.02) {

    bool has_new_break = false;
    int insert_pos = -1;
    Eigen::Vector3f break_point;

    // 遍历每一对端点 -By = Ax + C, B = -1
    for (int i = 0; i < np_end_points.cols() - 1; ++i) {
        float max_dis_point_to_line = 0.0f; // 最大距离初始化为0
        int break_point_index = -1; // 断点索引初始化-1

        // 计算斜率A
        float var_A = (np_end_points(1, i + 1) - np_end_points(1, i)) / (np_end_points(0, i + 1) - np_end_points(0, i));
        float var_B = -1.0f;
        float var_C = np_end_points(1, i) - var_A * np_end_points(0, i);

        // 遍历端点之间的点
        int start_index = static_cast<int>(np_end_points(2, i));
        int end_insdex = static_cast<int>(np_end_points(2, i + 1));
        // std::cout << "np end points: " << np_end_points.cols() << std::endl;
        for (int j = start_index + 1; j < end_insdex; ++j) {
            // 跳过首尾点
            // if (j == 0 || j == np_end_points(2, i)) continue;
            // 计算点到直线的距离
            // float dis_point_to_line = std::fabs((var_A * points[j].y + var_B * points[j].z + var_C) / (std::sqrt(var_A * var_A + var_B * var_B)));
            float dis_point_to_line = std::fabs((var_A * points[j].x + var_B * points[j].y + var_C) / (std::sqrt(var_A * var_A + var_B * var_B)));
            // std::cout << "dis point to line: " << dis_point_to_line << std::endl;
            // 如果距离超过阈值，检查是否为最大值
            if (dis_point_to_line > dis_threshold && dis_point_to_line > max_dis_point_to_line) {
                max_dis_point_to_line = dis_point_to_line;
                break_point_index = j;
            }
        }

        if (break_point_index != -1) {
            has_new_break = true;
            insert_pos = i + 1;
            // break_point = Eigen::Vector3f(static_cast<float>(points[break_point_index].y), static_cast<float>(points[break_point_index].z),
            //                               static_cast<float>(break_point_index));
            break_point = Eigen::Vector3f(static_cast<float>(points[break_point_index].x), static_cast<float>(points[break_point_index].y),
                                          static_cast<float>(break_point_index));
            break;
        }
    }

    // 如果没有新断点，终止递归
    if (!has_new_break) return np_end_points;

    // 列
    int old_cols = np_end_points.cols();
    int new_cols = old_cols + 1;
    // 插入列必须新建矩阵
    Eigen::MatrixXf new_np_end_points(3, new_cols);

    // 复制插入点之前的值
    if (insert_pos > 0) {
        // 0行0列开始，取3行insert_pos列
        new_np_end_points.block(0, 0, 3, insert_pos) = np_end_points.block(0, 0, 3, insert_pos);
    }
    // 插入新断点
    new_np_end_points.col(insert_pos) = break_point;
    // 复制插入点之后的值
    if ((old_cols - insert_pos) > 0) {
        new_np_end_points.block(0, insert_pos + 1, 3, old_cols - insert_pos) = np_end_points.block(0, insert_pos, 3, old_cols - insert_pos);
    }

    np_end_points = new_np_end_points;

    np_end_points = IepfFunction(points, np_end_points, dis_threshold);

    return np_end_points;
}

float Distance3D(const Point3D& p1, const Point3D& p2) {
    return std::sqrt(
        (p2.x - p1.x) * (p2.x - p1.x) +
        (p2.y - p1.y) * (p2.y - p1.y) +
        (p2.z - p1.z) * (p2.z - p1.z)
    );
}

std::vector<Point3D> InterpolatePoints(const Point3D& p1, const Point3D& p2, float step) {
    std::vector<Point3D> points;
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist < 1e-6) return {};  // 两点太近

    int num_steps = static_cast<int>(dist / step);
    points.reserve(num_steps);

    for (int i = 0; i <= num_steps; ++i) {
        float ratio = static_cast<float>(i) / num_steps;
        Point3D p;
        p.x = p1.x + ratio * dx;
        p.y = p1.y + ratio * dy;
        p.z = p1.z + ratio * dz;
        points.push_back(p);
    }

    return points;
}

std::pair<int, std::vector<Point3D>> FindGap(const std::vector<Point3D>& points) {
    int key = -1;
    std::vector<Point3D> points_in_laser = points;
    std::pair<int, std::vector<Point3D>> pair_out(key, points_in_laser);

    float  gap_dis = 0.01f;
    size_t gap_num = 0;
    std::vector<std::pair<Point3D, Point3D>> gaps_sides_points;

    if (!points.empty()) {
        float max_dis = 0;
        for (size_t i = 0; i < points.size() - 1; ++i) {
            Point3D p1 = points[i];
            Point3D p2 = points[i + 1];

            float dis = Distance3D(p1, p2);
            if (dis > max_dis) max_dis = dis;

            if (dis > gap_dis) {
                ++gap_num;
                std::pair<Point3D, Point3D> gap_sides_points(p1, p2);
                gaps_sides_points.emplace_back(gap_sides_points);
            }
        }

        Point3D wall_p;
        Point3D ground_p;
        if (gap_num > 0) {
            wall_p = gaps_sides_points.back().first;
            ground_p = gaps_sides_points.back().second;
            max_dis = Distance3D(wall_p, ground_p);
            
            if(max_dis > gap_dis) {
                if (std::fabs(wall_p.x - ground_p.x) > 0.01) {
                    std::cout << "real gap" << std::endl;
                    key = 1;
                } else {
                    // 补偿gap处的点云数据
                    std::vector<Point3D> inter_points_in_laser;
                    inter_points_in_laser = InterpolatePoints(wall_p, ground_p, 0.001);
                    points_in_laser.insert(points_in_laser.end(), inter_points_in_laser.begin(), inter_points_in_laser.end());
                    key = 0;
                }
            } else {
                std::cout << "normal wall" << std::endl;
            }
        } else {
            key = 0;
        }
    } else {
        key = 0;
        return pair_out;
    }

    pair_out.first = key;
    pair_out.second = points_in_laser;

    return pair_out;
}

int main() {
#ifdef GROUND
    // std::string   filename = "/home/shan2/Mypro/create3_ws/src/edge_following/data/line_points_in_odom.txt";
    std::string   filename = "/home/shan2/Mypro/create3_ws/src/edge_following/data/line_points_in_laser.txt";
#endif
#ifdef GAP
    std::string   filename = "/home/shan2/Mypro/create3_ws/src/edge_following/data/line_points_in_laser.txt";
#endif 
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return -1;
    }

    std::vector<Frame> frames;
    Frame              current_frame;
    std::string        line;

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;

        if (line.substr(0, 3) == "---") {
            if (!current_frame.xyz_points.empty()) frames.push_back(current_frame);
            current_frame = Frame();
        } else if (line.substr(0, 9) == "timestamp") {
            current_frame.timestamp = line.substr(line.find(":") + 1);
        } else {
            std::istringstream iss(line);
            float              x, y, z;
            if (!(iss >> x >> y >> z)) continue;
#ifdef GROUND
            current_frame.xyz_points.emplace_back(x, y, z);
#endif
#ifdef GAP
            current_frame.xyz_points.emplace_back(x, y, z);
#endif
        }
    }
    if (!current_frame.xyz_points.empty()) frames.push_back(current_frame);
    ifs.close();

    int width = 1500, height = 1000;
    cv::namedWindow("Laser View", cv::WINDOW_NORMAL);

    float scale = 2000.0f;
    int   offset_x = width / 2;
    int   offset_y = height / 2;

    int idx = 0;  // 当前帧 index，默认第 0 帧

    while (true) {
        if (idx < 0) idx = 0;
        if (idx >= (int)frames.size()) idx = frames.size() - 1;
        auto&                frame = frames[idx];

        cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

// laser in odom find ground
#ifdef GROUND
        std::vector<Point3D> points_in_odom;
        // 绘制原始点（绿色）
        for (auto& p : frame.xyz_points) {
            points_in_odom.emplace_back(Point3D(p.x, p.y, p.z));
            // int img_x = (int)(p.x * scale + offset_x);
            // int img_y = (int)(height - (p.y * scale + offset_y));
            // if (img_x >= 0 && img_x < width && img_y >= 0 && img_y < height) cv::circle(img, cv::Point(img_x, img_y), 3, cv::Scalar(0, 255, 0), -1);
        }

        // 找地面
        Eigen::MatrixXf np_end_points(3, 2);
        // 首端点
        np_end_points(0, 0) = points_in_odom.front().x; // y
        np_end_points(1, 0) = points_in_odom.front().y; // z
        np_end_points(2, 0) = 0; // index
        // 末端点
        np_end_points(0, 1) = points_in_odom.back().x;
        np_end_points(1, 1) = points_in_odom.back().y;
        np_end_points(2, 1) = points_in_odom.size() - 1;
        
        Eigen::MatrixXf lines = IepfFunction(points_in_odom, np_end_points);
        std::cout << "-----" << std::endl;
        std::cout << "line size: " << lines.cols() << std::endl;
        int ground_point_index = -1;
        bool has_ground_points = false;
        float min_k = std::numeric_limits<float>::max();
        for (int i = 0; i < lines.cols() - 1; ++i) {
            float k = (lines(1, i + 1) - lines(1, i)) / (lines(0, i + 1) - lines(0, i));
            
            cv::Point p1((int)(lines(0, i) * scale + offset_x), (int)(height - (-lines(1, i) * scale + offset_y)));
            cv::Point p2((int)(lines(0, i+1) * scale + offset_x), (int)(height - (-lines(1, i+1) * scale + offset_y)));
            // 绘制检测到的直线（红色）
            cv::line(img, p1, p2, cv::Scalar(0, 0, 255), 2);

            std::cout << "k: " << k << std::endl;
            if (std::fabs(k) > 1) continue;
            if (std::fabs(k) < std::fabs(min_k)) {
                min_k = k;
                ground_point_index = i;
                has_ground_points = true;
            }
        }

        std::vector<Point3D> ground_points;
        if (has_ground_points) {
            ground_points.resize((int)(lines(2, ground_point_index + 1) - lines(2, ground_point_index) + 1));
            std::copy(points_in_odom.begin() + (int)(lines(2, ground_point_index)), points_in_odom.begin() + (int)(lines(2, ground_point_index + 1)) + 1,
                      ground_points.begin());
        }

        // std::cout << "ground_points: " << ground_points.size() << std::endl;

        // 非地面点
        std::vector<Point3D> diff = symmetricDifference(points_in_odom, ground_points);

        // 绘制地面（绿色）
        for (auto& p : ground_points) {
            int img_x = (int)(p.x * scale + offset_x);
            int img_y = (int)(height - (-p.y * scale + offset_y));
            cv::circle(img, cv::Point(img_x, img_y), 2, cv::Scalar(0, 255, 0), -1);
        }

        // 绘制其他点（黄色）
        for (auto& p : diff) {
            int img_x = (int)(p.x * scale + offset_x);
            int img_y = (int)(height - (-p.y * scale + offset_y));
            cv::circle(img, cv::Point(img_x, img_y), 2, cv::Scalar(0, 255, 255), -1);
        }

#endif

// laser in laser find gap
#ifdef GAP
        std::vector<Point3D> points_in_laser;
        // 绘制原始点（绿色）
        for (auto& p : frame.xyz_points) {
            points_in_laser.emplace_back(Point3D(p.x, p.y, p.z));
            // int img_x = (int)(p.y * scale + offset_x);
            // int img_y = (int)(height - (p.z * scale + offset_y));
            int img_x = (int)(p.x * scale + offset_x);
            int img_y = (int)(height - (p.y * scale + offset_y));
            if (img_x >= 0 && img_x < width && img_y >= 0 && img_y < height) cv::circle(img, cv::Point(img_x, img_y), 3, cv::Scalar(0, 255, 0), -1);
        }

        std::pair<int, std::vector<Point3D>> pair = FindGap(points_in_laser);
        std::cout << "pair first: " << pair.first << std::endl;
        if (pair.first == 0) {
            for (auto& p : pair.second) {
                int img_x = (int)(p.x * scale + offset_x);
                int img_y = (int)(height - (p.y * scale + offset_y));
                cv::circle(img, cv::Point(img_x, img_y), 3, cv::Scalar(255, 255, 255), -1);
            }
        } else if (pair.first == 1) {
            for (auto& p : pair.second) {
                int img_x = (int)(p.x * scale + offset_x);
                int img_y = (int)(height - (p.y * scale + offset_y));
                cv::circle(img, cv::Point(img_x, img_y), 3, cv::Scalar(0, 255, 255), -1);
            }
        }
#endif

        // 在图像上显示帧信息
        std::string text1 = "Frame: " + std::to_string(idx) + " / " + std::to_string(frames.size() - 1);
        std::string text2 = "Timestamp: " + frame.timestamp;
        cv::putText(img, text1, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        cv::putText(img, text2, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        cv::imshow("Laser View", img);

        // 键盘控制
        int key = cv::waitKey(0);

        if (key == 27)  // ESC
            break;
        else if (key == 'd' || key == 'D')  // 下一帧
            ++idx;
        else if (key == 'a' || key == 'A')  // 上一帧
            --idx;
        else if (key == ' ')
            ++idx;
    }

    cv::destroyAllWindows();
    return 0;
}
