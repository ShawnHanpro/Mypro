#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>  // std::this_thread::sleep_for
#include <vector>

#include <Eigen/Dense>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <random>
#include <sstream>

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

struct Frame {
    std::string          timestamp;
    std::vector<Point3D> xyz_points;
};

struct Color {
    static constexpr const char* RED = "\033[0;31m";
    static constexpr const char* GREEN = "\033[0;32m";
    static constexpr const char* YELLOW = "\033[0;33m";
    static constexpr const char* BLUE = "\033[0;34m";
    static constexpr const char* MAGENTA = "\033[0;35m";
    static constexpr const char* CYAN = "\033[0;36m";
    static constexpr const char* WHITE = "\033[0;37m";

    static constexpr const char* RED_BOLD = "\033[1;31m";
    static constexpr const char* GREEN_BOLD = "\033[1;32m";
    static constexpr const char* YELLOW_BOLD = "\033[1;33m";
    static constexpr const char* BLUE_BOLD = "\033[1;34m";
    static constexpr const char* MAGENTA_BOLD = "\033[1;35m";
    static constexpr const char* CYAN_BOLD = "\033[1;36m";
    static constexpr const char* WHITE_BOLD = "\033[1;37m";

    static constexpr const char* RESET = "\033[0m";
};

void LoadParam(std::string param_dir, std::unordered_map<std::string, std::string>& param) {
    std::ifstream infile(param_dir);
    if (!infile.is_open()) {
        std::cerr << "Failed to open config.txt" << std::endl;
        return;
    }

    std::string key, value;
    while (infile >> key >> value) {
        param[key] = value;
    }

    infile.close();
}

void ReadTxtToLaserPoints3D(std::string filename, std::vector<Frame>& frames) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return;
    }

    Frame       current_frame;
    std::string line;

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
            current_frame.xyz_points.emplace_back(x, y, z);
        }
    }
    if (!current_frame.xyz_points.empty()) frames.push_back(current_frame);
    ifs.close();
}

#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

void AddXYPlane(pcl::visualization::PCLVisualizer::Ptr viewer, float z_value, float xmin, float xmax, float ymin, float ymax,
                const std::string& plane_id, double r, double g, double b, double opacity) {
    // 定义 4 个顶点 —— z 全部相同
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    plane_cloud->push_back(pcl::PointXYZ(xmin, ymin, z_value));
    plane_cloud->push_back(pcl::PointXYZ(xmax, ymin, z_value));
    plane_cloud->push_back(pcl::PointXYZ(xmax, ymax, z_value));
    plane_cloud->push_back(pcl::PointXYZ(xmin, ymax, z_value));

    // 转成 mesh
    vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
    for (auto& p : plane_cloud->points) vtk_points->InsertNextPoint(p.x, p.y, p.z);

    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolygon>   polygon = vtkSmartPointer<vtkPolygon>::New();
    polygon->GetPointIds()->SetNumberOfIds(4);

    polygon->GetPointIds()->SetId(0, 0);
    polygon->GetPointIds()->SetId(1, 1);
    polygon->GetPointIds()->SetId(2, 2);
    polygon->GetPointIds()->SetId(3, 3);

    polygons->InsertNextCell(polygon);

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(vtk_points);
    polydata->SetPolys(polygons);

    // 添加到 viewer
    viewer->addModelFromPolyData(polydata, plane_id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, plane_id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, plane_id);
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
    // std::cout << "break_point: " << break_point.z() << std::endl;
    // 复制插入点之后的值
    if ((old_cols - insert_pos) > 0) {
        new_np_end_points.block(0, insert_pos + 1, 3, old_cols - insert_pos) = np_end_points.block(0, insert_pos, 3, old_cols - insert_pos);
    }

    np_end_points = new_np_end_points;

    np_end_points = IepfFunction(points, np_end_points, dis_threshold);

    return np_end_points;
}

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

template <typename T>
T StringToValue(const std::string& s) {
    std::stringstream ss(s);
    T                 value;
    ss >> value;
    return value;
}

// bool类型特殊处理
template <>
bool StringToValue<bool>(const std::string& s) {
    return (s == "true");
}

template <typename T>
T GetParam(const std::unordered_map<std::string, std::string>& param, const std::string& key) {
    auto it = param.find(key);
    if (it == param.end()) {
        std::cerr << Color::RED_BOLD << "Config error: missing key: " << key << Color::RESET << std::endl;
        return T();
    }

    T value = StringToValue<T>(it->second);

    if constexpr (std::is_same<T, bool>::value) {
        std::cout << key << ": " << (value ? Color::GREEN_BOLD : Color::RED_BOLD) << (value ? "true" : "false") << Color::RESET << std::endl;
    } else {
        std::cout << key << ": " << Color::YELLOW_BOLD << value << Color::RESET << std::endl;
    }

    return value;
}

bool next_plane = false;

// 键盘回调函数
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event) {
    if (event.getKeySym() == "space" && event.keyDown()) {
        next_plane = true;
    }
}

// Bresenham 生成从 (0,0) 到 (x1,y1) 的离散像素点
std::vector<std::pair<int,int>> bresenham(int x1, int y1) {
    std::vector<std::pair<int,int>> pts;
    int x0 = 0, y0 = 0;

    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        pts.push_back({x0, y0});
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    return pts;
}

// 查找 360° 最近点
std::vector<Point3D> findClosest360(const std::vector<Point3D>& cloud) {
    std::vector<Point3D> closest(360);
    std::vector<float>   bestDist(360, std::numeric_limits<float>::max());

    const float RAY_LENGTH = 5.0f;  // 最大搜索半径
    const float TOL = 0.03f;        // 点落在射线上 ±3cm 范围

    for (int angle = 0; angle < 360; angle++) {
        float rad = angle * M_PI / 180.0f;
        int   rx = std::round(RAY_LENGTH * std::cos(rad) * 100);  // 转为整数像素
        int   ry = std::round(RAY_LENGTH * std::sin(rad) * 100);

        auto ray = bresenham(rx, ry);

        for (const auto& pt : cloud) {
            float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);

            for (auto& pix : ray) {
                float fx = pix.first / 100.0f;
                float fy = pix.second / 100.0f;

                float d = std::hypot(pt.x - fx, pt.y - fy);

                if (d < TOL && dist < bestDist[angle]) {
                    bestDist[angle] = dist;
                    closest[angle] = pt;
                }
            }
        }
    }
    return closest;
}

std::vector<Point3D> findClosest_halfDeg(const std::vector<Point3D>& cloud) {
    const int   bins = 720;       // 720 桶 → 0.5° 分辨率
    const float bin_size = 0.5f;  // 每个桶的角度宽度
    const float inv_bin = 1.0f / bin_size;

    std::vector<Point3D> closest(bins);
    std::vector<float>   bestDist(bins, std::numeric_limits<float>::max());

    for (const auto& pt : cloud) {
        float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        if (r < 0.0001f) continue;

        float angle = std::atan2(pt.y, pt.x);  // [-pi, pi]

        // 角度 → 桶编号
        // (angle + π) 转换为 [0, 2π]
        float deg = (angle + M_PI) * 180.0f / M_PI;  // [0, 360)
        int   bin = (int)std::floor(deg * inv_bin);  // 0.5° resolution

        if (bin < 0) bin = 0;
        if (bin >= bins) bin = bins - 1;

        if (r < bestDist[bin]) {
            bestDist[bin] = r;
            closest[bin] = pt;
        }
    }

    return closest;
}

int main() {
    // 加载参数
    std::unordered_map<std::string, std::string> param;
    LoadParam("/home/shan2/Mypro/create3_ws/src/pcl_simulation/yaml/config.txt", param);

    bool plane_show_in_order = GetParam<bool>(param, "plane_show_in_order");
    bool plane_show_one_in_order = GetParam<bool>(param, "plane_show_one_in_order");
    bool cloud_show_one_in_order = GetParam<bool>(param, "cloud_show_one_in_order");
    bool show_planes = GetParam<bool>(param, "show_planes");
    bool show_xyplane = GetParam<bool>(param, "show_xyplane");
    bool z_pass = GetParam<bool>(param, "z_pass");
    bool KDTree_ece = GetParam<bool>(param, "KDTree_ece");
    bool show_cloud = GetParam<bool>(param, "show_cloud");
    bool show_all_cloud_in_white = GetParam<bool>(param, "show_all_cloud_in_white");
    bool detect_the_ground = GetParam<bool>(param, "detect_the_ground");

    float z_plane_dis = GetParam<float>(param, "z_plane_dis");
    float z_plane_opacity = GetParam<float>(param, "z_plane_opacity");
    float ece_dis = GetParam<float>(param, "ece_dis");

    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZ>);  // 计算使用

    std::vector<Frame> odom_points_frames;
    ReadTxtToLaserPoints3D("/home/shan2/Mypro/create3_ws/src/edge_following/data/line_points_in_odom.txt", odom_points_frames);
    std::vector<Frame> laser_points_frames;
    ReadTxtToLaserPoints3D("/home/shan2/Mypro/create3_ws/src/edge_following/data/line_points_in_laser.txt", laser_points_frames);

    // 
    std::vector<Point3D> points_2d;


    for (size_t i = 0; i < odom_points_frames.size(); ++i) {

        if (detect_the_ground) {
            // 分割地面点
            Eigen::MatrixXf np_end_points(3, 2);
            // 首端点
            np_end_points(0, 0) = laser_points_frames[i].xyz_points.front().x;
            np_end_points(1, 0) = laser_points_frames[i].xyz_points.front().y;
            np_end_points(2, 0) = 0; // index
            // 末端点
            np_end_points(0, 1) = laser_points_frames[i].xyz_points.back().x;
            np_end_points(1, 1) = laser_points_frames[i].xyz_points.back().y;
            np_end_points(2, 1) = laser_points_frames[i].xyz_points.size() - 1;
            
            Eigen::MatrixXf lines = IepfFunction(laser_points_frames[i].xyz_points, np_end_points);
    
            // 解析地面点
            int ground_point_index = -1;
            bool has_ground_points = false;
            float min_k = std::numeric_limits<float>::max();
            for (int i = 0; i < lines.cols() - 1; ++i) {
                float k = (lines(1, i + 1) - lines(1, i)) / (lines(0, i + 1) - lines(0, i));
    
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
                std::copy(odom_points_frames[i].xyz_points.begin() + (int)(lines(2, ground_point_index)), odom_points_frames[i].xyz_points.begin() + (int)(lines(2, ground_point_index + 1)) + 1,
                        ground_points.begin());
            }
    
            std::vector<Point3D> wall_points = symmetricDifference(odom_points_frames[i].xyz_points, ground_points);
    
    
            for (auto& p : wall_points) {
                // pcl_points->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
                pcl_points->points.push_back(pcl::PointXYZ(p.x, p.y, 0.0));
                // points_2d.push_back(Point3D(p.x, p.y, 0.0));
            }
        } else {
            for (auto& p : odom_points_frames[i].xyz_points) {
                pcl_points->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
            }
        }
    }

    pcl_points->width = pcl_points->points.size();
    pcl_points->height = 1;  // 非组织化点云
    pcl_points->is_dense = true;

    // 初始化平面分割模型
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ>  extract;  // 用于从原始点云中提取某些索引对应的点

    seg.setOptimizeCoefficients(true);  // 设置是否优化平面系数，true为RANSAC找到平面后会对模型进行最小二乘优化得到更精确的平面系数
    seg.setModelType(pcl::SACMODEL_PLANE);  // 指定要拟合的模型类型 plane为平面
    seg.setMethodType(pcl::SAC_RANSAC);     // 设置分割方法 这里为ransac
    seg.setDistanceThreshold(0.005);        // 平面距离阈值

    // 初始化显示窗口
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.2);  // 可视化坐标系

    
    // show all points in white
    if (show_all_cloud_in_white) {
        // std::vector<Point3D> close_points_2d =  findClosest_halfDeg(points_2d);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr show_close_points(new pcl::PointCloud<pcl::PointXYZ>);
        // for (auto& cp : close_points_2d) {
        //     show_close_points->points.push_back(pcl::PointXYZ(cp.x, cp.y, cp.z));
        // }
        // show_close_points->width = show_close_points->points.size();
        // show_close_points->height = 1;  // 非组织化点云
        // show_close_points->is_dense = true;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  white_color(pcl_points, 255, 255, 255);  // R G B
        // 设置点云大小
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 3, "cloud_white");
        viewer->addPointCloud<pcl::PointXYZ>(pcl_points, white_color, "cloud_white");
    }

    if (show_xyplane)
        // z轴上平行于xy的平面
        AddXYPlane(viewer,
                   z_plane_dis,       // z = 1.0 的平面
                   -1.0, 1.0,         // xmin xmax
                   -1.0, 1.0,         // ymin ymax
                   "xy_plane",        // 平面名字
                   0.2, 0.8, 0.2,     // green color
                   z_plane_opacity);  // 透明度

    // 注册键盘按键回调函数
    if (plane_show_in_order || plane_show_one_in_order) viewer->registerKeyboardCallback(keyboardEventOccurred);

    // 平面计数
    int         plane_id = 0;
    std::string cloud_name, plane_name;

    while (pcl_points->size() > 50) {
        pcl::PointIndices::Ptr      inliers(new pcl::PointIndices);            // 存储平面内点的索引
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);  // 存储平面模型系数[a, b, c, d]

        seg.setInputCloud(pcl_points);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cout << "No planer surface found." << std::endl;
            break;
        }

        // 提取平面点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_plane_points(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(pcl_points);
        extract.setIndices(inliers);
        extract.setNegative(false);  // 设置提取模式 false为提取平面点 true为提取平面以外的点
        extract.filter(*pcl_plane_points);

        // kdtree欧式聚类
        if (KDTree_ece) {
            // 聚类判断是否为连通点云
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(pcl_plane_points);  // 构建kdtree索引

            std::vector<pcl::PointIndices> clusters;  // 保存聚类的结果 每个pcl::PointIndices是一个索引几何，表示一个簇
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  // 欧式聚类算法对象
            ec.setClusterTolerance(ece_dis);                    // 设置两个点被认为是一个簇的最大距离
            ec.setMinClusterSize(100);                          // 设置簇的最小点数，少的点会被认为是噪声
            ec.setMaxClusterSize(50000);                        // 设置簇的最大点数
            ec.setSearchMethod(tree);                           // 设置查找类型
            ec.setInputCloud(pcl_plane_points);
            ec.extract(clusters);

            // 从clusters找最多点云的簇作为聚类结果
            int    max_cluster_index = -1;
            size_t max_cluster_size = 0;
            for (size_t i = 0; i < clusters.size(); ++i) {
                size_t cluster_size = clusters[i].indices.size();
                if (cluster_size > max_cluster_size) {
                    max_cluster_size = cluster_size;
                    max_cluster_index = i;
                }
            }

            // 最大的点云簇索引
            pcl::PointIndices::Ptr local_indices(new pcl::PointIndices); // 注意这个是局部点云的索引不能与全局放在一起计算！！！
            local_indices->indices = clusters[max_cluster_index].indices;
            
            // 将pcl_plane_points最大簇索引映射回pcl_points
            pcl::PointIndices::Ptr global_indices(new pcl::PointIndices);
            for (int local_idx : local_indices->indices) {
                global_indices->indices.push_back(inliers->indices[local_idx]);
            }

            // 再次提取点云
            pcl::ExtractIndices<pcl::PointXYZ> extract2;
            extract2.setInputCloud(pcl_points); // 局部点云
            extract2.setIndices(global_indices);    // 最大簇的原始索引
            // 提取显示点云
            extract2.setNegative(false);  // 设置提取模式 false为提取平面点 true为提取平面以外的点
            pcl_plane_points->clear();    // 清除数据
            extract2.filter(*pcl_plane_points);

            // 提取剩余点云
            extract2.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            extract2.filter(*cloud_filtered);
            pcl_points = cloud_filtered;
        } else {
            // 提取剩余点云
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*cloud_filtered);
            pcl_points = cloud_filtered;
        }

        // 过滤点云数量较少的平面
        if (pcl_plane_points->points.size() < 900) continue;

        std::cout << "Plane detected: " << inliers->indices.size() << " inliers, coefficients: [" << coefficients->values[0] << ", "
                  << coefficients->values[1] << ", " << coefficients->values[2] << ", " << coefficients->values[3] << std::endl;

        // 删除上一个平面
        if (plane_show_one_in_order) {
            if (!plane_name.empty()) {
                viewer->removeShape(plane_name);
                if (cloud_show_one_in_order) viewer->removePointCloud(cloud_name);
            }
        }

        // 随机颜色
        int r = rand() % 255;
        int g = rand() % 255;
        int b = rand() % 255;

        // 点云名
        cloud_name = "cloud_" + std::to_string(plane_id);
        // 平面名
        plane_name = "plane_" + std::to_string(plane_id);

        if (z_pass) {
            // filter大于z_dis的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered_plane_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ>     pass;
            pass.setInputCloud(pcl_plane_points);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(-1000.0, z_plane_dis);  // 保留 z <= 0.2
            pass.filter(*pcl_filtered_plane_points);
            // 设置颜色
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcl_filtered_plane_points, r, g, b);

            // 显示点云
            viewer->addPointCloud<pcl::PointXYZ>(pcl_filtered_plane_points, color, cloud_name);
        } else {
            // 设置颜色
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pcl_plane_points, r, g, b);

            // 显示点云
            if(show_cloud) viewer->addPointCloud<pcl::PointXYZ>(pcl_plane_points, color, cloud_name);
        }

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);  // 设置点云渲染属性（此处为大小）
        if (plane_id == 0) viewer->initCameraParameters();

        if (show_planes) {
            viewer->addPlane(*coefficients, plane_name);
            // 设置平面颜色：红色
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r / 255.0f, g / 255.0f, b / 255.0f, plane_name);
            // 设置平面透明度 alpha=0.5（50%透明）
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, plane_name);
        }

        // 等待按空格显示下一个平面
        if (plane_show_in_order || plane_show_one_in_order) {
            next_plane = false;
            while (!next_plane && !viewer->wasStopped()) {
                viewer->spinOnce(50);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        ++plane_id;
    }

    // 循环显示
    while (!viewer->wasStopped()) {
        viewer->spinOnce(10);
    }

    return 0;
}