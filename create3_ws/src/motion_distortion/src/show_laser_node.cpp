#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

// 定义IMU数据结构
struct ImuData {
    uint64_t           timestamp_ns;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d    angular_velocity;
    Eigen::Vector3d    linear_acceleration;
};

// 定义Odometry数据结构
struct OdomData {
    uint64_t           timestamp_ns;
    Eigen::Vector3d    position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d    linear_velocity;
    Eigen::Vector3d    angular_velocity;
};

// 定义Laser帧数据结构
struct LaserFrame {
    uint64_t timestamp_ns;
    double   time_increment;  // Time increment between measurements
    double   angle_min;
    double   angle_max;
    double   angle_increment;
    double   range_min;
    double   range_max;
    int      points_count;
    std::vector<cv::Point2f>     points;         // For 2D display initially
    std::vector<Eigen::Vector3d> raw_points_3d;  // Raw 3D points
    std::vector<Eigen::Vector3d> corrected_points_3d;  // Corrected 3D points
};

// 解析imu数据
std::vector<ImuData> ParseImuData(const std::string& file_path) {
    std::vector<ImuData> imu_data_vec;
    std::ifstream        file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open IMU file " << file_path
                  << std::endl;
        return imu_data_vec;
    }

    std::string line;
    ImuData     current_imu;
    while (std::getline(file, line)) {
        if (line.rfind("---", 0) == 0) {          // New entry
            if (current_imu.timestamp_ns != 0) {  // Save previous if exists
                imu_data_vec.push_back(current_imu);
            }
            current_imu = {};  // Reset for new entry
        } else if (line.find("timestamp_ns:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_imu.timestamp_ns;
        } else if (line.find("orientation:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_imu.orientation.x() >>
                current_imu.orientation.y() >> current_imu.orientation.z() >>
                current_imu.orientation.w();
        } else if (line.find("angular_velocity:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_imu.angular_velocity.x() >>
                current_imu.angular_velocity.y() >>
                current_imu.angular_velocity.z();
        } else if (line.find("linear_acceleration:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_imu.linear_acceleration.x() >>
                current_imu.linear_acceleration.y() >>
                current_imu.linear_acceleration.z();
        }
    }
    // Add the last entry
    if (current_imu.timestamp_ns != 0) {
        imu_data_vec.push_back(current_imu);
    }
    file.close();
    return imu_data_vec;
}

// 解析odom数据
std::vector<OdomData> ParseOdomData(const std::string& file_path) {
    std::vector<OdomData> odom_data_vec;
    std::ifstream         file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open Odometry file " << file_path
                  << std::endl;
        return odom_data_vec;
    }

    std::string line;
    OdomData    current_odom;
    while (std::getline(file, line)) {
        if (line.rfind("---", 0) == 0) {           // New entry
            if (current_odom.timestamp_ns != 0) {  // Save previous if exists
                odom_data_vec.push_back(current_odom);
            }
            current_odom = {};  // Reset for new entry
        } else if (line.find("timestamp_ns:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_odom.timestamp_ns;
        } else if (line.find("position:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_odom.position.x() >>
                current_odom.position.y() >> current_odom.position.z();
        } else if (line.find("orientation:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_odom.orientation.x() >>
                current_odom.orientation.y() >> current_odom.orientation.z() >>
                current_odom.orientation.w();
        } else if (line.find("linear_velocity:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_odom.linear_velocity.x() >>
                current_odom.linear_velocity.y() >>
                current_odom.linear_velocity.z();
        } else if (line.find("angular_velocity:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_odom.angular_velocity.x() >>
                current_odom.angular_velocity.y() >>
                current_odom.angular_velocity.z();
        }
    }
    // Add the last entry
    if (current_odom.timestamp_ns != 0) {
        odom_data_vec.push_back(current_odom);
    }
    file.close();
    return odom_data_vec;
}

// 解析laser数据
std::vector<LaserFrame> ParseLaserData(const std::string& file_path) {
    std::vector<LaserFrame> all_frames;
    std::ifstream           file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open Laser file " << file_path
                  << std::endl;
        return all_frames;
    }

    std::string line;
    LaserFrame  current_frame;
    bool        in_points_section = false;

    while (std::getline(file, line)) {
        if (line.rfind("---", 0) == 0) {
            if (current_frame.timestamp_ns != 0) {
                all_frames.push_back(current_frame);
            }
            current_frame = {};         
            in_points_section = false;  
        } else if (line.find("timestamp_ns:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_frame.timestamp_ns;
        } else if (line.find("time_increment:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_frame.time_increment;
        } else if (line.find("angle_min:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_frame.angle_min;
        } else if (line.find("angle_max:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_frame.angle_max;
        } else if (line.find("angle_increment:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_frame.angle_increment;
        } else if (line.find("range_min:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_frame.range_min;
        } else if (line.find("range_max:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_frame.range_max;
        } else if (line.find("points_count:") != std::string::npos) {
            std::stringstream ss(line);
            std::string       key;
            ss >> key >> current_frame.points_count;
        } else if (line.find("points: # format [x, y, z]") !=
                   std::string::npos) {
            in_points_section = true;
        } else if (in_points_section) {
            std::stringstream ss(line);
            float             x, y, z;
            if (ss >> x >> y >> z) {
                current_frame.raw_points_3d.push_back(Eigen::Vector3d(x, y, z));
                current_frame.points.push_back(
                    cv::Point2f(x, y));
            } else {
                in_points_section = false;
            }
        }
    }
    
    if (current_frame.timestamp_ns != 0) {
        all_frames.push_back(current_frame);
    }
    file.close();
    return all_frames;
}

// 查找最近时间戳
template <typename T>
size_t FindClosestTimestampIndex(uint64_t              target_timestamp,
                                 const std::vector<T>& data_vec) {
    if (data_vec.empty()) {
        return -1;
    }

    auto it = std::lower_bound(
        data_vec.begin(), data_vec.end(), target_timestamp,
        [](const T& data, uint64_t ts) { return data.timestamp_ns < ts; });

    if (it == data_vec.end()) {
        return data_vec.size() - 1;
    }
    if (it == data_vec.begin()) {
        return 0;
    }

    size_t idx = std::distance(data_vec.begin(), it);
    // Compare with the previous element to find the closest
    if ((target_timestamp - (it - 1)->timestamp_ns) <
        (it->timestamp_ns - target_timestamp)) {
        return idx - 1;
    } else {
        return idx;
    }
}

// Slerp for Quaternion (for orientation interpolation)
Eigen::Quaterniond slerp(const Eigen::Quaterniond& q1,
                         const Eigen::Quaterniond& q2, double t) {
    return q1.slerp(t, q2);
}

// Linear interpolation for Vector3d (for position/velocity)
Eigen::Vector3d lerp(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
                     double t) {
    return v1 + t * (v2 - v1);
}

// 插值imu数据
ImuData interpolateImu(uint64_t                    target_timestamp,
                       const std::vector<ImuData>& imu_data_vec) {
    size_t idx = FindClosestTimestampIndex(target_timestamp, imu_data_vec);

    if (idx == -1) {  // No IMU data
        return {};
    }

    if (imu_data_vec.size() == 1 ||
        imu_data_vec[idx].timestamp_ns == target_timestamp) {
        return imu_data_vec[idx];
    }

    ImuData imu1 = imu_data_vec[idx];
    ImuData imu2;

    if (imu1.timestamp_ns < target_timestamp && idx + 1 < imu_data_vec.size()) {
        imu2 = imu_data_vec[idx + 1];
    } else if (imu1.timestamp_ns > target_timestamp && idx > 0) {
        imu2 = imu_data_vec[idx - 1];
        imu1 = imu_data_vec[idx];  // Swap to ensure imu1 is earlier
    } else {
        return imu_data_vec[idx];  // Should not happen if idx is valid and not
                                   // exact match
    }

    if (imu1.timestamp_ns == imu2.timestamp_ns) {  // Avoid division by zero
        return imu1;
    }

    double t = static_cast<double>(target_timestamp - imu1.timestamp_ns) /
               (imu2.timestamp_ns - imu1.timestamp_ns);

    ImuData interpolated_imu;
    interpolated_imu.timestamp_ns = target_timestamp;
    interpolated_imu.orientation = slerp(imu1.orientation, imu2.orientation, t);
    interpolated_imu.angular_velocity =
        lerp(imu1.angular_velocity, imu2.angular_velocity, t);
    interpolated_imu.linear_acceleration =
        lerp(imu1.linear_acceleration, imu2.linear_acceleration, t);

    return interpolated_imu;
}

// 插值odom数据
OdomData InterpolateOdom(uint64_t                     target_timestamp,
                         const std::vector<OdomData>& odom_data_vec) {
    size_t idx = FindClosestTimestampIndex(target_timestamp, odom_data_vec);

    if (idx == -1) {
        return {};
    }

    if (odom_data_vec.size() == 1 ||
        odom_data_vec[idx].timestamp_ns == target_timestamp) {
        return odom_data_vec[idx];
    }

    OdomData odom1 = odom_data_vec[idx];
    OdomData odom2;

    if (odom1.timestamp_ns < target_timestamp &&
        idx + 1 < odom_data_vec.size()) {
        odom2 = odom_data_vec[idx + 1];
    } else if (odom1.timestamp_ns > target_timestamp && idx > 0) {
        odom2 = odom_data_vec[idx - 1];
        odom1 = odom_data_vec[idx];  // Swap to ensure odom1 is earlier
    } else {
        return odom_data_vec[idx];  // Should not happen if idx is valid and not
                                    // exact match
    }

    if (odom1.timestamp_ns == odom2.timestamp_ns) {  // Avoid division by zero
        return odom1;
    }

    double t = static_cast<double>(target_timestamp - odom1.timestamp_ns) /
               (odom2.timestamp_ns - odom1.timestamp_ns);

    OdomData interpolated_odom;
    interpolated_odom.timestamp_ns = target_timestamp;
    interpolated_odom.position = lerp(odom1.position, odom2.position, t);
    interpolated_odom.orientation =
        slerp(odom1.orientation, odom2.orientation, t);
    interpolated_odom.linear_velocity =
        lerp(odom1.linear_velocity, odom2.linear_velocity, t);
    interpolated_odom.angular_velocity =
        lerp(odom1.angular_velocity, odom2.angular_velocity, t);

    return interpolated_odom;
}

// 插值去畸变
void UndistortLaserFrame(LaserFrame&                  laser_frame,
                         const std::vector<OdomData>& odom_data_vec) {
    if (laser_frame.raw_points_3d.empty()) {
        return;
    }

    OdomData odom_start =
        InterpolateOdom(laser_frame.timestamp_ns, odom_data_vec);
    if (odom_start.timestamp_ns == 0) {
        std::cerr << "Warning: Could not interpolate Odom for laser frame "
                  << laser_frame.timestamp_ns << std::endl;
        laser_frame.corrected_points_3d =
            laser_frame.raw_points_3d;
        return;
    }

    // Create transformation from world to start pose
    Eigen::Matrix3d R_start = odom_start.orientation.toRotationMatrix();
    Eigen::Vector3d t_start = odom_start.position;
    Eigen::Affine3d T_world_to_robot_start = Eigen::Affine3d::Identity();
    T_world_to_robot_start.translate(t_start);
    T_world_to_robot_start.rotate(R_start);
    Eigen::Affine3d T_robot_start_to_world = T_world_to_robot_start.inverse();

    laser_frame.corrected_points_3d.clear();
    laser_frame.corrected_points_3d.reserve(laser_frame.raw_points_3d.size());

    // 对于不同频率的激光需要使用time_increment变量来计算采集一帧数据的时间
    double scan_duration_ns = 166.666667 * 1000 * 1000;
    // if (laser_frame.time_increment > 0) {
    //     scan_duration_ns = static_cast<double>(laser_frame.points_count) *
    //                        laser_frame.time_increment * 1e9;
    // }

    double time_per_point_ns =
        scan_duration_ns / laser_frame.raw_points_3d.size();

    for (size_t i = 0; i < laser_frame.raw_points_3d.size(); ++i) {
        Eigen::Vector3d raw_point = laser_frame.raw_points_3d[i];

        // Estimate timestamp for this point
        uint64_t point_timestamp_ns =
            laser_frame.timestamp_ns +
            static_cast<uint64_t>(i * time_per_point_ns);

        // Interpolate Odometry at the point's timestamp
        OdomData odom_point_pose =
            InterpolateOdom(point_timestamp_ns, odom_data_vec);

        // If interpolation failed for some reason, use the start pose or skip
        if (odom_point_pose.timestamp_ns == 0) {
            laser_frame.corrected_points_3d.push_back(raw_point);
            continue;
        }

        // Transformation from laser_start_pose to point_pose_in_world
        Eigen::Matrix3d R_point =
            odom_point_pose.orientation.toRotationMatrix();
        Eigen::Vector3d t_point = odom_point_pose.position;
        Eigen::Affine3d T_world_to_point = Eigen::Affine3d::Identity();
        T_world_to_point.translate(t_point);
        T_world_to_point.rotate(R_point);

        Eigen::Affine3d T_robot_start_to_point = T_robot_start_to_world * T_world_to_point;

        // Apply the inverse transform to correct the point
        Eigen::Vector3d corrected_point =
            T_robot_start_to_point.inverse() * raw_point;
        laser_frame.corrected_points_3d.push_back(corrected_point);
    }

    // Update the 2D points for display
    laser_frame.points.clear();
    for (const auto& p_3d : laser_frame.corrected_points_3d) {
        laser_frame.points.push_back(cv::Point2f(p_3d.x(), p_3d.y()));
    }
}

void DisplayLaserFrame(const std::vector<cv::Point2f>& points, int frame_index,
                       cv::Mat& image, int img_width, int img_height,
                       bool corrected = false) {
    image = cv::Mat::zeros(img_height, img_width, CV_8UC3);

    if (points.empty()) {
        std::cout << "帧 " << frame_index
                  << (corrected ? " (校正后)" : " (原始)")
                  << " 没有点数据，跳过显示。" << std::endl;
        return;
    }

    // 找到X和Y的最小值和最大值，用于缩放(自适应缩放)
    float min_x = points[0].x, max_x = points[0].x;
    float min_y = points[0].y, max_y = points[0].y;

    for (const auto& p : points) {
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
    }

    // 计算缩放因子和偏移量
    float padding_ratio = 0.1f;
    float range_x = max_x - min_x;
    float range_y = max_y - min_y;

    if (range_x == 0) range_x = 0.001f;
    if (range_y == 0) range_y = 0.001f;

    float padded_min_x = min_x - range_x * padding_ratio;
    float padded_max_x = max_x + range_x * padding_ratio;
    float padded_min_y = min_y - range_y * padding_ratio;
    float padded_max_y = max_y + range_y * padding_ratio;

    float scale_x = img_width / (padded_max_x - padded_min_x);
    float scale_y = img_height / (padded_max_y - padded_min_y);

    float scale = std::min(scale_x, scale_y);

    float center_data_x = (padded_min_x + padded_max_x) / 2.0f;
    float center_data_y = (padded_min_y + padded_max_y) / 2.0f;

    float center_img_x = img_width / 2.0f;
    float center_img_y = img_height / 2.0f;

    float offset_x = center_img_x - center_data_x * scale;
    float offset_y = center_img_y - center_data_y * scale;

    // 绘制点
    for (const auto& p : points) {
        int display_x = static_cast<int>(p.x * scale + offset_x);
        int display_y = static_cast<int>(
            p.y * scale + offset_y);  // OpenCV Y-axis is downwards

        if (display_x >= 0 && display_x < img_width && display_y >= 0 &&
            display_y < img_height) {
            cv::circle(image, cv::Point(display_x, display_y), 1,
                       cv::Scalar(0, 255, 0), -1);
        }
    }

    std::string window_name = corrected ? "Laser (Corrected)" : "Laser (Raw)";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::imshow(window_name, image);
}

void DisplayBothLaserFrame(const std::vector<cv::Point2f>& raw_points,
                           const std::vector<cv::Point2f>& corrected_points,
                           int frame_index, cv::Mat& both_image, int img_width,
                           int img_height) {

    both_image = cv::Mat::zeros(img_height, img_width, CV_8UC3);

    if (corrected_points.empty()) {
        std::cout << "帧 " << frame_index
                  << " 没有点数据，跳过显示。" << std::endl;
        return;
    }

    float scale = 100;

    float offset_x = img_width / 2.0f;
    float offset_y = img_height / 2.0f;

    // 绘制点
    for (const auto& r_p : raw_points) {
        int display_x = static_cast<int>(r_p.x * scale + offset_x);
        int display_y = static_cast<int>(r_p.y * scale + offset_y);

        if (display_x >= 0 && display_x < img_width && display_y >= 0 &&
            display_y < img_height) {
            cv::circle(both_image, cv::Point(display_x, display_y), 1,
                       cv::Scalar(0, 0, 255), -1);
        }
    }
    for (const auto& c_p : corrected_points) {
        int display_x = static_cast<int>(c_p.x * scale + offset_x);
        int display_y = static_cast<int>(c_p.y * scale + offset_y);

        if (display_x >= 0 && display_x < img_width && display_y >= 0 &&
            display_y < img_height) {
            cv::circle(both_image, cv::Point(display_x, display_y), 1,
                       cv::Scalar(0, 255, 0), -1);
        }
    }

    cv::namedWindow("both", cv::WINDOW_NORMAL);
    cv::imshow("both", both_image);
}

int main(int argc, char* argv[]) {
    // 检查命令行参数数量
    if (argc < 4) {
        std::cerr
            << "用法: " << argv[0]
            << " <激光数据文件路径> <IMU数据文件路径> <Odometry数据文件路径>"
            << std::endl;
        return -1;
    }

    const std::string laser_file_path = argv[1];
    const std::string odom_file_path = argv[2];
    const std::string imu_file_path = argv[3];

    // 解析数据
    std::vector<LaserFrame> all_laser_frames = ParseLaserData(laser_file_path);
    std::vector<OdomData>   all_odom_data = ParseOdomData(odom_file_path);
    // std::vector<ImuData>    all_imu_data = ParseImuData(imu_file_path);

    if (all_laser_frames.empty()) {
        std::cerr << "错误：未从文件 " << laser_file_path
                  << " 中找到任何有效的激光数据帧。" << std::endl;
        return -1;
    }
    if (all_odom_data.empty()) {
        std::cerr << "错误：未从文件 " << odom_file_path
        << " 中找到任何有效的Odometry数据。" << std::endl;
        return -1;
    }
    // if (all_imu_data.empty()) {
    //     std::cerr << "错误：未从文件 " << imu_file_path
    //               << " 中找到任何有效的IMU数据。" << std::endl;
    //     return -1;
    // }

    const int img_width = 1000;
    const int img_height = 1000;
    cv::Mat   raw_image = cv::Mat::zeros(img_height, img_width, CV_8UC3);
    cv::Mat   corrected_image = cv::Mat::zeros(img_height, img_width, CV_8UC3);
    cv::Mat   both_image = cv::Mat::zeros(img_height, img_width, CV_8UC3);

    int frame_count = 0;
    for (auto& frame : all_laser_frames) {
        frame_count++;
        std::cout << "正在处理帧: " << frame_count
                  << " (时间戳: " << frame.timestamp_ns << ")" << std::endl;

        // Display raw frame
        // DisplayLaserFrame(frame.points, frame_count, raw_image, img_width,
        //                   img_height, false);

        std::vector<cv::Point2f> raw_points = frame.points;
        // Perform undistortion
        UndistortLaserFrame(frame, all_odom_data);
        std::vector<cv::Point2f> corrected_points = frame.points;

        // Display corrected frame
        // DisplayLaserFrame(frame.points, frame_count, corrected_image, img_width,
        //                   img_height, true);

        // Show both images
        DisplayBothLaserFrame(raw_points, corrected_points,
                              frame_count, both_image, img_width, img_height);

        int key = cv::waitKey(0);

        if (key == 27) {  // ESC key
            std::cout << "检测到 ESC 键，停止显示。" << std::endl;
            break;
        }
    }

    cv::destroyAllWindows();

    return 0;
}