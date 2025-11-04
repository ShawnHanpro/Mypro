#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

struct FrameData {
    long long timestamp;
    std::vector<cv::Point2f> points;
};

// 读取激光点文件
std::vector<FrameData> loadLaserFile(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open: " << filename << std::endl;
        return {};
    }

    std::vector<FrameData> frames;
    FrameData current;
    std::string line;

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;

        if (line.rfind("---", 0) == 0) {
            // 新帧开始
            if (!current.points.empty()) {
                frames.push_back(current);
                current = FrameData();
            }
        } 
        else if (line.rfind("timestamp:", 0) == 0) {
            std::stringstream ss(line.substr(10));
            ss >> current.timestamp;
        } 
        else {
            std::stringstream ss(line);
            float x, y;
            if (ss >> x >> y)
                current.points.emplace_back(x, y);
        }
    }

    if (!current.points.empty())
        frames.push_back(current);

    std::cout << "Loaded " << frames.size() << " frames." << std::endl;
    return frames;
}

int main() {
    std::string file_path = "/home/shan2/Mypro/create3_ws/src/play_point_laser_bag/data/point_laser.txt";
    auto frames = loadLaserFile(file_path);
    if (frames.empty()) return -1;

    int width = 1500, height = 800;
    cv::Mat ori_img(height, width, CV_8UC3);
    int idx = 0;

    const float scale = 100.0; // 缩放比例：1m = 100像素
    const cv::Point2f center(width/2.0f, height/2.0f);

    while (true) {
        ori_img.setTo(cv::Scalar(30, 30, 30));

        // 灰度图
        // cv::Mat gray;
        // cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        // 二值图
        cv::Mat binary_img = cv::Mat::zeros(height, width, CV_8UC1);

        // 创建 FastLineDetector 对象（EDLines）
        cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(
            10,      // length_threshold：最短直线段长度，单位像素
            1.414f,  // distance_threshold：拟合点之间最大距离
            50,      // canny_th1：Canny 边缘检测低阈值
            150,     // canny_th2：Canny 边缘检测高阈值
            3,       // canny_aperture_size：Sobel 核大小
            true     // do_merge：是否合并共线线段
        );

        // 绘制当前帧点
        for (const auto& p : frames[idx].points) {
            int px = (int)(500+center.x + p.x * scale);
            int py = (int)(2*center.y - p.y * scale); // y轴反向
            if (px >= 0 && px < width && py >= 0 && py < height) {
                cv::circle(ori_img, {px, py}, 2, {0,255,0}, -1);
                binary_img.at<uchar>(py, px) = 255;   // 在mask中打点
            }
        }

        std::vector<cv::Vec4i> lines;
        // 霍夫直线检测
        // 参数: rho=1 theta=1° 投票数=40 最短线长=30px 最大断点=10px
        cv::HoughLinesP(binary_img, lines, 1, CV_PI/180, 40, 30, 10);

        // edlines直线检测
        // fld->detect(binary_img, lines);

        // 绘制直线
        for (auto& l : lines) {
            cv::line(ori_img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                     cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }

        // 绘制坐标轴
        cv::line(ori_img, {0, (int)center.y}, {width, (int)center.y}, {80,80,80}, 1);
        cv::line(ori_img, {(int)center.x, 0}, {(int)center.x, height}, {80,80,80}, 1);

        // 显示时间戳
        cv::putText(ori_img, "timestamp: " + std::to_string(frames[idx].timestamp),
                    {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 1);

        // 显示帧索引
        cv::putText(ori_img, "Frame " + std::to_string(idx+1) + "/" + std::to_string(frames.size()),
                    {20, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 1);

        cv::imshow("Laser Playback", ori_img);

        int key = cv::waitKey(0);
        // std::cout << "key: " << key << std::endl;
        if (key == 27 || key == 'q') break;            // 退出
        else if (key == 44) { if (idx > 0) idx--; }    // ← 上一帧
        else if (key == 46) { if (idx < (int)frames.size()-1) idx++; } // → 下一帧
        else if (key == 32) { idx = (idx + 1) % frames.size(); }          // 自动循环
    }

    return 0;
}
