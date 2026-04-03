#pragma once

#include <iostream>
#include <vector>
#include <cmath>
// opencv
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "map/submap.h"
#include "type/pose2d.h"
#include "type/laser_point.h"
#include "map/grid2d.h"

#define DEBUG 0

struct PointCloud;
struct Point2D;

void Image2Points(cv::Mat& image, std::vector<LaserPoint>& points, int threshold = 200);

void Image2Points(cv::Mat& image, std::vector<Eigen::Vector2f>& points, int threshold = 50);

void Image2Grid(cv::Mat& image, Grid2D& grid, int threshold=50);

void Grid2Image(const Grid2D& grid, cv::Mat& image, double prob=0.8);

void Grid2PointCloud(Grid2D& grid, PointCloud& cloud, double prob=0.8);

void Grid2LaserPoint(Grid2D& grid, std::vector<LaserPoint>& laser_point, double prob=0.8);

void Grid2Point2D(Grid2D &grid, std::vector<Point2D> &points, double prob=0.8);

cv::Mat DHT2Image(const std::vector<std::vector<float>>& dht);

cv::Mat RenderSubmaps(const std::vector<Submap>& submaps);

cv::Mat StitchSubmaps(const cv::Mat& img1, const cv::Mat& img2, const Pose2D& pose1, const Pose2D& pose2);

struct SE2{
    float x;
    float y;
    float yaw;
};

// 点云结构
struct PointCloud{
    std::vector<Vec2> points;
};

// Candidate
struct Candidate{
    Pose2D pose;
    double score;
};

// Multi-resolution Grid
class MultiResolutionGrid{
public:
    MultiResolutionGrid(const Grid2D& base, int levels) {
        grids_.push_back(base);

        for (int i=1; i < levels; ++i) {
            grids_.push_back(downsample(grids_.back()));
        }
    }

    const Grid2D& getLevel(int level) const {
        return grids_[level];
    }

    int levels() const {
        return grids_.size();
    }

private:
    Grid2D downsample(const Grid2D& src) {
        int w = src.width()/2;
        int h = src.height()/2;
        double r = 2*src.resolution();

        Grid2D dst(w,h,r,src.origin_x(),src.origin_y());

        for(int x=0; x<w; ++x) {
            for (int y=0; y < h; ++y) {
                double max_prob = 0;

                for (int dx=0; dx<2; ++dx) {
                    for (int dy=0; dy<2; ++dy) {
                        max_prob = std::max(max_prob, src.getProbability(x*2+dx, y*2+dx));
                    }
                }

                dst.setProbability(x, y, max_prob);
            }
        }
        return dst;
    }

    std::vector<Grid2D> grids_;
};

// Fast Correlative Scan Matcher
class FastCorrelativeScanMatcher2D{
public:
    FastCorrelativeScanMatcher2D(const Grid2D& grid, int levels) : multi_grid_(grid, levels) {}

    /**
     * @brief 
     * 
     * @param cloud 
     * @param linear_window 
     * @param angular_window Angle range, with values representing half of all angles; recommended value is π.
     * @return Candidate 
     */
    Candidate match(const PointCloud& cloud, double angular_window) {
        Candidate best;
        best.score = -1;

        int angle_steps = 4;

        double max_linear_window = computeMaxWindow();
        const Grid2D& coarse_grid = multi_grid_.getLevel(multi_grid_.levels()-1);
        double center_x = coarse_grid.origin_x() + coarse_grid.width() * coarse_grid.resolution() / 2.0;
        double center_y = coarse_grid.origin_y() - coarse_grid.height() * coarse_grid.resolution() / 2.0;

        Vec2 centroid{0.0, 0.0};
        if (cloud.points.empty()) return best;
        for (const auto& p : cloud.points) {
            centroid.x += p.x;
            centroid.y += p.y;
        }
        centroid.x /= cloud.points.size();
        centroid.y /= cloud.points.size();

        PointCloud cloud_local;
        cloud_local.points.reserve(cloud.points.size());
        for (const auto& p : cloud.points) {
            cloud_local.points.push_back({p.x - centroid.x, p.y - centroid.y});
        }
    
        for(int i=0; i<angle_steps; ++i) {
            double theta = angular_window - i * 2*angular_window/angle_steps;

            searchLevel(multi_grid_.levels()-1, cloud_local, {center_x,center_y,theta}, max_linear_window, best);
            // std::cout << "global_best: " << best.score << std::endl;
        }

        double c = std::cos(best.pose.theta);
        double s = std::sin(best.pose.theta);

        Vec2 Rc;
        Rc.x = c * centroid.x - s * centroid.y;
        Rc.y = s * centroid.x + c * centroid.y;

        Pose2D real_pose;
        real_pose.theta = best.pose.theta;
        real_pose.x = best.pose.x - Rc.x;
        real_pose.y = best.pose.y - Rc.y;

        return {real_pose, best.score};
    }

private:
    void searchLevel(int level, const PointCloud& cloud, Pose2D center, double window, Candidate& global_best) {
        const Grid2D& grid = multi_grid_.getLevel(level);

#if DEBUG
        double step = window / 10.0;
#else
        double step = grid.resolution();
#endif

        Candidate local_best;
        local_best.score = -std::numeric_limits<double>::infinity();

        for (double dx=-window; dx<=window; dx+=step) {
            for (double dy=-window; dy<=window; dy+=step) {
                Pose2D pose = {center.x+dx, center.y+dy, center.theta};
                // Pose2D pose = {center.x, center.y, 0};

                double score = scorePose(grid, cloud, pose);

                if (score > local_best.score) {
                    local_best.score = score;
                    local_best.pose = pose;
                }

            }
        }

        // std::cout << "level: " << level << std::endl;        
        // std::cout << "local_best: " << local_best.score << std::endl;  

        if (level == 0) {
            if (local_best.score > global_best.score) {
                global_best = local_best;
            }
        } else {
            searchLevel(level - 1, cloud, local_best.pose, window/2.0, global_best);
        }
    }

    double scorePose(const Grid2D& grid, const PointCloud& cloud, const Pose2D& pose) {
        double score = 0;
        int valid_points = 0;

#if DEBUG
        cv::Mat image;
        Grid2Image(grid, image);
#endif

        for (const auto& p:cloud.points) {
            Vec2 tp = pose.transform(p);

            int ix = (tp.x - grid.origin_x()) / grid.resolution();
            int iy = (grid.origin_y() - tp.y) / grid.resolution();
            if (ix < 0 || ix >= grid.width() || iy < 0 || iy >= grid.height())
                continue;

#if DEBUG
            image.at<cv::Vec3b>(iy, ix) = cv::Vec3b(0, 0, 255);
#endif

            score += grid.getProbabilityWorld(tp.x, tp.y);
            valid_points++;
        }

#if DEBUG
        cv::namedWindow("Grid", cv::WINDOW_NORMAL);
        cv::imshow("Grid", image);
        cv::waitKey(0);
#endif

        if(valid_points == 0) return -1e9;

        return score/cloud.points.size();
    }

    double computeMaxWindow() {
        const Grid2D& coarse_grid = multi_grid_.getLevel(multi_grid_.levels() - 1);

        double width_world = coarse_grid.width() * coarse_grid.resolution();
        double height_world = coarse_grid.height() * coarse_grid.resolution();

        return std::sqrt(width_world * width_world + height_world * height_world) / 2.0;
    }

    // Multi-resolution grid map
    MultiResolutionGrid multi_grid_;
};

class Point2D {
public:
    Point2D() : x(0.0), y(0.0) {};
    Point2D(double input_x, double input_y) : x(input_x), y(input_y) {}

    Point2D operator+(const Point2D& other) const { return Point2D(x+other.x, y+other.y); }

    Point2D& operator+=(const Point2D& other) {
        x+=other.x;
        y+=other.y;
        return *this;
    }

// private:
    double x;
    double y;
};

class Point3D {
public:
    Point3D() : x(0.0), y(0.0), z(0.0) {};
    Point3D(float input_x, float input_y, float input_z) : x(input_x), y(input_y), z(input_z) {}

    Point3D operator+(const Point3D& other) const { return Point3D(x+other.x, y+other.y, z+other.z); }

    Point3D& operator+=(const Point3D& other) {
        x+=other.x;
        y+=other.y;
        z+=other.z;
        return *this;
    }

private:
    float x;
    float y;
    float z;
};

// 定义体素滤波器
class VoxelFilter {
public:
    VoxelFilter(float voxel_size) : voxel_size(voxel_size) {}

    std::vector<Eigen::Vector2f> filter(const std::vector<Eigen::Vector2f> &points) const {
        std::unordered_map<int, std::vector<Eigen::Vector2f>> voxel_map;

        // 将点云数据划分到体素中
        for (const auto &point : points) {
            int x_index     = static_cast<int>(std::floor(point[0] / voxel_size));
            int y_index     = static_cast<int>(std::floor(point[1] / voxel_size));
            int voxel_index = x_index * 100000 + y_index;  // 假设 x_index 和 y_index 不会超过 100000
            voxel_map[voxel_index].push_back(point);
        }

        // 计算每个体素的代表点（例如，体素内的平均值）
        std::vector<Eigen::Vector2f> filtered_points;
        for (const auto &entry : voxel_map) {
            const auto     &voxel_points = entry.second;
            Eigen::Vector2f centroid(0, 0);
            for (const auto &point : voxel_points) {
                centroid += point;
            }
            centroid /= static_cast<float>(voxel_points.size());
            filtered_points.push_back(centroid);
        }

        return filtered_points;
    }

private:
    float voxel_size;
};