#pragma once

#include "type/pose2d.h"
#include "map/grid2d.h"
// opencv
#include <opencv2/opencv.hpp>

class Submap {
public:
    Submap(const Grid2D& grid, const Pose2D& pose);
    Submap(const Grid2D& grid);

    const Grid2D& get_grid() const;
    const Pose2D& get_pose() const;

    cv::Mat to_image() const;


private:
    Grid2D grid_;
    Pose2D pose_;
};