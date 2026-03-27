#include "map/submap.h"

Submap::Submap(const Grid2D& grid, const Pose2D& pose) : grid_(grid), pose_(pose) {

}

Submap::Submap(const Grid2D& grid) : grid_(grid), pose_({0,0,0}) {

}

const Grid2D& Submap::get_grid() const {
    return grid_;
}

const Pose2D& Submap::get_pose() const {
    return pose_;
}

cv::Mat Submap::to_image() const {
    cv::Mat image(grid_.height(), grid_.width(), CV_8UC3, cv::Scalar(128, 128, 128));

    for (int y = 0; y < grid_.height(); ++y) {
        for (int x = 0; x < grid_.width(); ++x) {
            double p = grid_.getProbability(x,y);
            // subgrid: obstacle-1.0 unknown-0.5 passable-0.0

            image.at<cv::Vec3b>(y, x) = cv::Vec3b((1-p)*255, (1-p)*255, (1-p)*255);
        }
    }

    return image;
}