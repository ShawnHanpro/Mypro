#pragma once

#include <vector>

// Probability Grid
class Grid2D{
public:
    Grid2D(int width, int height, double resolution, double origin_x,
         double origin_y)
      : width_(width), height_(height), resolution_(resolution),
        origin_x_(origin_x), origin_y_(origin_y) {
            data_.resize(width * height, 0.0);
        }

    void setProbability(int x, int y, double prob) {
        if (inside(x, y)) data_[y * width_ + x] = prob;
    }

    double getProbability(int x, int y) const {
        if (!inside(x, y)) return 0.0;
        return data_[y * width_ + x];
    }

    double getProbabilityWorld(double wx, double wy) const {
        int x = (wx - origin_x_) / resolution_;
        // int y = (wy - origin_y_) / resolution_;
        // 按像素坐标系存储
        int y = (origin_y_ - wy) / resolution_;

        return getProbability(x, y);
    }

    int width() const {return width_;}
    int height() const {return height_;}

    double resolution() const {return resolution_;}
    double origin_x() const {return origin_x_;}
    double origin_y() const {return origin_y_;}

private:

    bool inside(int x, int y) const {
        return x>=0 && x<width_ && y>=0 && y<height_;
    }

    // bool isInsideWorld(double x, double y) {

    // }

    int width_;
    int height_;
    double resolution_;
    // 这个是栅格地图(0, 0)位置对应的世界坐标
    double origin_x_;
    double origin_y_;

    std::vector<double> data_;
};