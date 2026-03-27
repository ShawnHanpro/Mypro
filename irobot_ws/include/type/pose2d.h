#pragma once

#include <cmath>

struct Vec2{
    double x;
    double y;
};

struct Pose2D{
    double x;
    double y;
    double theta;
    bool valid = false;

    Vec2 transform(const Vec2& p) const {
        double c = std::cos(theta);
        double s = std::sin(theta);

        return {
            // R << cosθ, -sinθ,
            //      sinθ, cosθ;
            c * p.x - s * p.y + x,
            s * p.x + c * p.y + y
        };
    }
};