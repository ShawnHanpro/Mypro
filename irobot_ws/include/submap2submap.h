#include <iostream>
#include "baselib.h"

SE2 EstimateSE2From2Points(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2,
                           const Eigen::Vector2f &q1,
                           const Eigen::Vector2f &q2);

SE2 RansacSE2(const std::vector<Eigen::Vector2f> &src,
              const std::vector<Eigen::Vector2f> &tgt, int max_iter = 1000,
              float dist_thresh = 0.2f);
