#include "submap2submap.h"
#include <random>

SE2 EstimateSE2From2Points(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2,
                           const Eigen::Vector2f &q1,
                           const Eigen::Vector2f &q2) {
  Eigen::Vector2f dp = p2 - p1;
  Eigen::Vector2f dq = q2 - q1;

  float angle_p = std::atan2(dp.y(), dp.x());
  float angle_q = std::atan2(dq.y(), dq.x());

  float yaw = angle_q - angle_p;

  Eigen::Matrix2f R;
  R << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);

  Eigen::Vector2f t = q1 - R * p1;

  return {t.x(), t.y(), yaw};
}

SE2 RansacSE2(const std::vector<Eigen::Vector2f> &src,
              const std::vector<Eigen::Vector2f> &tgt, int max_iter = 1000,
              float dist_thresh = 0.2f) {
  std::default_random_engine rng;
  std::uniform_int_distribution<int> dist_src(0, src.size() - 1);
  std::uniform_int_distribution<int> dist_tgt(0, tgt.size() - 1);

  int best_inliers = 0;
  SE2 best_pose{0, 0, 0};

  for (int iter = 0; iter < max_iter; ++iter) {
    int i1 = dist_src(rng);
    int i2 = dist_src(rng);
    int j1 = dist_tgt(rng);
    int j2 = dist_tgt(rng);

    if (i1 == i2 || j1 == j2)
      continue;

    SE2 pose = EstimateSE2From2Points(src[i1], src[i2], tgt[j1], tgt[j2]);

    Eigen::Matrix2f R;
    R << std::cos(pose.yaw), -std::sin(pose.yaw), std::sin(pose.yaw),
        std::cos(pose.yaw);

    int inliers = 0;
    for (const auto &p : src) {
      Eigen::Vector2f p_trans = R * p + Eigen::Vector2f(pose.x, pose.y);

      // 最近邻（暴力版，submap 点数不大完全 OK）
      float min_dist = 1e9;
      for (const auto &q : tgt) {
        min_dist = std::min(min_dist, (p_trans - q).norm());
      }

      if (min_dist < dist_thresh)
        inliers++;
    }

    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_pose = pose;
    }
  }

  return best_pose;
}
