#include "baselib.h"

// 方便帧间匹配计算 
void Image2Points(cv::Mat& image, std::vector<LaserPoint>& points, int threshold) {
    int width = image.cols;
    int height = image.rows;
    int c_x = width / 2;
    int c_y = height / 2;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (image.at<uchar>(y, x) > threshold) {
                
                LaserPoint lp;
                lp.x = x - c_x;
                lp.y = c_y - y;

                points.emplace_back(lp);
            }
        }
    }
}

void Image2Grid(cv::Mat& image, Grid2D& grid, int threshold) {
    int width = image.cols;
    int height = image.rows;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (image.at<uchar>(y, x) < threshold) {

                grid.setProbability(x, y, 1.0);
            } else if (image.at<uchar>(y, x) < 200 && image.at<uchar>(y, x) > 50) {
                grid.setProbability(x, y, 0.5);
            }
        }
    }
}

void Grid2Image(const Grid2D& grid, cv::Mat& image, double prob) {
    image = cv::Mat(grid.height(), grid.width(), CV_8UC3, cv::Scalar(0, 0, 0));

    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            if (grid.getProbability(x,y) < prob) continue;

            image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
        }
    }
}

void Grid2PointCloud(Grid2D& grid, PointCloud& cloud, double prob) {
    
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            if (grid.getProbability(x,y) < prob) continue;

            double wx = (x+0.5) * grid.resolution() + grid.origin_x();
            double wy = grid.origin_y() - (y+0.5) * grid.resolution();

            cloud.points.push_back({wx, wy});
        }
    }
}

void Grid2LaserPoint(Grid2D &grid, std::vector<LaserPoint> &laser_point,
                     double prob) {
    laser_point.clear();

    double res = grid.resolution();

    double cx = grid.origin_x() + grid.width() * res * 0.5;

    double cy = grid.origin_y() - grid.height() * res * 0.5;

    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {

            if (grid.getProbability(x, y) < prob)
                continue;

            double wx = (x + 0.5) * res + grid.origin_x();
            double wy = grid.origin_y() - (y + 0.5) * res;

            LaserPoint lp;
            lp.x = wx - cx;
            lp.y = wy - cy;

            laser_point.push_back(lp);
        }
    }
}

void Grid2Point2D(Grid2D &grid, std::vector<Point2D> &points,
                     double prob) {
    points.clear();

    double res = grid.resolution();

    double cx = grid.origin_x() + grid.width() * res * 0.5;

    double cy = grid.origin_y() - grid.height() * res * 0.5;

    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {

            if (grid.getProbability(x, y) < prob)
                continue;

            double wx = (x + 0.5) * res + grid.origin_x();
            double wy = grid.origin_y() - (y + 0.5) * res;

            Point2D lp;
            lp.x = wx - cx;
            lp.y = wy - cy;

            points.push_back(lp);
        }
    }
}

cv::Mat RenderSubmaps(const std::vector<Submap>& submaps) {
    if (submaps.empty()) return cv::Mat();

    double res = submaps[0].get_grid().resolution();

    // ---------- 1. 计算世界范围 ----------
    double min_x = 1e9, min_y = 1e9;
    double max_x = -1e9, max_y = -1e9;

    for (const auto& sm : submaps)
    {
        const Grid2D& g = sm.get_grid();

        double w = g.width() * res;
        double h = g.height() * res;

        // 四个角
        std::vector<Vec2> corners = {
            {g.origin_x(), g.origin_y()},
            {g.origin_x() + w, g.origin_y()},
            {g.origin_x(), g.origin_y() - h},
            {g.origin_x() + w, g.origin_y() - h}
        };

        for (auto& c : corners)
        {
            Vec2 wc = sm.get_pose().transform(c);

            min_x = std::min(min_x, wc.x);
            min_y = std::min(min_y, wc.y);
            max_x = std::max(max_x, wc.x);
            max_y = std::max(max_y, wc.y);
        }
    }

    int width  = std::ceil((max_x - min_x) / res);
    int height = std::ceil((max_y - min_y) / res);

    // 创建世界画布（黑色）
    cv::Mat canvas(height, width, CV_8UC1, cv::Scalar(125));

    // ---------- 2. 渲染每个 submap ----------
    for (const auto& sm : submaps)
    {
        const Grid2D& g = sm.get_grid();

        for (int y = 0; y < g.height(); ++y)
        {
            for (int x = 0; x < g.width(); ++x)
            {
                double prob = g.getProbability(x, y);
                // if (prob < 0.5) continue;  // 只画占用

                // submap 内部 world 坐标
                double wx = g.origin_x() + x * res;
                double wy = g.origin_y() - y * res;

                // 转到全局世界
                Vec2 pw = sm.get_pose().transform({wx, wy});

                // int ix = (pw.x - min_x) / res;
                // int iy = (max_y - pw.y) / res;

                int ix = std::round((pw.x - min_x) / res); // 四舍五入
                int iy = std::round((max_y - pw.y) / res);

                if (ix < 0 || ix >= width ||
                    iy < 0 || iy >= height)
                    continue;
                    
                // if (ix == 181) {
                //     std::cout << "ix: " << ix << " iy: " << iy << std::endl;
                //     std::cout << "min_x: " << min_x << " max_y: " << max_y << std::endl;
                //     std::cout << "pw.x: " << pw.x << " pw.y: " << pw.y << std::endl;
                //     std::cout << "prob: " << prob << std::endl;
                // }

                if (prob == 0) {
                    if (canvas.at<uchar>(iy, ix) == 0) continue;
                    canvas.at<uchar>(iy, ix) = 255;
                } else if (prob > 0.8) {
                    canvas.at<uchar>(iy, ix) = 0;
                }
            }
        }
        
        // cv::namedWindow("canvas", cv::WINDOW_NORMAL);
        // cv::imshow("canvas", canvas);
        // cv::waitKey(0);
    }

    return canvas;
}

cv::Mat StitchSubmaps(const cv::Mat& img1, const cv::Mat& img2, const Pose2D& pose1, const Pose2D& pose2) {
    // 获取两个图像的大小
    int width1 = img1.cols, height1 = img1.rows;
    int width2 = img2.cols, height2 = img2.rows;

    // 根据 pose 计算拼接的画布大小
    double min_x = std::min(pose1.x, pose2.x);
    double min_y = std::min(pose1.y, pose2.y);
    double max_x = std::max(pose1.x + width1, pose2.x + width2);
    double max_y = std::max(pose1.y + height1, pose2.y + height2);

    int width  = std::ceil(max_x - min_x);
    int height = std::ceil(max_y - min_y);

    // 创建一个大画布，用于容纳两个拼接后的图像
    cv::Mat canvas(height, width, CV_8UC1, cv::Scalar(0));  // 初始化为黑色背景

    // 将第一个图像放置在合适的位置
    cv::Mat submap1 = img1.clone();
    cv::Rect roi1(pose1.x - min_x, pose1.y - min_y, width1, height1);
    submap1.copyTo(canvas(roi1));

    // 将第二个图像放置在合适的位置
    cv::Mat submap2 = img2.clone();
    cv::Rect roi2(pose2.x - min_x, pose2.y - min_y, width2, height2);
    submap2.copyTo(canvas(roi2));

    return canvas;
}

cv::Mat DHT2Image(const std::vector<std::vector<float>>& dht)
{
    int rows = dht.size();          // theta
    int cols = dht[0].size();       // rho

    cv::Mat img(rows, cols, CV_32F);

    float max_val = 0;

    // copy + find max
    for(int i=0;i<rows;i++)
        for(int j=0;j<cols;j++)
        {
            img.at<float>(i,j) = dht[i][j];
            max_val = std::max(max_val, dht[i][j]);
        }

    // normalize
    img /= (max_val + 1e-6f);

    cv::Mat img8u;
    img.convertTo(img8u, CV_8U, 255);

    // 放大方便观察
    cv::resize(img8u, img8u, cv::Size(), 3, 3, cv::INTER_NEAREST);

    return img8u;
}