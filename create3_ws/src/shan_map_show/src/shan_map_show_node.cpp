#include <mrpt/io/CFileInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <mrpt_file>\n";
        return 1;
    }

    const std::string filename = argv[1];

    try
    {
        mrpt::io::CFileInputStream f(filename);
        auto arch = mrpt::serialization::archiveFrom(f);

        while (true)
        {
            std::shared_ptr<mrpt::serialization::CSerializable> obj;
            try
            {
                arch >> obj;
            }
            catch (const mrpt::serialization::CExceptionEOF&)
            {
                break; // 文件末尾
            }

            if (!obj) continue;

            auto map = std::dynamic_pointer_cast<mrpt::maps::COccupancyGridMap2D>(obj);
            if (map)
            {
                std::cout << "Found COccupancyGridMap2D!\n";

                int size_x = map->getSizeX();
                int size_y = map->getSizeY();

                cv::Mat img(size_y, size_x, CV_8UC1, cv::Scalar(127));

                for (int x = 0; x < size_x; x++)
                {
                    for (int y = 0; y < size_y; y++)
                    {
                        float p = map->getCell(x, y);
                        img.at<uchar>(size_y - 1 - y, x) = (p < 0) ? 127 : static_cast<uchar>(255 * (1.0f - p));
                    }
                }

                cv::imshow("Map", img);
                cv::waitKey(0);
                break;
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
    }

    return 0;
}
