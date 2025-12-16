/*
    显示平面片
    -按质心位置显示
*/
#if 0
        // 计算平面中心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*pcl_plane_points, centroid); // 计算点云几何质心

        // 转换为 Eigen 点云矩阵
        Eigen::MatrixXf mat(pcl_plane_points->points.size(), 3);
        for (size_t i = 0; i < pcl_plane_points->points.size(); ++i)
        {
            mat(i, 0) = pcl_plane_points->points[i].x;
            mat(i, 1) = pcl_plane_points->points[i].y;
            mat(i, 2) = pcl_plane_points->points[i].z;
        }

        // PCA — 求平面方向
        Eigen::Vector3f mean = mat.colwise().mean();
        Eigen::MatrixXf centered = mat.rowwise() - mean.transpose();
        Eigen::Matrix3f cov = (centered.transpose() * centered) / float(mat.rows());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
        Eigen::Vector3f ev1 = eig.eigenvectors().col(2); // 最大特征向量
        Eigen::Vector3f ev2 = eig.eigenvectors().col(1); // 第二大特征向量

        // 平面矩形大小（根据点云范围自动确定）
        float scale1 = 0.3;
        float scale2 = 0.3;

        // 4 个角点
        Eigen::Vector3f c(centroid[0], centroid[1], centroid[2]);

        Eigen::Vector3f p1 = c + ev1 * scale1 + ev2 * scale2;
        Eigen::Vector3f p2 = c - ev1 * scale1 + ev2 * scale2;
        Eigen::Vector3f p3 = c - ev1 * scale1 - ev2 * scale2;
        Eigen::Vector3f p4 = c + ev1 * scale1 - ev2 * scale2;

        // 创建 mesh 对象
        pcl::PolygonMesh plane_mesh;
        pcl::PointCloud<pcl::PointXYZ> mesh_points;
        mesh_points.push_back(pcl::PointXYZ(p1.x(), p1.y(), p1.z()));
        mesh_points.push_back(pcl::PointXYZ(p2.x(), p2.y(), p2.z()));
        mesh_points.push_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
        mesh_points.push_back(pcl::PointXYZ(p4.x(), p4.y(), p4.z()));

        pcl::toPCLPointCloud2(mesh_points, plane_mesh.cloud);
        
        // 拆分为两个三角形
        plane_mesh.polygons.resize(2);
        plane_mesh.polygons[0].vertices = {0, 1, 2}; // Triangle 1
        plane_mesh.polygons[1].vertices = {0, 2, 3}; // Triangle 2

        // 添加 mesh
        std::string mesh_name = "plane_mesh_" + std::to_string(plane_id);
        viewer->addPolygonMesh(plane_mesh, mesh_name);

        // 设置随机颜色
        viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR,
                r / 255.0f, g / 255.0f, b / 255.0f,
                mesh_name);

        // 设置透明度
        viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY,
                0.8,
                mesh_name);

        // 使用 Gouraud 着色，让颜色生效
        viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_SHADING,
                pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD,
                mesh_name);
#endif