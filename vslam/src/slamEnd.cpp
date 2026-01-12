#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
using namespace std;

#include "slamBase.h"
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

// 给定index，读取一帧数据
FRAME readFrame( int index, ParameterReader& pd );
// 度量运动的大小
double normofTransform( cv::Mat rvec, cv::Mat tvec );

int main( int argc, char** argv ) {
    ParameterReader pd;
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );

    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // 当前索引为currIndex
    FRAME lastFrame = readFrame( currIndex, pd ); // 上一帧数据
    // 我们总是在比较currFrame和lastFrame
    CAMERA_INTRINSIC_PARAMETERS camera = GetDefaultCamera();
    OrbComputerKeyPointsAndDesp(lastFrame);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);
    
    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double max_norm = atof( pd.getData("max_norm").c_str() );


    // g2o初始化
    // 选择优化方法
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // 初始化求解器
    // 1.线性求解器（用unique_ptr）
    auto linearSolver = std::make_unique<SlamLinearSolver>();
    
    // 2.BlockSolver
    auto blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
    
    // 3.LM优化器
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    
    // 4.装入优化器
    g2o::SparseOptimizer globalOptimizer; // 最后使用的是这个
    globalOptimizer.setAlgorithm(solver);
    // 不要输出调试信息
    globalOptimizer.setVerbose(false);

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );

    int lastIndex = currIndex; // 上一帧的id

    /*
    for ( currIndex=startIndex+1; currIndex<endIndex; currIndex++ ) {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame
        OrbComputerKeyPointsAndDesp(currFrame);
        // 比较currFrame 和 lastFrame
        RESULT_OF_PNP result = EstimateMotion( lastFrame, currFrame, camera );
        if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
            continue;
        // 计算运动范围是否太大
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;

        Eigen::Isometry3d T = CvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;
        
        cloud = JoinPointCloud( cloud, currFrame, T, camera );
        
        if ( visualize == true )
            viewer.showCloud( cloud );

        lastFrame = currFrame;
    }
    */

    for ( currIndex=startIndex+1; currIndex<endIndex; currIndex++ ) {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame
        OrbComputerKeyPointsAndDesp(currFrame);
        // 比较currFrame 和 lastFrame
        RESULT_OF_PNP result = EstimateMotion( lastFrame, currFrame, camera );
        if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
            continue;
        // 计算运动范围是否太大
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;
        Eigen::Isometry3d T = CvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;
        
        // 去掉可视化的话，会快一些
        // if ( visualize == true )
        // {
        //     cloud = JoinPointCloud( cloud, currFrame, T, camera );
        //     viewer.showCloud( cloud );
        // }
        
        // 向g2o中增加这个顶点与上一帧联系的边
        // 顶点部分
        // 顶点只需设定id即可
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( currIndex );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        globalOptimizer.addVertex(v);
        // 边部分
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->vertices() [0] = globalOptimizer.vertex( lastIndex );
        edge->vertices() [1] = globalOptimizer.vertex( currIndex );
        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation( information );
        // 边的估计即是pnp求解之结果
        edge->setMeasurement( T );
        // 将此边加入图中
        globalOptimizer.addEdge(edge);

        lastFrame = currFrame;
        lastIndex = currIndex;
    }

    // 优化所有边
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    // globalOptimizer.save("../data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    // globalOptimizer.save( "../data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    // 取出所有优化后的位姿
    vector<Eigen::Isometry3d> optimizedPoses;
    for (int i = startIndex; i < endIndex; i++) {
        g2o::VertexSE3* v =
            dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex(i));
        if (v)
            optimizedPoses.push_back(v->estimate());
    }

    // 使用优化后的位姿拼接地图
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap(
        new pcl::PointCloud<pcl::PointXYZRGB>()
    );

    for (int i = startIndex; i < endIndex; i++) {
        FRAME f = readFrame(i, pd);
        OrbComputerKeyPointsAndDesp(f);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc =
            image2PointCloud(f.rgb, f.depth, camera);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_transformed(
            new pcl::PointCloud<pcl::PointXYZRGB>()
        );

        pcl::transformPointCloud(
            *pc, *pc_transformed,
            optimizedPoses[i - startIndex].matrix()
        );

        *globalMap += *pc_transformed;

        viewer.showCloud(globalMap);
    }

    // 显示地图
    // pcl::visualization::CloudViewer finalViewer("Final Optimized Map");
    // finalViewer.showCloud(globalMap);

    while (!viewer.wasStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    globalOptimizer.clear();

    return 0;
}

FRAME readFrame( int index, ParameterReader& pd ) {
    FRAME f;
    string rgbDir   =   pd.getData("rgb_dir");
    string depthDir =   pd.getData("depth_dir");
    
    string rgbExt   =   pd.getData("rgb_extension");
    string depthExt =   pd.getData("depth_extension");

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb = cv::imread( filename );

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec ) {
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}