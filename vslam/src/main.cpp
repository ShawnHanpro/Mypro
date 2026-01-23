#include <iostream>
#include <thread>
#include "slamBase.h"
using namespace std;

// OpenCV 特征检测模块
#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/nonfree/nonfree.hpp> // use this if you want to use SIFT or SURF
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

FRAME readFrame( int index, ParameterReader& pd );
double normofTransform( cv::Mat rvec, cv::Mat tvec );

// 检测两个帧，结果定义
enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME}; 

CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false );
// 检测近距离的回环
void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );
// 随机检测回环
void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );

int main( int argc, char** argv ) {
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera = GetDefaultCamera();
    int start = 1;
    int end = 500;

    FRAME last_frame = readFrame( start, pd );
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->detectAndCompute(last_frame.rgb, cv::Mat(), last_frame.kp, last_frame.desp);
    std::cout << "===== " << start << " =====" << std::endl;
    std::cout << "last frame kp: " << last_frame.kp.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = image2PointCloud(last_frame.rgb, last_frame.depth, camera);
    pcl::visualization::CloudViewer viewer("viewer");


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
    v->setId( start );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );

    vector< FRAME > keyframes; 
    keyframes.push_back( last_frame );

    int last_index = start;

#if 0
    bool check_loop_closure = pd.getData("check_loop_closure")==string("yes");

    for (int cur_index = start+1; cur_index < end; ++cur_index) {
        std::cout << "===== " << cur_index << " =====" << std::endl;
        FRAME cur_frame = readFrame( cur_index, pd );
        orb->detectAndCompute(cur_frame.rgb, cv::Mat(), cur_frame.kp, cur_frame.desp);

        CHECK_RESULT result = checkKeyframes( keyframes.back(), cur_frame, globalOptimizer ); //匹配该帧与keyframes里最后一帧

        switch (result) // 根据匹配结果不同采取不同策略
        {
        case NOT_MATCHED:
            //没匹配上，直接跳过
            cout<<RED"Not enough inliers."RESET<<endl;
            break;
        case TOO_FAR_AWAY:
            // 太近了，也直接跳
            cout<<RED"Too far away, may be an error."RESET<<endl;
            break;
        case TOO_CLOSE:
            // 太远了，可能出错了
            cout<<RED"Too close, not a keyframe"RESET<<endl;
            break;
        case KEYFRAME:
            cout<<GREEN"This is a new keyframe"RESET<<endl;
            // 不远不近，刚好
            /**
             * This is important!!
             * This is important!!
             * This is important!!
             * (very important so I've said three times!)
             */
            // 检测回环
            if (check_loop_closure)
            {
                checkNearbyLoops( keyframes, cur_frame, globalOptimizer );
                checkRandomLoops( keyframes, cur_frame, globalOptimizer );
            }
            keyframes.push_back( cur_frame );
            
            break;
        default:
            break;
        }
    }

        // 优化
    cout<<RESET"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    // globalOptimizer.save("../data//result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    // globalOptimizer.save( "../data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    // 拼接点云地图
    cout<<"saving the point cloud map..."<<endl;
    // PointCloud::Ptr output ( new PointCloud() ); //全局地图
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>()); //全局地图
    // PointCloud::Ptr tmp ( new PointCloud() );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel; // 网格滤波器，调整地图分辨率
    pcl::PassThrough<pcl::PointXYZRGB> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
    pass.setFilterFieldName("z");
    pass.setFilterLimits( 0.0, 6.0 ); //6m以上就不要了

    double gridsize = atof( pd.getData( "voxel_grid" ).c_str() ); //分辨图可以在parameters.txt里调
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    for (size_t i=0; i<keyframes.size(); i++)
    {
        // 从g2o里取出一帧
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud = image2PointCloud( keyframes[i].rgb, keyframes[i].depth, camera ); //转成点云
        // 以下是滤波
        voxel.setInputCloud( newCloud );
        voxel.filter( *tmp );
        pass.setInputCloud( tmp );
        pass.filter( *newCloud );
        // 把点云变换后加入全局地图中
        pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
        *output += *tmp;
        tmp->clear();
        newCloud->clear();
    }

    voxel.setInputCloud( output );
    voxel.filter( *tmp );


    viewer.showCloud(output);
    while (!viewer.wasStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif


#if 1
    for (int cur_index = start+1; cur_index < end; ++cur_index) {
        std::cout << "===== " << cur_index << " =====" << std::endl;
        FRAME cur_frame = readFrame( cur_index, pd );
        orb->detectAndCompute(cur_frame.rgb, cv::Mat(), cur_frame.kp, cur_frame.desp);
        std::cout << "cur frame kp: " << cur_frame.kp.size() << std::endl;

        RESULT_OF_PNP result = EstimateMotion( last_frame, cur_frame, camera );
        std::cout << "inliers: " << result.inliers << std::endl;
        if ( result.inliers < 5 ) //inliers不够，放弃该帧
            continue;
        double norm = normofTransform(result.rvec, result.tvec);
        cout << "norm: " << norm << endl;
        if ( norm >= 0.3 )
            continue;

        // cv::Mat img_show;
        // cv::drawKeypoints(cur_frame.rgb, cur_frame.kp, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // cv::imshow("keypoints", img_show);
        // cv::waitKey(0);

        // pnp计算的T是
        Eigen::Isometry3d T = CvMat2Eigen( result.rvec, result.tvec );
        Eigen::Isometry3d T_inv = T.inverse();
        static Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
        Twc = Twc * T_inv;

        // cout << "T=" << T.matrix() << endl;
        // cloud = JoinPointCloud( cloud, cur_frame, T, camera );
        // viewer.showCloud( cloud );

        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( cur_index );
        v->setEstimate(Twc);
        globalOptimizer.addVertex(v);

        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        edge->vertices() [0] = globalOptimizer.vertex( last_index );
        edge->vertices() [1] = globalOptimizer.vertex( cur_index );
        
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        information.diagonal() << 100,100,100, 100,100,100;

        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation( information );
        // 边的估计即是pnp求解之结果
        edge->setMeasurement( T_inv );
        globalOptimizer.addEdge(edge);
        
        last_frame = cur_frame;
        last_index = cur_index;   
    }

    // 优化所有边
    cout << "optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << endl;
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    cout << "Optimization done." << endl;

    // 取出所有优化后的位姿
    vector<Eigen::Isometry3d> optimizedPoses;
    for (int i = start; i < end; i++) {
        g2o::VertexSE3 *v =
            dynamic_cast<g2o::VertexSE3 *>(globalOptimizer.vertex(i));
        if (v)
            optimizedPoses.push_back(v->estimate());
    }

    // 使用优化后的位姿拼接地图
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap(
        new pcl::PointCloud<pcl::PointXYZRGB>()
    );

    for (int i = start; i < end; i++) {
        std::cout << "##### " << i << " #####" << std::endl;
        FRAME f = readFrame(i, pd);
        OrbComputerKeyPointsAndDesp(f);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc =
            image2PointCloud(f.rgb, f.depth, camera);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_transformed(
            new pcl::PointCloud<pcl::PointXYZRGB>()
        );

        pcl::transformPointCloud(
            *pc, *pc_transformed,
            optimizedPoses[i - start].matrix()
        );

        *globalMap += *pc_transformed;
        viewer.showCloud(globalMap);
    }

    while (!viewer.wasStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    globalOptimizer.clear();
#endif

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
    f.frameID = index;
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec ) {
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops) {
    static ParameterReader pd;
    static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    static CAMERA_INTRINSIC_PARAMETERS camera = GetDefaultCamera();
    // 比较f1 和 f2
    RESULT_OF_PNP result = EstimateMotion( f1, f2, camera );
    // std::cout << "inliers: " << result.inliers << std::endl;

    if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
        return NOT_MATCHED;
    // 计算运动范围是否太大
    double norm = normofTransform(result.rvec, result.tvec);
    if ( is_loops == false )
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;   // too adjacent frame
    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点部分
    // 顶点只需设定id即可
    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( f2.frameID );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
    // 边部分
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 连接此边的两个顶点id
    edge->setVertex( 0, opti.vertex(f1.frameID ));
    edge->setVertex( 1, opti.vertex(f2.frameID ));
    edge->setRobustKernel( new g2o::RobustKernelHuber() );
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
    Eigen::Isometry3d T = CvMat2Eigen( result.rvec, result.tvec );
    // edge->setMeasurement( T );
    edge->setMeasurement( T.inverse() );
    // 将此边加入图中
    opti.addEdge(edge);
    return KEYFRAME;
}

void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader pd;
    static int nearby_loops = atoi( pd.getData("nearby_loops").c_str() );
    
    // 就是把currFrame和 frames里末尾几个测一遍
    if ( frames.size() <= nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
}

void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader pd;
    static int random_loops = atoi( pd.getData("random_loops").c_str() );
    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测
    
    if ( frames.size() <= random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes( frames[index], currFrame, opti, true );
        }
    }
}