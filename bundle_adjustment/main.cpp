// for std
#include <iostream>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;

// 寻找两个图像中的对应点，像素坐标系
// 输入：img1, img2 两张图像
// 输出：points1, points2, 两组对应的2D点
bool findCorrespondingPoints( const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 );

// Intrinsic Camera parameters
double cx = 325.5;
double cy = 253.5;
double fx = 518.0;
double fy = 519.0;

int main( int argc, char** argv )
{

    // OpenCV part
        
    // read images
    string img1_path = "../pic/1.png";
    string img2_path = "../pic/2.png";
    cv::Mat img1 = cv::imread( img1_path ); 
    cv::Mat img2 = cv::imread( img2_path ); 
    
    // find corresponding points between two images.
    vector<cv::Point2f> pts1, pts2;

    if ( findCorrespondingPoints( img1, img2, pts1, pts2 ) == false )
    {
        cout<<"not enough points！"<<endl;
        return 0;
    }

    cout<<"found "<<pts1.size()<<" corresponding points"<<endl;

    // g2o part

    // Create linear solver (using Cholmod)
    // Define the dimension for solver: estimated is SE3 (R,t), 6 dimension, and observations are 3d key points.
    // Using BlockSolver_6_3 = BlockSolverPL<6,3>; // already included in header file.
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> ());


    // Create block solver, and initialize from pre-defined linerSolver.
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));

    // Set up block solver algorithm
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    // Create sparse solver and set up pre-defined algorithm
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    optimizer.setVerbose( false );
    
    // Construct pose graph
    
    // vertex part

    // 2 vertices of pose
    for ( int i=0; i<2; i++ )
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); // fix first camera pose (not moving after optimization)
        // set both camera poses to 0 because we don't know their poses.
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
    }
    // add feature points as vertices
    // take feature points from first camera view (the feature points will be at first camera frame)
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexPointXYZ* v = new g2o::VertexPointXYZ();
        v->setId( 2 + i );
        // don't know depth, so set it to 1
        double z = 1;
        double x = ( pts1[i].x - cx ) * z / fx; 
        double y = ( pts1[i].y - cy ) * z / fy; 
        v->setMarginalized(true);
        v->setEstimate( Eigen::Vector3d(x,y,z) );
        optimizer.addVertex( v );
    }
    
    // prepare camera parameters
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );
    
    // edge part
    // edge generated from key points at first frame
    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
        edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0); // invoke pre-defined camera parameters
        // kernel function
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    // second frame
    for ( size_t i=0; i<pts2.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);
        // kernel function
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    
    cout<<"start optimization"<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cout<<"end of optimization"<<endl;
    
    //我们比较关心两帧之间的变换矩阵
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    cout<<"Pose="<<endl<<pose.matrix()<<endl;
    
    // 以及所有特征点的位置
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexPointXYZ* v = dynamic_cast<g2o::VertexPointXYZ*> (optimizer.vertex(i+2));
        cout<<"vertex id "<<i+2<<", pos = ";
        Eigen::Vector3d pos = v->estimate();
        cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    }
    
    // 估计inlier的个数
    int inliers = 0;
    for ( auto e:edges )
    {
        e->computeError();
        // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
        if ( e->chi2() > 1 )
        {
            cout<<"error = "<<e->chi2()<<endl;
        }
        else 
        {
            inliers++;
        }
    }
    
    cout<<"inliers in total points: "<<inliers<<"/"<<pts1.size()+pts2.size()<<endl;
    optimizer.save("ba.g2o");
    
    return 0;
}


bool findCorrespondingPoints( const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 )
{
    // construct orb instance
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    vector<cv::KeyPoint> kp1, kp2;
    cv::Mat desp1, desp2;
    // detect FAST feature points
    cv::Ptr<cv::FeatureDetector> detector = orb;
    detector->detect(img1, kp1);
    detector->detect(img2, kp2);
    // compute BRIEF description
    cv::Ptr<cv::DescriptorExtractor> extractor = orb;
    extractor->compute(img1, kp1, desp1);
    extractor->compute(img2, kp2, desp2);

    cout<<"found "<<kp1.size()<<" keypoints for image_1 and "<<kp2.size()<<" keypoints for image_2."<<endl;
    
    cv::Ptr<cv::DescriptorMatcher>  matcher = cv::DescriptorMatcher::create( "BruteForce-Hamming");
    
    double knn_match_ratio=0.8;
    vector< vector<cv::DMatch> > matches_knn;
    matcher->knnMatch( desp1, desp2, matches_knn, 2 ); // check Common Interfaces of Descriptor Matchers website for detail.
    // because we assign k = 2, there are two nearest results from desp2 for the data in desp1.
    vector< cv::DMatch > goodmatches;

    for ( size_t i=0; i<matches_knn.size(); i++ )
    {
        if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance )
            goodmatches.push_back( matches_knn[i][0] );
            // we use knn_match_ration to select "excellent" features.
            // Remember smaller index has shorter distance than bigger index.
            // if the smallest index is smaller than knn_match_ratio * second-least distance, we say the feature is execellent (not ambiguous)
    }
    
    if (goodmatches.size() <= 20) // if matching points are less than 20.
        return false;
    
    for ( auto m:goodmatches )
    {
        points1.push_back( kp1[m.queryIdx].pt );
        points2.push_back( kp2[m.trainIdx].pt );
    }
    
    return true;
}
