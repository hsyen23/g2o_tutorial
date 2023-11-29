#include <iostream>
#include <Eigen/Core>
#include <cmath>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// for g2o
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>

#include <g2o/core/block_solver.h>

#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

#include <g2o/solvers/dense/linear_solver_dense.h>

// for Sophus, see https://blog.csdn.net/qq_42676511/article/details/124867819
#include "sophus/se2.hpp"

// Customize g2o vertex for ICP
class ICPVertex : public g2o::BaseVertex<3, Sophus::SE2d> // set variable to 3 because our Jacobian should be 2*3 matrix
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void setToOriginImpl() override
        {
            _estimate = Sophus::SE2d();
        }
        
        virtual void oplusImpl(const double *update) override
        {
            Eigen::Matrix<double, 3, 1> twist; // "update": a twist which first two components represent changing in x,y coordinates, last component represents changing in theta.
            twist << update[0], update[1], update[2];
            _estimate = _estimate * Sophus::SE2d::exp(twist); // right perturbation
        }
        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}
    };

// Customize g2o edge for ICP
class ICPEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, ICPVertex> // set variable to 2 because our observation is point_cloud_2 which uses x,y coordinates only
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ICPEdge(Eigen::Vector2d x) : BaseUnaryEdge(), _pt_1(x) {}

        virtual void computeError() override
        {
            const ICPVertex *v = static_cast<const ICPVertex *>(_vertices[0]);
            const Sophus::SE2d T = v->estimate();
            _error = T * _pt_1 - _measurement; // _measurement means point_cloud_2
        }

        virtual void linearizeOplus() override
        {
            const ICPVertex *v = static_cast<const ICPVertex *>(_vertices[0]);
            const Sophus::SE2d T = v->estimate();
            // get current theta state
            double theta = std::atan2(T.params()[1], T.params()[0]);
            Eigen::Matrix2d d_theta;
            d_theta << -sin(theta), -cos(theta), cos(theta), -sin(theta);
            // Jacobian, using right perturbation (de/dT = [de/dt, de/dR] )
            _jacobianOplusXi.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity(); // de/dt
            _jacobianOplusXi.block<2, 1>(0, 2) = d_theta * _pt_1; // de/dR
        }

        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}

    private:
        Eigen::Vector2d _pt_1;
    };

cv::Mat draw_point_cloud(Eigen::MatrixXd pt_1, Eigen::MatrixXd pt_2)
{
    cv::Mat img(400, 500, CV_8UC3, cv::Scalar(255,255, 255)); // create an empty image
    int left = -10;
    int right = 40;
    int bottom = -10;
    int top = 30;
    double resolution = 0.1;
    // draw point cloud 1
    for(int i = 0; i < pt_1.rows(); i++)
    {
        float px = (pt_1(i,0) - left) / resolution;
        float py = (top - pt_1(i,1)) / resolution;
        cv::Point2f p(px, py);
        cv::circle(img, p, 4, cv::Scalar(255, 0, 0), -1);
    }
    // draw point cloud 2
    for(int i = 0; i < pt_2.rows(); i++)
    {
        float px = (pt_2(i,0) - left) / resolution;
        float py = (top - pt_2(i,1)) / resolution;
        cv::Point2f p(px, py);
        cv::circle(img, p, 2, cv::Scalar(0, 0, 255), -1);
    }
    return img;
}

int main()
{
    // 1. data preparation
    
    // generate first point cloud
    const int num = 30;
    Eigen::Matrix<double, num, 2> pt_1;
    for(int i = 0; i<num; i++)
    {
        pt_1(i,0) = i;
        pt_1(i,1) = 0.2*i*sin(0.5*i);
    }
    //std::cout << "Here is the matrix pt_1:\n" << pt_1 << std::endl;

    // assign true rotation and translation
    float rotation = M_PI/6;
    Eigen::Matrix2d R;
    R << cos(rotation/4), -sin(rotation/4), sin(rotation/4), cos(rotation/4);
    std::cout << "rotation matrix R:\n" << R << std::endl;
    Eigen::Vector2d T;
    T << 2, 5;
    std::cout << "translation vector T:\n" << T << std::endl;

    // generate second point cloud
    Eigen::Matrix<double, num, 2> pt_2;
    for(int i = 0; i<num; i++)
    {
        Eigen::Vector2d newPt;
        newPt = R * pt_1.block(i,0,1,2).transpose() + T; // Eigen: dynamically extract submatrix, see https://amytabb.com/til/2022/02/27/eigen-extract-submatrices-2/
        pt_2(i,0) = newPt(0,0);
        pt_2(i,1) = newPt(1,0);
    }
    //std::cout << "Here is the matrix pt_2:\n" << pt_2 << std::endl;

    // 2. plot before ICP
    cv::Mat img_beforeICP = draw_point_cloud(pt_1, pt_2);
    cv::imshow("Before ICP", img_beforeICP);
    
    // 3. use g2o to solve ICP problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,2>> Block; // tip for dimension: <d of measurement, d of estimated>
    std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    //optimizer.setVerbose(true);

    // add vertex
    ICPVertex* v = new ICPVertex();
    v->setEstimate(Sophus::SE2d());
    v->setId(0);
    optimizer.addVertex(v);
    
    // add edge
    for (int i = 0; i < num; i++)
    {
        ICPEdge* edge = new ICPEdge(pt_1.block(i,0,1,2).transpose());
        edge->setId(i);
        edge->setVertex(0,v);
        edge->setMeasurement(pt_2.block(i,0,1,2).transpose());
        edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());
        optimizer.addEdge(edge);
    }

    // run
    std:: cout << "\nstart optimization\n" << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    Sophus::SE2d se2_estimate = v->estimate();
    Eigen::Matrix2d estimated_R;
    Eigen::Vector2d estimated_T;
    float estimated_theta = std::atan2(se2_estimate.params()[1], se2_estimate.params()[0]);
    estimated_R << cos(estimated_theta), -sin(estimated_theta), sin(estimated_theta), cos(estimated_theta);
    estimated_T << se2_estimate.params()[2], se2_estimate.params()[3];
    std::cout << "estimated R:\n" << estimated_R << std::endl;
    std::cout << "estimated T:\n" << estimated_T << std::endl;

    // after ICP plot
    Eigen::Matrix<double, num, 2> pt_1_af;
    for(int i = 0; i<num; i++)
    {
        Eigen::Vector2d newPt;
        newPt = estimated_R * pt_1.block(i,0,1,2).transpose() + estimated_T;
        pt_1_af(i,0) = newPt(0,0);
        pt_1_af(i,1) = newPt(1,0);
    }
    cv::Mat img_afterICP = draw_point_cloud(pt_1_af, pt_2);
    cv::imshow("After ICP", img_afterICP);

    cv::waitKey(0);
    return 0;
}