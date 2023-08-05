# Introduction
g2o is a popular graph-optimization tool. This repository contains 3 example for learning how to use g2o.
Examples are listed below:
1. Curve fitting
2. Bundle adjustment
3. ICP

Reference:
Basic Concept: https://blog.csdn.net/QLeelq/article/details/115497273
Basic Concept: https://www.guyuehome.com/37292
BA example: https://www.cnblogs.com/gaoxiang12/p/5304272.html

# Install
##### This installation process has been testified for `ubuntu:jammy-20230624` via docker.
### Install Requirement
```
sudo apt update
sudo apt install build-essential
sudo apt install cmake
sudo apt install libeigen3-dev
sudo apt install libspdlog-dev
sudo apt install libsuitesparse-dev
sudo apt install qtdeclarative5-dev
sudo apt install qt5-qmake
sudo apt install libqglviewer-dev-qt5
```
### Install git and clone g2o repository
```
sudo apt install git
cd /home
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
cmake --build .
sudo make install
```
We need to rename `g2oconfig.cmake` to `g2o-config.cmake` to avoid cmake error.
```
cd /usr/local/lib/cmake/g2o
mv g2oconfig.cmake g2o-config.cmake
```
### Install OpenCV (Optional)
```
cd /home
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake ..
cmake --build .
sudo make install
```

# g2o Pipeline
![Alt text](https://github.com/hsyen23/g2o_tutorial/blob/main/picture/g2o_pipeline.png "g2o pipeline")
```
1. build a linear solver LinearSolver.
2. build a BlockSolver, initialize it with predefined LinearSolver.
3. build a total solver selected from GN, LM, or DogLeg, initialize it with BlockSolver.
4. build the core of g2o: SparseOptimizer
5. define vertex and edge, adding them to SparseOptimizer.
6. setup parameters, run optimization.
```

#### step 1: build LinearSolver
First, we need to understand the dimension for our solver and define it.
e.g. vertex is 3d, edge is 2d.
```
typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,2>> Block;
```
There are some common dimension that are already included in the header file by using template.
```
template<int p, int l>
using BlockSolverPL = BlockSolver<BlockSolverTraits<p, l>>;

// solver for BA/3D SLAM
using BlockSolver_6_3 = BlockSolverPL<6, 3>;// pose is 6d, and observation point is 3d.

// solver for BA with scale
using BlockSolver_7_3 = BlockSolverPL<7, 3>;//adding one scale upon BlockSolver_6_3

// 2Dof landmarks 3Dof poses
using BlockSolver_3_2 = BlockSolverPL<3, 2>;// pise is 3d, and observation point is 2d.

// variable size solver
using BlockSolverX = BlockSolverPL<Eigen::Dynamic, Eigen::Dynamic>;
```
g2o provides five methods for liner solver.
* Cholmod
* CSparse
* PCG
* Dense
* Eigen

Chooes one and initiate it.
e.g.
```
Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>()
Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>()
Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>()
```

#### Step 2: build BlockSolver
```
Block* solver_ptr = new Block(linearSolver);
```

#### Step 3: build total solver
```
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr)
g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr)
g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr)
```
#### Step 4: build SparseOptimizer
```
g2o::SparseOptimizer optimizer;
optimizer.setAlgorithm(solver);
```
#### Step 5: define vertex and edge, adding them to SparseOptimizer
There are some predefined vertex from g2o showing below.
```
VertexSE2 : public BaseVertex<3, SE2>  //2D pose Vertex, (x,y,theta)
VertexSE3 : public BaseVertex<6, Isometry3>  //6d vector (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion)
VertexPointXY : public BaseVertex<2, Vector2>
VertexPointXYZ : public BaseVertex<3, Vector3>
VertexSBAPointXYZ : public BaseVertex<3, Vector3>

// SE3 Vertex parameterized internally with a transformation matrix and externally with its exponential map
VertexSE3Expmap : public BaseVertex<6, SE3Quat>

// SBACam Vertex, (x,y,z,qw,qx,qy,qz),(x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
// qw is assumed to be positive, otherwise there is an ambiguity in qx,qy,qz as a rotation
VertexCam : public BaseVertex<6, SBACam>

// Sim3 Vertex, (x,y,z,qw,qx,qy,qz),7d vector,(x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
VertexSim3Expmap : public BaseVertex<7, Sim3>
```
Add vertex and edge.
e.g. vertex
```
    for (size_t i = 0; i < Vertexs.size(); i++) {
        VertexSE2* v = new VertexSE2();
        v->setEstimate(Vertexs[i]);
        v->setId(i);
        if (i == 0) {
            v->setFixed(true);
        }
        optimizer.addVertex(v);
    }
```
e.g. edge
```
    for (size_t i = 0; i < Edges.size(); i++) {
        EdgeSE2* edge = new EdgeSE2();

        Edge tmpEdge = Edges[i];

        edge->setId(i);
        edge->setVertex(0, optimizer.vertices()[tmpEdge.xi]);
        edge->setVertex(1, optimizer.vertices()[tmpEdge.xj]);

        edge->setMeasurement(tmpEdge.measurement);
        edge->setInformation(tmpEdge.infoMatrix);
        optimizer.addEdge(edge);
    }
```
#### step 6: setup parameter and run optimization
```
optimizer.initializeOptimization();
```
```
optimizer.optimize(100);
```