cmake_minimum_required(VERSION 3.8)
project(bundle_adjustment)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_DIRS})

find_package(Eigen3 REQUIRED)

find_package(FMT REQUIRED)

add_executable(bundle_adjustment main.cpp)

target_link_libraries(bundle_adjustment ${OpenCV_LIBS} Eigen3::Eigen fmt::fmt)

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

target_link_libraries(bundle_adjustment ${G2O_LIBS})
