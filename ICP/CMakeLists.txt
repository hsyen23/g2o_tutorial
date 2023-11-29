cmake_minimum_required(VERSION 3.8)
project(g2o_ICP)

set(CMAKE_CXX_STANDARD 17)

find_package(G2O REQUIRED)
include_directories(${G2O_DIRS})

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

find_package(FMT REQUIRED)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(knownDataAssociation knownDataAssociation.cpp)
target_link_libraries(knownDataAssociation ${OpenCV_LIBS} g2o_core g2o_stuff Eigen3::Eigen fmt::fmt Sophus::Sophus)