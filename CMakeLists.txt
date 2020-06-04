cmake_minimum_required(VERSION 3.16)
project(chp10)

set(CMAKE_CXX_STANDARD 11)
Find_Package(Eigen3 REQUIRED)
Find_Package(Sophus REQUIRED)
find_package(g2o REQUIRED)

include_directories(${G2O_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3 g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})

add_executable(pose_graph_g2o_lie_algbra pose_graph_g2o_lie_algbra.cpp)
target_link_libraries(pose_graph_g2o_lie_algbra g2o_core g2o_stuff ${CHOLMOD_LIBRARIES} ${Sophus_LIBRARIES})
