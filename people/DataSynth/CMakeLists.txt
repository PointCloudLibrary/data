cmake_minimum_required(VERSION 2.8)

project(pcl_gpu_people_tools)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(Eigen REQUIRED)

include_directories(libBVH/include BVHDBSparsify/include BVHtoRGBD/include ${EIGEN_INCLUDE_DIRS})

add_subdirectory(libBVH/)
add_subdirectory(BVHDBSparsify/)
add_subdirectory(BVHtoRGBD/)

