cmake_minimum_required(VERSION 2.8)

project(pcl_gpu_people_tools)

find_package(Eigen REQUIRED)

include_directories(libBVH/include BVHDBSparsify/include BVHtoRGBD/include ${Eigen_INCLUDE_DIRS})

add_subdirectory(libBVH/)
add_subdirectory(BVHDBSparsify/)
add_subdirectory(BVHtoRGBD/)

