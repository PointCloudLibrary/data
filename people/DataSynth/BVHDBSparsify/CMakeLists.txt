cmake_minimum_required(VERSION 2.8)
project(BVHDBSparsify)

find_package(OpenMP QUIET)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_executable(${PROJECT_NAME} src/source.cpp)
target_link_libraries(${PROJECT_NAME} libBVH)
