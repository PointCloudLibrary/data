cmake_minimum_required(VERSION 2.8)
project(libBVH)

#following two lines added for test
set( SRC src/libBVH.cpp
         src/BVHTransforms.cpp
         src/BVHMesh.cpp
         src/BVHMesh_smooth.cpp
         src/BVHMotionFile.cpp )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3 -march=athlon64 -m3dnow -mmmx -msse -msse2 -mfpmath=sse  -Wextra")

add_library(${PROJECT_NAME} ${SRC})
