cmake_minimum_required(VERSION 2.8)
project(BVHtoRGBD)
# the mesh/mesh app
set( SRC src/source.cpp
         src/GLerrors.cpp
         src/RGBDRenderer.cpp
         src/OpenGLContext.cpp )

find_package(OpenCV REQUIRED)

# this was necessary cause /usr/lib/libGL.so links to the mesa implementation
LINK_DIRECTORIES( /usr/lib/nvidia-current)
# necessary to access newer stuff
ADD_DEFINITIONS( -DGL_GLEXT_PROTOTYPES )

add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} libBVH X11 m GL GLU glut ${OpenCV_LIBRARIES})
