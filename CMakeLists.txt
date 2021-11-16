cmake_minimum_required(VERSION 2.8.3)
project(template_render)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_COMPILER "/usr/bin/clang++" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

FIND_PACKAGE(OpenGL)
  IF(OPENGL_FOUND)
    MESSAGE("OpenGL found")
    INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
    #TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OPENGL_LIBRARIES})
  ELSE(OPENGL_FOUND)
    MESSAGE(FATAL_ERROR "OpenGL not found")
ENDIF(OPENGL_FOUND)

FIND_PACKAGE(GLUT)
  IF(GLUT_FOUND)
    MESSAGE("GLUT found")
    INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
    #TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${GLUT_LIBRARIES})
  ELSE(GLUT_FOUND)
    MESSAGE(FATAL_ERROR "GLUT not found")
ENDIF(GLUT_FOUND)

include_directories(
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
  include
)

# Describe catkin Project
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy roslib std_msgs
)

#ADD_LIBRARY(config STATIC src/libs/config.cc)
ADD_LIBRARY(render_common STATIC src/libs/render_common.cc)


##----------------------------------------------------------executables----------------------------------------------------------##

##############################
#      UV Sphere Render      #
##############################

#UV Sphere Render for 6D template generation

add_executable(uv_render_6d_original src/UV_Sphere_Render_6D_original.cpp)
target_link_libraries(uv_render_6d_original ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} config render_common)

add_executable(uv_render_6d_minimum src/UV_Sphere_Render_6D_minimum.cpp)
target_link_libraries(uv_render_6d_minimum ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} config render_common)

add_executable(uv_render_6d_WxH src/UV_Sphere_Render_6D_WxH.cpp)
target_link_libraries(uv_render_6d_WxH ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} config render_common)



##############################
#     ICO Sphere Render      #
##############################

#ICO Sphere Render for 6D template generation

add_executable(ico_render_6d_original src/ICO_Sphere_Render_6D_original.cpp src/libs/icosphere.cc)
target_link_libraries(ico_render_6d_original ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} config render_common)

add_executable(ico_render_6d_minimum src/ICO_Sphere_Render_6D_minimum.cpp src/libs/icosphere.cc)
target_link_libraries(ico_render_6d_minimum ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} config render_common)

add_executable(ico_render_6d_WxH src/ICO_Sphere_Render_6D_WxH.cpp src/libs/icosphere.cc)
target_link_libraries(ico_render_6d_WxH ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} config render_common)



##############################
#     OCT Sphere Render      #
##############################

#OCT Sphere Render for 6D template generation

add_executable(oct_render_6d_original src/OCT_Sphere_Render_6D_original.cpp src/libs/octsphere.cc)
target_link_libraries(oct_render_6d_original ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} config render_common)



##############################
#     Random Generation      #
##############################

#Random Positon test image generation

add_executable(random_generation src/Random_Generation.cpp)
target_link_libraries(random_generation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} config render_common)

add_executable(random_generation_per_layer src/Random_Generation_Per_Layer.cpp)
target_link_libraries(random_generation_per_layer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} config render_common)