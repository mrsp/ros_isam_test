cmake_minimum_required(VERSION 2.8.3)
project(isam_ros_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)
set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3")
set(TOON_INCLUDE_PATHS "$ENV{HOME}/src/TooN")
catkin_package(CATKIN_DEPENDS)


find_package(Eigen3 REQUIRED)
find_package(CUDA)
find_package(OpenCV)

include_directories(include 
  src
  ${TOON_INCLUDE_PATHS}
  ${catkin_INCLUDE_DIRS}
  ${TOON_INCLUDE_PATHS}  
  ${EIGEN3_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)

add_executable(isam_ros_test src/isam_ros_test.cpp src/Isam.cpp src/utils.cpp)


target_link_libraries(isam_ros_test
    ${catkin_LIBRARIES}  
    ${CUDA_LIBRARIES}
    isam cholmod  ${OpenCV_LIBS}  ${Eigen3_LIBRARIES}
)