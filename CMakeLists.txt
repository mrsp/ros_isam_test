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
catkin_package(CATKIN_DEPENDS)


find_package(Eigen3 REQUIRED)


include_directories(include 
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(isam_ros_test src/isam_ros_test.cpp src/Isam.cpp )

target_link_libraries(isam_ros_test
    ${catkin_LIBRARIES}  
    isam cholmod   ${Eigen3_LIBRARIES} 
)