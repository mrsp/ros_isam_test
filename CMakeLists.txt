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
SET( CUDA_PROPAGATE_HOST_FLAGS OFF )
find_package(CUDA)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-Xcompiler;-fPIC;" )

find_package(OpenCV)

set(cuda_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/kernels.cu
    ${CMAKE_CURRENT_LIST_DIR}/src/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/kernelscalls.cu

)

cuda_add_library(isam_ros_test_cuda   STATIC  ${cuda_srcs}  OPTIONS -std=c++11; -Xcompiler; -fPIC; -D_MWAITXINTRIN_H_INCLUDED -D_FORCE_INLINES
    -I${CMAKE_CURRENT_LIST_DIR}/src/
    -I${TOON_INCLUDE_PATHS}
    -I${EIGEN3_INCLUDE_DIR}
)

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
    isam cholmod  ${OpenCV_LIBS}  ${Eigen3_LIBRARIES} isam_ros_test_cuda
)