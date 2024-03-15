cmake_minimum_required(VERSION 3.0.2)
project(cloud_undistortion)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  eigen_conversions
  pcl_conversions
  pcl_ros
  tf2
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
	    ${EIGEN3_INCLUDE_DIR}
	    ${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs 
  pcl_conversions pcl_ros
  eigen_conversions
  DEPENDS PCL 
  INCLUDE_DIRS include
)

add_executable(cloud_undistortion src/cloud_dedistortion.cpp src/data_process.cpp src/gyr_int.cpp)

target_link_libraries(cloud_undistortion ${catkin_LIBRARIES} ${PCL_LIBRARIES})


