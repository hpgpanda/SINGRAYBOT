cmake_minimum_required(VERSION 3.0.2)
project(color_points)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  my_custom_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
)
find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(Threads)
find_package(Ceres REQUIRED)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs 
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(colorpoints src/colorpoints.cpp)
add_executable(filter src/filter.cpp)


target_link_libraries(colorpoints
   ${catkin_LIBRARIES}
 )
 target_link_libraries(filter
 ${catkin_LIBRARIES}
)
