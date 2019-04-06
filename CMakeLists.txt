#auto-regist no

cmake_minimum_required(VERSION 2.8.3)
project(ros_kinect)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
	cv_bridge
  	roscpp
	sensor_msgs
 	std_msgs
)

find_package(Threads REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_kinect
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "-pthread")
set( OpenCV_DIR "/usr/local/include/opencv2/" )
find_package( OpenCV REQUIRED )

set(CMAKE_MACOSX_RPATH 1)
set(INSTALL_RPATH "/home/migly/catkin_ws/src/ros_kinect")

include_directories(/home/migly/OpenNI-Linux-x64-2.2/Include)
include_directories(/home/migly/NiTE-Linux-x64-2.2/Include)


link_directories(/home/migly/OpenNI-Linux-x64-2.2/Redist)
link_directories(/home/migly/NiTE-Linux-x64-2.2/Redist)


set(myapp_sources src/main.cpp src/HumanTracker.hpp)
add_executable(ros_kinect ${myapp_sources})

target_link_libraries(ros_kinect  ${catkin_LIBRARIES} ${OpenCV_LIBS} OpenNI2 NiTE2 glut GL)
#add_executable(openni_kinect_sample src/openni_kinect_sample.cpp)
#target_link_libraries(openni_kinect_sample  ${catkin_LIBRARIES})
