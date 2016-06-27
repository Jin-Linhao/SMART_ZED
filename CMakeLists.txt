cmake_minimum_required(VERSION 2.8.7)

project(zed_smart)
###############################################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
###############################################################################

find_package(OpenCV 2.4 COMPONENTS core highgui imgproc REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  rosconsole
  sensor_msgs
  cv_bridge
  dynamic_reconfigure
)




catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    sensor_msgs
    opencv2
    cv_bridge
    image_transport
    dynamic_reconfigure
)

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

###############################################################################

###############################################################################
# EXECUTABLE

add_definitions(-std=c++11)# -m64) #-Wall)


add_executable(
  zed_smart
  src/img_converter.cpp
)


target_link_libraries(
  zed_smart
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}  
    )

add_executable(
  depth_viewer_node
  src/depth_viewer.cpp
)
target_link_libraries(
  depth_viewer_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}  
    )



#add_dependencies(zed_wrapper_node ${PROJECT_NAME}_gencfg)
###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
