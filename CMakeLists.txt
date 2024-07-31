cmake_minimum_required(VERSION 3.5)
project(gndlo)

## Set CXX optimization flags
##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++11")       For debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -Wall -Wextra")
## -mtune=native -march=native

set(CMAKE_CXX_STANDARD 14) # We require C++ 14 or later
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")
## Set build type to Release by default
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE) # set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif(NOT CMAKE_BUILD_TYPE)
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
else(CMAKE_BUILD_TYPE STREQUAL "Release")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
endif(CMAKE_BUILD_TYPE STREQUAL "Release")
message(STATUS "Flags: " ${CMAKE_CXX_FLAGS})

if(1)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif(1)


# ROS
# --------------------------------------
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV REQUIRED) ###
find_package(rosidl_default_generators REQUIRED) ###
find_package(pcl_conversions REQUIRED) ###
find_package(PCL REQUIRED)###
find_package(transformer REQUIRED)

# DEPENDENCIES
# --------------------------------------
#Looking for OpenCV
find_package( OpenCV REQUIRED )

#Looking for Eigen
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

#Looking for Ceres
find_package(Ceres REQUIRED)


# INCLUDE
# --------------------------------------
include_directories(
 include/${PROJECT_NAME}
)

### PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# CREATE LIBRARY
# --------------------------------------
file( GLOB LIB_SOURCES
${PROJECT_SOURCE_DIR}/src/Gndlo.cpp
${PROJECT_SOURCE_DIR}/src/Gndlo_Lidar.cpp
${PROJECT_SOURCE_DIR}/src/Gndlo_selection.cpp
${PROJECT_SOURCE_DIR}/src/Gndlo_planarity.cpp
${PROJECT_SOURCE_DIR}/src/Gndlo_solution.cpp
${PROJECT_SOURCE_DIR}/src/Gndlo_features.cpp
${PROJECT_SOURCE_DIR}/src/Gndlo_ground.cpp)

file( GLOB LIB_HEADERS ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h )


### CREATE THE PATCHES MESSAGE
# --------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Patches.msg"
  DEPENDENCIES std_msgs ### std_msgs/Header
)


# CREATE EXECUTABLES
# --------------------------------------
set(ODOM_EXECUTABLE "${PROJECT_NAME}_odom")
add_executable(${ODOM_EXECUTABLE} ${PROJECT_SOURCE_DIR}/src/main/Gndlo_main.cpp ${LIB_SOURCES} ${LIB_HEADERS})
add_executable(validator ${PROJECT_SOURCE_DIR}/src/Gndlo_transformation_validation.cpp)
add_executable(transformation ${PROJECT_SOURCE_DIR}/src/Gndlo_image_formation.cpp)
set_target_properties(${ODOM_EXECUTABLE} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# TARGET AND LINK
# --------------------------------------
#Target msgs
ament_target_dependencies( ${ODOM_EXECUTABLE}
		rclcpp
		sensor_msgs
		std_msgs
		geometry_msgs
		nav_msgs
		message_filters
		cv_bridge
		transformer)
		
ament_target_dependencies(validator
		rclcpp
		sensor_msgs
		std_msgs
		geometry_msgs
		nav_msgs
		message_filters
		cv_bridge
		Eigen3
		OpenCV
		pcl_conversions
		)
ament_target_dependencies(transformation
		rclcpp
		sensor_msgs
		std_msgs
		geometry_msgs
		nav_msgs
		message_filters
		cv_bridge
		Eigen3
		OpenCV
		transformer
		)
		
##Target custom messages
rosidl_get_typesupport_target(${ODOM_EXECUTABLE} ${PROJECT_NAME} "rosidl_typesupport_cpp")


target_link_libraries(${ODOM_EXECUTABLE} ${PROJECT_NAME}__rosidl_typesupport_cpp)


#Use OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${ODOM_EXECUTABLE} ${OpenCV_LIBS} )

#Use Eigen
ament_target_dependencies( ${ODOM_EXECUTABLE} Eigen3)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

include_directories(include ${EIGEN3_INCLUDE_DIR})

#Use Ceres
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(${ODOM_EXECUTABLE} ${CERES_LIBRARIES})
###
target_link_libraries(transformation ${PROJECT_NAME}__rosidl_typesupport_cpp ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(validator ${PROJECT_NAME}__rosidl_typesupport_cpp ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})


# INSTALL
# --------------------------------------
install(TARGETS ${ODOM_EXECUTABLE}
	ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
###
install(TARGETS transformation DESTINATION lib/${PROJECT_NAME})
install(TARGETS validator DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


# PACKAGE
# --------------------------------------
ament_package()
