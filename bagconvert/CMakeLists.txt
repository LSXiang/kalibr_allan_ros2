cmake_minimum_required(VERSION 3.5)

# Project name
project(bagconvert)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Matlab REQUIRED
    COMPONENTS
    MAIN_PROGRAM
    ENG_LIBRARY
    MAT_LIBRARY
    MX_LIBRARY)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3  -Wall")

# Include our header files
include_directories(
    src
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${Matlab_INCLUDE_DIRS})

# Set link libraries used by all binaries
list(APPEND parser_libraries
    ${Boost_LIBRARIES}
    ${Matlab_LIBRARIES}
    ${Matlab_MX_LIBRARY}
    ${Matlab_MEX_LIBRARY}
    ${Matlab_MAT_LIBRARY}
    ${Matlab_ENG_LIBRARY})

##################################################
# Make binary for the offline reader
##################################################
add_executable(bagconvert
    src/main.cc)
ament_target_dependencies(bagconvert
    rclcpp
    rcpputils
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    rosbag2_cpp)
target_link_libraries(bagconvert 
    ${parser_libraries})

# Install nodes
install(
    TARGETS bagconvert
    DESTINATION lib/${PROJECT_NAME})

ament_package()
