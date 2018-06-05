cmake_minimum_required(VERSION 2.8.3)
project(mbes_mapper)

# check c++14 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
     add_compile_options(-std=c++14 -Wall -Wextra)
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++11")
     add_compile_options(-std=c++11 -Wall -Wextra)
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    tf
    tf_conversions
    pcl_ros
    pcl_conversions
    laser_geometry
    std_msgs
    geometry_msgs
)

add_message_files(FILES
  PoseArrayWithCovariances.msg
)

#find_package(OpenCV REQUIRED)

#find_package(PCL REQUIRED)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbes_mapper
  CATKIN_DEPENDS pcl_ros roscpp tf message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
add_library(ping_object
    src/mbes_ping.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mbes_pcl_receptor
    src/mbes_receptor_node.cpp
    src/mbes_receptor.cpp
    )

add_dependencies(mbes_pcl_receptor
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(mbes_pcl_receptor
    ping_object
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )
