cmake_minimum_required(VERSION 3.2)
project(romea_path_matching)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    roslib
    nodelet
    tf2_ros
    nav_msgs
    visualization_msgs
    std_srvs

    romea_common_utils
    romea_path_msgs
    romea_path_utils
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(romea_core_common REQUIRED)
find_package(romea_core_path REQUIRED)
find_package(romea_core_path_matching REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${romea_core_common_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/path_matching_display.cpp
  src/path_matching_display_base.cpp
  src/path_matching_base.cpp
  src/path_matching.cpp
  src/path_matching_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  romea_core_common::romea_core_common
  romea_core_path::romea_core_path
  romea_core_path_matching::romea_core_path_matching
  ${catkin_LIBRARIES}
)

add_executable(path_matching_node
  src/path_matching_node.cpp
)

target_link_libraries(path_matching_node PRIVATE
  ${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()
