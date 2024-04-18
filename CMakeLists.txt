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
    # tf2
    tf2_ros
    # tf2_eigen
    nav_msgs
    # tf2_geometry_msgs
    visualization_msgs
    std_srvs

    romea_common_utils
    romea_path_msgs
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(romea_core_common REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(romea_common REQUIRED)
# find_package(romea_common_utils REQUIRED)
# find_package(romea_path_msgs REQUIRED)
# find_package(romea_path_utils REQUIRED)
# find_package(romea_core_path REQUIRED)
# find_package(romea_core_path_matching REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(diagnostic_updater REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${romea_core_common_INCLUDE_DIRS}
)

# ---------------- ament ----------------
add_library(${PROJECT_NAME} SHARED
  src/path_matching_base.cpp
  # src/path_matching.cpp
  src/path_matching_display.cpp
  src/path_matching_display_base.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  romea_core_common::romea_core_common
)


# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   rclcpp_components
#   rclcpp_lifecycle
#   nav_msgs
#   romea_common
#   romea_common_utils
#   romea_path_msgs
#   romea_path_utils
#   romea_core_path
#   romea_core_path_matching
#   tf2_ros
#   visualization_msgs
#   std_srvs
#   diagnostic_updater
# )
#
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# )

# ---------------- catkin ----------------
# include_directories(${catkin_INCLUDE_DIRS})
#
# add_library(path_matching_nodelet
#     src/path_matching.cpp
#     src/path_matching_base.cpp
#     src/path_matching_diagnostic.cpp
#     src/path_matching_diagnostic_base.cpp
#     src/path_matching_display.cpp
#     src/path_matching_display_base.cpp
#     src/path_matching_tf.cpp
#     src/path_matching_nodelet.cpp
#     src/path_comparator.cpp
#     src/uturn_generator.cpp
#     )
#
# add_library(on_the_fly_path_matching_nodelet
#     src/path_matching_base.cpp
#     src/path_matching_diagnostic_base.cpp
#     src/path_matching_display_base.cpp
#     src/on_the_fly_path_matching.cpp
#     src/on_the_fly_path_matching_diagnostic.cpp
#     src/on_the_fly_path_matching_display.cpp
#     src/on_the_fly_path_matching_nodelet.cpp
#     )
#
# add_executable(path_matching_node src/path_matching_node.cpp)
# add_executable(on_the_fly_path_matching_node src/on_the_fly_path_matching_node.cpp)
#
# add_library(path_matching_fsm_plugin
#   src/fsm_plugin.cpp
#   src/fsm_plugin.hpp
# )
#
# target_link_libraries(path_matching_node ${catkin_LIBRARIES})
# target_link_libraries(path_matching_nodelet ${catkin_LIBRARIES})
# target_link_libraries(path_matching_fsm_plugin ${catkin_LIBRARIES})
#
# target_link_libraries(on_the_fly_path_matching_node ${catkin_LIBRARIES})
# target_link_libraries(on_the_fly_path_matching_nodelet ${catkin_LIBRARIES})



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
