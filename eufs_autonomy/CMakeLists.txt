cmake_minimum_required(VERSION 3.8)
project(eufs_autonomy)


# Build settings

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Dependencies (ROS)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Optional EUFS interfaces (uncomment if you actually use eufs_msgs in code)
# find_package(eufs_msgs REQUIRED)


# Dependencies (3rd party)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)


# Common include dirs

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})


# Nodes


# GPS localization node
add_executable(gps_localization_node
  src/localization/gps_localization_node.cpp
)
ament_target_dependencies(gps_localization_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros
)
install(TARGETS gps_localization_node DESTINATION lib/${PROJECT_NAME})

# LiDAR cone detector node
add_executable(lidar_cone_detector
  src/perception/lidar_cone_detector.cpp
)
ament_target_dependencies(lidar_cone_detector
  rclcpp sensor_msgs geometry_msgs pcl_conversions
)
target_link_libraries(lidar_cone_detector ${PCL_LIBRARIES})
install(TARGETS lidar_cone_detector DESTINATION lib/${PROJECT_NAME})

# Camera cone verifier node
add_executable(camera_cone_verifier
  src/perception/camera_cone_verifier.cpp
)
ament_target_dependencies(camera_cone_verifier
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge
)
target_link_libraries(camera_cone_verifier ${OpenCV_LIBRARIES})
install(TARGETS camera_cone_verifier DESTINATION lib/${PROJECT_NAME})

# Cone mapper
add_executable(cone_mapper
  src/mapping/cone_mapper.cpp
)
ament_target_dependencies(cone_mapper
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)
install(TARGETS cone_mapper DESTINATION lib/${PROJECT_NAME})

# Centerline generator
add_executable(centerline_generator
  src/planning/centerline_generator.cpp
)
ament_target_dependencies(centerline_generator
  rclcpp geometry_msgs
)
install(TARGETS centerline_generator DESTINATION lib/${PROJECT_NAME})

# Pure pursuit controller
add_executable(pure_pursuit_controller
  src/control/pure_pursuit_controller.cpp
)
ament_target_dependencies(pure_pursuit_controller
  rclcpp geometry_msgs ackermann_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs
)
install(TARGETS pure_pursuit_controller DESTINATION lib/${PROJECT_NAME})


# Install launch & config files

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


# Testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
