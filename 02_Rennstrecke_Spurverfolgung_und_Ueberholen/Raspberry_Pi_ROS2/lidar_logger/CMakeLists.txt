cmake_minimum_required(VERSION 3.8)
project(lidar_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Set OpenCV
set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

#find packages for dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  cv_bridge
  rclcpp
  rclcpp_components
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
)
add_executable(lidar_logger src/lidar_logger.cpp)

ament_target_dependencies(
  lidar_logger "cv_bridge" "rclcpp" "rclcpp_components" "sensor_msgs"
   "stereo_msgs" "std_msgs" "vision_msgs" )
#ament_target_dependencies(parking_node rclcpp sensor_msgs nav_msgs std_msgs cv_bridge)

target_include_directories(lidar_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
  
target_link_libraries(lidar_logger ${OpenCV_LIBS})

target_compile_features(lidar_logger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS lidar_logger
  DESTINATION lib/${PROJECT_NAME})

#--------------LidarClientExecutable----------------
add_executable(lidar_filter src/simple_lidar_filter.cpp)
ament_target_dependencies(lidar_filter
  rclcpp
#  rclcpp_action
  sensor_msgs
  std_msgs
)

install(
     TARGETS lidar_filter
     DESTINATION lib/${PROJECT_NAME}
     )
    
add_executable(obstacle_detector src/obstacle_detector.cpp)
ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  std_msgs
)

install(
     TARGETS obstacle_detector
     DESTINATION lib/${PROJECT_NAME}
     )
     
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
#----------------------------------------------------     
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
