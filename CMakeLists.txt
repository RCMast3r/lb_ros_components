cmake_minimum_required(VERSION 3.12)
project(lidar_bike_components)

# ROS deps
# find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui)


find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
# non-ROS deps
find_package(httplib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# https://docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html
add_library(lidar_bike_components SHARED src/MCAPWriter.cpp) 
add_library(lidar_bike_calibration SHARED src/CalibNode.cpp)
set_target_properties(lidar_bike_components PROPERTIES CXX_STANDARD 20)
set_target_properties(lidar_bike_calibration PROPERTIES CXX_STANDARD 20)

ament_target_dependencies(lidar_bike_components PUBLIC rosbag2_cpp rclcpp sensor_msgs rclcpp_components message_filters cv_bridge)
ament_target_dependencies(lidar_bike_calibration PUBLIC rosbag2_cpp rclcpp sensor_msgs rclcpp_components message_filters cv_bridge pcl_conversions tf2 tf2_ros)

target_link_libraries(lidar_bike_components PUBLIC httplib::httplib)

target_link_libraries(lidar_bike_calibration PUBLIC Eigen3::Eigen opencv_highgui opencv_core)

target_include_directories(lidar_bike_components PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_include_directories(lidar_bike_calibration PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(lidar_bike_components
  PLUGIN "lidar_bike_components::MCAPRecorder"
  EXECUTABLE mcap_recorder
)

rclcpp_components_register_node(lidar_bike_calibration
  PLUGIN "lidar_bike_calibration::CalibNode"
  EXECUTABLE calib_node
)

install(TARGETS lidar_bike_components
        EXPORT export_lidar_bike_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS lidar_bike_calibration
        EXPORT export_lidar_bike_calibration
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    include
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_include_directories(include) 
ament_export_dependencies(rosidl_default_runtime)
ament_export_targets(export_lidar_bike_components)
ament_export_targets(export_lidar_bike_calibration)
ament_package()