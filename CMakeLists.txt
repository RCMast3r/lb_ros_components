cmake_minimum_required(VERSION 3.12)
project(mcap_writer_component)

find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(ouster_ros REQUIRED)
# https://docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html
add_library(mcap_writer_component SHARED src/MCAPWriter.cpp)

ament_target_dependencies(mcap_writer_component PUBLIC rosbag2_cpp rclcpp sensor_msgs rclcpp_components)

target_include_directories(mcap_writer_component PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# rclcpp_components_register_node(mcap_writer_component "MCAPRecorder::mcap_boi")

rclcpp_components_register_node(mcap_writer_component
  PLUGIN "mcap_writer_component::MCAPRecorder"
  EXECUTABLE mcap_recorder
)


install(TARGETS mcap_writer_component
        EXPORT export_mcap_writer_component
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
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_include_directories(include) 
ament_export_dependencies(rosidl_default_runtime)
ament_export_targets(export_mcap_writer_component)
ament_package()