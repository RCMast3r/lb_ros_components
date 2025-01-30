cmake_minimum_required(VERSION 3.12)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(ouster_ros REQUIRED)
# https://docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html
add_library(mcap_writer_component SHARED src/MCAPWriter.cpp)

ament_target_dependencies(mcap_writer_component PUBLIC rosbag2_cpp rclcpp sensor_msgs rclcpp_components)

target_include_directories(mcap_writer_component PUBLIC include)

rclcpp_components_register_nodes(mcap_writer_component "MCAPRecorder")

ament_export_targets(export_mcap_writer_component)
install(TARGETS mcap_writer_component
        EXPORT export_mcap_writer_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)