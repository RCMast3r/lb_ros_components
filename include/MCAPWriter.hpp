#ifndef __MCAPWRITER_H__
#define __MCAPWRITER_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

class MCAPRecorder : public rclcpp::Node
{
public:
    MCAPRecorder(const rclcpp::NodeOptions & options);

    ~MCAPRecorder();
    

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
    // Parameters
    std::string topic_;
    std::string output_file_;

    // ROS 2 entities
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    // Rosbag2 writer
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MCAPRecorder)

#endif // __MCAPWRITER_H__