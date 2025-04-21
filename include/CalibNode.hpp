#ifndef __CALIBNODE_H__
#define __CALIBNODE_H__


// rclcpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

// msg types from sensor_msgs

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>


namespace lidar_bike_components {
class CalibNode : public rclcpp::Node
{
    CalibNode(const rclcpp::NodeOptions & options);
    ~CalibNode();

private:
    void _image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void _pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription_pointcloud;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription_image;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_bike_components::CalibNode)
}

#endif // __CALIBNODE_H__