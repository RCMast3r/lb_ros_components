#ifndef __CALIBNODE_H__
#define __CALIBNODE_H__

// TODO
// - [ ] add ros2 parameter live callbacks for updating internal param values
// - [ ] display the image data in an opencv frame
// - [ ] add transformed lidar cloud into the opencv frame

// combine this: https://gitlab.com/KSU_EVT/autonomous-software/lidar-camera-fusion with this
// https://gitlab.com/KSU_EVT/autonomous-software/lidar_camera_tf_pub

// rclcpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

// msg types from sensor_msgs

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>


namespace lidar_bike_calibration {
class CalibNode : public rclcpp::Node
{
public:
    CalibNode(const rclcpp::NodeOptions & options);
    ~CalibNode() = default;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:

    std::vector<double> _distortion_params = {0,0,0,0,0}; // k1, k2, p1, p2, k3
    std::vector<double> _cam_matrix_params = {0,0,0,0,0, 0, 0, 0, 0}; // 3x3 row major fx, 0, cx, 0, fy, cy, 0,0,1


    std::shared_ptr<rclcpp::ParameterEventHandler> _param_subscriber;

    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription_pointcloud;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription_image;

    std::shared_ptr<rclcpp::ParameterCallbackHandle> _cb_handle;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> _cb_handle_2;
};


}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_bike_calibration::CalibNode)
#endif // __CALIBNODE_H__