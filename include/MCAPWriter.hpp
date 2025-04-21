#ifndef __MCAPWRITER_H__
#define __MCAPWRITER_H__

#include <thread>
#include <atomic>

// for http webserver
#include <httplib.h>



// msg types from sensor_msgs
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// rclcpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

// rosbag2 api
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>


namespace lidar_bike_components {

class MCAPRecorder : public rclcpp::Node
{
public:
    MCAPRecorder(const rclcpp::NodeOptions & options);

    ~MCAPRecorder();
    

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
private:
    void start_http_server();
    void handle_start();
    void handle_stop();
    
    std::thread http_thread_;
    std::atomic<bool> _writing{true};
    std::atomic<bool> server_running_{true};

private:
    // Parameters
    std::string _pointcloud_topic, _imu_topic, _camera_topic, _fix_topic;
    std::string output_file_;

    // ROS 2 entities
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _subscription_gps;

    // Rosbag2 writer
    std::unique_ptr<rosbag2_cpp::Writer> writer_;


};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mcap_writer_component::MCAPRecorder)

#endif // __MCAPWRITER_H__