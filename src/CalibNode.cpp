#include <CalibNode.hpp>

namespace calib_node_component {
lidar_bike_components::CalibNode::CalibNode(const rclcpp::NodeOptions & options) : rclcpp::Node("mcap_recorder", options)
{
    // Declare parameters
    this->declare_parameter<std::string>("pointcloud_topic", "/points");
    this->declare_parameter<std::string>("image_topic", "/image_raw");
    // Get parameters
    _pointcloud_topic = 
    _camera_topic = this->get_parameter("image_topic").as_string();

    // Initialize the ROS 2 subscriptions
    _subscription_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->get_parameter("pointcloud_topic").as_string();, rclcpp::SensorDataQoS(),
        std::bind(&MCAPRecorder::pointcloud_callback, this, std::placeholders::_1));
    
    subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        this->get_parameter("image_topic").as_string(), rclcpp::QoS(10),
        std::bind(&MCAPRecorder::image_callback, this, std::placeholders::_1));
    
    // Set up the rosbag2 writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions storage_options;
    if(! (output_file_.back() == '/'))
    {
        output_file_ +="/";
    }

    storage_options.uri = output_file_ + "lb_rosbag_" + std::to_string(static_cast<int>(now().seconds()));
    storage_options.storage_id = "mcap";  // Ensure MCAP format
    writer_->open(storage_options);

    // Add topics to the bag with metadata
    rosbag2_storage::TopicMetadata writer_topic;
    
    writer_topic.name = _pointcloud_topic;
    writer_topic.type = "sensor_msgs/msg/PointCloud2";
    writer_topic.serialization_format = "cdr";
    writer_->create_topic(writer_topic);
    
    writer_topic.name = _imu_topic;
    writer_topic.type = "sensor_msgs/msg/Imu";
    writer_->create_topic(writer_topic);
    
    writer_topic.name = _camera_topic;
    writer_topic.type = "sensor_msgs/msg/Image";
    writer_->create_topic(writer_topic);

    writer_topic.name = _fix_topic;
    writer_topic.type = "sensor_msgs/msg/NavSatFix";
    writer_->create_topic(writer_topic);
    RCLCPP_DEBUG(this->get_logger(), "Recording topics: '%s', '%s', '%s', '%s', to '%s'",
                _pointcloud_topic.c_str(), _imu_topic.c_str(), _camera_topic.c_str(), _fix_topic.c_str(), output_file_.c_str());

    http_thread_ = std::thread(&MCAPRecorder::start_http_server, this);

}
}