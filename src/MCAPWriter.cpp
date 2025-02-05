#include <MCAPWriter.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

mcap_writer_component::MCAPRecorder::MCAPRecorder(const rclcpp::NodeOptions & options) : rclcpp::Node("mcap_recorder", options)
{
    // Declare parameters
    this->declare_parameter<std::string>("pointcloud_topic", "/points");
    this->declare_parameter<std::string>("imu_topic", "/imu");
    this->declare_parameter<std::string>("image_topic", "/image_raw");
    this->declare_parameter<std::string>("output_base", "/media/data");

    // Get parameters
    _pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    _imu_topic = this->get_parameter("imu_topic").as_string();
    _camera_topic = this->get_parameter("image_topic").as_string();
    output_file_ = this->get_parameter("output_base").as_string();

    // Initialize the ROS 2 subscriptions
    subscription_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        _pointcloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&MCAPRecorder::pointcloud_callback, this, std::placeholders::_1));
    
    subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        _imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&MCAPRecorder::imu_callback, this, std::placeholders::_1));
    
    subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        _camera_topic, rclcpp::QoS(10),
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

    RCLCPP_INFO(this->get_logger(), "Recording topics: '%s', '%s', '%s' to '%s'",
                _pointcloud_topic.c_str(), _imu_topic.c_str(), _camera_topic.c_str(), output_file_.c_str());
}

void mcap_writer_component::MCAPRecorder::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    writer_->write(*msg, _pointcloud_topic, now());
    RCLCPP_DEBUG(this->get_logger(), "Recorded PointCloud2 message");
}

void mcap_writer_component::MCAPRecorder::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    writer_->write(*msg, _imu_topic, now());
    RCLCPP_DEBUG(this->get_logger(), "Recorded IMU message");
}

void mcap_writer_component::MCAPRecorder::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    writer_->write(*msg, _camera_topic, now());
    RCLCPP_DEBUG(this->get_logger(), "Recorded Image message");
}

mcap_writer_component::MCAPRecorder::~MCAPRecorder()
{
    // Ensure rosbag2 writer cleanup
    if (writer_)
    {
        writer_.reset();
    }
    RCLCPP_INFO(this->get_logger(), "Shutting down MCAPRecorder");
}
