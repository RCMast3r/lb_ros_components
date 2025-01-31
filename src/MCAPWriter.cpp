#include <MCAPWriter.hpp>

mcap_writer_component::MCAPRecorder::MCAPRecorder(const rclcpp::NodeOptions & options) : rclcpp::Node("mcap_recorder", options)
{
    // Declare parameters
    this->declare_parameter<std::string>("topic", "/pointcloud");
    this->declare_parameter<std::string>("output_file", "pointcloud_recording");

    // Get parameters
    topic_ = this->get_parameter("topic").as_string();
    output_file_ = this->get_parameter("output_file").as_string();

    // Initialize the ROS 2 subscription
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_, rclcpp::QoS(10), std::bind(&MCAPRecorder::pointcloud_callback, this, std::placeholders::_1));

    // Set up the rosbag2 writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = output_file_;
    storage_options.storage_id = "mcap";  // Ensure MCAP format
    writer_->open(storage_options);

    // Add topic to the bag with metadata
    rosbag2_storage::TopicMetadata writer_topic;
    writer_topic.name = topic_;
    writer_topic.type = "sensor_msgs/msg/PointCloud2";
    writer_topic.serialization_format = "cdr";
    writer_->create_topic(writer_topic);

    RCLCPP_INFO(this->get_logger(), "Recording PointCloud2 topic '%s' to '%s'", topic_.c_str(), output_file_.c_str());
}

void mcap_writer_component::MCAPRecorder::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    writer_->write(*msg, "/pointcloud", now());
    RCLCPP_DEBUG(this->get_logger(), "Recorded PointCloud2 message");
}

mcap_writer_component::MCAPRecorder::~MCAPRecorder()
{
    // Ensure rosbag2 writer cleanup
    if (writer_)
    {
        writer_.reset();
    }
    RCLCPP_INFO(this->get_logger(), "Shutting down PointCloudRecorder");
}

