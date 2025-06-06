#include <MCAPWriter.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include <format>

#include <iostream>

lidar_bike_components::MCAPRecorder::MCAPRecorder(const rclcpp::NodeOptions & options) : rclcpp::Node("mcap_recorder", options)
{
    // Declare parameters
    this->declare_parameter<std::string>("pointcloud_topic", "/points");
    this->declare_parameter<std::string>("imu_topic", "/imu");
    this->declare_parameter<std::string>("image_topic", "/image_raw");
    this->declare_parameter<std::string>("output_base", "/media/data");
    this->declare_parameter<std::string>("fix_topic", "/fix");
    // Get parameters
    _pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    _imu_topic = this->get_parameter("imu_topic").as_string();
    _camera_topic = this->get_parameter("image_topic").as_string();
    output_file_ = this->get_parameter("output_base").as_string();
    _fix_topic = this->get_parameter("fix_topic").as_string();

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
    
    _subscription_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        _fix_topic, rclcpp::QoS(10), std::bind(&MCAPRecorder::gps_callback, this, std::placeholders::_1));

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

void lidar_bike_components::MCAPRecorder::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if(_writing)
    {
        writer_->write(*msg, _pointcloud_topic, now());
        RCLCPP_DEBUG(this->get_logger(), "Recorded PointCloud2 message");
    }
    
}

void lidar_bike_components::MCAPRecorder::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    
    if(_writing)
    {
        writer_->write(*msg, _imu_topic, now());
        RCLCPP_DEBUG(this->get_logger(), "Recorded IMU message");
    }
    
}

void lidar_bike_components::MCAPRecorder::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if(_writing)
    {
        writer_->write(*msg, _camera_topic, now());
        RCLCPP_DEBUG(this->get_logger(), "Recorded Image message");
    }
    
}

void lidar_bike_components::MCAPRecorder::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if(_writing)
    {
        writer_->write(*msg, _fix_topic, now());
        RCLCPP_DEBUG(this->get_logger(), "Recorded gps message");
    }
}

void lidar_bike_components::MCAPRecorder::start_http_server()
{
    httplib::Server svr;

    // Serve the webpage with Start/Stop buttons
    svr.Get("/", [this](const httplib::Request&, httplib::Response& res) {

        std::string website_content = 
        R"(<!DOCTYPE html>
            <html>
            <head>
                <title>Recording Control</title>
                <script>
                    function sendRequest(action) {
                        fetch('/' + action, { method: 'GET' })
                        .then(response => response.text())
                        .then(data => document.getElementById('status').innerText = data);
                    }
                </script>
            </head>
            <body>
                <h1>Recording Control</h1>
                <button onclick='sendRequest("start")'>Start Recording</button>
                <button onclick='sendRequest("stop")'>Stop Recording</button>
                <p id='status'>Status: )"+ std::string(_writing ? "recording": "not recording") + 
        R"(</p>
        </body>
        </html>)";
        
        res.set_content(website_content, "text/html");
    });
    
    // Start recording
    svr.Get("/start", [this](const httplib::Request &, httplib::Response &res) {
        handle_start();
        std::cout << "got start" <<std::endl;
        res.set_content("Recording started", "text/plain");
    });

    // Stop recording
    svr.Get("/stop", [this](const httplib::Request &, httplib::Response &res) {
        std::cout << "got stop" <<std::endl;
        handle_stop();
        res.set_content("Recording stopped", "text/plain");
    });

    RCLCPP_DEBUG(this->get_logger(), "HTTP server running on port 8080...");
    svr.listen("0.0.0.0", 8080);
}

void lidar_bike_components::MCAPRecorder::handle_start()
{
    if (!_writing)
    {
        rosbag2_storage::StorageOptions storage_options;
        if(! (output_file_.back() == '/'))
        {
            output_file_ +="/";
        }
        storage_options.uri = output_file_ + "lb_rosbag_" + std::to_string(static_cast<int>(now().seconds()));
        storage_options.storage_id = "mcap";  // Ensure MCAP format
        writer_->open(storage_options);
        _writing = true;
        RCLCPP_DEBUG(this->get_logger(), "Started recording.");
    }
}

void lidar_bike_components::MCAPRecorder::handle_stop()
{
    if (_writing)
    {
        _writing = false;
        writer_->close();
        RCLCPP_DEBUG(this->get_logger(), "Stopped recording.");
    }
}

lidar_bike_components::MCAPRecorder::~MCAPRecorder()
{
    // Ensure rosbag2 writer cleanup
    if (writer_)
    {
        writer_->close();
        writer_.reset();
    }
    server_running_ = false;
    if (http_thread_.joinable())
    {
        http_thread_.join();
    }
    RCLCPP_DEBUG(this->get_logger(), "Shutting down MCAPRecorder");
}
