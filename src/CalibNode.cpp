#include <CalibNode.hpp>

lidar_bike_calibration::CalibNode::CalibNode(const rclcpp::NodeOptions & options) : rclcpp::Node("calib_node", options)
{
    // Declare "init time" parameters
    this->declare_parameter<std::string>("pointcloud_topic", "/points");
    this->declare_parameter<std::string>("image_topic", "/image_raw");
    this->declare_parameter<std::vector<double>>("distortion_coefficients", _distortion_params);
    this->declare_parameter<std::vector<double>>("camera_matrix_row_major", _cam_matrix_params);
    // Initialize the ROS 2 subscriptions
    _subscription_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->get_parameter("pointcloud_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&CalibNode::pointcloud_callback, this, std::placeholders::_1));
    
    _subscription_image = this->create_subscription<sensor_msgs::msg::Image>(
        this->get_parameter("image_topic").as_string(), rclcpp::QoS(10),
        std::bind(&CalibNode::image_callback, this, std::placeholders::_1));
    
    // Declare "runtime" parameters
    _param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // camera distortion coefficients (5 parameters)
    this->get_parameter("distortion_coefficients", _distortion_params);
    this->get_parameter("camera_matrix_row_major", _cam_matrix_params);


    auto cb = [this](const rclcpp::Parameter & p) {

        if(p.get_name() == std::string("distortion_coefficients")) {
            _distortion_params = p.get_value<rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY>();
        } else if(p.get_name() == std::string("camera_matrix_row_major"))
        {
            _cam_matrix_params = p.get_value<rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY>();
        }
      RCLCPP_INFO(
        this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s",
        p.get_name().c_str(),
        p.get_type_name().c_str());
    };
  _cb_handle = _param_subscriber->add_parameter_callback("distortion_coefficients", cb);
  _cb_handle_2 = _param_subscriber->add_parameter_callback("camera_matrix_row_major", cb);
}

void lidar_bike_calibration::CalibNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{

}
void lidar_bike_calibration::CalibNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

}