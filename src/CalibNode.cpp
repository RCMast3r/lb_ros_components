#include <CalibNode.hpp>

// [[1.16315864e+03 0.00000000e+00 2.80054226e+02]
// [0.00000000e+00 1.13928846e+03 1.86520649e+02]
// [0.00000000e+00 0.00000000e+00 1.00000000e+00]]


lidar_bike_calibration::CalibNode::CalibNode(const rclcpp::NodeOptions & options) : rclcpp::Node("calib_node", options)
{
    // Declare "init time" parameters
    this->declare_parameter<std::string>("pointcloud_topic", "/points");
    this->declare_parameter<std::string>("image_topic", "/image_raw");
    
    this->declare_parameter<std::vector<double>>("distortion_coefficients", _distortion_params);
    this->declare_parameter<std::vector<double>>("camera_matrix_row_major", _cam_matrix_params);
    this->declare_parameter<std::vector<double>>("tf_params", _tf_params); // x y z r p y
    this->declare_parameter<std::string>("image_frame", "image"); 
    this->declare_parameter<std::string>("lidar_frame", "ouster"); 
    
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
    this->get_parameter("tf_params", _tf_params);
    this->get_parameter("image_frame", _image_frame);
    this->get_parameter("lidar_frame", _lidar_frame);
    auto cb = [this](const rclcpp::Parameter & p) {

        if(p.get_name() == std::string("distortion_coefficients")) {
            _distortion_params = p.get_value<rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY>();
            for(auto param : _distortion_params)
            {
                std::cout << param <<std::endl;
            }
        } else if(p.get_name() == std::string("camera_matrix_row_major"))
        {
            _cam_matrix_params = p.get_value<rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY>();

            for(auto param : _distortion_params)
            {
                std::cout << param <<std::endl;
            }
        } else if(p.get_name() == std::string("tf_params"))
        {
            _tf_params = p.get_value<rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY>();
        }
      RCLCPP_INFO(
        this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s",
        p.get_name().c_str(),
        p.get_type_name().c_str());
    };
  _cb_handle = _param_subscriber->add_parameter_callback("distortion_coefficients", cb);
  _cb_handle_2 = _param_subscriber->add_parameter_callback("camera_matrix_row_major", cb);
    _tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void lidar_bike_calibration::CalibNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = msg->header.stamp;
    t.header.frame_id = _lidar_frame;
    t.child_frame_id = _image_frame;

    t.transform.translation.x = _tf_params[0];
    t.transform.translation.y = _tf_params[1];
    t.transform.translation.z = _tf_params[2];

    tf2::Quaternion quat;
    quat.setRPY(_tf_params[3], _tf_params[4], _tf_params[5]);

    t.transform.rotation.x = quat.x();
    t.transform.rotation.y = quat.y();
    t.transform.rotation.z = quat.z();
    t.transform.rotation.w = quat.w();

    _tf_static_broadcaster->sendTransform(t);
}

void lidar_bike_calibration::CalibNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Example Euler angles
    // double ai = 0.0, aj = 0.0, ak = 0.0;
    // Eigen::Quaterniond q = _quat_from_euler(_tf_params[3], _tf_params[4], _tf_params[5]);

    // // Example translation
    // Eigen::Vector3d t(_tf_params[0], _tf_params[1], _tf_params[2]);

    // // Convert ROS PointCloud2 to PCL PointCloud
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_raw(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*msg, *pcl_raw);

    // // Allocate or reset your member variable
    // _cur_pntcld = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // _cur_pntcld->header = pcl_raw->header;
    // _cur_pntcld->is_dense = pcl_raw->is_dense;
    // _cur_pntcld->width = pcl_raw->width;
    // _cur_pntcld->height = pcl_raw->height;

    // // Resize and transform each point
    // _cur_pntcld->points.resize(pcl_raw->points.size());

    // for (std::size_t i = 0; i < pcl_raw->points.size(); ++i)
    // {
    //     const auto& pt_in = pcl_raw->points[i];
    //     Eigen::Vector3d p(pt_in.x, pt_in.y, pt_in.z);
    //     Eigen::Vector3d p_transformed = q * p + t;

    //     auto& pt_out = _cur_pntcld->points[i];
    //     pt_out.x = static_cast<float>(p_transformed.x());
    //     pt_out.y = static_cast<float>(p_transformed.y());
    //     pt_out.z = static_cast<float>(p_transformed.z());
    // }
    // RCLCPP_INFO(this->get_logger(), "pointcloud cb");
}

// roll, pitch, yaw
Eigen::Quaterniond lidar_bike_calibration::CalibNode::_quat_from_euler(double ai, double aj, double ak)
{
    ai /= 2.0;
    aj /= 2.0;
    ak /= 2.0;
    auto ci = std::cos(ai);
    auto si = std::sin(ai);
    auto cj = std::cos(aj);
    auto sj = std::sin(aj);
    auto ck = std::cos(ak);
    auto sk = std::sin(ak);
    auto cc = ci * ck;
    auto cs = ci * sk;
    auto sc = si * ck;
    auto ss = si * sk;

    // This matches the scalar-last ordering (x, y, z, w)
    double x = cj * sc - sj * cs;
    double y = cj * ss + sj * cc;
    double z = cj * cs - sj * sc;
    double w = cj * cc + sj * ss;

    return Eigen::Quaterniond(w, x, y, z);  // Eigen uses w, x, y, z ordering
}
