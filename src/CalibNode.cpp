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
}

void lidar_bike_calibration::CalibNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "image cb");
    auto mb = cv_bridge::toCvCopy((*msg), msg->encoding);
    cv::Mat image_test = mb->image;
    cv::imshow("Display window 2", image_test);
    if(_cur_pntcld)
    {
    //     Eigen::Matrix<float, 3, 4> transform;
    // transform << -0.103133168138869, -0.994667367582627, 0.000614406939186479, 0.0230245696651290,
    //     0.0738907892845924, -0.00827743798191837, -0.997231986690838, -0.0939116569250688,
    //     0.991919200786304, -0.102802295143166, 0.0743504352694564, 0.103364651051617;
    Eigen::Matrix3f calib;
    calib << _cam_matrix_params[0], _cam_matrix_params[1], _cam_matrix_params[2],
             _cam_matrix_params[3], _cam_matrix_params[4], _cam_matrix_params[5],
             _cam_matrix_params[6], _cam_matrix_params[7], _cam_matrix_params[8];
             
    std::vector<cv::Point2d> cameraFramePoints;

    auto mat_bridged = cv_bridge::toCvCopy((*msg), msg->encoding);

    cv::Mat image = mat_bridged->image;
    for (auto p : (_cur_pntcld->points) )
    {
      Eigen::Vector3f e_point(p.x, p.y, p.z);

      Eigen::Vector3f e_point_inv = calib * e_point; // point should already be transformed
      cv::Point2d cv_point(e_point_inv(0) / e_point_inv(2), e_point_inv(1) / e_point_inv(2));
      cameraFramePoints.push_back(cv_point);
    }
    for (auto p : cameraFramePoints)
    {
      if (p.x == std::clamp(p.x, (double)0.0, (double)1920.0))
      {
        if (p.y == std::clamp(p.y, (double)0.0, (double)1080.0))
        {
          cv::circle(image, cv::Point(p.x, p.y), 5, cv::Scalar(255, 255, 255), cv::FILLED, 8, 0);
        }
      }
    }

    cv::imshow("Display window", image);


        // TODO transform into image frame
        // TODO display points on image with opencv
    }
}

void lidar_bike_calibration::CalibNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Example Euler angles
    // double ai = 0.0, aj = 0.0, ak = 0.0;
    Eigen::Quaterniond q = _quat_from_euler(_tf_params[3], _tf_params[4], _tf_params[5]);

    // Example translation
    Eigen::Vector3d t(_tf_params[0], _tf_params[1], _tf_params[2]);

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_raw);

    // Allocate or reset your member variable
    _cur_pntcld = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    _cur_pntcld->header = pcl_raw->header;
    _cur_pntcld->is_dense = pcl_raw->is_dense;
    _cur_pntcld->width = pcl_raw->width;
    _cur_pntcld->height = pcl_raw->height;

    // Resize and transform each point
    _cur_pntcld->points.resize(pcl_raw->points.size());

    for (std::size_t i = 0; i < pcl_raw->points.size(); ++i)
    {
        const auto& pt_in = pcl_raw->points[i];
        Eigen::Vector3d p(pt_in.x, pt_in.y, pt_in.z);
        Eigen::Vector3d p_transformed = q * p + t;

        auto& pt_out = _cur_pntcld->points[i];
        pt_out.x = static_cast<float>(p_transformed.x());
        pt_out.y = static_cast<float>(p_transformed.y());
        pt_out.z = static_cast<float>(p_transformed.z());
    }
    RCLCPP_INFO(this->get_logger(), "pointcloud cb");
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
