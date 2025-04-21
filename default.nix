{ buildRosPackage, ament-cmake, rclcpp, rclcpp-components, sensor-msgs, ament-cmake-ros, ament-lint-auto, ament-lint-common, ros-environment, ouster-ros, rosbag2-cpp, launch-ros, v4l2-camera, httplib, pcl, perception-pcl, cv-bridge}:
buildRosPackage {
  pname = "lidar-bike-components";
  version = "0.0.1a";
  src = ./.;

  buildType = "ament_cmake";
  propagatedBuildInputs = [ ament-cmake-ros rclcpp sensor-msgs rclcpp-components ament-cmake ros-environment ouster-ros rosbag2-cpp launch-ros v4l2-camera httplib pcl perception-pcl cv-bridge ];
  checkInputs = [ ament-lint-auto ament-lint-common ];
  nativeBuildInputs = [ ament-cmake-ros ament-cmake ];

  meta = {
    description = "ros2 components for the lidar bike project";
  };
}