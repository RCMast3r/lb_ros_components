{ buildRosPackage, ament-cmake, rclcpp, rclcpp-components, sensor-msgs, ament-cmake-ros, ament-lint-auto, ament-lint-common, ament-cmake-auto, ros-environment, rosbag2-cpp, launch-ros, image-transport, libuvc, libusb1, src }:
buildRosPackage {
  pname = "libuvc-cam";
  version = "0.0.1a";
  inherit src;

  buildType = "ament_cmake";
  propagatedBuildInputs = [ ament-cmake-ros rclcpp sensor-msgs image-transport rclcpp-components ament-cmake ros-environment rosbag2-cpp launch-ros libuvc libusb1 ];
  checkInputs = [ ament-lint-auto ament-lint-common ];
  nativeBuildInputs = [ ament-cmake-ros ament-cmake ament-cmake-auto ];

  meta = {
    description = "ros2 libuvc-cam";
  };
}