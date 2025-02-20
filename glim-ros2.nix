{ buildRosPackage, ament-cmake, rclcpp, rclcpp-components, sensor-msgs, ament-cmake-ros, ament-lint-auto, ament-lint-common, ament-cmake-auto, ros-environment, launch-ros, glim, src, image-transport, ament-index-cpp, cv-bridge, tf2-ros, nav-msgs, gtsam_pkg }:
buildRosPackage {
  pname = "glim_ros2";
  version = "0.0.1a";
  inherit src;
  buildType = "ament_cmake";
  propagatedBuildInputs = [ ament-cmake-ros rclcpp sensor-msgs image-transport rclcpp-components ament-cmake ros-environment launch-ros glim ament-index-cpp cv-bridge tf2-ros nav-msgs gtsam_pkg ];
  checkInputs = [ ament-lint-auto ament-lint-common ];
  nativeBuildInputs = [ ament-cmake-ros ament-cmake ament-cmake-auto ];

  meta = {
    description = "glim ros2";
  };
}