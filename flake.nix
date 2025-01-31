{
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default my-ros-overlay ];
        };

        jazzy_ros_packages = with pkgs.rosPackages.jazzy; [ ament-cmake geometry-msgs launch launch-ros ouster-sensor-msgs pcl-conversions pcl-ros rclcpp rclcpp-components rclcpp-lifecycle rosidl-default-runtime sensor-msgs std-msgs std-srvs tf2-ros ];
        my_overlay = final: prev: {
          lidar-bike-components = final.callPackage ./default.nix { };
          ouster-ros = prev.ouster-ros.overrideAttrs (finalAttrs: previousAttrs: {
            buildType = "ament_cmake";
            buildInputs = with pkgs; [ libtins spdlog rosPackages.jazzy.ament-cmake eigen pcl rosPackages.jazzy.rosidl-default-generators rosPackages.jazzy.tf2-eigen ] ++ jazzy_ros_packages;
            checkInputs = [ pkgs.gtest ];
            propagatedBuildInputs = with pkgs; [ curl jsoncpp ] ++ jazzy_ros_packages;
            nativeBuildInputs = with pkgs.rosPackages.jazzy; [ ament-cmake rosidl-default-generators ];
            src = pkgs.fetchurl {
              url = "https://github.com/ros2-gbp/ouster-ros-release/archive/release/jazzy/ouster_ros/0.13.2.tar.gz";
              name = "0.13.2.tar.gz";
              sha256 = "sha256-TEO7xqCYxkDCcXejx0qV/sSL1VQccntUI5+q2KtjOJA=";
            };
          });
        };

        my-ros-overlay = final: prev: {
          rosPackages = prev.rosPackages // { jazzy = prev.rosPackages.jazzy.overrideScope my_overlay; };
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "lidar-bike-env";
          packages = [
            pkgs.colcon
            # ... other non-ROS packages
            (with pkgs.rosPackages.jazzy; buildEnv {
              paths = [
                ros-core
                ros-base
                rclcpp-components
                sensor-msgs
                ouster-ros
                rosbag2-storage-mcap
                ouster-ros
                rmw-cyclonedds-cpp
                ublox
                lidar-bike-components
                ament-cmake
                ament-cmake-core
                ament-cmake-ros
                nmea-navsat-driver
              ];
            })
          ];
        };

        packages = pkgs;
        legacyPackages =
          import nixpkgs {
            inherit system;
            overlays = [
              nix-ros-overlay.overlays.default
              my-ros-overlay
            ];
          };
      });

}
