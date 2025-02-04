{
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    libuvc-cam-src.url = "github:RCMast3r/libuvc_cam";
    libuvc-cam-src.flake = false;
    # ros2-v4l2-camera-src.url = "https://gitlab.com/muzhyk.belarus/ros2_v4l2_camera.git";
    ros2-v4l2-camera-src.url = "gitlab:rcmast3r1/ros2_v4l2_camera/compressed_formats";
    ros2-v4l2-camera-src.flake = false;
  };
  outputs = { self, nix-ros-overlay, nixpkgs, libuvc-cam-src, ros2-v4l2-camera-src }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default my-ros-overlay ouster-ros-override-overlay ];
        };
        test_pkg = (with pkgs.rosPackages.jazzy; buildEnv {
              paths = [
                ros-core
                ros-base
                rclcpp-components
                sensor-msgs
                ouster-ros
                rosbag2-storage-mcap
                ouster-ros
                foxglove-bridge
                rmw-cyclonedds-cpp
                ublox
                lidar-bike-components
                ament-cmake
                ament-cmake-core
                ament-cmake-ros
                nmea-navsat-driver
              ];
            });

        jazzy_ros_packages = with pkgs.rosPackages.jazzy; [ ament-cmake geometry-msgs launch launch-ros ouster-sensor-msgs pcl-conversions pcl-ros rclcpp rclcpp-components rclcpp-lifecycle rosidl-default-runtime sensor-msgs std-msgs std-srvs tf2-ros ];
        
        devshell_overlay = final: prev: {
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
          
            v4l2-camera = prev.v4l2-camera.overrideAttrs (prev: {
              src = ros2-v4l2-camera-src;
              propagatedBuildInputs = prev.propagatedBuildInputs ++ [ pkgs.rosPackages.jazzy.cv-bridge ];
              buildInputs = prev.buildInputs ++ [ pkgs.rosPackages.jazzy.cv-bridge ];
            });
          
        };
        ouster-ros-override-overlay = final: prev: {
          rosPackages = prev.rosPackages // { jazzy = prev.rosPackages.jazzy.overrideScope devshell_overlay; };
        };
        my_overlay = final: prev: {
          lidar-bike-components = final.callPackage ./default.nix { };
          libuvc-cam = final.callPackage ./libuvc_cam.nix {src = libuvc-cam-src; };
        };

        my-ros-overlay = final: prev: {
          rosPackages = prev.rosPackages // { jazzy = prev.rosPackages.jazzy.overrideScope my_overlay; };
        };


      in
      {
        overlays = my-ros-overlay;
        devShells.default = pkgs.mkShell {
          name = "lidar-bike-env";
          packages = [
            pkgs.colcon
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
                libuvc-cam
                foxglove-bridge
                v4l2-camera
              ];
            })
          ];
        };

        packages.asdfasdf = test_pkg;
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
