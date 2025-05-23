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
    
    gtsam-points-src = {
      url = "github:RCMast3r/gtsam_points";
      flake = false;
    };
    
    nebs-packages.url = "github:RCMast3r/nebs_packages"; # packages for glim-ros2
    nebs-packages.inputs.gtsam-points-src.follows = "gtsam-points-src";

    
    glim-ros2-src.url = "github:RCMast3r/glim_ros2";
    glim-ros2-src.flake = false;
    nixgl.url = "github:nix-community/nixGL";
  };
  outputs = { self, nix-ros-overlay, nixpkgs, libuvc-cam-src, ros2-v4l2-camera-src, nebs-packages, glim-ros2-src, nixgl, ...}:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default my-ros-overlay ouster-ros-override-overlay nebs-packages.overlays.default nixgl.overlay ];
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
          
        };
        ouster-ros-override-overlay = final: prev: {
          rosPackages = prev.rosPackages // { jazzy = prev.rosPackages.jazzy.overrideScope devshell_overlay; };
        };
        my_overlay = final: prev: {
          v4l2-camera = prev.v4l2-camera.overrideAttrs (prev: {
            src = ros2-v4l2-camera-src;
            propagatedBuildInputs = prev.propagatedBuildInputs ++ [ pkgs.rosPackages.jazzy.cv-bridge ];
              # buildInputs = prev.buildInputs ++ [ pkgs.rosPackages.jazzy.cv-bridge ];
            });
          glim-ros2 = final.callPackage ./glim-ros2.nix {src = glim-ros2-src; };
          lidar-bike-components = final.callPackage ./default.nix { };
          # libuvc-cam = final.callPackage ./libuvc_cam.nix { src = libuvc-cam-src; };
        };

        

        my-ros-overlay = final: prev: {
          rosPackages = prev.rosPackages // { jazzy = prev.rosPackages.jazzy.overrideScope my_overlay; };
        };


      in
      {
        overlays = my-ros-overlay;
        devShells.default = pkgs.mkShell {
          name = "lidar-bike-env";
          # auto-completion yeet
          shellHook = ''
            eval "$(register-python-argcomplete ros2)"
            eval "$(register-python-argcomplete colcon)"
            export GLIM_PATH=${pkgs.glim}
            export GLIM_ROS_PATH=${pkgs.rosPackages.jazzy.glim-ros2}
            sudo sysctl -w net.core.rmem_max=2147483647
          '';
          RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp";
          ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST";
          RMW_CONNEXT_PUBLICATION_MODE="ASYNCHRONOUS";
          CYCLONEDDS_URI="file://config/ddsconfig.xml";
          NIXPKGS_ALLOW_UNFREE=1;
          packages = [
            pkgs.colcon
            pkgs.nixgl.auto.nixGLDefault
            (with pkgs.rosPackages.jazzy; buildEnv {
              paths = [
                rviz2
                ros-core
                ros-base
                rclcpp-components
                sensor-msgs
                ouster-ros
                rosbag2-storage-mcap
                ouster-ros
                rmw-cyclonedds-cpp
                ublox
                ament-cmake
                ament-cmake-core
                ament-cmake-ros
                nmea-navsat-driver
                foxglove-bridge
                v4l2-camera
                lidar-bike-components
                pcl-ros
                glim-ros2
                tf2-ros
                tf2-tools
              ];
            })
            
          ];
        };

        devShells.humble = pkgs.mkShell {
          name = "lidar-bike-env";
          # auto-completion yeet
          shellHook = ''
            eval "$(register-python-argcomplete ros2)"
            eval "$(register-python-argcomplete colcon)"
            sudo sysctl -w net.core.rmem_max=2147483647
          '';
          RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp";
          ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST";
          RMW_CONNEXT_PUBLICATION_MODE="ASYNCHRONOUS";
          CYCLONEDDS_URI="file://config/ddsconfig.xml";
          NIXPKGS_ALLOW_UNFREE=1;
          packages = [
            pkgs.colcon
            pkgs.nixgl.auto.nixGLDefault
            (with pkgs.rosPackages.humble; buildEnv {
              paths = [
                rviz2
                ros-core
                ros-base
                rclcpp-components
                sensor-msgs
                ouster-ros
                rosbag2-storage-mcap
                ouster-ros
                rmw-cyclonedds-cpp
                ublox
                ament-cmake
                ament-cmake-core
                ament-cmake-ros
                nmea-navsat-driver
                foxglove-bridge
                v4l2-camera
                lidar-bike-components
                pcl-ros
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
              nebs-packages.overlays.default
              my-ros-overlay
            ];
          };
      });

}
