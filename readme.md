docs:
why can ros not handle linking to all their docs? it has been YEARS.


https://docs.ros.org/en/jazzy/p/rosbag2_cpp/ 

https://docs.ros.org/en/jazzy/p/rosbag2_storage/


## running

### shell 1:
start:
`ros2 launch lidar_bike_components launch.py`

### shell 2:
configure:
`ros2 lifecycle set driver configure`

start:
`ros2 lifecycle set driver activate`

## working with pointclouds

`ros2 run pcl_ros pointcloud_to_pcd`

`ros2 bag play --start-offset 430 --remap /points:=input`

## rviz2 running reqs
```export NIXPKGS_ALLOW_UNFREE=1```
```nix develop --impure```