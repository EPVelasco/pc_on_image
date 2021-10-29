# PointCloud on Image
The code implemented in ROS projects a point cloud obtained by a Velodyne VLP16 3D-Lidar sensor on an image from an RGB camera. The example used the ROS package to calibrate a camera and a LiDAR from [lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration).

## Requisites
- [ROS](http://wiki.ros.org/ROS/Installation) Kinetic or Melodic
- [Velodyne](https://github.com/ros-drivers/velodyne) repository
- [PCL](https://pointclouds.org/) (Point Cloud Library)

## Clone repository
```
    cd ~/catkin_ws/src
    git clone https://github.com/EPVelasco/pc_on_image.git
    cd ..
    catkin_make --only-pkg-with-deps pc_on_image
```
## Ros Launch
```
   roslaunch pc_on_img vlp16OnImg.launch
```
