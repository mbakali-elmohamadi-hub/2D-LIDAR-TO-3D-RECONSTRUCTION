# File Description

## `firmware/arduino_z_axis_controller/arduino_z_axis_controller.ino`
Arduino firmware for controlling the Z-axis stepper motor. It receives the scan height, waits for a button press, moves the lead screw, and streams Z positions to the host computer.

## `src/cpp/lidar_acquisition/main.cpp`
Main C++ acquisition program. It reads RPLiDAR 2D scan data, receives Z positions from Arduino, converts polar LiDAR samples into 3D Cartesian coordinates, and writes the reconstructed point cloud.

## `src/python/filter_lof.py`
Python post-processing script that removes spatial outliers using Local Outlier Factor.

## `src/java/viewer/Surface.java`
Java Swing GUI for launching scans, selecting COM ports, running the filtering script, and visualizing reconstructed 3D point clouds.

## `data/sample/raw_points_example.txt`
Example raw reconstructed point cloud before filtering.

## `data/sample/filtered_points_example.txt`
Example point cloud after LOF filtering.

## `third_party/rplidar_sdk/`
Third-party SLAMTEC/RPLiDAR SDK source files required to build the acquisition program.
