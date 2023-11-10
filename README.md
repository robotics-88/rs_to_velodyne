# RS to Velodyne
A ros tool for converting Robosense pointcloud or AirSim lidar to Velodyne pointcloud format, which can be directly used for downstream algorithm, such as LOAM, LEGO-LOAM, LIO-SAM, etc.

## Currently support:

### 1. [Airsim XYZI] to [velodyne XYZIR]:
Airsim Lidar2 point cloud

### 2. [robosense XYZIRT] to [velodyne XYZIRT / XYZIR / XYZI]:
RS-16, RS-32, RS-Ruby, RS-BP and RS-Helios LiDAR point cloud.

### 3. [robosense XYZI] to [velodyne XYZIR]:
RS-16 and RS-Ruby LiDAR point cloud, More LiDAR model support is coming soon. 



### Notice: For rslidar_sdk v1.5+(2022), minor modifications should be made in the code. Please see https://github.com/HViktorTsoi/rs_to_velodyne/issues/11. This fix will soon be merged into the main stream.

## Usage

Launch using
```
roslaunch rs_to_velodyne rs_to_velodyne.launch
```

### Arguments
`raw_lidar_topic`: The raw lidar topic to be converted.

`input_cloud_type`: Input cloud format type (the type of pointcloud published by `raw_lidar_topic`). Options are XYZI, XYZIT, or XYZIRT.

`output_cloud`: Output cloud format type. Options are XYZI, XYZIR, or XYZIRT.

## Subscribes
`<raw_lidar_topic>`: sensor_msgs.PointCloud2, from Robosense LiDAR or AirSim LiDAR. 

## Publishes
`/velodyne_points`: sensor_msgs.PointCloud2, the frame_id is `velodyne`.
