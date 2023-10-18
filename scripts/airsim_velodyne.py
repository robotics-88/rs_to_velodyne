#!/usr/bin/env python3

import rospy
import numpy as np
import json
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import sys

class LiDARSubscriber:
    def __init__(self):
        self.channels = None
        self.lower_fov = None
        self.upper_fov = None
        self.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('ring', 16, PointField.UINT16, 1)
        ]

        args = rospy.myargv(argv=sys.argv)
        if len(args)!=3:
            print("Missing arguments vehicle_frame and json_config")
        elif len(args) == 3:
            vehicle_enu_ned = args[1]
            self.file_path = args[2]

        vehicle_topic = "/airsim_ros_node/" + vehicle_enu_ned + "/lidar/Lidar2"

        self.read_airsim_settings()
        rospy.init_node('airsim_lidar2velodyne', anonymous=True)
        self.lidar_pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=1)
        rospy.Subscriber(vehicle_topic, PointCloud2, self.lidar_callback)

    def lidar_callback(self, data):
        # This function will be called every time a new message is received on the lidar_topic
        # Access LiDAR data from the PointCloud2 message
        header = data.header
        header.frame_id = 'velodyne'
        lidar_data = np.array([[x, y, z, intensity] for x, y, z, intensity in point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=True)])
        # print(lidar_data[:,2])
        norm = np.linalg.norm(lidar_data[:, :3], 2, axis=1)
        pitch = np.arcsin(lidar_data[:, 2] / norm)

        fov_down = self.lower_fov / 180.0 * np.pi
        fov = (abs(self.lower_fov) + abs(self.upper_fov)) / 180.0 * np.pi

        ring = (pitch + abs(fov_down)) / fov
        ring *= self.channels
        ring = np.floor(ring)
        ring = np.minimum(self.channels - 1, ring)
        ring = np.maximum(0, ring).astype(np.uint16)
        ring = ring.reshape(-1, 1)
        lidar_data = lidar_data.tolist()
        [lidar_data[i].insert(4, *ring[i]) for i in range(len(lidar_data))]
        point_cloud_msg = create_cloud(header, self.fields, lidar_data)
        self.lidar_pub.publish(point_cloud_msg)


    def read_airsim_settings(self):
        with open(self.file_path, 'r') as file:
            json_data = file.read()

        config = json.loads(json_data)
        lidar_info = config.get("Vehicles", {}).get("base_link_frd", {}).get("Sensors", {}).get("Lidar2", {})
        self.channels = lidar_info.get("NumberOfChannels")
        self.lower_fov = lidar_info.get("VerticalFOVLower")
        self.upper_fov = lidar_info.get("VerticalFOVUpper")
        

    def run(self):
        rospy.spin()  # Keeps the node running

if __name__ == '__main__':
    try:
        lidar_subscriber = LiDARSubscriber()
        lidar_subscriber.run()
    except rospy.ROSInterruptException:
        pass
