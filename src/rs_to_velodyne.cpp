#include "rs_to_velodyne.h"

namespace rs_to_velodyne
{

RsToVelodyne::RsToVelodyne(const rclcpp::NodeOptions& options)
    : Node("rs_converter", options)
{
    
    std::string lidar_topic = "/airsim_ros_node/MyVehicle/lidar/Lidar2";
    std::string input_cloud_type = "XYZIT";
    output_cloud_type = "XYZIR";
    this->get_parameter("raw_lidar_topic", lidar_topic);
    this->get_parameter("input_cloud_type", input_cloud_type);
    this->get_parameter("output_cloud_type", output_cloud_type);

    rclcpp::QoS qos(40);

    if (lidar_topic.empty()) {
        RCLCPP_WARN(this->get_logger(), "Please set raw_lidar_topic param in rs_to_velodyne launch file");
    }

    if (input_cloud_type == "XYZI") {
        subRobosensePC = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, 1, std::bind(&RsToVelodyne::rsHandler_XYZI, this, std::placeholders::_1));
    } 
    // It is very unclear why uncommenting this case causes laserMapping to fail when running fast_lio_lc,
    // even when the input_cloud_type is XYZIT. Weird!!!
    // else if (input_cloud_type == "XYZIRT") {
    //     subRobosensePC = nh.subscribe(lidar_topic, 1, rsHandler_XYZIRT);
    // } 
    else if (input_cloud_type == "XYZIT") {
        subRobosensePC = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, 1, std::bind(&RsToVelodyne::rsHandler_XYZI_XYZIRT, this, std::placeholders::_1));
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Please set input_cloud_type param in rs_to_velodyne launch file to XYZI or XYZIRT");
    }

    if (!(output_cloud_type == "XYZI" ||
        output_cloud_type == "XYZIR" ||
        output_cloud_type == "XYZRT")) {
        
        RCLCPP_WARN(this->get_logger(), "Please set output_cloud_type param in rs_to_velodyne launch file to XYZI, XYZIR, or XYZRT");
    }

    pubRobosensePC = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 1);
}

void RsToVelodyne::rsHandler_XYZI(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_new(new pcl::PointCloud<VelodynePointXYZIR>());
    pcl::fromROSMsg(*pc_msg, *pc);

    // to new pointcloud
    for (int point_id = 0; point_id < pc->points.size(); ++point_id) {
        if (has_nan(pc->points[point_id]))
            continue;

        VelodynePointXYZIR new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        new_point.intensity = pc->points[point_id].intensity;
        // remap ring id
        if (pc->height == 16) {
            new_point.ring = RING_ID_MAP_16[point_id / pc->width];
        } else if (pc->height == 128) {
            new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
        }
        pc_new->points.push_back(new_point);
    }

    publish_points(pc_new, pc_msg);
}

void RsToVelodyne::rsHandler_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in(new pcl::PointCloud<RsPointXYZIRT>());
    pcl::fromROSMsg(*pc_msg, *pc_in);

    if (output_cloud_type == "XYZIRT") {
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIRT>());
        handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        add_ring<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        add_time<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    } else if (output_cloud_type == "XYZIR") {
        pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIR>());
        handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
        add_ring<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    } else if (output_cloud_type == "XYZI") {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZI>());
        handle_pc_msg<RsPointXYZIRT, pcl::PointXYZI>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    }
}

void RsToVelodyne::rsHandler_XYZI_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_new(new pcl::PointCloud<VelodynePointXYZIRT>());
    pcl::fromROSMsg(*pc_msg, *pc);

    // to new pointcloud
    for (int point_id = 0; point_id < pc->points.size(); ++point_id) {
        if (has_nan(pc->points[point_id]))
            continue;

        VelodynePointXYZIRT new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        new_point.intensity = pc->points[point_id].intensity;
        // remap ring id
        if (pc->height == 16) {
            new_point.ring = RING_ID_MAP_16[point_id / pc->width];
        } else if (pc->height == 128) {
            new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
        }
        new_point.time = 0.0;
        pc_new->points.push_back(new_point);
    }

    publish_points(pc_new, pc_msg);
}
}