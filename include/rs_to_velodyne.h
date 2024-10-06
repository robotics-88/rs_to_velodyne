#ifndef RS_TO_VELODYNE_H_
#define RS_TO_VELODYNE_H_

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"


static int RING_ID_MAP_RUBY[] = {
        3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
        35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
        67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
        99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
        7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
        39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
        71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
        103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};
static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

// rslidar和velodyne的格式有微小的区别
// rslidar的点云格式
struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    std::uint8_t intensity;
    std::uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)
                                        (std::uint16_t, ring, ring)(double, timestamp, timestamp));

// velodyne的点云格式
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                        (std::uint16_t, ring, ring)(float, time, time));

struct VelodynePointXYZIR {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    std::uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIR,
                                (float, x, x)(float, y, y)
                                        (float, z, z)(float, intensity, intensity)
                                        (std::uint16_t, ring, ring));


namespace rs_to_velodyne {

class RsToVelodyne : public rclcpp::Node
{
public:
    explicit RsToVelodyne(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~RsToVelodyne(){};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subRobosensePC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRobosensePC;

    std::string output_cloud_type;
    void rsHandler_XYZI(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);
    void rsHandler_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);
    void rsHandler_XYZI_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);


    template<typename T>
    bool has_nan(T point) {

        // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
        // pcl remove nan not work normally
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
            return true;
        } else {
            return false;
        }
    }

    template<typename T>
    void publish_points(T &new_pc, const sensor_msgs::msg::PointCloud2::SharedPtr &old_msg) {
        // pc properties
        new_pc->is_dense = true;

        // publish
        sensor_msgs::msg::PointCloud2 pc_new_msg;
        pcl::toROSMsg(*new_pc, pc_new_msg);
        pc_new_msg.header = old_msg->header;
        pc_new_msg.header.frame_id = "velodyne";
        pubRobosensePC->publish(pc_new_msg);
    }

    template<typename T_in_p, typename T_out_p>
    void handle_pc_msg(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                    const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {

        // to new pointcloud
        for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;
            T_out_p new_point;
            new_point.x = pc_in->points[point_id].x;
            new_point.y = pc_in->points[point_id].y;
            new_point.z = pc_in->points[point_id].z;
            new_point.intensity = pc_in->points[point_id].intensity;
    //        // 计算相对于第一个点的相对时间
            pc_out->points.push_back(new_point);
        }
    }

    template<typename T_in_p, typename T_out_p>
    void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
        // to new pointcloud
        int valid_point_id = 0;
        for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;
            // 跳过nan点
            pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
        }
    }

    template<typename T_in_p, typename T_out_p>
    void add_time(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
        // to new pointcloud
        int valid_point_id = 0;
        for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;
            pc_out->points[valid_point_id++].time = float(pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
            // 跳过nan点
        }
    }

};

}

#endif