#include <rclcpp/rclcpp.hpp>
#include "rs_to_velodyne.h"

int main(int argc, char **argv) {

    const rclcpp::NodeOptions options;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rs_to_velodyne::RsToVelodyne>(options));
    rclcpp::shutdown();
    return 0;
}