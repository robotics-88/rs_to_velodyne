#include <rclcpp/rclcpp.hpp>
#include "rs_to_velodyne.h"

int main(int argc, char **argv) {

    const rclcpp::NodeOptions options;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rs_to_velodyne::RsToVelodyne>(options));
    rclcpp::shutdown();
    return 0;
    // rclcpp::init(argc, argv);
    // rclcpp::NodeOptions node_options;
    // node_options.automatically_declare_parameters_from_overrides(true);
    // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("rs_converter", node_options);

    // rs_to_velodyne::RsToVelodyne rs_to_velodyne(node);
    // rclcpp::spin(node);

    // return 0;
}