#include <iostream>

#include <r2/r2_node.hpp>

using nhk24_2nd_ws::r2::r2_node::R2Node;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<R2Node>());
	rclcpp::shutdown();
}
