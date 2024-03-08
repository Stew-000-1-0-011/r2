#include <iostream>

#include <r2/filter_node.hpp>

using nhk24_2nd_ws::r2::filter_node::FilterNode;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FilterNode>());
	rclcpp::shutdown();
}
