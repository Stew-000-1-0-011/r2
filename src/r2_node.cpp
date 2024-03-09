#include <iostream>

#include <r2/manual_stop_node.hpp>
#include <r2/r2_node.hpp>

using nhk24_2nd_ws::r2::manual_stop_node::ManualStopNode;
using nhk24_2nd_ws::r2::r2_node::R2Node;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(std::make_shared<ManualStopNode>());
	executor.add_node(std::make_shared<R2Node>());
	executor.spin();
	rclcpp::shutdown();
}
