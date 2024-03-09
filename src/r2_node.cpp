#include <iostream>

#include <include/debug_print.hpp>

#include <r2/manual_stop_node.hpp>
#include <r2/r2_node.hpp>

using nhk24_2nd_ws::debug_print::printlns;
using nhk24_2nd_ws::r2::manual_stop_node::ManualStopNode;
using nhk24_2nd_ws::r2::r2_node::R2Node;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto manual_stop_node = std::make_shared<ManualStopNode>();
	auto r2_node = std::make_shared<R2Node>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(manual_stop_node);
	executor.add_node(r2_node);
	printlns("r2_node start.");
	executor.spin();

	rclcpp::shutdown();
}
