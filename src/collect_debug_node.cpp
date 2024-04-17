#include <rclcpp/rclcpp.hpp>
#include <r2/collect_debug_node.hpp>
#include <r2/manual_undercarriage_node.hpp>
#include <r2/manual_stop_node.hpp>

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto exec = rclcpp::executors::MultiThreadedExecutor();

	auto collect_debug_node = std::make_shared<nhk24_2nd_ws::r2::collect_debug_node::CollectDebugNode>();
	// auto manual_undercarriage_node = std::make_shared<nhk24_2nd_ws::r2::manual_undercarriage_node::ManualUndercarriageNode>();
	// auto manual_stop_node = std::make_shared<nhk24_2nd_ws::r2::manual_stop_node::ManualStopNode>();
	exec.add_node(collect_debug_node);
	// exec.add_node(manual_undercarriage_node);
	// exec.add_node(manual_stop_node);

	exec.spin();
	rclcpp::shutdown();
}