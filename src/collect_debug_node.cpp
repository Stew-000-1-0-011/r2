#include <r2/collect_debug_node.hpp>

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto exec = rclcpp::executors::MultiThreadedExecutor();

	auto collect_debug_node = std::make_shared<nhk24_2nd_ws::r2::collect_debug_node::CollectDebugNode>();
	exec.add_node(collect_debug_node);

	exec.spin();
	rclcpp::shutdown();
}