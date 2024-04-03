#include <iostream>

#include <nav2_amcl/amcl_node.hpp>
#include <r2/filter_node.hpp>

using nhk24_2nd_ws::r2::filter_node::FilterNode;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto exec = rclcpp::executors::MultiThreadedExecutor();
	exec.add_node(std::make_shared<FilterNode>());

	rclcpp::NodeOptions amcl_options{};
	std::array<const char *, 4> amcl_argv = {
		"filter_node_amcl"
		, "--ros-args"
		, "--params-file"
		, "launch/amcl.yaml"
	};
	auto amcl_context = rclcpp::Context::make_shared();
	amcl_context->init(amcl_argv.size(), amcl_argv.data());
	amcl_options.context(amcl_context);
	auto amcl = std::make_shared<nav2_amcl::AmclNode>(amcl_options);
	exec.add_node(amcl->get_node_base_interface());

	exec.spin();
	rclcpp::shutdown();
}
