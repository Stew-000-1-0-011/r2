#include <rclcpp/rclcpp.hpp>
#include <r2/silo_position_publish_node.hpp>

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto exec = rclcpp::executors::MultiThreadedExecutor();

	auto silo_position_publish_node = std::make_shared<nhk24_2nd_ws::r2::silo_position_publish_node::SiloPositionPublishNode>(rclcpp::NodeOptions());
	exec.add_node(silo_position_publish_node);

	exec.spin();
	rclcpp::shutdown();
}