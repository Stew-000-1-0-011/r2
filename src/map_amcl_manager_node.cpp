#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <my_include/debug_print.hpp>

#include <r2/map_amcl_manager_node.hpp>

using namespace std::chrono_literals;

using nhk24_2nd_ws::debug_print::printlns_to;
using nhk24_2nd_ws::r2::map_amcl_manager_node::MapAmclManagerNode;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);


	auto map_amcl_manager_node = std::make_shared<MapAmclManagerNode>();

	rclcpp::on_shutdown( [
		map_amcl_manager_node
	] {
		map_amcl_manager_node->kill();

		while (
			!map_amcl_manager_node->done.get()
		) {
			std::this_thread::sleep_for(500ms);
			printlns_to(std::osyncstream{std::cout}, "waiting for map_amcl_manager are killed.", map_amcl_manager_node->done.get());
		}
	});

	rclcpp::executors::MultiThreadedExecutor executor{};
	executor.add_node(map_amcl_manager_node);

	printlns_to(std::osyncstream{std::cout}, "map_amcl_manager_node spin start.");
	executor.spin();
	printlns_to(std::osyncstream{std::cout}, "map_amcl_manager_node spin end.");
}
