#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <my_include/mutexed.hpp>
#include <my_include/debug_print.hpp>

#include <r2/r2_node.hpp>
#include <r2/map_amcl_manager_node.hpp>

using namespace std::chrono_literals;

using nhk24_2nd_ws::mutexed::Mutexed;
using nhk24_2nd_ws::debug_print::printlns_to;
using nhk24_2nd_ws::r2::r2_node::R2Node;
using nhk24_2nd_ws::r2::r2_node::make_r2_controller_unique;
using nhk24_2nd_ws::r2::map_amcl_manager_node::MapAmclManagerNode;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	std::shared_ptr<Mutexed<bool>> r2_controller_killed = std::shared_ptr<Mutexed<bool>>(new Mutexed<bool>(Mutexed<bool>::make(false)));

	auto r2_node = std::make_shared<R2Node>();
	auto map_amcl_manager_node = std::make_shared<MapAmclManagerNode>();
	auto r2_controller = make_r2_controller_unique (
		r2_node,
		*r2_controller_killed
	);

	rclcpp::on_shutdown([r2_node, map_amcl_manager_node, r2_controller_killed] {
		r2_node->kill();
		map_amcl_manager_node->kill();

		while(!r2_controller_killed->get() || !map_amcl_manager_node->done.get()) {
			std::this_thread::sleep_for(500ms);
			printlns_to(std::osyncstream{std::cout}, "waiting for r2_controller and map_amcl_manager are killed.", r2_controller_killed->get(), map_amcl_manager_node->done.get());
		}
	});

	rclcpp::executors::MultiThreadedExecutor executor{};
	executor.add_node(r2_node);
	executor.add_node(map_amcl_manager_node);

	std::jthread r2_controller_thread([r2_controller = std::move(r2_controller)] mutable {
		printlns_to(std::osyncstream{std::cout}, "r2_controller_thread start.");
		r2_controller->run();
		printlns_to(std::osyncstream{std::cout}, "r2_controller_thread end.");
	});

	printlns_to(std::osyncstream{std::cout}, "r2_node spin start.");
	executor.spin();
	printlns_to(std::osyncstream{std::cout}, "r2_node spin end.");
}
