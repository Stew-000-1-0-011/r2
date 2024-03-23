#include <iostream>

#include <my_include/debug_print.hpp>
#include <my_include/state_machine.hpp>

#include <r2/robot_io.hpp>
#include <r2/r2_node.hpp>
#include <r2/states/states.hpp>
#include <r2/manual_stop_node.hpp>

using nhk24_2nd_ws::debug_print::printlns;
using nhk24_2nd_ws::state_machine::StateMachine;
using nhk24_2nd_ws::r2::robot_io::Io;
using nhk24_2nd_ws::r2::r2_node::R2Node;
using nhk24_2nd_ws::r2::r2_node::make_node;
using nhk24_2nd_ws::r2::r2_node::make_node;
namespace transit_state = nhk24_2nd_ws::r2::transit_state;
using nhk24_2nd_ws::r2::manual_stop_node::ManualStopNode;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto manual_stop_node = std::make_shared<ManualStopNode>();
	auto [node, io_fut] = make_node();

	rclcpp::executors::MultiThreadedExecutor executor{};
	executor.add_node(manual_stop_node);
	executor.add_node(node);

	std::jthread state_machine_thread([io_fut = std::move(io_fut)] mutable {
		printlns("state_machine_thread start.");
		auto machine = StateMachine<Io>::make(
			transit_state::to_pass_area1()
		);
		auto io = io_fut.get();
		machine.run(*io);
		printlns("state_machine_thread end.");
	});

	printlns("r2_node start.");
	executor.spin();

	rclcpp::shutdown();
}
