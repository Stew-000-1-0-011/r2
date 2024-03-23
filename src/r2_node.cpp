#include <csignal>
#include <atomic>
#include <memory>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

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

volatile std::atomic_flag kill_interrupt = ATOMIC_FLAG_INIT;
volatile std::atomic_flag user_defined_interrupt_done = ATOMIC_FLAG_INIT;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	std::signal(SIGINT, [](int) -> void {
		kill_interrupt.test_and_set();
	});

	auto manual_stop_node = std::make_shared<ManualStopNode>();
	auto [node, io_fut] = make_node(&kill_interrupt, &user_defined_interrupt_done);

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

	printlns("r2_node end.");
}
