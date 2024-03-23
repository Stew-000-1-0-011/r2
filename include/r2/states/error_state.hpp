#pragma once

#include <string_view>
#include <thread>
#include <memory>
#include <chrono>

#include <my_include/state_machine.hpp>
#include <my_include/debug_print.hpp>

#include "transit_state.hpp"

namespace nhk24_2nd_ws::r2::transit_state {
	auto to_error_state(const std::string_view error_message) -> std::unique_ptr<StateBase> {
		using namespace std::chrono_literals;
		using state_machine::make_non_loop_state;
		using debug_print::printlns;
		using robot_io::Io;
		
		auto state = make_non_loop_state<Io> (
			[error_message = std::string(error_message)](Io&) -> std::unique_ptr<StateBase> {
				printlns("ErrorState: ", error_message);
				std::this_thread::sleep_for(1s);
				return nullptr;
			}
		);
		return std::make_unique<decltype(state)>(std::move(state));
	}
}