#pragma once

#include <memory>
#include <chrono>
#include <thread>

#include <my_include/xyth.hpp>
#include <my_include/state_machine.hpp>
#include <my_include/debug_print.hpp>

#include "../robot_config.hpp"
#include "../robot_io.hpp"

#include "transit_state.hpp"

namespace nhk24_2nd_ws::r2::go_up_slope::impl {
	using namespace std::chrono_literals;
	using xyth::Xy;
	using xyth::Xyth;
	using state_machine::make_non_loop_state;
	using debug_print::printlns;
	using robot_config::slope_speed;
	using robot_config::go_up_slope_duration;
	using robot_config::storage_entry_point;
	using robot_io::Io;
	using robot_io::MapName;
	using transit_state::StateBase;
	using transit_state::to_pass_area2;
	using transit_state::to_dancing;

	template<class NextStateGenerator_>
	requires requires(std::remove_cvref_t<NextStateGenerator_> nsg) {
		{nsg()} -> std::convertible_to<std::unique_ptr<StateBase>>;
	}
	inline auto to_go_up_slope(NextStateGenerator_&& nsg, const Xyth& speed) -> std::unique_ptr<StateBase> {
		auto state = make_non_loop_state<Io> (
			[nsg = std::move(nsg), speed](Io& io) -> std::unique_ptr<StateBase> {
				io.body_speed.set(speed);
				std::this_thread::sleep_for(go_up_slope_duration);
				io.body_speed.set(Xyth::zero());
				std::this_thread::sleep_for(0.5s);
				return nsg();
			}
		);

		return std::make_unique<decltype(state)>(std::move(state));
	}
}

namespace nhk24_2nd_ws::r2::transit_state {
	inline auto to_slope_1to2() -> std::unique_ptr<StateBase> {
		using namespace go_up_slope::impl;
		return to_go_up_slope(to_pass_area2, slope_speed);
	}

	inline auto to_slope_2to3() -> std::unique_ptr<StateBase> {
		using namespace go_up_slope::impl;
		return to_go_up_slope(to_dancing, slope_speed);
	}

	inline auto to_slope_YtoS() -> std::unique_ptr<StateBase> {
		using namespace go_up_slope::impl;
		return to_go_up_slope(to_plunge_ball, slope_speed);
	}
}