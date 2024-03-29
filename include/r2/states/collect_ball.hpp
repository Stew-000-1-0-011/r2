#pragma once

#include "transit_state.hpp"

namespace nhk24_2nd_ws::r2::collect_ball::impl {
	using namespace std::chrono_literals;
	using state_machine::StateBase;
	using state_machine::make_state;
	using robot_io::Io;

	struct CollectBall final {
		
	};
}

namespace nhk24_2nd_ws::r2::transit_state {
	inline auto to_collect_ball() -> std::unique_ptr<StateBase> {
		using namespace collect_ball::impl;

		auto state = make_state<Io, CollectBall> (
			[](Io&) -> CollectBall {
				return {};
			}
			, std::tuple {
				[](CollectBall&, Io&) -> std::unique_ptr<StateBase> {
					return nullptr;
				}
			}
			, 20ms
		);

		return std::make_unique<decltype(state)>(std::move(state));
	}
}