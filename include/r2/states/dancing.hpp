#pragma once

#include <memory>
#include <chrono>
#include <optional>
#include <tuple>

#include <my_include/xyth.hpp>
#include <my_include/state_machine.hpp>

#include "../robot_io.hpp"
#include "../lap_timer.hpp"

#include "transit_state.hpp"

namespace nhk24_2nd_ws::r2::dancing::impl {
	using namespace std::chrono_literals;
	using xyth::Xy;
	using xyth::Xyth;
	using state_machine::make_state;
	using robot_io::Io;
	using robot_io::ManualAuto;
	using transit_state::StateBase;
	using transit_state::to_manual;
	using lap_timer::LapTimer;

	struct Dancing final {
		LapTimer turn_timer;
		std::chrono::duration<double> turn_duration;
		bool is_turning_left;
	};
}
namespace nhk24_2nd_ws::r2::transit_state {
	inline auto to_dancing() -> std::unique_ptr<StateBase> {
		using namespace dancing::impl;

		auto state = make_state<Io, Dancing> (
			[](Io&) -> Dancing {
				return Dancing {
					LapTimer::make()
					, 1s
					, true
				};
			}
			, std::tuple {
				[](Dancing& d, Io& io) -> std::optional<std::unique_ptr<StateBase>> {
					if(const auto manual_auto = io.change_manual_auto.get(); manual_auto.has_value() && *manual_auto == ManualAuto::manual){
						io.body_speed.set (
							Xyth::make(Xy::zero(), d.is_turning_left ? 3.14 : -3.14)
						);
						return to_manual(to_dancing());
					}
					return std::nullopt;
				}
				, [](Dancing& d, Io& io) -> std::optional<std::unique_ptr<StateBase>> {
					if(d.turn_timer.watch() >= d.turn_duration) {
						d.is_turning_left = !d.is_turning_left;
						d.turn_duration += 1s;
						d.turn_timer.reset();
						
						io.body_speed.set (
							Xyth::make(Xy::zero(), d.is_turning_left ? 3.14 : -3.14)
						);
					}
					return std::nullopt;
				}
			}
			, 20ms
		);

		return std::make_unique<decltype(state)>(std::move(state));
	}
}