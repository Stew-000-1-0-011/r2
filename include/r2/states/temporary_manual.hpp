#pragma once

#include <memory>
#include <optional>
#include <chrono>
#include <utility>
#include <tuple>

#include <my_include/state_machine.hpp>
#include <my_include/debug_print.hpp>

#include "../robot_io.hpp"

#include "transit_state.hpp"

namespace nhk24_2nd_ws::r2::temporary_manual::impl {
	using namespace std::chrono_literals;
	using state_machine::make_state;
	using debug_print::printlns;
	using robot_io::Io;
	using robot_io::ManualAuto;
	using robot_io::StateName;
	using transit_state::StateBase;
	using transit_state::to_pass_area1;
	using transit_state::to_slope_1to2;
	using transit_state::to_pass_area2;
	using transit_state::to_slope_2to3;
	using transit_state::to_pass_yellow;
	using transit_state::to_slope_YtoS;
	using transit_state::to_plunge_balls;
	using transit_state::to_collect_ball;
	using transit_state::to_dancing;
	using transit_state::to_pass_yellow;
	using transit_state::to_plunge_balls;
	using transit_state::to_collect_ball;

	struct TemporaryManual final {};

	inline auto manual_to_auto(std::unique_ptr<StateBase>&& evacuated_state) {
		return [evacuated_state = std::move(evacuated_state)](TemporaryManual&, Io& io) mutable -> std::optional<std::unique_ptr<StateBase>> {
			if(const auto manual_auto = io.change_manual_auto.get()) {
				if(*manual_auto == ManualAuto::auto_specified) {
					auto recover_state = io.manual_recover_state.get();
					if(recover_state.has_value()) {
						switch(*recover_state) {
							case StateName::pass_area1:
								return to_pass_area1();
							case StateName::slope_1to2:
								return to_slope_1to2();
							case StateName::pass_area2:
								return to_pass_area2();
							case StateName::slope_2to3:
								return to_slope_2to3();
							case StateName::dancing:
								return to_dancing();
							case StateName::pass_yellow:
								return to_pass_yellow();
							case StateName::slope_YtoS:
								return to_slope_YtoS();
							case StateName::plunge_balls:
								return to_plunge_balls();
							case StateName::collect_ball:
								return to_collect_ball();
							default:
								return nullptr;
						}
					}
					else {
						printlns("TemporaryManual: recover_state is empty.");
					}
				}
				else if(*manual_auto == ManualAuto::auto_evacuated) {
					return std::move(evacuated_state);
				}
			}
			return std::nullopt;
		};
	}

	inline auto manual_control(TemporaryManual&, Io& io) -> std::optional<std::unique_ptr<StateBase>> {
		const auto manual_speed = io.manual_speed.get();
		io.body_speed.set(manual_speed);
		return std::nullopt;
	}
}

namespace nhk24_2nd_ws::r2::transit_state {
	auto to_manual(std::unique_ptr<StateBase>&& evacuated_state) -> std::unique_ptr<StateBase> {
		using namespace nhk24_2nd_ws::r2::temporary_manual::impl;
		
		auto state = make_state<Io, TemporaryManual> (
			[](Io&) {
				return TemporaryManual{};
			}
			, std::tuple {
				manual_to_auto(std::move(evacuated_state))
				, [](auto&& ... args){return manual_control(std::forward<decltype(args)>(args) ...);}
			}
			, 20ms
		);

		return std::make_unique<decltype(state)>(std::move(state));
	}
}