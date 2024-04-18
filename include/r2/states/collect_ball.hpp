#pragma once

#include <my_include/xyth.hpp>
#include <my_include/pid.hpp>
#include <my_include/state_machine.hpp>
#include <my_include/debug_print.hpp>

#include "../robot_io.hpp"
#include "../lap_timer.hpp"
#include <my_include/xyth.hpp>
#include <my_include/pid.hpp>
#include <my_include/state_machine.hpp>

#include "../robot_io.hpp"
#include "../lap_timer.hpp"
#include "transit_state.hpp"

namespace nhk24_2nd_ws::r2::collect_ball::impl {
	using namespace std::chrono_literals;
	using xyth::Xyth;
	using xyth::Xy;
	using xyth::XyOp;
	using pid::Pid;
	using xyth::Xyth;
	using xyth::Xy;
	using xyth::XyOp;
	using pid::Pid;
	using state_machine::StateBase;
	using state_machine::make_state;
	using debug_print::printlns;
	using robot_io::Io;
	using robot_io::ManualAuto;
	using lap_timer::LapTimer;
	using lap_timer::LapTimer;

	struct CollectBall final {
		Pid<double> pid{Pid<double>::make(0.3, 0.0, 0.0)};
		LapTimer dt{LapTimer::make()};
		LapTimer assume_lost{LapTimer::make()};
	};

	inline auto in_safety_area(const xyth::Xyth& pose) -> bool {
		return pose.xy.x < 5.875 && pose.xy.y < 4.0;
	}
}

namespace nhk24_2nd_ws::r2::transit_state {
	inline auto to_collect_ball() -> std::unique_ptr<StateBase> {
		using namespace collect_ball::impl;

		auto state = make_state<Io, CollectBall> (
			[](Io&) -> CollectBall {
				return {};
			}
			, std::tuple {
				[](CollectBall&, Io& io) -> std::optional<std::unique_ptr<StateBase>> {
					if(const auto manual_auto = io.change_manual_auto.get(); manual_auto.has_value() && *manual_auto == ManualAuto::manual){
						return to_manual(to_collect_ball());
					}
					return std::nullopt;
				}
				, [](CollectBall& cb, Io& io) -> std::optional<std::unique_ptr<StateBase>> {
					const auto current_pose = io.current_pose.get();
					const auto current_speed = io.current_speed.get();
					const auto direction = io.ball_direction.get();

					printlns("direction: ", direction);


					if(not direction.has_value() || not in_safety_area(current_pose)) {
						if(cb.assume_lost.update() > 0.5s) {
							return to_manual(to_plunge_balls());
						}
					} else {
						cb.assume_lost.reset();

						const auto rotation_speed = cb.pid.update_with_derivative (
							*direction - 0.0
							, current_speed.th
							, cb.dt.update().count()
						);

						// io.body_speed.set (
						// 	xyth::Xyth::make (
						// 		xyth::Xy::make(0.0, 0.0)
						// 		, rotation_speed
						// 	)
						// );
					}

					if(io.ball_collected_correctly.get()) {
						return to_manual(to_exit_storage());
					}

					return std::nullopt;
				}
			}
			, 20ms
		);

		return std::make_unique<decltype(state)>(std::move(state));
	}
}