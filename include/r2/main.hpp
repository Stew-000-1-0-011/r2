#pragma once

#include <array>
#include <tuple>
#include <vector>
#include <optional>
#include <chrono>
#include <memory>

#include <include/std_types.hpp>
#include <include/xyth.hpp>
#include <include/pid.hpp>

#include "omni4.hpp"
#include "pacman.hpp"
#include "lap_timer.hpp"
#include "state_machine.hpp"

namespace nhk24_2nd_ws::r2::main::impl {
	using xyth::Xy;
	using xyth::Xyth;
	using pid::Pid;
	using omni4::Omni4;
	using pacman::Pacman;
	using lap_timer::LapTimer;
	using state_machine::state_info_like;
	using state_machine::StateBase;
	
	using XyPid = Pid<Xy, double, pid::trivial, xyth::XyOp{}, xyth::XyOp{}>;
	using ThPid = Pid<double, double>;

	namespace {
		struct TemporaryManual final {
			struct In final {
				Xyth body_speed;
				bool change_to_auto;
			};

			struct Out final {
				std::array<double, 4> motor_speeds;
			};

			struct TransitArg final {};

			struct Content final {
				Omni4 omni;
				LapTimer timer;

				static auto make() -> Content {
					return Content{Omni4::make(), LapTimer::make()};
				}
			};

			static auto update(Content& s, In&& in) -> std::tuple<Out, std::optional<TransitArg>> {
				return {
					Out{s.omni.update(in.body_speed, s.timer.update().count())}
					, in.change_to_auto ? std::make_optional(TransitArg{})
					: std::nullopt
				};
			}
		};

		struct GotoArea final {
			struct In final {
				Xyth current_pose;
				bool change_to_manual;
			};

			struct Out final {
				std::array<double, 4> motor_speeds;
			};

			struct Content final {
				Omni4 omni;
				Pacman pacman;
				LapTimer timer;
				double xy_threashold;
				double th_threashold;

				static auto make (
					std::vector<Xyth>&& path,
					const XyPid& xy_pid,
					const ThPid& th_pid
				) -> Content {
					return Content {
						Omni4::make()
						, Pacman::make(std::move(path), xy_pid, th_pid)
						, LapTimer::make()
						, 0.05  // 5cm
						, 0.13  // 15deg
					};
				}
			};

			struct TransitArg final {
				bool change_to_manual;
			};

			static auto update(Content& s, In&& in) -> std::tuple<Out, std::optional<TransitArg>> {
				const auto dt = s.timer.update().count();
				const auto [xyth, is_reached] = s.pacman.update(in.current_pose, dt, s.xy_threashold, s.th_threashold);
				const auto motor_speeds = s.omni.update(xyth, dt);
				
				return {
					Out{motor_speeds}
					, in.change_to_manual ? std::optional<TransitArg>{TransitArg{true}}
					: is_reached ? std::optional<TransitArg>{TransitArg{false}}
						: std::nullopt
				};
			}
		};

		struct Dancing final {
			struct In final {
				bool change_to_manual;
			};

			struct Out final {
				std::array<double, 4> motor_speeds;
			};

			struct TransitArg final {
				bool change_to_manual;
			};

			struct Content {
				Omni4 omni;
				LapTimer turn_change_timer;
				LapTimer dt_timer;
				u32 turn_count;
				bool left_turn;
				const std::chrono::seconds turn_duration;

				static auto make (
					const std::chrono::seconds turn_duration
					, const bool left_turn
				) -> Content {
					return Content {
						Omni4::make()
						, LapTimer::make()
						, LapTimer::make()
						, 0
						, left_turn
						, turn_duration
					};
				}
			};

			static auto update(Content& s, In&& in) -> std::tuple<Out, std::optional<TransitArg>> {
				if (s.turn_change_timer.watch() > s.turn_duration) {
					s.turn_change_timer.reset();
					s.left_turn = !s.left_turn;
					++s.turn_count;
				}
				return {
					Out{s.omni.update(Xyth::make(Xy::zero(), s.left_turn ? 3.14 : -3.14), s.dt_timer.update().count())}
					, in.change_to_manual ? std::optional<TransitArg>{TransitArg{true}}
					: s.turn_count == 2 ? std::optional<TransitArg>{TransitArg{false}}
						: std::nullopt
				};
			}
		};
	}
}

namespace nhk24_2nd_ws::r2::main {
	using impl::GotoArea;
	using impl::Dancing;
	using impl::TemporaryManual;
}