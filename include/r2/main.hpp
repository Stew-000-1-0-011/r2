#pragma once

#include <array>
#include <tuple>
#include <vector>
#include <optional>
#include <chrono>

#include <include/std_types.hpp>
#include <include/xyth.hpp>
#include <include/pid.hpp>

#include "omni4.hpp"
#include "pacman.hpp"

namespace nhk24_2nd_ws::r2::main::impl {
	using xyth::Xy;
	using xyth::Xyth;
	using pid::Pid;
	using omni4::Omni4;
	using pacman::Pacman;
	
	using XyPid = Pid<Xy, double, pid::trivial, xyth::XyOp{}, xyth::XyOp{}>;
	using ThPid = Pid<double, double>;

	namespace {
		struct GotoArea {
			struct In final {
				Xyth current_pose;
				double dt;
			};

			struct Out final {
				std::array<double, 4> motor_speeds;
			};

			struct TransitArg final {};

			struct Content final {
				Omni4 omni;
				Pacman pacman;
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
						, 0.05  // 5cm
						, 0.13  // 15deg
					};
				}
			};

			static auto update(Content& s, In&& in) -> std::tuple<Out, std::optional<TransitArg>> {
				const auto [xyth, is_reached] = s.pacman.update(in.current_pose, in.dt, s.xy_threashold, s.th_threashold);
				const auto motor_speeds = s.omni.update(xyth, in.dt);
				
				return {
					Out{motor_speeds}
					, is_reached ? std::optional<TransitArg>{TransitArg{}}
						: std::nullopt
				};
			}
		};

		template<class Clock_>
		requires std::chrono::is_clock_v<Clock_>
		struct Dancing final {
			struct In final {
				std::chrono::time_point<Clock_> now;
				double dt;
			};

			struct Out final {
				std::array<double, 4> motor_speeds;
			};

			struct TransitArg final {
				std::chrono::seconds turn_duration;
				bool left_turn;
			};

			struct Content {
				Omni4 omni;
				std::chrono::time_point<Clock_> last_turned;
				u32 turn_count;
				bool left_turn;
				const std::chrono::seconds turn_duration;

				static auto make (
					const std::chrono::seconds turn_duration
					, const bool left_turn
				) -> Content {
					return Content {
						Omni4::make()
						, Clock_::now()
						, 0
						, left_turn
						, turn_duration
					};
				}
			};

			static auto update(Content& s, In&& in) -> std::tuple<Out, std::optional<TransitArg>> {
				if (in.now - s.last_turned >= s.turn_duration) {
					s.last_turned = in.now;
					s.left_turn = !s.left_turn;
					++s.turn_count;
				}
				return {
					Out{s.omni.update(Xyth::make(Xy::zero(), s.left_turn ? 3.14 : -3.14), in.dt)}
					, s.turn_count == 2 ? std::optional<TransitArg>{TransitArg{s.turn_duration, s.left_turn}}
						: std::nullopt
				};
			}
		};
	}
}

namespace nhk24_2nd_ws::r2::main {
	using impl::GotoArea;
	using impl::Dancing;
}