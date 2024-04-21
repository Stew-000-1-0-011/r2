#pragma once

#include <utility>
#include <optional>

#include <my_include/void.hpp>
#include <my_include/xyth.hpp>
#include <my_include/lap_timer.hpp>
#include <my_include/debug_print.hpp>

#include "../mode.hpp"
#include "../pacman.hpp"

namespace nhk24_2nd_ws::r2::modes::collect_ball::impl {
	using void_::Void;
	using xyth::Xy;
	using xyth::Xyth;
	using xyth::XythScalar;
	using xyth::XyOp;
	using xyth::XythOp;
	using lap_timer::LapTimer;
	using debug_print::printlns_to;
	using mode::ModeName;
	using mode::ModeOutput;
	using pacman::XythPid;

	struct CollectBall final {
		static constexpr auto name = ModeName::collect_ball;

		struct In final {
			Xyth current_speed;
			std::optional<double> ball_direction;
			double forward_speed;
			bool collected_correctly;
		};

		struct Out final {
			Xyth target_speed;
		};

		LapTimer lost_timer{LapTimer::make()};
		LapTimer dt_timer{LapTimer::make()};
		double last_direction{0.0};
		XythPid pid{XythPid::make (
			XythScalar::from(1.0)
			, XythScalar::from(0.0)
			, XythScalar::from(0.0)
		)};

		static auto make(const mode::Arg<name>&) -> CollectBall {
			return CollectBall{};
		}

		auto update(const In& in) -> ModeOutput<Out> {
			if(in.collected_correctly) {
				printlns_to(std::osyncstream{std::cout}, "ball collected correctly.");
				return ModeOutput<Out>::template change<ModeName::goto_silo_watch_point>(Void{});
			}

			if(in.ball_direction) {
				this->lost_timer.reset();
				this->last_direction = *in.ball_direction;

				const auto target_speed = this->pid.update (
					in.current_speed -XythOp{}- Xyth::make(in.forward_speed *XyOp{}* Xy::unit_y(), *in.ball_direction)
					, XythScalar::from(dt_timer.update().count())
				);

				return ModeOutput<Out>::output(Out{target_speed});
			}
			else {
				if(this->lost_timer.watch().count() > 1.0) {
					printlns_to(std::osyncstream{std::cout}, "lost ball.");
					return ModeOutput<Out>::template change<ModeName::goto_center_storage>(Void{});
				}

				const auto target_speed = pid.update (
					Xyth::make(Xy::zero(), this->last_direction < 0.5 ? 0.5 : this->last_direction) -XythOp{}- in.current_speed
					, XythScalar::from(this->dt_timer.update().count())
				);

				return ModeOutput<Out>::output(Out{target_speed});
			}
		}
	};
}

namespace nhk24_2nd_ws::r2::modes::collect_ball {
	using impl::CollectBall;
}