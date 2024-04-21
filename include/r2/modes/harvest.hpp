#pragma once

#include <utility>
#include <optional>

#include <my_include/void.hpp>
#include <my_include/xyth.hpp>
#include <my_include/lap_timer.hpp>
#include <my_include/debug_print.hpp>

#include "../mode.hpp"
#include "../pacman.hpp"
#include "../field_constant.hpp"

namespace nhk24_2nd_ws::r2::modes::harvest::impl {
	using void_::Void;
	using xyth::Xyth;
	using xyth::XythScalar;
	using xyth::XythOp;
	using lap_timer::LapTimer;
	using debug_print::printlns_to;
	using mode::ModeName;
	using mode::ModeOutput;
	using pacman::XythPid;
	using field_constant::SiloIndex;
	namespace poses = field_constant::poses;
	namespace points = field_constant::points;


	struct Harvest final {
		static constexpr auto name = ModeName::harvest;

		struct In final {};

		struct Out final {
			Xyth target_speed;
			bool servo_open;
		};

		LapTimer servo_timer{LapTimer::make()};
		LapTimer dt_timer{LapTimer::make()};
		XythPid pid{XythPid::make (
			XythScalar::from(1.0)
			, XythScalar::from(0.0)
			, XythScalar::from(0.2)
		)};
		bool closing{false};

		static auto make(const mode::Arg<ModeName::harvest>&) -> Harvest {
			return Harvest{};
		}

		auto update(const In&) -> ModeOutput<Out> {
			const auto target_speed = Xyth::zero();

			if(this->servo_timer.watch().count() > 3.0) {
				this->closing = true;
				this->servo_timer.reset();
				return ModeOutput<Out>::output(Out{target_speed, true});
			}
			else if(closing && this->servo_timer.watch().count() > 0.5) {
				return ModeOutput<Out>::template change<ModeName::goto_center_storage>(Void{});
			}
			else {
				return ModeOutput<Out>::output(Out{target_speed, false});
			}
		}
	};
}

namespace nhk24_2nd_ws::r2::modes::harvest {
	using impl::Harvest;
}