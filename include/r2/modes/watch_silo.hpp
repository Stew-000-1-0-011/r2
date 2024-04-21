#pragma once

#include <utility>
#include <optional>

#include <my_include/xyth.hpp>
#include <my_include/lap_timer.hpp>
#include <my_include/debug_print.hpp>

#include "../mode.hpp"
#include "../pacman.hpp"
#include "../field_constant.hpp"

namespace nhk24_2nd_ws::r2::modes::watch_silo::impl {
	using xyth::Xyth;
	using xyth::XythScalar;
	using xyth::XythOp;
	using lap_timer::LapTimer;
	using mode::ModeName;
	using mode::ModeOutput;
	using pacman::XythPid;
	using field_constant::SiloIndex;
	namespace poses = field_constant::poses;
	
	struct WatchSilo final {
		static constexpr auto name = ModeName::watch_silo;

		struct In final {
			std::optional<SiloIndex::Enum> target_silo;
			Xyth current_pose;
			Xyth current_speed;
		};

		struct Out final {
			Xyth target_speed;
		};

		XythPid pid {
			XythPid::make (
				XythScalar::from(1.0)
				, XythScalar::from(0.0)
				, XythScalar::from(0.2)
			)
		};
		LapTimer dt_timer{LapTimer::make()};

		static auto make(const mode::Arg<ModeName::watch_silo>&) -> WatchSilo {
			return WatchSilo{};
		}

		auto update(const In& in) -> ModeOutput<Out> {
			if(not in.target_silo) {
				const auto target_speed = this->pid.update_with_derivative (
					poses::look_around_silo -XythOp{}- in.current_pose
					, in.current_speed
					, XythScalar::from(dt_timer.update().count())
				);

				return ModeOutput<Out>::output(Out{target_speed});
			}
			else {
				return ModeOutput<Out>::template change<ModeName::goto_silo>(*in.target_silo);
			}
		}
	};
}

namespace nhk24_2nd_ws::r2::modes::watch_silo {
	using impl::WatchSilo;
}