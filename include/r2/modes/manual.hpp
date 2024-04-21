#pragma once

#include <utility>
#include <optional>
#include <variant>

#include <my_include/void.hpp>
#include <my_include/xyth.hpp>
#include <my_include/debug_print.hpp>

#include "../mode.hpp"

namespace nhk24_2nd_ws::r2::modes::manual::impl {
	using void_::Void;
	using xyth::Xyth;
	using debug_print::printlns_to;
	using mode::ModeName;
	using mode::ModeOutput;
	
	struct Manual final {
		static constexpr auto name = ModeName::manual;

		struct ModeContinue final {};

		struct In final {
			Xyth manual_speed;
			std::optional<std::variant<ModeContinue, ModeName::Enum>> change_to_auto;
		};

		struct Out final {
			Xyth target_speed;
		};

		ModeName::Enum last_mode;

		static auto make(const mode::Arg<ModeName::manual>& arg) -> Manual {
			return Manual {
				arg
			};
		}

		auto update(const In& in) -> ModeOutput<Out> {
			if(in.change_to_auto) {
				const auto next_mode = in.change_to_auto->index() == 0 ? this->last_mode : std::get<ModeName::Enum>(*in.change_to_auto);

				switch(next_mode) {
					case ModeName::terminate:
						return ModeOutput<Out>::template change<ModeName::terminate>(Void{});
					case ModeName::area1_start:
						return ModeOutput<Out>::template change<ModeName::area1_start>(Void{});
					case ModeName::area2_start:
						return ModeOutput<Out>::template change<ModeName::area2_start>(Void{});
					case ModeName::goto_center_storage:
						return ModeOutput<Out>::template change<ModeName::goto_center_storage>(Void{});
					case ModeName::collect_ball:
						return ModeOutput<Out>::template change<ModeName::collect_ball>(Void{});
					case ModeName::goto_silo_watch_point:
						return ModeOutput<Out>::template change<ModeName::goto_silo_watch_point>(Void{});
					case ModeName::watch_silo:
						return ModeOutput<Out>::template change<ModeName::watch_silo>(Void{});
					case ModeName::goto_silo:
					{
						printlns_to(std::osyncstream{std::cerr}, "cannot change to goto_silo from manual.");
						break;
					}
					case ModeName::harvest:
						return ModeOutput<Out>::template change<ModeName::harvest>(Void{});
					default:;
				}
			}
			return ModeOutput<Out>::output(Out{in.manual_speed});
		}
	};
}

namespace nhk24_2nd_ws::r2::modes::manual {
	using impl::Manual;
}