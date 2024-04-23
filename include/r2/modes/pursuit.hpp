#pragma once

#include <utility>
#include <optional>
#include <vector>
#include <string>
#include <iostream>
#include <syncstream>
#include <expected>

#include <my_include/void.hpp>
#include <my_include/xyth.hpp>
#include <my_include/debug_print.hpp>

#include "../mode.hpp"
#include "../pacman.hpp"
#include "../path_parser.hpp"
#include "../field_constant.hpp"

namespace nhk24_2nd_ws::r2::modes::pursuit::impl {
	using void_::Void;
	using xyth::Xy;
	using xyth::Xyth;
	using xyth::XythScalar;
	using debug_print::printlns_to;
	using mode::ModeName;
	using mode::ModeOutput;
	using pacman::Pacman;
	using pacman::Milestone;
	using pacman::XythPid;
	using path_parser::path_load;
	namespace poses = field_constant::poses;
	namespace points = field_constant::points;

	inline auto make_path_to_pose(const Xyth& goal, const XythScalar& tolerance) -> std::vector<Milestone> {
		return std::vector<Milestone> {
			Milestone::make (
				goal
				, tolerance
				, std::nullopt
			)
		};
	}

	struct PursuitIn final {
		Xyth current_pose;
		Xyth current_speed;
	};

	struct PursuitOut final {
		Xyth target_speed;
	};

	template<ModeName::Enum now_, ModeName::Enum next_>
	struct Pursuit final {
		static constexpr ModeName::Enum name = now_;

		using In = PursuitIn;
		using Out = PursuitOut;

		std::optional<Pacman> pacman;

		static auto make(const mode::Arg<now_>& arg) -> Pursuit {
			auto path = [arg]() -> std::expected<std::vector<Milestone>, std::string> {
				using namespace std::string_literals;
				if constexpr(now_ == ModeName::goto_silo) {
					return make_path_to_pose (
						Xyth::make(points::silos[arg], poses::direction_to_storage)
						, XythScalar::make(0.01, 0.02)
					);
				}
				else {
					switch(now_) {
						case ModeName::area1_start:
							return path_load("path/"s + std::string{ModeName::to_string(now_)} + ".txt");
						case ModeName::area2_start:
							return path_load("path/"s + std::string{ModeName::to_string(now_)} + ".txt");
						case ModeName::goto_center_storage:
							return make_path_to_pose(poses::look_around_storage, XythScalar::make(0.03, 0.05));
						case ModeName::goto_silo_watch_point:
							return make_path_to_pose(poses::look_around_silo, XythScalar::make(0.01, 0.02));
					}
				}
			}();

			if(path) {
				return Pursuit {
					Pacman::make (
						std::move(*path)
						, {}
						, XythPid::make (
							XythScalar::from(1.0)
							, XythScalar::from(0.0)
							, XythScalar::from(0.0)
						)
					)
				};
			}
			else {
				printlns_to(std::osyncstream{std::cout}, "cannot make path: ", path.error());
				return Pursuit{std::nullopt};
			}
		}

		auto update(const In& in) -> ModeOutput<Out> {
			if(not pacman) {  // ここ気持ち悪い。Modeのmakeが失敗し得ないものとして設計してしまったしわ寄せ。
				return ModeOutput<Out>::template change<ModeName::terminate>(Void{});
			}

			auto global_target_speed = pacman->update(in.current_pose, in.current_speed);
			if(global_target_speed) {
				const auto target_speed = global_target_speed->norm().xy > 0.3 ? Xyth::make (
					global_target_speed->xy
					, 0.0
				) : Xyth::make (
					Xy::zero()
					, global_target_speed->th
				);

				return ModeOutput<Out>::output(Out{target_speed});
			}
			else {
				return ModeOutput<Out>::template change<next_>(Void{});
			}
		}
	};

	using Area1Start = Pursuit<ModeName::area1_start, ModeName::collect_ball>;
	using Area2Start = Pursuit<ModeName::area2_start, ModeName::collect_ball>;
	using GotoCenterStorage = Pursuit<ModeName::goto_center_storage, ModeName::collect_ball>;
	using GotoSiloWatchPoint = Pursuit<ModeName::goto_silo_watch_point, ModeName::watch_silo>;
	using GotoSilo = Pursuit<ModeName::goto_silo, ModeName::harvest>;
}

namespace nhk24_2nd_ws::r2::modes::pursuit {
	using impl::PursuitIn;
	using impl::PursuitOut;
	using impl::Area1Start;
	using impl::Area2Start;
	using impl::GotoCenterStorage;
	using impl::GotoSiloWatchPoint;
	using impl::GotoSilo;
}