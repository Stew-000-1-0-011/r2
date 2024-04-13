#pragma once

#include <vector>
#include <tuple>
#include <optional>
#include <memory>
#include <chrono>
#include <concepts>
#include <type_traits>

#include <my_include/pid.hpp>
#include <my_include/xyth.hpp>
#include <my_include/state_machine.hpp>
#include <my_include/debug_print.hpp>

#include "../robot_config.hpp"
#include "../robot_io.hpp"
#include "../path_parser.hpp"
#include "../pacman.hpp"

#include "transit_state.hpp"

namespace nhk24_2nd_ws::r2::pass_area::impl {
	using namespace std::chrono_literals;
	using xyth::Xyth;
	using xyth::XythScalar;
	using state_machine::make_state;
	using debug_print::printlns;
	using path_parser::PathName;
	using robot_io::Io;
	using robot_io::ManualAuto;
	using robot_io::MapName;
	using pacman::Pacman;
	using pacman::XythPid;
	using transit_state::StateBase;
	using transit_state::to_error_state;
	using transit_state::to_manual;
	using transit_state::to_slope_1to2;
	using transit_state::to_slope_2to3;

	using XyPid = pid::Pid<xyth::Xy, double, pid::trivial, xyth::XyOp{}, xyth::XyOp{}>;
	using ThPid = pid::Pid<double, double>;

	struct PassArea final {
		Pacman pacman;
		MapName::Enum map;
		Xyth initialpose;
	};

	template<class NextStateGenerator>
	requires requires(std::remove_cvref_t<NextStateGenerator> nsg) {
		{nsg()} -> std::convertible_to<std::unique_ptr<StateBase>>;
	}
	inline auto to_pass_area(const MapName::Enum map, const Xyth& initialpose, std::vector<Xyth>&& path, NextStateGenerator&& nsg) -> std::unique_ptr<StateBase> {
		auto state = make_state<Io, PassArea> (
			[path = std::move(path), map, initialpose](Io& io) {

				io.change_map(map, initialpose).get();
				std::this_thread::sleep_for(3s);  // wait for position to stabilize
				printlns("map changed: ", MapName::to_filepath(map), " (", initialpose, ")");

				return PassArea {
					Pacman::make (
						std::move(path)
						, XythPid::make (
							XythScalar::from(0.1)
							, XythScalar::from(0.0)
							, XythScalar::from(1.0)
						)
					)
					, map
					, initialpose
				};
			}
			, std::tuple {
				[nsg](PassArea& pa, Io& io) -> std::optional<std::unique_ptr<StateBase>> {
					if(const auto manual_auto = io.change_manual_auto.get(); manual_auto.has_value() && *manual_auto == ManualAuto::manual){
						return to_manual(to_pass_area(pa.map, pa.initialpose, std::move(pa.pacman.path), nsg));
					}
					return std::nullopt;
				}
				, [nsg = std::move(nsg)](PassArea& pa, Io& io) mutable -> std::optional<std::unique_ptr<StateBase>> {
					const auto current_pose = io.current_pose.get();
					const auto current_speed = io.current_speed.get();

					const auto [body_speed, is_reached] = pa.pacman.update (
						current_pose
						, current_speed
						, XythScalar::make(0.1, 0.25)
					);
					
					
					if(is_reached) {
						printlns("reached!!!");
						io.body_speed.set(Xyth::zero());
						std::this_thread::sleep_for(1s);
						return nsg();
					}
					else {
						io.body_speed.set(body_speed);
						return std::nullopt;
					}
				}
			}
			, 20ms
		);

		return std::make_unique<decltype(state)>(std::move(state));
	}
}

namespace nhk24_2nd_ws::r2::transit_state {
	inline auto to_pass_area1() -> std::unique_ptr<StateBase> {
		using namespace pass_area::impl;

		auto path = PathName::load_path(PathName::start_to_area2);
		if(!path.has_value()) {
			return to_error_state(std::string("Failed to load path: ") + path.error());
		}
		else {
			return to_pass_area (
				MapName::area1
				, robot_config::area1_initialpose
				, std::move(*path)
				, to_slope_1to2
			);
		}
	}

	inline auto to_pass_area2() -> std::unique_ptr<StateBase> {
		using namespace pass_area::impl;

		auto path = PathName::load_path(PathName::area2_to_yellow);
		if(!path.has_value()) {
			return to_error_state(std::string("Failed to load path: ") + path.error());
		}
		else {
			return to_pass_area (
				MapName::area2
				, robot_config::area2_initialpose
				, std::move(*path)
				, to_slope_2to3
			);
		}
	}

	inline auto to_pass_yellow() -> std::unique_ptr<StateBase> {
		using namespace pass_area::impl;

		auto path = PathName::load_path(PathName::yellow_to_storage);
		if(!path.has_value()) {
			return to_error_state(std::string("Failed to load path: ") + path.error());
		}
		else {
			return to_pass_area (
				MapName::area3_yellow
				, robot_config::yellow_initialpose
				, std::move(*path)
				, to_plunge_balls
			);
		}
	}

	inline auto to_plunge_balls() -> std::unique_ptr<StateBase> {
		using namespace pass_area::impl;

		auto path = PathName::load_path(PathName::storage_into_center);
		if(!path.has_value()) {
			return to_error_state(std::string("Failed to load path: ") + path.error());
		}
		else {
			return to_pass_area (
				MapName::area3_storage
				, robot_config::storage_entry_point
				, std::move(*path)
				, to_collect_ball
			);
		}
	}

	inline auto to_exit_storage() -> std::unique_ptr<StateBase> {
		using namespace pass_area::impl;

		auto path = PathName::load_path(PathName::center_to_exit);
		if(!path.has_value()) {
			return to_error_state(std::string("Failed to load path: ") + path.error());
		}
		else {
			return to_pass_area (
				MapName::area3_storage
				, robot_config::storage_entry_point
				, std::move(*path)
				, to_dancing
			);
		}
	}
}