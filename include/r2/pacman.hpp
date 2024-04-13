#pragma once

#include <vector>
#include <tuple>

#include <my_include/std_types.hpp>
#include <my_include/operator_generator.hpp>
#include <my_include/xyth.hpp>
#include <my_include/pid.hpp>

#include "lap_timer.hpp"

namespace nhk24_2nd_ws::r2::pacman::impl {
	using xyth::Xy;
	using xyth::XyOp;
	using xyth::Xyth;
	using xyth::XythScalar;
	using xyth::XythScalarOp;
	using xyth::XythOp;
	using pid::Pid;
	using lap_timer::LapTimer;

	
	namespace {
		using XythPid = Pid<Xyth, XythScalar, XythScalarOp{}, XythOp{}, XythOp{}>;
	}

	namespace {
		struct Pacman final {
			std::vector<Xyth> path;
			usize index;
			LapTimer dt_timer;
			XythPid xyth_pid;

			static auto make (
				const std::vector<Xyth>& path,
				XythPid&& xyth_pid
			) -> Pacman {
				return Pacman{path, 0, LapTimer::make(), std::move(xyth_pid)};
			}

			auto update (
				const Xyth& current_pose
				, const Xyth& current_speed
				, const XythScalar& threashold
			) noexcept -> std::tuple<Xyth, bool>
			// [[expects: index < path.size()]]
			{
				const auto target = path[index];
				const auto xyth = xyth_pid.update_with_derivative(target -XythOp{}- current_pose, current_speed, XythScalar::from(dt_timer.update().count()));
				
				if(xyth.norm() < threashold) {
					++index;
				}

				if(index == path.size()) {
					return std::tuple{Xyth::zero(), true};
				}
				else {
					return std::tuple{xyth, false};
				}
			}
		};
	}
}

namespace nhk24_2nd_ws::r2::pacman {
	using impl::Pacman;
	using impl::XythPid;
}