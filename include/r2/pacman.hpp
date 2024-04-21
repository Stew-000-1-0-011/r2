#pragma once

#include <vector>
#include <unordered_map>
#include <optional>
#include <string>
#include <tuple>
#include <functional>

#include <my_include/std_types.hpp>
#include <my_include/operator_generator.hpp>
#include <my_include/xyth.hpp>
#include <my_include/pid.hpp>
#include <my_include/lap_timer.hpp>

namespace nhk24_2nd_ws::r2::pacman::impl {
	using xyth::Xy;
	using xyth::XyOp;
	using xyth::Xyth;
	using xyth::XythScalar;
	using xyth::XythScalarOp;
	using xyth::XythOp;
	using pid::Pid;
	using lap_timer::LapTimer;

	using XythPid = Pid<Xyth, XythScalar, XythScalarOp{}, XythOp{}, XythOp{}>;

	struct Milestone final {
		Xyth point;
		XythScalar tolerance;
		std::optional<std::u8string> event;

		static auto make(const Xyth& xyth, const XythScalar& tolerance, const std::optional<std::u8string>& event) -> Milestone {
			return Milestone{xyth, tolerance, event};
		}
	};

	struct Pacman final {
		std::vector<Milestone> path;
		std::unordered_map<std::u8string, std::function<void ()>> event_handlers;
		usize index;
		LapTimer dt_timer;
		XythPid xyth_pid;

		static auto make (
			std::vector<Milestone>&& path,
			std::unordered_map<std::u8string, std::function<void ()>>&& event_handlers,
			XythPid&& xyth_pid
		) -> Pacman {
			return Pacman{std::move(path), std::move(event_handlers), 0, LapTimer::make(), std::move(xyth_pid)};
		}

		auto update (
			const Xyth& current_pose
			, const Xyth& current_speed
		) noexcept -> std::optional<Xyth>
		// [[expects: index < path.size()]]
		{
			if(index == path.size()) {
				return std::nullopt;
			}
			
			const auto [target, tolerance, event] = path[index];
			const auto xyth = xyth_pid.update_with_derivative(target -XythOp{}- current_pose, current_speed, XythScalar::from(dt_timer.update().count()));
			
			if(xyth.norm() < tolerance) {
				++index;

				if(event) {
					if(event_handlers.contains(*event)) {
						event_handlers[*event]();
					}
				}
			}

			return xyth;
		}
	};
}

namespace nhk24_2nd_ws::r2::pacman {
	using impl::Milestone;
	using impl::Pacman;
	using impl::XythPid;
}