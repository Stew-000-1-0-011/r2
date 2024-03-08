#pragma once

#include <vector>
#include <tuple>

#include <include/std_types.hpp>
#include <include/xyth.hpp>
#include <include/pid.hpp>

namespace nhk24_2nd_ws::r2::pacman::impl {
	using nhk24_2nd_ws::xyth::Xy;
	using nhk24_2nd_ws::xyth::XyOp;
	using nhk24_2nd_ws::xyth::Xyth;
	using nhk24_2nd_ws::xyth::XythNearistDiffOp;
	using nhk24_2nd_ws::pid::Pid;
	using nhk24_2nd_ws::pid::trivial;

	using XyPid = Pid<Xy, double, trivial, XyOp{}, XyOp{}>;
	using ThPid = Pid<double, double>;

	inline constexpr auto ndf = XythNearistDiffOp{};
	namespace {
		struct XythPid final {
			XyPid xy_pid;
			ThPid th_pid;

			auto update (
				const Xyth& current_pose,
				const Xyth& target,
				const double dt
			) noexcept -> Xyth
			{
				const auto error = target -ndf- current_pose;
				const auto xy = xy_pid.update(error.xy, dt);
				const auto th = th_pid.update(error.th, dt);

				return Xyth::make(xy, th);
			}
		};
	}

	namespace {
		struct Pacman final {
			std::vector<Xyth> path;
			usize index;
			XythPid xyth_pid;

			static auto make (
				const std::vector<Xyth>& path,
				const XyPid& xy_pid,
				const ThPid& th_pid
			) -> Pacman {
				return Pacman{path, 0, XythPid{xy_pid, th_pid}};
			}

			auto update (
				const Xyth& current_pose
				, const double dt
				, const double xy_threashold
				, const double th_threashold
			) noexcept -> std::tuple<Xyth, bool>
			// [[expects: index < path.size()]]
			{
				const auto target = path[index];
				const auto xyth = xyth_pid.update(current_pose, target, dt);
				
				if(xyth.xy.norm() < xy_threashold && std::abs(xyth.th) < th_threashold) {
					++index;
				}

				if(index == path.size()) {
					return {Xyth::zero(), true};
				}
				else {
					return {xyth, false};
				}
			}
		};
	}
}

namespace nhk24_2nd_ws::r2::pacman {
	using impl::Pacman;
}