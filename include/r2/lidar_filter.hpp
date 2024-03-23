#pragma once

#include <cmath>
#include <concepts>
#include <tuple>
#include <utility>
#include <vector>
#include <ranges>
#include <type_traits>

#include <my_include/std_types.hpp>
#include <my_include/xyth.hpp>

namespace nhk24_2nd_ws::r2::lidar_filter::impl {
	using nhk24_2nd_ws::xyth::Xy;
	using nhk24_2nd_ws::xyth::Xyth;
	inline constexpr auto xyop = nhk24_2nd_ws::xyth::impl::XyOp{};

	struct LidarScan final {
		std::vector<float> range;
		float angle_min;
		float angle_increment;

		static constexpr auto make(std::vector<float>&& range, const float angle_min, const float angle_max) noexcept -> LidarScan {
			return LidarScan{std::move(range), angle_min, (angle_max - angle_min) / range.size()};
		}

		constexpr auto r_th(const usize i) const noexcept -> std::pair<float, float> {
			return {range[i], angle_min + i * angle_increment};
		}
	};

	struct ChainableFilterMarker {};

	template<class T_>
	concept chainable_filter = requires(T_ filter, const double r, const double th) {
		requires std::derived_from<T_, ChainableFilterMarker>;
		{filter.update(r, th)} -> std::same_as<bool>;
	};

	template<chainable_filter ... Filters_>
	struct ChainedFilter final : ChainableFilterMarker {
		std::tuple<Filters_...> filters;

		auto update(const double r, const double th) -> bool {
			return []<usize ... indices_>(std::index_sequence<indices_ ...>, const double r, const double th, auto& filters) {
				return (std::get<indices_>(filters).update(r, th) && ...);
			}(std::index_sequence_for<Filters_...>{}, r, th, filters);
		}
	};

	inline auto filter_chain(chainable_filter auto&& ... filters) {
		return ChainedFilter<std::remove_cvref_t<decltype(filters)> ...>{{}, {std::forward<decltype(filters)>(filters) ...}};
	}

	template<class Filter_>
	requires chainable_filter<std::remove_cvref_t<Filter_>>
	inline auto apply_filter(LidarScan&& scan, Filter_&& filter) -> LidarScan {
		for(usize i = 0; i < scan.range.size(); i++) {
			const auto [r, th] = scan.r_th(i);
			if(not filter.update(r, th)) scan.range[i] = std::numeric_limits<float>::quiet_NaN();
		}
		return std::move(scan);
	}

	struct BoxFilter final : ChainableFilterMarker {
		Xyth lidar_pose;
		Xyth box_center_from_map;
		Xy box_half_diagonal;

		static constexpr auto make(const Xyth& lidar_pose, const Xyth& box_center_from_map, const Xy& box_half_diagonal) -> BoxFilter {
			return BoxFilter{{}, lidar_pose, box_center_from_map, box_half_diagonal};
		}

		constexpr auto update(const double r, const double th) -> bool {
			const auto point_xy_from_map = Xy::from_polar(r, th + lidar_pose.th) +xyop+ lidar_pose.xy;
			const auto point_xy_from_box_center = (point_xy_from_map -xyop- box_center_from_map.xy).rot(-box_center_from_map.th);
			return -box_half_diagonal.x <= point_xy_from_box_center.x && point_xy_from_box_center.x <= box_half_diagonal.x
				&& -box_half_diagonal.y <= point_xy_from_box_center.y && point_xy_from_box_center.y <= box_half_diagonal.y;
		}
	};

	struct ShadowFilter final : ChainableFilterMarker {
		float threshold_tan;
		u32 i;
		u32 size;
		const LidarScan * scan;
		u16 window;

		static constexpr auto make(const float threshold_angle, const u16 window, const LidarScan& scan) noexcept -> ShadowFilter {
			return ShadowFilter{{}, std::tan(threshold_angle), 0u, (u32)scan.range.size(), &scan, window};
		}

		constexpr auto update(const double r, const double th) -> bool {
			constexpr auto range_intersect = []<class T_>(const std::pair<T_, T_>& lhs, const std::pair<T_, T_>& rhs) noexcept -> std::pair<T_, T_> {
				return {std::max(lhs.first, rhs.first), std::min(lhs.second, rhs.second)};
			};

			const auto windowed = range_intersect.template operator()<i32>({(i32)i - (i32)window, (i32)i + (i32)window}, {0, (i32)size});
			for(const auto j : std::views::iota(windowed.first, windowed.second)) {
				const auto [other_r, other_th] = scan->r_th(j);
				if(is_shadow(r, other_r, other_th - th)) return false;
			}
			return true;
		}

		constexpr auto is_shadow(const float r, const float other_r, const float delta_theta) noexcept -> bool {
			const float diff_x = r * std::sin(delta_theta);
			const float diff_y = other_r - r * std::cos(delta_theta);
			return std::fabs(diff_y / diff_x) < threshold_tan;
		}
	};
}

namespace nhk24_2nd_ws::r2::lidar_filter {
	using impl::LidarScan;
	using impl::filter_chain;
	using impl::apply_filter;
	using impl::BoxFilter;
	using impl::ShadowFilter;
}