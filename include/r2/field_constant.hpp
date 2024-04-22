#pragma once

#include <variant>

#include <my_include/std_types.hpp>
#include <my_include/xyth.hpp>
#include <my_include/debug_print.hpp>

namespace nhk24_2nd_ws::r2::field_constant::impl {
	using xyth::Xy;
	using xyth::Xyth;

	constexpr double xy_error_max = 0.10;

	struct Section final {
		enum Enum : u8 {
			area1
			, area2
			, yellow
			, storage

			, slope12
			, slope2Y
			, slopeYS
		};

		static constexpr auto to_filepath(const Enum s) noexcept -> std::string_view {
			switch(s) {
				case Enum::area1:
					return "area1";
				case Enum::area2:
					return "area2";
				case Enum::yellow:
					return "yellow";
				case Enum::storage:
					return "storage";
				case Enum::slope12:
					return "slope12";
				case Enum::slope2Y:
					return "slope2Y";
				case Enum::slopeYS:
					return "slopeYS";
				default:
					return "unreachable: " __FILE__ ": to_filepath";
			}
		}
	};

	

	inline constexpr auto is_plane(const Section::Enum s) noexcept -> bool {
		return static_cast<u8>(s) <= static_cast<u8>(Section::storage);
	}

	struct BetweenSections final {
		Section::Enum a;
		Section::Enum b;

		constexpr auto is_one_side(const Section::Enum s) const noexcept -> bool {
			return this->a == s || this->b == s;
		}

		constexpr auto to_section() const noexcept -> Section::Enum {
			if(is_plane(this->a)) {
				return this->a;
			}
			else {
				return this->b;
			}
		}
	};

	inline constexpr auto is_in(const Section::Enum sec, const std::variant<Section::Enum, BetweenSections>& v) noexcept -> bool {
		if(auto p = std::get_if<Section::Enum>(&v)) {
			return *p == sec;
		}
		else {
			return std::get<BetweenSections>(v).is_one_side(sec);
		}
	}

	struct SiloIndex final {
		enum Enum : u8 {
			silo1
			, silo2
			, silo3
			, silo4
			, silo5
			
			, N
		};
	};

	#if 1
	namespace points {
		// start point of robot
		inline constexpr Xy area1_start = Xy::make(5.375, 0.350);
		inline constexpr Xy area2_start = Xy::make(5.525, 5.500);


		// begin and end point of slopes
		inline constexpr Xy slope12_1 = Xy::make(5.375, 3.000);
		inline constexpr Xy slope12_2 = Xy::make(5.375, 4.000);
		inline constexpr Xy slope2Y_2 = Xy::make(1.575, 7.000);
		inline constexpr Xy slope2Y_Y = Xy::make(1.575, 8.000);
		inline constexpr Xy slope_YS_Y = Xy::make(2.625, 10.00);
		inline constexpr Xy slope_YS_S = Xy::make(3.625, 10.00);

		// center of storage
		inline constexpr Xy storage_center = Xy::make(4.750, 10.000);

		// silo watch point
		inline constexpr Xy silo_watch_point = Xy::make(0.2300, 10.00);

		// front points of silo
		inline constexpr Xy silos[5] = {
			Xy::make(0.310, 8.500)
			, Xy::make(0.310, 9.250)
			, Xy::make(0.310, 10.00)
			, Xy::make(0.310, 10.75)
			, Xy::make(0.310, 11.50)
		};
	}

	namespace poses {
		inline constexpr double direction_to_storage = -std::numbers::pi / 2;
		inline constexpr auto area1_starting = Xyth::make(points::area1_start, 0.0);
		inline constexpr auto area2_starting = Xyth::make(points::area2_start, 0.0);
		inline constexpr auto look_around_storage = Xyth::make(points::storage_center, direction_to_storage);
		inline constexpr auto look_around_silo = Xyth::make(points::silo_watch_point, direction_to_storage);
	}

	inline constexpr auto get_zone(const Xy& p) noexcept -> std::variant<Section::Enum, BetweenSections> {
		if(p.y <= 4.000) {
			if(p.x <= 4.775 || p.y <= 3.000 - xy_error_max) {
				return Section::area1;
			} else if(p.y <= 3.000 + xy_error_max) {
				return BetweenSections{Section::area1, Section::slope12};
			} else if(p.y <= 4.000 - xy_error_max) {
				return Section::slope12;
			} else {
				return BetweenSections{Section::slope12, Section::area2};
			}
		}
		else if(p.y <= 8.000) {
			if(4.775 <= p.x && p.y <= 4.000 + xy_error_max) {
				return BetweenSections{Section::slope12, Section::area2};
			}
			else if(p.x <= 1.025 || 2.125 <= p.x || p.y <= 7.000 - xy_error_max) {
				return Section::area2;
			}
			else if(p.y <= 7.000 + xy_error_max) {
				return BetweenSections{Section::area2, Section::slope2Y};
			}
			else if(p.y <= 8.000 - xy_error_max) {
				return Section::slope2Y;
			}
			else {
				return BetweenSections{Section::slope2Y, Section::yellow};
			}
		}
		else {
			if(1.025 <= p.x && p.x <= 2.125 && p.y <= 8.000 + xy_error_max) {
				return BetweenSections{Section::slope2Y, Section::yellow};
			}
			else if(p.x <= 2.625 - xy_error_max) {
				return Section::yellow;
			}
			else if(p.x <= 2.625 + xy_error_max) {
				return BetweenSections{Section::yellow, Section::slopeYS};
			}
			else if(p.x <= 3.625 - xy_error_max) {
				return Section::slopeYS;
			}
			else if(p.x <= 3.625 + xy_error_max) {
				return BetweenSections{Section::slopeYS, Section::storage};
			}
			else {
				return Section::storage;
			}
		}
	}
	#else

	#endif

	inline constexpr auto where_am_i(const Xy& p, const Section::Enum last_i_am) noexcept -> Section::Enum {
		const auto zone = get_zone(p);
		// if(zone.index() == 0) {
		// 	debug_print::printlns_to(std::osyncstream{std::cout}, "zone: Section: ", int(std::get<0>(zone)));
		// }
		// else {
		// 	const auto [a, b] = std::get<1>(zone);
		// 	debug_print::printlns_to(std::osyncstream{std::cout}, "zone: betweenSection: ", int(a), int(b));
		// }
		if(is_in(last_i_am, zone)) {
			return last_i_am;
		}
		else {
			if(auto p = std::get_if<Section::Enum>(&zone)) {
				return *p;
			}
			else {
				return std::get<BetweenSections>(zone).to_section();
			}
		}
	}
}

namespace nhk24_2nd_ws::r2::field_constant {
	using impl::Section;
	using impl::BetweenSections;
	using impl::SiloIndex;
	using impl::is_plane;
	using impl::get_zone;
	using impl::where_am_i;
	namespace points = impl::points;
	namespace poses = impl::poses;
}