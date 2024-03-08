/**
 * @file robot_config.hpp
 * @brief ロボットの設定を定義する
 */

#pragma once

#include <array>
#include <optional>

#include <include/std_types.hpp>

namespace nhk24_2nd_ws::r2::robot_config {
	constexpr auto ids = std::array<std::optional<u32>, 4> {
		std::make_optional(0x160)
		, std::make_optional(0x144)
		, std::make_optional(0x110)
		, std::make_optional(0x154)
	};
}