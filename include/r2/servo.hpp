#pragma once

#include <cstring>

#include <can_plugins2/msg/frame.hpp>

#include <my_include/std_types.hpp>

namespace nhk24_2nd_ws::r2::servo {
	inline auto change_mode_frame(const bool is_activate) -> can_plugins2::msg::Frame {
		can_plugins2::msg::Frame ret{};
		ret.is_rtr = false;
		ret.is_extended = false;
		ret.is_error = false;
		ret.dlc = 1;
		ret.id = 0x300;
		ret.data[0] = is_activate;
		return ret;
	}

	inline auto change_targets03_frame(const u64 targets) -> can_plugins2::msg::Frame {
		can_plugins2::msg::Frame ret{};
		ret.is_rtr = false;
		ret.is_extended = false;
		ret.is_error = false;
		ret.dlc = 8;
		ret.id = 0x301;
		std::memcpy(ret.data.data(), &targets, sizeof(targets));
		return ret;
	}

	inline auto change_targets47_frame(const u64 targets) -> can_plugins2::msg::Frame {
		can_plugins2::msg::Frame ret{};
		ret.is_rtr = false;
		ret.is_extended = false;
		ret.is_error = false;
		ret.dlc = 8;
		ret.id = 0x302;
		std::memcpy(ret.data.data(), &targets, sizeof(targets));
		return ret;
	}
}