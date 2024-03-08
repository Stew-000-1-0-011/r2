#pragma once

#include <cstring>
#include <utility>

#include <can_plugins2/msg/frame.hpp>

#include <include/std_types.hpp>

namespace nhk24_2nd_ws::r2::shirasu::impl {
	enum class Command : u8 {
		shutdown
		, recover
		, home
		, get_status
		, recover_current
		, recover_velocity
		, recover_position
	};
	
	inline auto command_frame(const u32 id, Command command) noexcept -> can_plugins2::msg::Frame {
		can_plugins2::msg::Frame ret{};
		ret.is_rtr = false;
		ret.is_extended = false;
		ret.is_error = false;
		ret.dlc = 1;
		ret.id = id;
		ret.data[0] = static_cast<u8>(command);
		return ret;
	}

	inline auto target_frame(const u32 id, const float target) noexcept -> can_plugins2::msg::Frame {
		can_plugins2::msg::Frame ret{};
		ret.is_rtr = false;
		ret.is_extended = false;
		ret.is_error = false;
		ret.dlc = 4;
		ret.id = id + 1;

		std::memcpy(ret.data.data(), &target, sizeof(target));
		std::swap(ret.data[0], ret.data[3]);
		std::swap(ret.data[1], ret.data[2]);

		return ret;
	}
}

namespace nhk24_2nd_ws::r2::shirasu {
	using impl::Command;
	using impl::command_frame;
	using impl::target_frame;
}