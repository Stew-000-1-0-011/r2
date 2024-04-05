#pragma once

#include <my_include/std_types.hpp>

#include <robomas_plugins/msg/robomas_frame.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nhk24_2nd_ws::r2::robomasu {
	enum class Mode : u8 {
		Disable = 0,
		Velocity = 1,
		Position = 4
	};

	inline auto make_target_frame(const float target) -> robomas_plugins::msg::RobomasTarget {
		robomas_plugins::msg::RobomasTarget msg{};
		msg.target = target;
		return msg;
	}

	auto make_param_frame(const u16 id, const Mode mode) -> robomas_plugins::msg::RobomasFrame {
		robomas_plugins::msg::RobomasFrame frame{};
		frame.mode = static_cast<u8>(mode);
		frame.motor = id;
		frame.c620 = 0;
		frame.temp = 50;
		frame.velkp = 0.25;
		frame.velki = 9;
		frame.poskp = 0.5;
		frame.stable_pos_limit_vel = 25;
		return frame;
	}
}