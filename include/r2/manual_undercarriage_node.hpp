#pragma once

#include <cmath>
#include <utility>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <can_plugins2/msg/frame.hpp>

#include <my_include/xyth.hpp>
#include <my_include/mutexed.hpp>

#include "robot_config.hpp"
#include "logicool.hpp"
#include "shirasu.hpp"
#include "omni4.hpp"

namespace nhk24_2nd_ws::r2::manual_undercarriage_node::impl {
	using namespace std::chrono_literals;
	using xyth::Xyth;
	using mutexed::Mutexed;
	using logicool::Axes;
	using shirasu::Command;
	using shirasu::command_frame;
	using shirasu::target_frame;
	using omni4::Omni4;

	struct ManualUndercarriageNode final : rclcpp::Node {
		Mutexed<Xyth> target;
		Omni4 omni4;

		rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_tx;
		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy;

		ManualUndercarriageNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
			: rclcpp::Node("manual_undercarriage_node", options)
			, target(Mutexed<Xyth>::make(Xyth::zero()))
			, omni4(Omni4::make())
			, can_tx(this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10))
			, timer(this->create_wall_timer(10ms, [this]() {
				const auto speeds = omni4.update(target.get());
				for(const auto i : {0, 1, 2, 3}) if(const auto [id, speed] = std::pair{robot_config::ids[i], speeds[i]}; id) {
					can_tx->publish(target_frame(*id, speed));
					rclcpp::sleep_for(1ms);
				}
			}))
			, joy(this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
				Xyth target;
				target.xy.x = -msg->axes[Axes::l_stick_LR] * robot_config::max_vxy / std::sqrt(2);
				target.xy.y = msg->axes[Axes::l_stick_UD] * robot_config::max_vxy / std::sqrt(2);
				target.th = msg->axes[Axes::r_stick_LR] * robot_config::max_vth;
				this->target.set(target);
			}))
		{}
	};
}

namespace nhk24_2nd_ws::r2::manual_undercarriage_node {
	using impl::ManualUndercarriageNode;
}