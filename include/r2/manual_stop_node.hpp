#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <can_plugins2/msg/frame.hpp>

#include "shirasu.hpp"
#include "logicool.hpp"
#include "robot_config.hpp"

namespace nhk24_2nd_ws::r2::manual_stop_node::impl {
	using namespace std::chrono_literals;
	using shirasu::Command;
	using shirasu::command_frame;
	using logicool::Buttons;

	struct ManualStopNode final : rclcpp::Node {
		rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_pub;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

		ManualStopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
			: rclcpp::Node("manual_stop_node", options)
			, can_pub(this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10))
			, joy_sub(this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
				if (msg->buttons[Buttons::back]) {
					for(const auto id : robot_config::ids) {
						can_pub->publish(command_frame(*id, Command::shutdown));
						rclcpp::sleep_for(10ms);
					}
				} else if (msg->buttons[Buttons::start]) {
					for(const auto id : robot_config::ids) {
						can_pub->publish(command_frame(*id, Command::recover_velocity));
						rclcpp::sleep_for(10ms);
					}
				}
			}))
		{}
	};
}

namespace nhk24_2nd_ws::r2::manual_stop_node {
	using nhk24_2nd_ws::r2::manual_stop_node::impl::ManualStopNode;
}