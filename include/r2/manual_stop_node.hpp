#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <robomas_plugins/msg/robomas_frame.hpp>

#include "shirasu.hpp"
#include "robomasu.hpp"
#include "servo.hpp"
#include "logicool.hpp"
#include "robot_config.hpp"

namespace nhk24_2nd_ws::r2::manual_stop_node::impl {
	using namespace std::chrono_literals;
	using shirasu::Command;
	using shirasu::command_frame;
	using robomasu::Mode;
	using robomasu::make_param_frame;
	using servo::change_mode_frame;
	using logicool::Buttons;

	struct ManualStopNode final : rclcpp::Node {
		// rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_pub;
		rclcpp::Publisher<robomas_plugins::msg::RobomasFrame>::SharedPtr robomas_pub;
		rclcpp::Publisher<robomas_plugins::msg::RobomasFrame>::SharedPtr robomas2_pub;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

		ManualStopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
			: rclcpp::Node("manual_stop_node", options)
			// , can_pub(this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10))
			, robomas_pub(this->create_publisher<robomas_plugins::msg::RobomasFrame>("robomas_frame", 10))
			, robomas2_pub(this->create_publisher<robomas_plugins::msg::RobomasFrame>("robomas_frame2", 10))
			, joy_sub(this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
				if (msg->buttons[Buttons::back]) {
					for(u32 i = 0; i < 5; ++i) {
						robomas_pub->publish(make_param_frame(i, Mode::Disable));
						rclcpp::sleep_for(10ms);
					}
					for(u32 i = 0; i < 4; ++i) {
						robomas2_pub->publish(make_param_frame(i, Mode::Disable));
						rclcpp::sleep_for(10ms);
					}
				} else if (msg->buttons[Buttons::start]) {
					for(u32 i = 0; i < 4; ++i) {
						robomas_pub->publish(make_param_frame(i, Mode::Velocity));
						rclcpp::sleep_for(10ms);
					}
					robomas_pub->publish(make_param_frame(4, Mode::Position));

					for(u32 i = 0; i < 4; ++i) {
						robomas2_pub->publish(make_param_frame(i, Mode::Velocity));
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