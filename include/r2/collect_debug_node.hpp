#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>

#include "logicool.hpp"

namespace nhk24_2nd_ws::r2::collect_debug_node::impl {
	using logicool::Axes;

	class CollectDebugNode final : public rclcpp::Node {
		float cw_speed{1000.0f};
		float ccw_speed{1000.0f};

		rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target_pub;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cw_sub;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ccw_sub;

		public:
		CollectDebugNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
			: rclcpp::Node("collect_debug_node", options)
			, robomas_target_pub(this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target", 10))
			, joy_sub(this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
				robomas_plugins::msg::RobomasTarget target{};

				if(msg->axes[Axes::cross_UD] > 0.5) {
					target.target = this->cw_speed;
				} else if(msg->axes[Axes::cross_UD] < -0.5) {
					target.target = this->ccw_speed;
				} else {
					target.target = 0.0f;
				}

				this->robomas_target_pub->publish(target);
			}))
			, cw_sub(this->create_subscription<std_msgs::msg::Float32>("cw_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
				this->cw_speed = msg->data;
			}))
			, ccw_sub(this->create_subscription<std_msgs::msg::Float32>("ccw_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
				this->ccw_speed = msg->data;
			}))
		{}
	};
}

namespace nhk24_2nd_ws::r2::collect_debug_node {
	using impl::CollectDebugNode;
}