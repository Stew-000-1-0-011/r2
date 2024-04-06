#pragma once

#include <array>
#include <chrono>
#include <shared_mutex>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>

#include <my_include/mutexed.hpp>
#include "logicool.hpp"
#include "robomasu.hpp"

namespace nhk24_2nd_ws::r2::collect_debug_node::impl {
	using namespace std::chrono_literals;
	using mutexed::Mutexed;
	using logicool::Axes;
	using logicool::Buttons;
	using robomasu::make_target_frame;

	enum class Rotation {
		stop
		, cw
		, ccw
	};

	enum class Lift {
		down
		, up
	};

	class CollectDebugNode final : public rclcpp::Node {
		std::shared_mutex speeds_mtx{};
		std::shared_mutex rotate_mtx{};
		std::array<float, 4> cw_speeds{1000.0f, 1000.0f, 1000.0f, 1000.0f};
		std::array<float, 4> ccw_speeds{1000.0f, 1000.0f, 1000.0f, 1000.0f};
		float low_position{0.0f};
		float high_position{2300.0f};
		Rotation collect{Rotation::stop};
		Rotation load{Rotation::stop};
		Lift lift{Lift::down};

		std::array<rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr, 5> robomas_target_pubs;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
		std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> cw_subs;
		std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> ccw_subs;
		rclcpp::TimerBase::SharedPtr timer;

		public:
		CollectDebugNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
			: rclcpp::Node("collect_debug_node", options)
			, robomas_target_pubs([this]{
				std::array<rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr, 5> pubs{};
				for(size_t i = 0; i < pubs.size(); ++i) {
					pubs[i] = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target" + std::to_string(i), 10);
				}
				return pubs;
			}())
			, joy_sub(this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
				std::lock_guard lock(this->rotate_mtx);

				if(msg->buttons[Buttons::lb]) {
					this->collect = Rotation::cw;
				} else if(msg->buttons[Buttons::rb]) {
					this->collect = Rotation::ccw;
				} else {
					this->collect = Rotation::stop;
				}

				if(msg->buttons[Buttons::x]) {
					this->load = Rotation::cw;
				} else if(msg->buttons[Buttons::y]) {
					this->load = Rotation::ccw;
				} else {
					this->load = Rotation::stop;
				}
				
				if(msg->axes[Axes::cross_UD] > 0.5) {
					this->lift = Lift::up;
				} else if(msg->axes[Axes::cross_UD] < -0.5) {
					this->lift = Lift::down;
				}
			}))
			, cw_subs([this]() -> std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> {
				std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> subs{};
				for(size_t i = 0; i < 4; ++i) {
					subs[i] = this->create_subscription<std_msgs::msg::Float32>("cw_speed" + std::to_string(i), 10, [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
						std::lock_guard lock(this->speeds_mtx);
						this->cw_speeds[i] = msg->data;
					});
				}

				return subs;
			}())
			, ccw_subs([this]() -> std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> {
				std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> subs{};
				for(size_t i = 0; i < subs.size(); ++i) {
					subs[i] = this->create_subscription<std_msgs::msg::Float32>("ccw_speed" + std::to_string(i), 10, [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
						std::lock_guard lock(this->speeds_mtx);
						this->ccw_speeds[i] = msg->data;
					});
				}
				return subs;
			}())
		{
			this->timer = this->create_wall_timer(10ms, [this] {
				const auto [collect, load, lift, cw_speeds, ccw_speeds] = [this]() -> std::tuple<Rotation, Rotation, Lift, std::array<float, 4>, std::array<float, 4>> {
					std::shared_lock lock(this->rotate_mtx);
					std::shared_lock lock2(this->speeds_mtx);

					return {this->collect, this->load, this->lift, this->cw_speeds, this->ccw_speeds};
				}();

				switch(this->collect) {
					case Rotation::stop:
						for(size_t i = 0; i < 3; ++i) {
							this->robomas_target_pubs[i]->publish(make_target_frame(0.0f));
							rclcpp::sleep_for(1ms);
						}
						break;
					case Rotation::cw:
						for(size_t i = 0; i < 3; ++i) {
							this->robomas_target_pubs[i]->publish(make_target_frame(cw_speeds[i]));
							rclcpp::sleep_for(1ms);
						}
						break;
					case Rotation::ccw:
						for(size_t i = 0; i < 3; ++i) {
							this->robomas_target_pubs[i]->publish(make_target_frame(-ccw_speeds[i]));
							rclcpp::sleep_for(1ms);
						}
						break;
				}

				switch(load) {
					case Rotation::stop:
						this->robomas_target_pubs[3]->publish(make_target_frame(0.0f));
						break;
					case Rotation::cw:
						this->robomas_target_pubs[3]->publish(make_target_frame(cw_speeds[3]));
						break;
					case Rotation::ccw:
						this->robomas_target_pubs[3]->publish(make_target_frame(-ccw_speeds[3]));
						break;
				}
				rclcpp::sleep_for(1ms);

				switch(lift) {
					case Lift::down:
						this->robomas_target_pubs[4]->publish(make_target_frame(low_position));
						break;
					case Lift::up:
						this->robomas_target_pubs[4]->publish(make_target_frame(high_position));
						break;
				}
				rclcpp::sleep_for(1ms);
			});
		}
	};
}

namespace nhk24_2nd_ws::r2::collect_debug_node {
	using impl::CollectDebugNode;
}