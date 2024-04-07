#pragma once

#include <array>
#include <chrono>
#include <shared_mutex>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>
#include <can_plugins2/msg/frame.hpp>

#include <my_include/std_types.hpp>
#include <my_include/mutexed.hpp>
#include "logicool.hpp"
#include "robomasu.hpp"
#include "servo.hpp"

namespace nhk24_2nd_ws::r2::collect_debug_node::impl {
	using namespace std::chrono_literals;
	using mutexed::Mutexed;
	using logicool::Axes;
	using logicool::Buttons;
	using robomasu::make_target_frame;
	using servo::change_targets03_frame;
	using servo::change_targets47_frame;

	enum class Rotation {
		stop
		, cw
		, ccw
	};

	enum class Lift {
		down
		, up
	};

	enum class Servo {
		close
		, open
	};

	class CollectDebugNode final : public rclcpp::Node {
		Mutexed<std::array<float, 4>> cw_speeds{Mutexed<std::array<float, 4>>::make({1000.0f, 1000.0f, 1000.0f, 1000.0f})};
		Mutexed<std::array<float, 4>> ccw_speeds{Mutexed<std::array<float, 4>>::make({1000.0f, 1000.0f, 1000.0f, 1000.0f})};
		Mutexed<float> low_position{Mutexed<float>::make(0.0f)};
		Mutexed<float> high_position{Mutexed<float>::make(2300.0f)};
		Mutexed<std::array<u16, 2>> servo_angles{Mutexed<std::array<u16, 2>>::make({20000, 50000})};
		Mutexed<Rotation> collect{Mutexed<Rotation>::make(Rotation::stop)};
		Mutexed<Rotation> load{Mutexed<Rotation>::make(Rotation::stop)};
		Mutexed<Lift> lift{Mutexed<Lift>::make(Lift::down)};
		Mutexed<Servo> servo{Mutexed<Servo>::make(Servo::close)};

		std::array<rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr, 5> robomas_target_pubs;
		rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_tx;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
		std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> cw_subs;
		std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> ccw_subs;
		std::array<rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr, 2> servo_subs;
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
			, can_tx(this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10))
			, joy_sub(this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
				if(msg->buttons[Buttons::lb]) {
					this->collect.set(Rotation::cw);
				} else if(msg->buttons[Buttons::rb]) {
					this->collect.set(Rotation::ccw);
				} else {
					this->collect.set(Rotation::stop);
				}

				if(msg->buttons[Buttons::x]) {
					this->load.set(Rotation::cw);
				} else if(msg->buttons[Buttons::y]) {
					this->load.set(Rotation::ccw);
				} else {
					this->load.set(Rotation::stop);
				}
				
				if(msg->axes[Axes::cross_UD] > 0.5) {
					this->lift.set(Lift::up);
				} else if(msg->axes[Axes::cross_UD] < -0.5) {
					this->lift.set(Lift::down);
				}

				if(msg->axes[Axes::cross_LR] > 0.5) {
					this->servo.set(Servo::close);
				} else if(msg->axes[Axes::cross_LR] < -0.5) {
					this->servo.set(Servo::open);
				}
			}))
			, cw_subs([this]() -> std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> {
				std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> subs{};
				for(size_t i = 0; i < 4; ++i) {
					subs[i] = this->create_subscription<std_msgs::msg::Float32>("cw_speed" + std::to_string(i), 10, [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
						const auto data = msg->data;
						this->cw_speeds.modify([i, data](auto& cw_speeds){cw_speeds[i] = data;});
					});
				}

				return subs;
			}())
			, ccw_subs([this]() -> std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> {
				std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> subs{};
				for(size_t i = 0; i < subs.size(); ++i) {
					subs[i] = this->create_subscription<std_msgs::msg::Float32>("ccw_speed" + std::to_string(i), 10, [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
						const auto data = msg->data;
						this->ccw_speeds.modify([i, data](auto& ccw_speeds){ccw_speeds[i] = data;});
					});
				}
				return subs;
			}())
			, servo_subs([this]() -> std::array<rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr, 2> {
				std::array<rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr, 2> subs{};
				subs[0] = this->create_subscription<std_msgs::msg::UInt16>("servo_close", 10, [this](const std_msgs::msg::UInt16::SharedPtr msg) {
					const auto data = msg->data;
					this->servo_angles.modify([data](auto& servo_angles){servo_angles[0] = data;});
				});
				subs[1] = this->create_subscription<std_msgs::msg::UInt16>("servo_open", 10, [this](const std_msgs::msg::UInt16::SharedPtr msg) {
					const auto data = msg->data;
					this->servo_angles.modify([data](auto& servo_angles){servo_angles[1] = data;});
				});
				return subs;
			}())
		{
			this->timer = this->create_wall_timer(50ms, [this] {
				const auto [collect, load, lift, servo, cw_speeds, ccw_speeds, servo_angles] = [this]() -> std::tuple<Rotation, Rotation, Lift, Servo, std::array<float, 4>, std::array<float, 4>, std::array<u16, 2>> {
					return {this->collect.get(), this->load.get(), this->lift.get(), this->servo.get(), this->cw_speeds.get(), this->ccw_speeds.get(), this->servo_angles.get()};  // fmapほしい！！
				}();

				switch(collect) {
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
						this->robomas_target_pubs[4]->publish(make_target_frame(this->low_position.get()));
						break;
					case Lift::up:
						this->robomas_target_pubs[4]->publish(make_target_frame(this->high_position.get()));
						break;
				}
				rclcpp::sleep_for(1ms);

				switch(servo) {
					case Servo::close:
						this->can_tx->publish(change_targets03_frame((u64)servo_angles[0]));  // サイズでdlcが決まるので注意
						break;
					case Servo::open:
						this->can_tx->publish(change_targets03_frame((u64)servo_angles[1]));  // 同上
						break;
				}
			});
		}
	};
}

namespace nhk24_2nd_ws::r2::collect_debug_node {
	using impl::CollectDebugNode;
}