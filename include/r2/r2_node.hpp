/**
 * @file r2_node.hpp
 * @brief R2のメインノードを定義する
 */

#pragma once

#include <utility>
#include <optional>
#include <array>
#include <string>
#include <string_view>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <robomas_plugins/msg/robomas_target.hpp>
#include <robomas_plugins/msg/frame.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <nhk24_utils/msg/balls.hpp>

#include <my_include/void.hpp>
#include <my_include/xyth.hpp>
#include <my_include/std_types.hpp>
#include <my_include/debug_print.hpp>
// #include <my_include/sum_last_n.hpp>
#include <my_include/operator_generator.hpp>
#include <my_include/mutexed.hpp>
#include <my_include/lap_timer.hpp>

#include <nhk24_utils/vec3d.hpp>

#include "geometry_msgs_convertor.hpp"
#include "robot_config.hpp"
#include "omni4.hpp"
#include "logicool.hpp"
#include "ros2_utils.hpp"
#include "robomasu.hpp"
#include "servo.hpp"
#include "mode.hpp"
#include "field_constant.hpp"
#include "r2_controller.hpp"

#include "modes/manual.hpp"

namespace nhk24_2nd_ws::r2::r2_node::impl {
	using namespace std::chrono_literals;

	using void_::Void;
	using xyth::Xy;
	using xyth::Xyth;
	using xyth::XythScalar;
	using xyth::XyOp;
	using xyth::XythOp;
	using debug_print::printlns_to;
	// using sum_last_n::SumLastN;
	using operator_generator::BinaryLeftOp;
	using mutexed::Mutexed;
	using lap_timer::LapTimer;

	using nhk24_utils::stew::vec3d::Vec3d;

	using omni4::Omni4;
	using logicool::Buttons;
	using logicool::Axes;
	using ros2_utils::get_pose;
	using robomasu::make_target_frame;
	using servo::change_targets03_frame;
	using mode::ModeName;
	using mode::ModeArg;
	using field_constant::SiloIndex;
	using r2_controller::R2Controller;
	
	using modes::pursuit::Area1Start;
	using modes::pursuit::Area2Start;
	using modes::pursuit::GotoCenterStorage;
	using modes::pursuit::GotoSiloWatchPoint;
	using modes::pursuit::GotoSilo;
	using modes::pursuit::PursuitIn;
	using modes::pursuit::PursuitOut;
	using modes::collect_ball::CollectBall;
	using modes::watch_silo::WatchSilo;
	using modes::harvest::Harvest;
	using modes::manual::Manual;
	using ModeContinue = Manual::ModeContinue;

	struct XythLinear final
		: BinaryLeftOp<"+", +[](const Xyth& lhs, const Xyth& rhs) -> Xyth {
			return Xyth::make(lhs.xy +XyOp{}+ rhs.xy, lhs.th + rhs.th);
		}>
		, BinaryLeftOp<"-", +[](const Xyth& lhs, const Xyth& rhs) -> Xyth {
			return Xyth::make(lhs.xy -XyOp{}- rhs.xy, lhs.th - rhs.th);
		}>
		, BinaryLeftOp<"/", +[](const Xyth& lhs, const double rhs) -> Xyth {
			return Xyth::make(lhs.xy /XyOp{}/ rhs, lhs.th / rhs);
		}>
	{};

	struct R2Node final : rclcpp::Node {
		Mutexed<bool> to_manual;
		Mutexed<bool> killed;

		Mutexed<Xyth> current_pose;
		Mutexed<Xyth> current_speed;
		Mutexed<std::optional<double>> ball_direction;
		Mutexed<double> forward_speed;
		Mutexed<bool> collected_correctly;
		Mutexed<std::optional<SiloIndex::Enum>> target_silo;
		Mutexed<Xyth> manual_speed;
		Mutexed<std::optional<std::variant<ModeContinue, ModeName::Enum>>> change_to_auto;
		
		Mutexed<Xyth> target_speed;
		Mutexed<bool> servo_open;

		LapTimer dt_timer;
		Omni4 omni4;
		Mutexed<Xyth> last_odompose;
		Mutexed<std::optional<ModeName::Enum>> recover_mode;

		tf2_ros::TransformBroadcaster tf2_broadcaster;
		tf2_ros::Buffer tf2_buffer;
		tf2_ros::TransformListener tf2_listener;

		std::array<rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr, 4> robomas_target_frame_pubs;
		rclcpp::Publisher<robomas_plugins::msg::Frame>::SharedPtr robomas_can_tx_pub;
		
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
		// 本当はサービスにすべきだが、サービスを作るのめんどいので...
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_recover_mode_sub;
		rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_silo_sub;
		rclcpp::Subscription<nhk24_utils::msg::Balls>::SharedPtr balls_sub;

		rclcpp::TimerBase::SharedPtr timer;

		R2Node (
			const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
		)
			: rclcpp::Node("r2", options)
			, to_manual{Mutexed<bool>::make(false)}
			, killed{Mutexed<bool>::make(false)}
			, current_pose{Mutexed<Xyth>::make(Xyth::zero())}
			, current_speed{Mutexed<Xyth>::make(Xyth::zero())}
			, ball_direction{Mutexed<std::optional<double>>::make(std::nullopt)}
			, forward_speed{Mutexed<double>::make(0.0)}
			, collected_correctly{Mutexed<bool>::make(false)}
			, target_silo{Mutexed<std::optional<SiloIndex::Enum>>::make(std::nullopt)}
			, manual_speed{Mutexed<Xyth>::make(Xyth::zero())}
			, change_to_auto{Mutexed<std::optional<std::variant<ModeContinue, ModeName::Enum>>>::make(std::nullopt)}
			, target_speed{Mutexed<Xyth>::make(Xyth::zero())}
			, servo_open{Mutexed<bool>::make(false)}
			, dt_timer{LapTimer::make()}
			, omni4{Omni4::make()}
			, last_odompose{Mutexed<Xyth>::make(Xyth::zero())}
			, recover_mode{Mutexed<std::optional<ModeName::Enum>>::make(std::nullopt)}
			, tf2_broadcaster{*this}
			, tf2_buffer{this->get_clock()}
			, tf2_listener{tf2_buffer}
			, robomas_target_frame_pubs {
				[this]() -> std::array<rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr, 4> {
					std::array<rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr, 4> pubs;
					for(u32 i = 0; i < 4; ++i) {
						pubs[i] = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target2_" + std::to_string(i), 10);
					}
					return pubs;
				}()
			}
			, robomas_can_tx_pub{this->create_publisher<robomas_plugins::msg::Frame>("robomas_can_tx", 10)}
			, joy_sub(this->create_subscription<sensor_msgs::msg::Joy>("joy", 1,
				[this](const sensor_msgs::msg::Joy::SharedPtr joy) {
					this->joy_callback(std::move(*joy));
				}
			))
			, manual_recover_mode_sub(this->create_subscription<std_msgs::msg::String>("r2/manual_recover_mode", 1,
				[this](const std_msgs::msg::String::SharedPtr msg) {
					auto received = ModeName::from_string(msg->data);
					if(received.has_value()) {
						this->recover_mode.set(std::move(received));
					}
					else {
						printlns_to(std::osyncstream{std::cerr}, "invalid recover_mode: ", msg->data);
					}
				}
			))
			, target_silo_sub(this->create_subscription<std_msgs::msg::UInt8>("r2/target_silo", 1,
				[this](const std_msgs::msg::UInt8::SharedPtr msg) {
					const u8 data = msg->data - 1;
					if(data < SiloIndex::N) this->target_silo.set(static_cast<SiloIndex::Enum>(data));
					else printlns_to(std::osyncstream{std::cout}, "invalid target_silo: ", msg->data);
				}
			))
			, balls_sub(this->create_subscription<nhk24_utils::msg::Balls>("balls", 1,
				[this](const nhk24_utils::msg::Balls::SharedPtr balls) {
					this->balls_callback(std::move(*balls));
				}
			))
			, timer(this->create_wall_timer(10ms, [this]() {
				this->timer_callback();
			}))
		{}

		void joy_callback(sensor_msgs::msg::Joy&& joy) {
			if(joy.buttons[Buttons::back]) {
				this->to_manual.set(true);
			}
			// else if(joy.axes[Axes::cross_UD] < 0.5) {
			if(joy.buttons[Buttons::a]) {
				if(const auto next_mode = this->recover_mode.get()) this->change_to_auto.set(*next_mode);
				else printlns_to(std::osyncstream{std::cout}, "recover_mode is not set.");
			}
			else if(joy.buttons[Buttons::b]) {
				this->change_to_auto.set(ModeContinue{});
			}
			// }
			const auto manual_speed = Xyth::make (
				Xy::make (
					-joy.axes[Axes::l_stick_LR] * robot_config::max_vxy / std::sqrt(2.0)
					, joy.axes[Axes::l_stick_UD] * robot_config::max_vxy / std::sqrt(2.0)
				)
				, joy.axes[Axes::r_stick_LR] * robot_config::max_vth
			);
			this->manual_speed.set(manual_speed);
			this->forward_speed.set(manual_speed.xy.y);

			if(joy.buttons[Buttons::l_push]) {
				this->collected_correctly.set(true);
			}
		}

		void balls_callback(nhk24_utils::msg::Balls&& balls) {
			if(balls.balls.empty()) {
				this->ball_direction.set(std::nullopt);
			}
			else {
				constexpr auto calc_direction = [](const nhk24_utils::msg::Ball& ball) -> double {
					const auto v = Vec3d::from_msg<geometry_msgs::msg::Point>(ball.position);
					return std::atan2(v.x, v.z);
				};

				const double last_direction = this->ball_direction.get().value_or(0.0);
				double direction = calc_direction(balls.balls[0]);
				for(const auto& ball : balls.balls) {
					const double d = calc_direction(ball);
					if(std::abs(d - last_direction) < std::abs(direction - last_direction)) {
						direction = d;
					}
				}

				this->ball_direction.set(direction);
			}
		}

		void timer_callback() {
			const double dt = this->dt_timer.update().count();

			const auto current_pose = get_pose(this->tf2_buffer, "map", "true_base_link");
			if(current_pose.has_value()) {
				this->current_pose.set(*current_pose);
			}
			// printlns_to(std::osyncstream{std::cout}, "current_pose: ", this->current_pose.get());

			const auto odompose = get_pose(this->tf2_buffer, "odom", "true_base_link");
			if(odompose.has_value()) {
				const auto last_pose = this->last_odompose.modify(
					[odompose](auto& last) {
						const auto last_pose = last;
						last = *odompose;
						return last_pose;
					}
				);
				this->current_speed.set((*odompose -XythOp{}- last_pose) /XythOp{}/ XythScalar::from(dt));
			}

			// output
			const auto target_speed = this->target_speed.get();
			const auto motor_speeeds = this->omni4.update(target_speed);
			this->send_motor_speeds(motor_speeeds);

			const auto servo_open = this->servo_open.get();
			(void)servo_open;  // do nothing for now
		}

		void send_motor_speeds(const std::array<double, 4>& speeds) {
			this->robomas_target_frame_pubs[0]->publish(make_target_frame(-speeds[0]));
			this->robomas_target_frame_pubs[3]->publish(make_target_frame(-speeds[1]));
			this->robomas_target_frame_pubs[2]->publish(make_target_frame(-speeds[2]));
			this->robomas_target_frame_pubs[1]->publish(make_target_frame(-speeds[3]));
		}

		void kill() {
			this->killed.set(true);
			rclcpp::sleep_for(100ms);
			this->send_motor_speeds({0.0, 0.0, 0.0, 0.0});
			rclcpp::sleep_for(100ms);
		}
	};

	struct CommonIo final {
		std::shared_ptr<R2Node> node;
		Mutexed<bool> * accept_killed;

		auto killed() const -> bool {
			return this->node->killed.get();
		}

		auto to_manual() const -> bool {
			return this->node->to_manual.get();
		}

		void notify_killed() {
			this->accept_killed->set(true);
		}
	};

	template<class Inputor_, class Outputor_>
	struct Executor {
		Inputor_ input;
		Outputor_ output;
	};

	inline auto make_executor (
		auto&& inputor
		, auto&& outputor
	) {
		return [inputor = std::forward<decltype(inputor)>(inputor), outputor = std::forward<decltype(outputor)>(outputor)] {
				return Executor<std::remove_cvref_t<decltype(inputor)>, std::remove_cvref_t<decltype(outputor)>> {
				std::forward<decltype(inputor)>(inputor)
				, std::forward<decltype(outputor)>(outputor)
			};
		};
	}

	inline auto make_r2_controller_unique(const std::shared_ptr<R2Node>& node, Mutexed<bool>& killed) {
		return r2_controller::make_r2_controller_unique (
			make_executor (
				[node]() -> PursuitIn {
					return PursuitIn {
						node->current_pose.get()
						, node->current_speed.get()
					};
				}
				, [node](PursuitOut&& out) {
					node->target_speed.set(out.target_speed);
				}
			)
			, make_executor (
				[node]() -> CollectBall::In {
					return CollectBall::In {
						node->current_pose.get()
						, node->ball_direction.get()
						, node->forward_speed.get()
						, node->collected_correctly.get()
					};
				}
				, [node](CollectBall::Out&& out) {
					node->target_speed.set(out.target_speed);
				}
			)
			, make_executor (
				[node]() -> WatchSilo::In {
					return WatchSilo::In {
						node->target_silo.get()
						, node->current_pose.get()
						, node->current_speed.get()
					};
				}
				, [node](WatchSilo::Out&& out) {
					node->target_speed.set(out.target_speed);
				}
			)
			, make_executor (
				[node]() -> Harvest::In {
					return Harvest::In{};
				}
				, [node](Harvest::Out&& out) {
					node->target_speed.set(out.target_speed);
					node->servo_open.set(out.servo_open);
				}
			)
			, make_executor (
				[node]() -> Manual::In {
					return Manual::In {
						node->manual_speed.get()
						, node->change_to_auto.get()
					};
				}
				, [node](Manual::Out&& out) {
					node->target_speed.set(out.target_speed);
				}
			)
			, CommonIo {
				node
				, &killed
			}
			, ModeArg::template make<ModeName::area1_start>(Void{})
		);
	}
}

namespace nhk24_2nd_ws::r2::r2_node {
	using impl::R2Node;
	using impl::make_r2_controller_unique;
}