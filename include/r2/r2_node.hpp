/**
 * @file r2_node.hpp
 * @brief R2のメインノードを定義する
 * @todo タイマ周期が複数箇所で指定されており、危険。直す良い方法を検討する
 */

#pragma once

#include <memory>
#include <stop_token>
#include <thread>
#include <shared_mutex>
#include <chrono>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <can_plugins2/msg/frame.hpp>

#include <include/xyth.hpp>
#include <include/pid.hpp>
#include <include/mutexed.hpp>
#include <include/debug_print.hpp>

#include "state_machine.hpp"
#include "main.hpp"
#include "path_parser.hpp"
#include "ros2_utils.hpp"
#include "shirasu.hpp"
#include "logicool.hpp"
#include "robot_config.hpp"

namespace nhk24_2nd_ws::r2::r2_node::impl {
	using namespace std::chrono_literals;
	
	using xyth::Xy;
	using xyth::Xyth;
	using pid::Pid;
	using mutexed::Mutexed;
	using debug_print::printlns;
	using state_machine::StateBase;
	using state_machine::StateMachine;
	using state_machine::make_state;
	using main::TemporaryManual;
	using main::GotoArea;
	using main::Dancing;
	using main::GoUpSlope;
	using path_parser::path_load;
	using ros2_utils::get_pose;
	using shirasu::target_frame;
	using logicool::Axes;
	using logicool::Buttons;

	using XyPid = Pid<Xy, double, pid::trivial, xyth::XyOp{}, xyth::XyOp{}>;
	using ThPid = Pid<double, double>;
	
	enum class Mode : u8 {
		manual
		, automatic
	};

	namespace {
		struct R2Node final : rclcpp::Node {
			Mutexed<Xyth> current_pose;
			Mutexed<Xyth> manual_speed;
			Mutexed<std::optional<Mode>> change_mode;

			Mutexed<std::array<double, 4>> motor_speeds;

			StateMachine state_machine;
			std::jthread thd;

			tf2_ros::Buffer tf2_buffer;
			tf2_ros::TransformListener tf2_listener;

			rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_tx;
			
			rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
			rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub;
			rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client;
			rclcpp::TimerBase::SharedPtr timer;

			R2Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
				: rclcpp::Node("r2", options)
				, current_pose{Mutexed<Xyth>::make(Xyth::zero())}
				, manual_speed{Mutexed<Xyth>::make(Xyth::zero())}
				, motor_speeds{Mutexed<std::array<double, 4>>::make(std::array<double, 4>{0.0, 0.0, 0.0, 0.0})}
				, state_machine(StateMachine::make(this->goto_area2()))
				, thd([this](std::stop_token stoken) {
					state_machine.run(stoken);
				})
				, tf2_buffer{this->get_clock()}
				, tf2_listener{tf2_buffer}
				, can_tx{create_publisher<can_plugins2::msg::Frame>("can_tx", 10)}
				, joy_sub(create_subscription<sensor_msgs::msg::Joy>("joy", 1,
					[this](const sensor_msgs::msg::Joy::SharedPtr joy) {
						this->joy_callback(std::move(*joy));
					}
				))
				, stop_sub(create_subscription<std_msgs::msg::Empty>("r2/stop", 1,
					[this](const std_msgs::msg::Empty::SharedPtr) {
						thd.request_stop();
					}
				))
				, load_map_client(create_client<nav2_msgs::srv::LoadMap>("nav2/load_map"))
				, timer{}
			{
				this->timer = this->create_wall_timer(10ms, [this]() {
					this->timer_callback();
				});
			}

			void joy_callback(sensor_msgs::msg::Joy&& joy) {
				if(joy.buttons[Buttons::back]) {
					this->change_mode.set(Mode::manual);
				}
				else if(joy.buttons[Buttons::a]) {
					this->change_mode.set(Mode::automatic);
				}
				manual_speed.set(Xyth::make (
					Xy::make (
						joy.axes[Axes::l_stick_LR] * robot_config::max_vxy / std::sqrt(2.0)
						, joy.axes[Axes::l_stick_UD] * robot_config::max_vxy / std::sqrt(2.0)
					)
					, joy.axes[Axes::r_stick_LR] * robot_config::max_vth
				));
			}

			void timer_callback() {
				// printlns("r2_node: timer_callback");
				// input
				const auto current_pose = get_pose(this->tf2_buffer, "map", "base_link");
				if(current_pose.has_value()) {
					this->current_pose.set(*current_pose);
				}

				// output
				printlns(this->motor_speeds.get());
				this->send_motor_speeds(this->motor_speeds.get());

				// debug
				const auto manual_speed = this->manual_speed.get();
				printlns(manual_speed.xy.x, manual_speed.xy.y, manual_speed.th);
			}

			void send_motor_speeds(const std::array<double, 4>& speeds) {
				for(u32 i = 0; i < 4; ++i) {
					if(const auto id = robot_config::ids[i]; id) {
						this->can_tx->publish(target_frame(*id, speeds[i]));
					}
				}
			}

			auto to_manual(std::unique_ptr<StateBase>&& next_state) -> std::unique_ptr<StateBase> {
				auto state = make_state<TemporaryManual> (
					TemporaryManual::Content::make()
					, [this]() -> TemporaryManual::In {
						rclcpp::sleep_for(10ms);
						const auto manual_speed = this->manual_speed.get();

						return TemporaryManual::In {
							manual_speed
							, this->change_mode.get() == Mode::automatic
						};
					}
					, [this](TemporaryManual::Out&& out) {
						printlns("motor_speeds in output: ", out.motor_speeds);
						this->motor_speeds.set(out.motor_speeds);
					}
					, [this, next_state = std::move(next_state)](TemporaryManual::TransitArg&&) mutable -> std::unique_ptr<StateBase> {
						return {std::move(next_state)};
					}
				);

				return std::make_unique<decltype(state)>(std::move(state));
			}

			auto goto_area(const std::string_view pathfile, auto&& next_state, auto&& recover_manual) -> std::unique_ptr<StateBase>
			requires requires {
				{next_state()} -> std::same_as<std::unique_ptr<StateBase>>;
				{recover_manual()} -> std::same_as<std::unique_ptr<StateBase>>;
			}
			{
				auto path = path_load(pathfile);
				if(path.has_value()) {
					auto state = make_state<GotoArea> (
						GotoArea::Content::make (
							std::move(*path)
							, XyPid::make(1.0, 0.0, 0.0)
							, ThPid::make(1.0, 0.0, 0.0)
						)
						, [this]() -> GotoArea::In {
							rclcpp::sleep_for(10ms);
							const auto current_pose = this->current_pose.get();
							const auto change_to_manual = this->change_mode.get() == Mode::manual;

							return GotoArea::In {
								current_pose
								, change_to_manual
							};
						}
						, [this](GotoArea::Out&& out) {
							this->motor_speeds.set(out.motor_speeds);
						}
						, [this, next_state = std::move(next_state), recover_manual = std::move(recover_manual)](GotoArea::TransitArg&& targ) mutable {
							if(not targ.change_to_manual) return next_state();
							else return this->to_manual(recover_manual());
						}
					);
					return std::make_unique<decltype(state)>(std::move(state));
				}
				else {
					RCLCPP_ERROR_STREAM(this->get_logger(), "r2_node: failed to load path: " << path.error());
					return this->to_manual(recover_manual());
				}
			}

			auto goto_area2() -> std::unique_ptr<StateBase> {
				return this->goto_area("path/to_area2.txt", [this]() {
					return this->go_up_slope (
						[this]{return this->goto_area3();}
						, [this]{return this->goto_area2();}
					);
				}
				, [this] {
					return this->to_manual(this->goto_area2());
				});
			}

			auto goto_area3() -> std::unique_ptr<StateBase> {
				return this->goto_area("path/to_area3.txt", [this]() {
					return this->go_up_slope (
					[this]{return this->dancing(2s, true);}
					, [this]{return this->goto_area3();}
					);
				}
				, [this] {
					return this->to_manual(this->goto_area3());
				});
			}

			auto go_up_slope(auto&& next_state, auto&& recover_manual) -> std::unique_ptr<StateBase>
			requires requires {
				{next_state()} -> std::same_as<std::unique_ptr<StateBase>>;
				{recover_manual()} -> std::same_as<std::unique_ptr<StateBase>>;
			}
			{
				auto state = make_state<GoUpSlope> (
					GoUpSlope::Content::make()
					, [this]() -> GoUpSlope::In {
						rclcpp::sleep_for(10ms);
						const auto change_to_manual = this->change_mode.get() == Mode::manual;

						return GoUpSlope::In {
							change_to_manual
						};
					}
					, [this](GoUpSlope::Out&& out) {
						this->motor_speeds.set(out.motor_speeds);
					}
					, [this, next_state = std::move(next_state), recover_manual = std::move(recover_manual)](GoUpSlope::TransitArg&& targ) mutable {
						if(not targ.change_to_manual) return next_state();
						else return this->to_manual(recover_manual());
					}
				);

				return std::make_unique<decltype(state)>(std::move(state));
			}

			auto dancing(const std::chrono::seconds& duration, const bool left_turn) -> std::unique_ptr<StateBase> {
				auto state = make_state<Dancing> (
					Dancing::Content::make (
						duration
						, left_turn
					)
					, [this]() -> Dancing::In {
						rclcpp::sleep_for(10ms);
						const auto change_to_manual = this->change_mode.get() == Mode::manual;

						return Dancing::In {
							change_to_manual
						};
					}
					, [this](Dancing::Out&& out) {
						this->motor_speeds.set(out.motor_speeds);
					}
					, [this, duration, left_turn](Dancing::TransitArg&& targ) {
						if(targ.change_to_manual) return this->dancing(duration + 1s, !left_turn);
						else return this->to_manual(this->dancing(duration, left_turn));
					}
				);

				return std::make_unique<decltype(state)>(std::move(state));
			}
		};
	}
}

namespace nhk24_2nd_ws::r2::r2_node {
	using impl::R2Node;
}