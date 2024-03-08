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
	using main::GotoArea;
	using Dancing = main::Dancing<std::chrono::system_clock>;
	using path_parser::path_load;
	using ros2_utils::get_pose;
	using ros2_utils::get_system_timepoint;
	using shirasu::Command;
	using shirasu::command_frame;
	using shirasu::target_frame;

	using XyPid = Pid<Xy, double, pid::trivial, xyth::XyOp{}, xyth::XyOp{}>;
	using ThPid = Pid<double, double>;
	
	namespace {
		struct R2Node final : rclcpp::Node {
			Mutexed<GotoArea::In> goto_area_in;
			Mutexed<Dancing::In> dancing_in;
			Mutexed<std::array<double, 4>> motor_speeds;

			StateMachine state_machine;
			std::jthread thd;

			tf2_ros::Buffer tf2_buffer;
			tf2_ros::TransformListener tf2_listener;

			rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_tx;
			
			rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub;
			rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client;
			rclcpp::TimerBase::SharedPtr timer;

			R2Node()
				: rclcpp::Node("r2")
				, goto_area_in {
					Mutexed<GotoArea::In>::make(GotoArea::In {
						Xyth::zero()
						, (10ms).count()
					})
				}
				, dancing_in {
					Mutexed<Dancing::In>::make(Dancing::In {
						get_system_timepoint(*this->get_clock())
						, (10ms).count()
					})
				}
				, motor_speeds {
					Mutexed<std::array<double, 4>>::make(std::array<double, 4>{0.0, 0.0, 0.0, 0.0})
				}
				, state_machine(StateMachine::make(this->goto_area2()))
				, thd([this](std::stop_token stoken) {
					state_machine.run(stoken);
				})
				, tf2_buffer{this->get_clock()}
				, tf2_listener{tf2_buffer}
				, can_tx{create_publisher<can_plugins2::msg::Frame>("can_tx", 10)}
				, stop_sub(create_subscription<std_msgs::msg::Empty> (
					"r2/stop",
					1,
					[this](const std_msgs::msg::Empty::SharedPtr) {
						thd.request_stop();
					}
				))
				, load_map_client(create_client<nav2_msgs::srv::LoadMap>("nav2/load_map"))
				, timer{}
			{
				for(const auto id : robot_config::ids) {
					if(id.has_value()) {
						can_tx->publish(command_frame(*id, Command::recover_velocity));
						rclcpp::sleep_for(200ms);
					}
				}

				timer = create_wall_timer(10ms, [this]() {
					this->timer_callback();
				});
			}

			static auto make() -> std::unique_ptr<R2Node> {
				return std::make_unique<R2Node>();
			}

			void timer_callback() {
				auto now = get_system_timepoint(*this->get_clock());
				const auto current_pose = get_pose(this->tf2_buffer, "map", "base_link");
				this->goto_area_in.set(GotoArea::In {
					current_pose ? *current_pose : this->goto_area_in.get().current_pose
					, (10ms).count()
				});
				this->dancing_in.set(Dancing::In {
					now
					, (10ms).count()
				});

				const auto motor_speeds = this->motor_speeds.get();
				printlns("r2_node: motor_speeds: ", motor_speeds);
				for(u32 i = 0; i < 4; ++i) {
					if(robot_config::ids[i].has_value()) {
						this->can_tx->publish(target_frame(*robot_config::ids[i], motor_speeds[i]));
						rclcpp::sleep_for(200us);
					}
				}
			}

			auto goto_area2() -> std::unique_ptr<StateBase> {
				auto path = path_load("path/to_area2.txt");
				if(path.has_value()) {
					auto state = make_state<GotoArea> (
						GotoArea::Content::make (
							std::move(*path)
							, XyPid::make(1.0, 0.0, 0.0)
							, ThPid::make(1.0, 0.0, 0.0)
						)
						, [this]() -> GotoArea::In {
							rclcpp::sleep_for(10ms);
							return this->goto_area_in.get();
						}
						, [this](GotoArea::Out&& out) {
							this->motor_speeds.set(out.motor_speeds);
						}
						, [this](GotoArea::TransitArg&&) {
							return this->goto_area3();
						}
					);
					return std::make_unique<decltype(state)>(std::move(state));
				}
				else {
					RCLCPP_ERROR_STREAM(this->get_logger(), "r2_node: failed to load path: " << path.error());
					return nullptr;
				}
			}

			auto goto_area3() -> std::unique_ptr<StateBase> {
				nav2_msgs::srv::LoadMap::Request::SharedPtr request{};
				request->map_url = "map/area2.yaml";
				auto result = this->load_map_client->async_send_request(std::move(request));
				result.wait_for(3s);

				auto path = path_load("path/to_area3.txt");
				if(path.has_value()) {
					auto state = make_state<GotoArea> (
						GotoArea::Content::make (
							std::move(*path)
							, XyPid::make(1.0, 0.0, 0.0)
							, ThPid::make(1.0, 0.0, 0.0)
						)
						, [this]() -> GotoArea::In {
							rclcpp::sleep_for(10ms);
							return this->goto_area_in.get();
						}
						, [this](GotoArea::Out&& out) {
							this->motor_speeds.set(out.motor_speeds);
						}
						, [this](GotoArea::TransitArg&&) {
							return this->dancing(1s, true);
						}
					);
					return std::make_unique<decltype(state)>(std::move(state));
				}
				else {
					RCLCPP_ERROR_STREAM(this->get_logger(), "r2_node: failed to load path: " << path.error());
					return nullptr;
				}
			}

			auto dancing(const std::chrono::seconds& duration, const bool left_turn) -> std::unique_ptr<StateBase> {
				auto state = make_state<Dancing> (
					Dancing::Content::make (
						duration
						, left_turn
					)
					, [this]() -> Dancing::In {
						rclcpp::sleep_for(10ms);
						return this->dancing_in.get();
					}
					, [this](Dancing::Out&& out) {
						this->motor_speeds.set(out.motor_speeds);
					}
					, [this](Dancing::TransitArg&& targ) {
						return this->dancing(targ.turn_duration + 1s, !targ.left_turn);
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