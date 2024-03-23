/**
 * @file r2_node.hpp
 * @brief R2のメインノードを定義する
 */

#pragma once

#include <cmath>
#include <atomic>
#include <utility>
#include <chrono>
#include <tuple>
#include <memory>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <can_plugins2/msg/frame.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <nav2_msgs/srv/load_map.hpp>

#include <my_include/void.hpp>
#include <my_include/xyth.hpp>
#include <my_include/debug_print.hpp>

#include "robot_config.hpp"
#include "robot_io.hpp"
#include "omni4.hpp"
#include "logicool.hpp"
#include "ros2_utils.hpp"
#include "shirasu.hpp"

namespace nhk24_2nd_ws::r2::r2_node::impl {
	using namespace std::chrono_literals;

	using void_::Void;
	using xyth::Xy;
	using xyth::Xyth;
	using debug_print::printlns;

	using robot_io::Io;
	using robot_io::MapName;
	using robot_io::StateName;
	using robot_io::ManualAuto;
	using omni4::Omni4;
	using logicool::Buttons;
	using logicool::Axes;
	using ros2_utils::get_pose;
	using shirasu::target_frame;

	namespace {
		struct R2Node final : rclcpp::Node {
			Io * io;
			const volatile std::atomic_flag * kill_interrupt;
			const volatile std::atomic_flag * user_defined_interrupt_done;
			Omni4 omni4;

			tf2_ros::TransformBroadcaster tf2_broadcaster;
			tf2_ros::Buffer tf2_buffer;
			tf2_ros::TransformListener tf2_listener;

			rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_tx;
			rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
			
			rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
			// 本当はサービスにすべきだが、サービスを作るのめんどいので...
			rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_recover_state_sub;

			rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr map_server_change_state_client;
			rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr amcl_change_state_client;
			rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client;
			rclcpp::TimerBase::SharedPtr timer;

			R2Node (
				Io *const io
				, const volatile std::atomic_flag *const kill_interrupt
				, const volatile std::atomic_flag *const user_defined_interrupt_done
				, const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
			)
				: rclcpp::Node("r2", options)
				, io{io}
				, kill_interrupt{kill_interrupt}
				, user_defined_interrupt_done{user_defined_interrupt_done}
				, omni4{Omni4::make()}
				, tf2_broadcaster{*this}
				, tf2_buffer{this->get_clock()}
				, tf2_listener{tf2_buffer}
				, can_tx{this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10)}
				, initial_pose_pub{this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1)}
				, joy_sub(this->create_subscription<sensor_msgs::msg::Joy>("joy", 1,
					[this](const sensor_msgs::msg::Joy::SharedPtr joy) {
						this->joy_callback(std::move(*joy));
					}
				))
				, manual_recover_state_sub(this->create_subscription<std_msgs::msg::String>("r2/manual_recover_state", 1,
					[this](const std_msgs::msg::String::SharedPtr msg) {
						auto received = StateName::from_string(msg->data);
						if(received.has_value()) {
							this->io->manual_recover_state.set(std::move(received));
						}
						else {
							printlns("invalid manual_recover_state: ", msg->data);
						}
					}
				))
				, map_server_change_state_client(this->create_client<lifecycle_msgs::srv::ChangeState>("map_server/change_state"))
				, amcl_change_state_client(this->create_client<lifecycle_msgs::srv::ChangeState>("amcl/change_state"))
				, load_map_client(this->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map"))
				, timer(this->create_wall_timer(10ms, [this]() {
					this->timer_callback();
				}))
			{
				io->change_map = [this](const MapName::Enum map_name, const Xyth& initial_pose) -> std::future<void> {
					return std::async(std::launch::async, [this, map_name, initial_pose]() {
						this->change_map(map_name, initial_pose);
					});
				};
			}

			void joy_callback(sensor_msgs::msg::Joy&& joy) {
				if(joy.buttons[Buttons::back]) {
					this->io->change_manual_auto.set(ManualAuto::manual);
				}
				else if(joy.axes[Axes::cross_UD] < 0.5) {
					if(joy.buttons[Buttons::a]) {
						this->io->change_manual_auto.try_set(ManualAuto::auto_specified);
					}
					else if(joy.buttons[Buttons::b]) {
						this->io->change_manual_auto.try_set(ManualAuto::auto_evacuated);
					}
				}
				this->io->manual_speed.set(Xyth::make (
					Xy::make (
						joy.axes[Axes::l_stick_LR] * robot_config::max_vxy / std::sqrt(2.0)
						, joy.axes[Axes::l_stick_UD] * robot_config::max_vxy / std::sqrt(2.0)
					)
					, joy.axes[Axes::r_stick_LR] * robot_config::max_vth
				));
			}

			void timer_callback() {
				// check kill_interrupt
				if(this->kill_interrupt->test()) {
					this->io->change_manual_auto.set(ManualAuto::manual);
					this->io->manual_speed.set(Xyth::make(Xy::make(0.0, 0.0), 0.0));
					this->io->kill_interrupt.set(Void{});
					rclcpp::shutdown();
					return;
				}

				// input
				const auto current_pose = get_pose(this->tf2_buffer, "map", "base_link");
				if(current_pose.has_value()) {
					this->io->current_pose.set(*current_pose);
				}

				// output
				const auto body_speed = this->io->body_speed.get();
				const auto motor_speeeds = this->omni4.update(body_speed);
				this->send_motor_speeds(motor_speeeds);

				// debug
				printlns("body_speed: ", body_speed);
				printlns("motor_speeeds: ", motor_speeeds);
				const auto manual_speed = this->io->manual_speed.get();
				printlns("manual_speed: ", manual_speed);
			}

			void send_motor_speeds(const std::array<double, 4>& speeds) {
				for(u32 i = 0; i < 4; ++i) {
					if(const auto id = robot_config::ids[i]; id) {
						this->can_tx->publish(target_frame(*id, speeds[i]));
					}
				}
			}

			void wait_for_service() const {
				this->map_server_change_state_client->wait_for_service();
				this->amcl_change_state_client->wait_for_service();
				this->load_map_client->wait_for_service();
			}

			void change_map(const MapName::Enum filepath, const Xyth& initial_pose) {
				// disable amcl
				{
					auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
					req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
					this->amcl_change_state_client->async_send_request(std::move(req)).get();
				}
				
				// change map
				{
					auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
					req->map_url = std::string{"map/"} + std::string(MapName::to_filepath(filepath)) + ".yaml";
					this->load_map_client->async_send_request(std::move(req)).get();
				}

				// set initialpose
				{
					auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
					initial_pose_msg.header.frame_id = "map";
					initial_pose_msg.pose.pose.position.x = initial_pose.xy.x;
					initial_pose_msg.pose.pose.position.y = initial_pose.xy.y;
					initial_pose_msg.pose.pose.orientation.z = std::sin(initial_pose.th / 2);  // ここら辺ほんとにあってるか不安(copilotくんはこう出してる)
					initial_pose_msg.pose.pose.orientation.w = std::cos(initial_pose.th / 2);
					this->initial_pose_pub->publish(initial_pose_msg);
				}

				// enable amcl
				{
					auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
					req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
					this->amcl_change_state_client->async_send_request(std::move(req)).get();
				}
			}
		};

		inline auto make_node (
			const volatile std::atomic_flag *const kill_interrupt
			, const volatile std::atomic_flag *const user_defined_interrupt_done
			, const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
		) -> std::tuple<std::shared_ptr<R2Node>, std::future<std::unique_ptr<Io>>> {
			auto io = Io::make_unique();
			auto node = std::make_shared<R2Node>(io.get(), kill_interrupt, user_defined_interrupt_done, options);
			auto fut = std::async(std::launch::async, [io = std::move(io), node = node]() mutable -> std::unique_ptr<Io> {
				// ここ順番が大事。
				// どいつがどのライフサイクルか、どのライフサイクルでどのクエリが投げられるかを考えること
				{
					// map_serverの準備ができるまで待つ
					node->map_server_change_state_client->wait_for_service();

					// map_serverをコンフィグレーション状態にする
					auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
					req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
					node->map_server_change_state_client->async_send_request(std::move(req)).get();

					// map_serverをアクティブ状態にする
					auto req2 = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
					req2->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
					node->map_server_change_state_client->async_send_request(std::move(req2)).get();
				}
				{
					// amclの準備ができるまで待つ
					node->amcl_change_state_client->wait_for_service();

					// amclをコンフィグレーション状態にする
					auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
					req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
					node->amcl_change_state_client->async_send_request(std::move(req)).get();
					
					// amclをアクティブ状態にする
					auto req2 = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
					req2->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
					node->amcl_change_state_client->async_send_request(std::move(req2)).get();
				}
				{
					// map_serverのload_mapの準備ができるまで待つ
					node->load_map_client->wait_for_service();
				}
				/// @todo change_mapをここで呼び、map_serverにyaml渡したりamclのyamlに初期位置書くのをやめられるならやめるべきかも
				// {
				// 	node->change_map(MapName::area1, /* todo */);
				// }

				return std::move(io);
			});
			return {std::move(node), std::move(fut)};
		}
	}
}

namespace nhk24_2nd_ws::r2::r2_node {
	using impl::R2Node;
	using impl::make_node;
}