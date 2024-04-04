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
#include <future>
#include <array>
#include <thread>
#include <optional>
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
#include <my_include/sum_last_n.hpp>
#include <my_include/operator_generator.hpp>
#include <my_include/mutexed.hpp>

#include "geometry_msgs_convertor.hpp"
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
	using xyth::XyOp;
	using debug_print::printlns;
	using sum_last_n::SumLastN;
	using operator_generator::BinaryLeftOp;
	using mutexed::Mutexed;

	using robot_io::Io;
	using robot_io::MapName;
	using robot_io::StateName;
	using robot_io::ManualAuto;
	using omni4::Omni4;
	using logicool::Buttons;
	using logicool::Axes;
	using ros2_utils::get_pose;
	using shirasu::target_frame;

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

	namespace {
		struct R2Node final : rclcpp::Node {
			Io * io;
			Omni4 omni4;
			Mutexed<SumLastN<Xyth, XythLinear>> current_pose_sum;

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
				, const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
			)
				: rclcpp::Node("r2", options)
				, io{io}
				, omni4{Omni4::make()}
				, current_pose_sum{Mutexed<SumLastN<Xyth, XythLinear>>::make(SumLastN<Xyth, XythLinear>::make(10))}
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
						-joy.axes[Axes::l_stick_LR] * robot_config::max_vxy / std::sqrt(2.0)
						, joy.axes[Axes::l_stick_UD] * robot_config::max_vxy / std::sqrt(2.0)
					)
					, joy.axes[Axes::r_stick_LR] * robot_config::max_vth
				));
			}

			void timer_callback() {
				// input
				const auto current_pose = get_pose(this->tf2_buffer, "map", "base_link");
				if(current_pose.has_value()) {
					auto current_pose_average = this->current_pose_sum.modify([current_pose](auto& sum) -> Xyth {
						sum.push(*current_pose);
						return sum.get_average();
					});
					this->io->current_pose.set(current_pose_average);
				}
				// printlns("current_pose: ", this->io->current_pose.get());

				// output
				const auto body_speed = this->io->body_speed.get();
				const auto motor_speeeds = this->omni4.update(body_speed);
				this->send_motor_speeds(motor_speeeds);

				// debug
				// printlns("body_speed: ", body_speed);
				// printlns("motor_speeeds: ", motor_speeeds);
				// const auto manual_speed = this->io->manual_speed.get();
				// printlns("manual_speed: ", manual_speed);
			}

			void send_motor_speeds(const std::array<double, 4>& speeds) {
				for(u32 i = 0; i < 4; ++i) {
					if(const auto id = robot_config::ids[i]; id) {
						this->can_tx->publish(target_frame(*id, speeds[i]));
					}
				}
			}

			void change_map(const MapName::Enum filepath, const Xyth& initial_pose) {
				{
					this->current_pose_sum.modify([](auto& sum) {
						sum.clear();
					});
					io->busy_loop([this]() -> std::optional<Void> {
						// amclを非アクティブ状態にする
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
						if(this->amcl_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});

					printlns("amcl is deactivated.");
				}
				
				{
					io->busy_loop([this, filepath]() -> std::optional<Void> {
						// map_serverに地図をロードさせる
						auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
						req->map_url = std::string{"map/"} + std::string(MapName::to_filepath(filepath)) + ".yaml";
						if(this->load_map_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});

					printlns("map is loaded.");
				}

				{
					// 初期位置を設定
					auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
					initial_pose_msg.header.frame_id = "map";
					initial_pose_msg.pose.pose.position.x = initial_pose.xy.x;
					initial_pose_msg.pose.pose.position.y = initial_pose.xy.y;
					initial_pose_msg.pose.pose.orientation.z = std::sin(initial_pose.th / 2);  // ここら辺ほんとにあってるか不安(copilotくんはこう出してる)
					initial_pose_msg.pose.pose.orientation.w = std::cos(initial_pose.th / 2);
					this->initial_pose_pub->publish(initial_pose_msg);

					// 最初のtransformを吐く
					geometry_msgs::msg::TransformStamped transform{};
					transform.header.frame_id = "map";
					transform.header.stamp = this->now();
					transform.child_frame_id = "odom";
					transform.transform.rotation = initial_pose_msg.pose.pose.orientation;
					transform.transform.translation.x = initial_pose.xy.x;
					transform.transform.translation.y = initial_pose.xy.y;
					this->tf2_broadcaster.sendTransform(transform);

					rclcpp::sleep_for(500ms);

					printlns("initial_pose is set.");
				}

				{
					io->busy_loop([this]() -> std::optional<Void> {
						// amclをアクティブ状態にする
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
						if(this->amcl_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});

					printlns("amcl is activated.");
				}
			}

			void kill() {
				this->io->kill_interrupt.test_and_set();
				rclcpp::sleep_for(100ms);
				this->send_motor_speeds({0.0, 0.0, 0.0, 0.0});
				rclcpp::sleep_for(100ms);
			}
		};

		inline auto make_node (
			const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
		) -> std::tuple<std::shared_ptr<R2Node>, std::future<std::optional<Io *>>, std::unique_ptr<Io>> {
			auto io = Io::make_unique();
			auto node = std::make_shared<R2Node>(io.get(), options);
			auto fut = std::async(std::launch::async, [io = io.get(), node = node]() mutable -> std::optional<Io *> {
				// ここ順番が大事。
				// どいつがどのライフサイクルか、どのライフサイクルでどのクエリが投げられるかを考えること
				{
					io->busy_loop([node]() -> std::optional<Void> {
						// map_server_change_state_clientを使えるようにする
						if(node->map_server_change_state_client->wait_for_service(100ms)) {
							return Void{};
						}
						return std::nullopt;
					});

					io->busy_loop([node]() -> std::optional<Void> {
						// map_serverをコンフィグレーション状態にする
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
						if(node->map_server_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});

					io->busy_loop([node]() -> std::optional<Void> {
						// map_serverをアクティブ状態にする
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
						if(node->map_server_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});

					printlns("map_server is ready.");
				}
				{
					io->busy_loop([node]() -> std::optional<Void> {
						// amcl_change_state_clientを使えるようにする
						printlns("amcl_change_state_client wait_for_service.");
						if(node->amcl_change_state_client->wait_for_service(100ms)) {
							return Void{};
						}
						return std::nullopt;
					});

					io->busy_loop([node]() -> std::optional<Void> {
						// amclをコンフィグレーション状態にする
						printlns("amcl_change_state_client to configure.");
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
						if(node->amcl_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});

					io->busy_loop([node]() -> std::optional<Void> {
						// amclをアクティブ状態にする
						printlns("amcl_change_state_client to active.");
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
						if(node->amcl_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});

					printlns("amcl is ready.");
				}
				{
					io->busy_loop([node]() -> std::optional<Void> {
						// load_map_clientを使えるようにする
						if(node->load_map_client->wait_for_service(100ms)) {
							return Void{};
						}
						return std::nullopt;
					});

					printlns("load_map is ready.");
				}
				{
					// 地図と初期位置を設定
					node->change_map(MapName::area1, robot_config::area1_initialpose);
				}

				return io;
			});
			return {std::move(node), std::move(fut), std::move(io)};
		}
	}
}

namespace nhk24_2nd_ws::r2::r2_node {
	using impl::R2Node;
	using impl::make_node;
}