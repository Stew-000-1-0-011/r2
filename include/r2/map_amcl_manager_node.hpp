#pragma once

#include <cmath>
#include <memory>
#include <tuple>
#include <stop_token>
#include <chrono>
#include <future>
#include <functional>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <nhk24_utils/msg/twist2d.hpp>

#include <my_include/void.hpp>
#include <my_include/xyth.hpp>
#include <my_include/mutexed.hpp>
#include <my_include/debug_print.hpp>
#include <my_include/lap_timer.hpp>

#include "position_based_behaviour.hpp"
#include "field_constant.hpp"
#include "ros2_utils.hpp"

namespace nhk24_2nd_ws::r2::map_amcl_manager_node::impl {
	using namespace std::chrono_literals;
	using void_::Void;
	using xyth::Xy;
	using xyth::Xyth;
	using xyth::XyOp;
	using mutexed::Mutexed;
	using debug_print::printlns_to;
	using lap_timer::LapTimer;
	using position_based_bahaviour::PositionBasedBehaviour;
	using field_constant::Section;
	namespace poses = field_constant::poses;
	using ros2_utils::get_pose;

	struct MapAmclManagerNode : rclcpp::Node {
		tf2_ros::TransformBroadcaster tf2_broadcaster;
		tf2_ros::Buffer tf2_buffer;
		tf2_ros::TransformListener tf2_listener;

		rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
		rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr map_server_change_state_client;
		rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr amcl_change_state_client;
		rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client;

		PositionBasedBehaviour behaviour;
		std::future<void> setup_fut;
		Mutexed<bool> killed;
		Mutexed<bool> done;

		rclcpp::Subscription<nhk24_utils::msg::Twist2d>::SharedPtr initpose_sub;
		rclcpp::TimerBase::SharedPtr timer;

		MapAmclManagerNode()
		: Node("map_amcl_manager")
		, tf2_broadcaster{this}
		, tf2_buffer{this->get_clock()}
		, tf2_listener{this->tf2_buffer}
		, initial_pose_pub{this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10)}
		, map_server_change_state_client(this->create_client<lifecycle_msgs::srv::ChangeState>("map_server/change_state"))
		, amcl_change_state_client(this->create_client<lifecycle_msgs::srv::ChangeState>("amcl/change_state"))
		, load_map_client(this->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map"))
		, behaviour {
			PositionBasedBehaviour::make (
				Section::area1
				, [this](std::stop_token&& st, const bool change_to_active) -> std::future<void> {
					return this->change_amcl(std::move(st), change_to_active);
				}
				, [this](std::stop_token&& st, const std::string_view map_name) -> std::future<void> {
					return this->change_map(std::move(st), map_name);
				}
			)
		}
		, setup_fut{}
		, killed{Mutexed<bool>::make(false)}
		, done{Mutexed<bool>::make(false)}
		, initpose_sub{this->create_subscription<nhk24_utils::msg::Twist2d>("r2/initialpose", 1, [this](const nhk24_utils::msg::Twist2d::SharedPtr msg) {
			const auto pose_in_map = Xyth::make(Xy::make(msg->linear.x, msg->linear.y), msg->angular);
			this->reset_pose(pose_in_map);
		})}
		, timer{this->create_wall_timer(10ms, [this]() {
			if(const auto current_pose = get_pose(this->tf2_buffer, "map", "true_base_link")) {
				this->behaviour.update(current_pose.value().xy);
			}
			else {
				printlns_to(std::osyncstream{std::cout}, "fail to get current pose: ", current_pose.error());
			}
		})}
		{
			this->setup_fut = this->setup();
		}

		void kill() {
			this->killed.set(true);
		}

		void reset_pose(const Xyth& pose_in_map) {
			auto wait_timer = LapTimer::make();
			while(wait_timer.watch().count() < 0.5) {
				if(auto pose_in_odom = get_pose(this->tf2_buffer, "odom", "true_base_link")) {
					auto t = make_transform_stamped("map", "odom", this->now(), calc_transform(pose_in_map, *pose_in_odom));
					tf2_broadcaster.sendTransform(t);
					auto p = make_pose_stamped("map", this->now(), pose_in_map);
					initial_pose_pub->publish(p);
				}
			}
			printlns_to(std::osyncstream{std::cout}, __FILE__, __LINE__, "fail to get pose in odom.");
		}

		auto change_amcl(std::stop_token&& st, const bool change_to_active) -> std::future<void> {
			auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
			req->transition.id = change_to_active ? lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE : lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
			auto res = this->map_server_change_state_client->async_send_request(req);
			return std::async(std::launch::async, [st = std::move(st), res = std::move(res)]() mutable {
				while(!st.stop_requested()) {
					if(res.wait_for(0s) == std::future_status::ready) {
						res.get();
						break;
					}
				}
			});
		}

		auto change_map(std::stop_token&& st, const std::string_view map_name) -> std::future<void> {
			auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
			req->map_url = std::string(map_name);
			auto res = this->load_map_client->async_send_request(req);
			return std::async(std::launch::async, [st = std::move(st), res = std::move(res)]() mutable {
				while(!st.stop_requested()) {
					if(res.wait_for(0s) == std::future_status::ready) {
						res.get();
						break;
					}
				}
			});
		}

		

		auto busy_loop(auto&& f) -> std::optional<std::remove_cvref_t<decltype(*f())>> {
			while(!this->killed.get()) {
				if(const auto res = f()) {
					return res;
				}
			}

			return std::nullopt;
		}

		auto setup() -> std::future<void> {
			return std::async(std::launch::async, [this] {
				// ここ順番が大事。
				// どいつがどのライフサイクルか、どのライフサイクルでどのクエリが投げられるかを考えること
				std::make_optional<Void>(Void{})
				.and_then([this](Void) -> std::optional<Void> {
					return this->busy_loop([this]() -> std::optional<Void> {
						// map_server_change_state_clientを使えるようにする
						if(this->map_server_change_state_client->wait_for_service(100ms)) {
							return Void{};
						}
						return std::nullopt;
					});
				}).and_then([this](Void) -> std::optional<Void> {
					return this->busy_loop([this]() -> std::optional<Void> {
						// map_serverをコンフィグレーション状態にする
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
						if(this->map_server_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});
				})
				.and_then([this](Void) -> std::optional<Void> {
					return this->busy_loop([this]() -> std::optional<Void> {
						// map_serverをアクティブ状態にする
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
						if(this->map_server_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});
				})
				.and_then([](Void) -> std::optional<Void> {
					printlns_to(std::osyncstream{std::cout}, "map_server is ready.");
					return Void{};
				})
				.and_then([this](Void) -> std::optional<Void> {
					return this->busy_loop([this]() -> std::optional<Void> {
						// amcl_change_state_clientを使えるようにする
						if(this->amcl_change_state_client->wait_for_service(100ms)) {
							return Void{};
						}
						return std::nullopt;
					});
				}).and_then([this](Void) -> std::optional<Void> {
					return this->busy_loop([this]() -> std::optional<Void> {
						// amclをコンフィグレーション状態にする
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
						if(this->amcl_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});
				}).and_then([this](Void) -> std::optional<Void> {
					return this->busy_loop([this]() -> std::optional<Void> {
						// amclをアクティブ状態にする
						auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
						req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
						if(this->amcl_change_state_client->async_send_request(std::move(req)).wait_for(100ms) == std::future_status::ready) {
							return Void{};
						}
						return std::nullopt;
					});
				})
				.and_then([](Void) -> std::optional<Void> {
					printlns_to(std::osyncstream{std::cout}, "amcl is ready.");
					return Void{};
				})
				.and_then([this](Void) -> std::optional<Void> {
					return this->busy_loop([this]() -> std::optional<Void> {
						// load_map_clientを使えるようにする
						if(this->load_map_client->wait_for_service(100ms)) {
							return Void{};
						}
						return std::nullopt;
					});
				})
				.and_then([](Void) -> std::optional<Void> {
					printlns_to(std::osyncstream{std::cout}, "load_map is ready.");
					return Void{};
				})
				.and_then(
					[this](Void) -> std::optional<Void> {
						std::stop_source ssource{};
						// 地図と初期位置を設定
						auto fut = this->change_map(ssource.get_token(), Section::to_filepath(Section::area1));

						auto ret = this->busy_loop([this, &ssource, &fut]() -> std::optional<Void> {
							if(fut.wait_for(100ms) == std::future_status::ready) {
								fut.get();
								return Void{};
							}
							return std::nullopt;
						});

						ssource.request_stop();

						return ret;
					}
				)
				.and_then([this](Void) -> std::optional<Void> {
					this->reset_pose(poses::area1_starting);
					return Void{};
				})
				.and_then([](Void) -> std::optional<Void> {
					printlns_to(std::osyncstream{std::cout}, "setup completed.");
					return Void{};
				});

				this->done.set(true);
			});
		}

		static auto make_pose_stamped(const std::string_view frame_id, const rclcpp::Time& time, const Xyth& xyth) -> geometry_msgs::msg::PoseWithCovarianceStamped {
			geometry_msgs::msg::PoseWithCovarianceStamped p{};
			p.header.frame_id = frame_id;
			p.header.stamp = time;
			p.pose.pose.position.x = xyth.xy.x;
			p.pose.pose.position.y = xyth.xy.y;
			p.pose.pose.position.z = 0.0;
			p.pose.pose.orientation.x = 0.0;
			p.pose.pose.orientation.y = 0.0;
			p.pose.pose.orientation.z = std::sin(xyth.th / 2);
			p.pose.pose.orientation.w = std::cos(xyth.th / 2);
			return p;
		}

		static auto make_transform_stamped(const std::string_view frame_id, const std::string_view child_frame_id, const rclcpp::Time& time, const Xyth& xyth) -> geometry_msgs::msg::TransformStamped {
			geometry_msgs::msg::TransformStamped t{};
			t.header.frame_id = frame_id;
			t.header.stamp = time;
			t.child_frame_id = child_frame_id;
			t.transform.translation.x = xyth.xy.x;
			t.transform.translation.y = xyth.xy.y;
			t.transform.translation.z = 0.0;
			t.transform.rotation.x = 0.0;
			t.transform.rotation.y = 0.0;
			t.transform.rotation.z = std::sin(xyth.th / 2);
			t.transform.rotation.w = std::cos(xyth.th / 2);
			return t;
		}

		static auto calc_transform(const Xyth& pose_from, const Xyth& pose_to) -> Xyth {
			const double th = pose_to.th - pose_from.th;
			const Xy xy = pose_to.xy -XyOp{}- pose_from.xy.rot(-th);
			return Xyth::make(xy, th);
		}
	};
}

namespace nhk24_2nd_ws::r2::map_amcl_manager_node {
	using impl::MapAmclManagerNode;
}