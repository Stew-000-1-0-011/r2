#pragma once

#include <utility>
#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <my_include/xyth.hpp>
#include <my_include/debug_print.hpp>

#include "ros2_utils.hpp"
#include "lidar_filter.hpp"
#include "robot_config.hpp"

namespace nhk24_2nd_ws::r2::filter_node::impl {
	using xyth::Xy;
	using xyth::XyOp;
	using xyth::Xyth;
	using debug_print::printlns;
	using ros2_utils::get_pose;
	using lidar_filter::LidarScan;
	using lidar_filter::FilterOp;
	using lidar_filter::BoxFilter;
	using lidar_filter::ShadowFilter;
	using lidar_filter::filter_chain;
	using lidar_filter::apply_filter;
	using robot_config::footprint_half_diagonal;
	using robot_config::area_half_diagonal;
	using robot_config::shadow_filter_threshold_angle;
	using robot_config::shadow_window;

	struct FilterNode final : rclcpp::Node {
		sensor_msgs::msg::LaserScan last_msgs{};
		std::future<std::shared_ptr<std_srvs::srv::Empty_Response>> last_future{};

		tf2_ros::Buffer tf_buffer;
		tf2_ros::TransformListener tf_listener;

		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
		rclcpp::Client<std_srvs::srv::Empty>::SharedPtr request_nomotion_update_client;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
		
		FilterNode()
			: Node("filter_node")
			, tf_buffer(this->get_clock())
			, tf_listener(tf_buffer)
			, pub(create_publisher<sensor_msgs::msg::LaserScan>("scan", 1))
			, request_nomotion_update_client(create_client<std_srvs::srv::Empty>("request_nomotion_update"))
			, sub(create_subscription<sensor_msgs::msg::LaserScan>("scan_nonfiltered", 1, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
				this->callback(std::move(msg));
			}))
		{
			RCLCPP_INFO(get_logger(), "filter_node has been started");
		}

		void callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
			auto scan = LidarScan::make(std::move(msg->ranges), msg->angle_min, msg->angle_max);

			const auto lidar_pose = get_pose(tf_buffer, "map", "lidar_link");
			const auto base_pose = get_pose(tf_buffer, "map", "base_link");

			if(not lidar_pose) {
				RCLCPP_ERROR(get_logger(), lidar_pose.error().c_str());
				return;
			}
			if(not base_pose) {
				RCLCPP_ERROR(get_logger(), base_pose.error().c_str());
				return;
			}

			auto chained = filter_chain (
				!FilterOp{}* BoxFilter::make (
					*lidar_pose
					, *base_pose
					, footprint_half_diagonal
				)
				, BoxFilter::make (
					*lidar_pose
					, Xyth::make(area_half_diagonal +XyOp{}+ Xy::make(0.050, 0.050), 0.0)
					// , *base_pose
					, area_half_diagonal *XyOp{}* 1.1
				)
				// , ShadowFilter::make (
				// 	shadow_filter_threshold_angle
				// 	, shadow_window
				// 	, scan
				// )
			);

			auto filtered_scan = apply_filter(std::move(scan), std::move(chained));

			sensor_msgs::msg::LaserScan filtered_scan_msg{};
			filtered_scan_msg.header = msg->header;
			filtered_scan_msg.angle_min = msg->angle_min;
			filtered_scan_msg.angle_max = msg->angle_max;
			filtered_scan_msg.angle_increment = msg->angle_increment;
			filtered_scan_msg.time_increment = msg->time_increment;
			filtered_scan_msg.scan_time = msg->scan_time;
			filtered_scan_msg.range_min = msg->range_min;
			filtered_scan_msg.range_max = msg->range_max;
			filtered_scan_msg.ranges = std::move(filtered_scan.range);
			filtered_scan_msg.intensities = std::move(msg->intensities);
			
			pub->publish(filtered_scan_msg);

			RCLCPP_INFO(this->get_logger(), "filter_node publish scan.");

			// if(not this->last_future.valid() || this->last_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
			// 	printlns("update.");
			// 	auto req = std::make_shared<std_srvs::srv::Empty::Request>();
			// 	this->last_future = std::move(request_nomotion_update_client->async_send_request(std::move(req)).future);
			// }
		}

		// static auto make_marker(const auto& filtered_msg, const LidarScan& scan) -> visualization_msgs::msg::Marker {
		// 	auto scale = geometry_msgs::msg::Vector3{};
		// 	scale.x = 0.01;

		// 	auto color = std_msgs::msg::ColorRGBA{};
		// 	color.r = 0.0;
		// 	color.g = 1.0;
		// 	color.b = 0.0;
		// 	color.a = 1.0;

		// 	auto points = std::vector<geometry_msgs::msg::Point>();
		// 	points.reserve(indices.size());
		// 	for(const auto i : indices)
		// 	{
		// 		const auto p = scan.nth_rtheta(i).to_vec2d();
		// 		geometry_msgs::msg::Point point{};
		// 		point.x = p.x;
		// 		point.y = p.y;
		// 		point.z = 0.0;
		// 		points.push_back(point);
		// 	}

		// 	auto marker_msg = visualization_msgs::msg::builder::Init_Marker_header()
		// 		.header(filtered_msg.header)
		// 		.ns("segment_line")
		// 		.id(0)
		// 		.type(visualization_msgs::msg::Marker::LINE_STRIP)
		// 		.action(visualization_msgs::msg::Marker::ADD)
		// 		.pose(geometry_msgs::msg::Pose{})
		// 		.scale(scale)
		// 		.color(color)
		// 		.lifetime(rclcpp::Duration::from_nanoseconds(0))
		// 		.frame_locked(false)
		// 		.points(std::move(points))
		// 		.colors(std::vector<std_msgs::msg::ColorRGBA>{})
		// 		.texture_resource("")
		// 		.texture(sensor_msgs::msg::CompressedImage{})
		// 		.uv_coordinates(std::vector<visualization_msgs::msg::UVCoordinate>{})
		// 		.text("")
		// 		.mesh_resource("")
		// 		.mesh_file(visualization_msgs::msg::MeshFile{})
		// 		.mesh_use_embedded_materials(false)
		// 	;

		// 	return marker_msg;
		// }
	};
}

namespace nhk24_2nd_ws::r2::filter_node {
	using filter_node::impl::FilterNode;
}
