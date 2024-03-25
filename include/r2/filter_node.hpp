#pragma once

#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <my_include/xyth.hpp>

#include "ros2_utils.hpp"
#include "lidar_filter.hpp"
#include "robot_config.hpp"

namespace nhk24_2nd_ws::r2::filter_node::impl {
	using xyth::Xy;
	using xyth::XyOp;
	using xyth::Xyth;
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

		tf2_ros::Buffer tf_buffer;
		tf2_ros::TransformListener tf_listener;

		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
		
		FilterNode()
			: Node("filter_node")
			, tf_buffer(this->get_clock())
			, tf_listener(tf_buffer)
			, pub(create_publisher<sensor_msgs::msg::LaserScan>("scan", 1))
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
		}
	};
}

namespace nhk24_2nd_ws::r2::filter_node {
	using filter_node::impl::FilterNode;
}
