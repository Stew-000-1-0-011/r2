#pragma once

#include <cmath>
#include <functional>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nhk24_utils/msg/filter_param.hpp>

#include <nhk24_utils/std_type.hpp>
#include <nhk24_utils/vec2d.hpp>
#include <nhk24_utils/scan.hpp>
#include "laser_filters.hpp"
#include "segment_line.hpp"

namespace nhk24_use_amcl::stew::filter_node::impl {
	using namespace crs_lib::integer_types;
	using Vec2d = nhk24_utils::stew::vec2d::Vec2d_<float>;
	using nhk24_utils::stew::scan::Scan;
	using laser_filters::filter_chain;
	using laser_filters::ShadowFilter;
	using laser_filters::BoxFilter;
	using segment_line::SegmentLine;

	struct FilterNode final : rclcpp::Node
	{
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
		rclcpp::Subscription<nhk24_utils::msg::FilterParam>::SharedPtr sub_filter_param;
		u16 seg_window;
		float seg_threshold;
		Vec2d base_to_lidar;
		Vec2d footprint_size;
		float shadow_filter_threshold_angle;
		u16 shadow_filter_window;

		FilterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: Node("nhk24_1st_filter_node", options)
			, pub_scan(create_publisher<sensor_msgs::msg::LaserScan>("scan", 10))
			, pub_marker(create_publisher<visualization_msgs::msg::Marker>("segment_line", 10))
			, sub_scan(create_subscription<sensor_msgs::msg::LaserScan>("scan_nonfiltered", 10, std::bind(&FilterNode::scan_callback, this, std::placeholders::_1)))
			, sub_filter_param(create_subscription<nhk24_utils::msg::FilterParam>("filter_param", 10, std::bind(&FilterNode::callback_filter_param, this, std::placeholders::_1)))
			, seg_window(10)
			, seg_threshold(0.100f)
			, base_to_lidar(0.000, -0.000)
			, footprint_size(0.500f, 0.500f)
			, shadow_filter_threshold_angle(0.100f)
			, shadow_filter_window(10)
		{}

		private:
		void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
		{
			auto[scan, filtered_msg] = Scan::from_msg(std::move(*msg));

			const auto chain = filter_chain(
				ShadowFilter::make(this->shadow_filter_threshold_angle, this->shadow_filter_window)
				, BoxFilter::make(this->base_to_lidar, this->footprint_size)
			);

			auto seg_line = SegmentLine::make(this->seg_window, this->seg_threshold);
			for(u16 i = 0; i < scan.rs.size(); ++i)
			{
				scan.rs[i] = chain(scan, i);
				seg_line = std::move(seg_line).update(scan, i);
			}

			auto marker_msg = make_marker(filtered_msg, scan, seg_line.indices);
			pub_marker->publish(marker_msg);

			filtered_msg.ranges = std::move(scan.rs);

			pub_scan->publish(filtered_msg);
		}

		void callback_filter_param(const nhk24_utils::msg::FilterParam::SharedPtr msg)
		{
			if(msg->seg_window != 0) seg_window = msg->seg_window;
			if(msg->seg_threshold != 0.0) seg_threshold = msg->seg_threshold;
			if(const auto v = Vec2d{msg->base_to_lidar_x, msg->base_to_lidar_y}; v.norm2() != 0.0) base_to_lidar = v;
			if(const auto v = Vec2d{msg->footprint_size_x, msg->footprint_size_y}; v.norm2() != 0.0) footprint_size = v;
			if(msg->shadow_threshold_angle != 0.0) shadow_filter_threshold_angle = msg->shadow_threshold_angle;
			if(msg->shadow_window != 0) shadow_filter_window = msg->shadow_window;

			RCLCPP_INFO (
				this->get_logger()
				, "filter_param: window=%d"
					", threshold=%f"
					", base_to_lidar=(%f, %f)"
					", footprint_size=(%f, %f)"
					", shadow_threshold_angle=%f"
					", shadow_window=%d"
				, seg_window
				, seg_threshold
				, base_to_lidar.x
				, base_to_lidar.y
				, footprint_size.x
				, footprint_size.y
				, shadow_filter_threshold_angle
				, shadow_filter_window
			);
		}

		static auto make_marker(const auto& filtered_msg, const Scan& scan, const std::vector<u16>& indices) -> visualization_msgs::msg::Marker {
			auto scale = geometry_msgs::msg::Vector3{};
			scale.x = 0.01;

			auto color = std_msgs::msg::ColorRGBA{};
			color.r = 0.0;
			color.g = 1.0;
			color.b = 0.0;
			color.a = 1.0;

			auto points = std::vector<geometry_msgs::msg::Point>();
			points.reserve(indices.size());
			for(const auto i : indices)
			{
				const auto p = scan.nth_rtheta(i).to_vec2d();
				geometry_msgs::msg::Point point{};
				point.x = p.x;
				point.y = p.y;
				point.z = 0.0;
				points.push_back(point);
			}

			auto marker_msg = visualization_msgs::msg::builder::Init_Marker_header()
				.header(filtered_msg.header)
				.ns("segment_line")
				.id(0)
				.type(visualization_msgs::msg::Marker::LINE_STRIP)
				.action(visualization_msgs::msg::Marker::ADD)
				.pose(geometry_msgs::msg::Pose{})
				.scale(scale)
				.color(color)
				.lifetime(rclcpp::Duration::from_nanoseconds(0))
				.frame_locked(false)
				.points(std::move(points))
				.colors(std::vector<std_msgs::msg::ColorRGBA>{})
				.texture_resource("")
				.texture(sensor_msgs::msg::CompressedImage{})
				.uv_coordinates(std::vector<visualization_msgs::msg::UVCoordinate>{})
				.text("")
				.mesh_resource("")
				.mesh_file(visualization_msgs::msg::MeshFile{})
				.mesh_use_embedded_materials(false)
			;

			return marker_msg;
		}
	};
}

namespace nhk24_use_amcl::stew::filter_node {
	using impl::FilterNode;
}