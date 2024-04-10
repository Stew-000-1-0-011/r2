#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster_node.hpp>

#include "geometry_msgs_convertor.hpp"
#include "ros2_utils.hpp"
#include "robot_config.hpp"

namespace nhk24_2nd_ws::r2::silo_position_publish_node::impl {
	using geometry_msgs_convertor::MsgConvertor;
	using ros2_utils::xyth_to_transform;

	struct SiloPositionPublishNode : public rclcpp::Node {
		tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

		SiloPositionPublishNode(const rclcpp::NodeOptions & options)
			: Node("silo_position_publish_node", options)
			, static_tf_broadcaster(this)
		{
			for(u32 i = 0; i < robot_config::impl::silo_positions.size(); ++i) {
				geometry_msgs::msg::TransformStamped ts{};
				ts.header.stamp = rclcpp::Clock().now();
				ts.header.frame_id = "map";
				ts.child_frame_id = "silo" + std::to_string(i);
				ts.transform = MsgConvertor<tf2::Transform, geometry_msgs::msg::Transform>::toMsg(xyth_to_transform(robot_config::impl::silo_positions[i]));
				static_tf_broadcaster.sendTransform(ts);
			}
		}
	};
}

namespace nhk24_2nd_ws::r2::silo_position_publish_node {
	using nhk24_2nd_ws::r2::silo_position_publish_node::impl::SiloPositionPublishNode;
}