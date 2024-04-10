#pragma once

#include <expected>
#include <string>
#include <string_view>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>

#include <my_include/xyth.hpp>

#include "geometry_msgs_convertor.hpp"

namespace nhk24_2nd_ws::r2::ros2_utils::impl {
	using nhk24_2nd_ws::xyth::Xy;
	using nhk24_2nd_ws::xyth::Xyth;
	using nhk24_2nd_ws::r2::geometry_msgs_convertor::MsgConvertor;

	inline auto get_pose(tf2_ros::Buffer& tf2_buffer, const std::string_view from, const std::string_view to) -> std::expected<Xyth, std::string> {
		try {
			const auto transform = tf2_buffer.lookupTransform(from.data(), to.data(), tf2::TimePointZero).transform;
			const auto v = transform.translation;
			double roll, pitch, yaw;
			tf2::Matrix3x3{MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::fromMsg(transform.rotation)}
				.getRPY(roll, pitch, yaw);
			return Xyth::make(Xy::make(v.x, v.y), yaw);
		} catch(const tf2::TransformException& e) {
			using std::literals::operator""s;
			return std::unexpected{"fail to get transform from \""s + from.data() + "\" to \"" + to.data() + "\": " + e.what()};
		}
	}

	inline auto xyth_to_pose_msg(const Xyth& xyth) -> geometry_msgs::msg::Pose {
		geometry_msgs::msg::Pose pose{};
		pose.position.x = xyth.xy.x;
		pose.position.y = xyth.xy.y;
		tf2::Quaternion q{};
		q.setRPY(0, 0, xyth.th);
		pose.orientation = MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::toMsg(q);
		return pose;
	}

	inline auto xyth_to_transform(const Xyth& xyth) -> tf2::Transform {
		tf2::Transform transform{};
		transform.setOrigin(tf2::Vector3{xyth.xy.x, xyth.xy.y, 0});
		tf2::Quaternion q{};
		q.setRPY(0, 0, xyth.th);
		transform.setRotation(q);
		return transform;
	}

	inline auto get_system_timepoint(rclcpp::Clock& clock) -> std::chrono::system_clock::time_point {
		return std::chrono::system_clock::time_point{std::chrono::system_clock::duration{clock.now().nanoseconds()}};
	}
}

namespace nhk24_2nd_ws::r2::ros2_utils {
	using nhk24_2nd_ws::r2::ros2_utils::impl::get_pose;
	using nhk24_2nd_ws::r2::ros2_utils::impl::xyth_to_pose_msg;
	using nhk24_2nd_ws::r2::ros2_utils::impl::xyth_to_transform;
	using nhk24_2nd_ws::r2::ros2_utils::impl::get_system_timepoint;
}