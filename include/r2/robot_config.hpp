/**
 * @file robot_config.hpp
 * @brief ロボットの設定を定義する
 */

#pragma once

#include <array>
#include <optional>
#include <chrono>

#include <my_include/std_types.hpp>
#include <my_include/xyth.hpp>

namespace nhk24_2nd_ws::r2::robot_config::impl {
	using namespace std::chrono_literals;
	using nhk24_2nd_ws::xyth::Xy;
	using nhk24_2nd_ws::xyth::Xyth;

	inline constexpr Xy footprint_half_diagonal = Xy::make(0.275, 0.275);  // フットプリントの半対角線長[m]

	inline constexpr double max_v = 500.0 * 0.5;  // モーターの最大速度[rad/s]
	inline constexpr double max_a = 500.0 * 0.5;  // モーターの最大加速度[rad/s^2]
	inline constexpr double max_vxy = 0.5;  // 最大並進速度[m/s]
	inline constexpr double max_vth = std::numbers::pi / 3.0 * 0.3;  // 最大角速度[rad/s]
	inline constexpr double max_axy = 6.0;  // 最大並進加速度[m/s^2]
	inline constexpr double max_ath = std::numbers::pi / 3.0 * 0.6;  // 最大角加速度[rad/s^2]
	inline constexpr double center_to_wheel = footprint_half_diagonal.norm();  // 中心から駆動輪までの距離[m]
	inline constexpr double wheel_radius = 0.0635;  // 駆動輪の半径[m](雑)
	inline constexpr double wheel_to_motor_ratio = 32;  // 駆動輪からモーターへの倍速比

	inline constexpr Xy area_half_diagonal = Xy::make(5.975 / 2, 4.100 / 2);  // エリアの半対角線長[m]

	inline constexpr double shadow_filter_threshold_angle = std::numbers::pi / 15.0;  // シャドウフィルタの閾値[rad]
	inline constexpr u16 shadow_window = 10;  // シャドウフィルタの窓幅
}

namespace nhk24_2nd_ws::r2::robot_config {
	using impl::footprint_half_diagonal;
	using impl::max_v;
	using impl::max_a;
	using impl::max_vxy;
	using impl::max_vth;
	using impl::max_axy;
	using impl::max_ath;
	using impl::center_to_wheel;
	using impl::wheel_radius;
	using impl::wheel_to_motor_ratio;
	using impl::area_half_diagonal;
	using impl::shadow_filter_threshold_angle;
	using impl::shadow_window;
}

