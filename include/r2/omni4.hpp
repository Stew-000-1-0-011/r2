/**
 * @file omni4.hpp
 * @brief 
 * 
 */

#pragma once

#include <cmath>
#include <numbers>
#include <array>

#include <my_include/std_types.hpp>
#include <my_include/xyth.hpp>

#include "robot_config.hpp"
#include "lap_timer.hpp"

namespace nhk24_2nd_ws::r2::omni4::impl {
	using xyth::Xy;
	using xyth::Xyth;
	using xyth::XyOp;
	using robot_config::max_v;
	using robot_config::max_a;
	using robot_config::max_vxy;
	using robot_config::max_vth;
	using robot_config::max_axy;
	using robot_config::max_ath;
	using robot_config::center_to_wheel;
	using robot_config::wheel_radius;
	using robot_config::wheel_to_motor_ratio;
	using lap_timer::LapTimer;

	/**
	 * @brief 速度上限や加速度上限を考慮して、モーターの速度を制限する
	 * 
	 */
	struct MotorSpeedFixer final {
		

		double last_v{0.0};

		constexpr auto update(double velocity, const double dt) noexcept -> double {
			// 速度の制限
			if(velocity * velocity > max_v * max_v) {
				velocity = std::signbit(velocity) ? -max_v : max_v;
			}

			// 加速度の制限
			const double dv = velocity - last_v;
			const double max_dv = max_a * dt;
			if(dv * dv > max_dv * max_dv) {
				velocity = last_v + (std::signbit(dv) ? -max_dv : max_dv);
			}

			// 速度の更新
			last_v = velocity;

			return velocity;
		}
	};

	/**
	 * @brief 並進・回転それぞれの速度上限や加速度上限を考慮して、機体の速度を制限する
	 * 
	 */
	struct BodySpeedFixer final {
		Xyth last_velocity{Xyth::zero()};

		constexpr auto update(const Xyth& velocity, const double dt) noexcept -> Xyth {
			constexpr auto op = XyOp{};

			auto vxy = velocity.xy;

			// 並進速度の制限
			if(const double norm2 = vxy.norm2(); norm2 > max_vxy * max_vxy) {
				vxy = vxy.unitize() *op* max_vxy;  // max_xy * max_xyが十分大きいと仮定
			}

			// 並進加速度の制限
			auto dvxy = vxy -op- last_velocity.xy;
			const double max_dvxy = max_axy * dt;
			if(const double norm2 = dvxy.norm2(); norm2 > max_dvxy * max_dvxy) {
				dvxy = dvxy.unitize() *op* max_dvxy;  // max_dvxy * max_dvxyが十分大きいと仮定
				vxy = last_velocity.xy +op+ dvxy;
			}

			auto vth = velocity.th;

			// 角速度の制限
			if(vth * vth > max_vth * max_vth) {
				vth *= std::signbit(vth) ? -max_vth : max_vth;
			}

			// 角加速度の制限
			const double dvth = vth - last_velocity.th;
			const double max_dvth = max_ath * dt;
			if(dvth * dvth > max_dvth * max_dvth) {
				vth = last_velocity.th + (std::signbit(dvth) ? -max_dvth : max_dvth);
			}

			// 速度の更新
			last_velocity = Xyth{vxy, vth};
			
			return Xyth{vxy, vth};
		}
	};

	namespace {
		struct Omni4 final {
			private:
			LapTimer dt_timer{};
			BodySpeedFixer body_velocity_fixer{};
			std::array<MotorSpeedFixer, 4> motor_velocity_fixers{};

			public:
			static constexpr auto make() -> Omni4 {
				return Omni4{};
			}

			/**
			 * @brief 機体座標系における速度を入れると、各モーターの速度を返す
			 * 
			 * @param body_velocity 
			 * @param dt 
			 * @return std::array<double, 4> 
			 */
			auto update(const Xyth& body_velocity) -> std::array<double, 4> {
				const auto dt = dt_timer.update().count();
				constexpr auto op = XyOp{};
				const auto fixed_body_velocity = body_velocity_fixer.update(body_velocity, dt);

				// Pythonでいうところの内包表記みたいなやつ
				return [this, &fixed_body_velocity, dt, op]<u32 ... i>(std::integer_sequence<u32, i...>) {
					return std::array<double, 4> {
						[this, &fixed_body_velocity, dt, op]() -> double {
							const auto v = fixed_body_velocity.xy %op% Xy::unit_x().rot(3 * std::numbers::pi / 4.0 + std::numbers::pi / 2.0 * i) + fixed_body_velocity.th * center_to_wheel;
							return this->motor_velocity_fixers[i].update(v / wheel_radius * wheel_to_motor_ratio, dt);
						}() ...
					};
				}(std::make_integer_sequence<u32, 4>{});
			}
		};
	}
}

namespace nhk24_2nd_ws::r2::omni4 {
	using impl::Omni4;
}