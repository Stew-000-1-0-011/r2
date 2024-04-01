#pragma once

#include <string>
#include <atomic>
#include <array>
#include <memory>
#include <utility>
#include <string_view>
#include <concepts>
#include <future>
#include <type_traits>
#include <functional>
#include <optional>

#include <my_include/void.hpp>
#include <my_include/mutexed.hpp>
#include <my_include/wanko.hpp>
#include <my_include/xyth.hpp>

namespace nhk24_2nd_ws::r2::robot_io::impl {
	using void_::Void;
	using mutexed::Mutexed;
	using wanko::Wanko;
	using xyth::Xyth;

	struct MapName final {
		enum class Enum : u8 {
			area1,
			area2,
			area3_yellow,
			area3_storage
		};

		using Enum::area1;
		using Enum::area2;
		using Enum::area3_yellow;
		using Enum::area3_storage;

		static auto to_filepath(const Enum map_name) -> std::string_view {
			switch(map_name) {
				case area1: return "area1";
				case area2: return "area2";
				case area3_yellow: return "area3_yellow";
				case area3_storage: return "area3_storage";
				default: return "unreachable";
			}
		}
	};

	struct StateName final {
		enum class Enum : u8 {
			pass_area1,
			slope_1to2,
			pass_area2,
			slope_2to3,
			dancing
		};

		using Enum::pass_area1;
		using Enum::slope_1to2;
		using Enum::pass_area2;
		using Enum::slope_2to3;
		using Enum::dancing;

		static auto from_string(const std::string_view state_name) -> std::optional<Enum> {
			if(state_name == "pass_area1") {
				return pass_area1;
			} else if(state_name == "slope_1to2") {
				return slope_1to2;
			} else if(state_name == "pass_area2") {
				return pass_area2;
			} else if(state_name == "slope_2to3") {
				return slope_2to3;
			} else if(state_name == "dancing") {
				return dancing;
			} else {
				return std::nullopt;
			}
		}
	};

	enum class ManualAuto : u8 {
		manual,
		auto_specified,
		auto_evacuated
	};

	enum class LiftState : u8 {
		up,
		down
	};

	struct Io {
		Mutexed<Xyth> current_pose;
		Mutexed<Xyth> manual_speed;
		Mutexed<std::optional<double>> ball_direction;
		Mutexed<std::optional<StateName::Enum>> manual_recover_state;
		Wanko<ManualAuto> change_manual_auto;
		std::atomic_flag kill_interrupt;

		Mutexed<Xyth> body_speed;
		Wanko<LiftState> lift_state;

		std::function<auto (const MapName::Enum, const Xyth&) -> std::future<void>> change_map;

		Io()
			: current_pose(Mutexed<Xyth>::make(Xyth::zero()))
			, manual_speed(Mutexed<Xyth>::make(Xyth::zero()))
			, ball_direction(Mutexed<std::optional<double>>::make(std::nullopt))
			, manual_recover_state(Mutexed<std::optional<StateName::Enum>>::make(std::nullopt))
			, change_manual_auto(Wanko<ManualAuto>::make())
			, kill_interrupt(ATOMIC_FLAG_INIT)
			, body_speed(Mutexed<Xyth>::make(Xyth::zero()))
			, lift_state(Wanko<LiftState>::make())
			, change_map(nullptr)
		{}

		static auto make_unique() -> std::unique_ptr<Io> {
			return std::make_unique<Io>();
		}

		auto kill_interrupted() -> bool {
			return this->kill_interrupt.test();
		}

		template<class F_>
		requires requires (F_ f) {
			{[]<class T_>(const std::optional<T_>&){}(f())};
		}
		auto busy_loop(F_&& f) -> std::optional<std::decay_t<decltype(*f())>> {
			using Ret = std::optional<std::decay_t<decltype(*f())>>;
			
			while(!this->kill_interrupted()) {
				if(auto ret = f()) {
					return Ret{std::in_place, std::move(*ret)};
				}
			}
			return Ret{std::nullopt};
		}
	};
}

namespace nhk24_2nd_ws::r2::robot_io {
	using impl::Io;
	using impl::MapName;
	using impl::StateName;
	using impl::ManualAuto;
	using impl::LiftState;
}