/**
 * @file states.hpp
 * @brief 各状態を記述。
 * 2-phase name lookupが無いと木構造でない関数呼び出しがきついね。
 */

#pragma once

#include <functional>
#include <memory>
#include <utility>
#include <chrono>
#include <optional>
#include <tuple>
#include <concepts>
#include <type_traits>

#include <my_include/xyth.hpp>
#include <my_include/debug_print.hpp>
#include <my_include/state_machine.hpp>

#include "../r2_node.hpp"
#include "../robot_io.hpp"
#include "../robot_config.hpp"
#include "../pacman.hpp"
#include "../lap_timer.hpp"

namespace nhk24_2nd_ws::r2::transit_state {
	using StateBase = state_machine::StateBase<robot_io::Io>;

	inline auto to_error_state(const std::string_view error_message) -> std::unique_ptr<StateBase>;
	inline auto to_manual(std::unique_ptr<StateBase>&& evacuated_state) -> std::unique_ptr<StateBase>;
	inline auto to_pass_area1() -> std::unique_ptr<StateBase>;
	inline auto to_slope_1to2() -> std::unique_ptr<StateBase>;
	inline auto to_pass_area2() -> std::unique_ptr<StateBase>;
	inline auto to_slope_2to3() -> std::unique_ptr<StateBase>;
	inline auto to_pass_yellow() -> std::unique_ptr<StateBase>;
	inline auto to_slope_YtoS() -> std::unique_ptr<StateBase>;
	inline auto to_plunge_balls() -> std::unique_ptr<StateBase>;
	inline auto to_collect_ball() -> std::unique_ptr<StateBase>;
	inline auto to_exit_storage() -> std::unique_ptr<StateBase>;
	inline auto to_goto_silo() -> std::unique_ptr<StateBase>;
	inline auto to_dancing() -> std::unique_ptr<StateBase>;
}
