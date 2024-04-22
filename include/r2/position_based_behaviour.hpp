#pragma once

#include <utility>
#include <stop_token>
#include <future>
#include <functional>
#include <string_view>
#include <optional>
#include <memory>
#include <concepts>
#include <type_traits>
#include <chrono>

#include <my_include/std_types.hpp>
#include <my_include/xyth.hpp>
#include <my_include/lap_timer.hpp>
#include <my_include/debug_print.hpp>

#include "field_constant.hpp"

namespace nhk24_2nd_ws::r2::position_based_bahaviour::impl {
	using namespace std::chrono_literals;
	using xyth::Xy;
	using lap_timer::LapTimer;
	using debug_print::printlns_to;
	using field_constant::Section;
	using field_constant::is_plane;
	using field_constant::where_am_i;

	inline constexpr double wait_duration = 0.5;

	/// @brief 結果が得られるまで勝手に再実行されるfuture
	/// @tparam Arg_
	/// @attention 非同期での使用は不可能。やりたいなら各メンバ関数間での排他制御が必要
	/// @note future<void>のみを対象としているため、処理が色々と雑
	template<class Arg_>
	requires std::copyable<Arg_> && std::same_as<Arg_, std::remove_cvref_t<Arg_>>
	struct BufferedFuture final {
		struct Task final {
			Arg_ arg;
			std::future<void> fut;
			std::stop_source ssource;
			LapTimer wait_timer;
		};

		std::optional<Arg_> next_arg;
		std::optional<Task> task;
		std::function<auto (std::stop_token&&, Arg_) -> std::future<void>> generator;

		static auto make (
			auto&& generator
		) noexcept -> BufferedFuture {
			return BufferedFuture<Arg_> {
				.next_arg = std::nullopt
				, .task = std::nullopt
				, .generator = std::forward<decltype(generator)>(generator)
			};
		}

		static auto make_task(auto& generator, const Arg_& arg) -> Task {
			std::stop_source ssource{};
			return Task {
				.arg = arg
				, .fut = generator(ssource.get_token(), arg)
				, .ssource = std::move(ssource)
				, .wait_timer = LapTimer::make()
			};
		}

		void update() {
			if(not this->task) {
				if(this->next_arg) {
					this->task = make_task(this->generator, *this->next_arg);
					this->next_arg.reset();
				}
			}
			else {
				if(this->task->fut.wait_for(0s) == std::future_status::ready) {
					this->task.reset();
				}
				else if(this->task->wait_timer.watch().count() > wait_duration) {
					this->task->ssource.request_stop();
					if(this->next_arg) {
						this->task = make_task(this->generator, *this->next_arg);
						this->next_arg.reset();
					}
					else this->task = make_task(this->generator, this->task->arg);
				}
			}
		}

		void set(const Arg_& arg) noexcept {
			this->next_arg = arg;
		}
	};

	struct PositionBasedBehaviour final {
		Section::Enum now;
		BufferedFuture<bool> amcl_fut;
		BufferedFuture<std::string_view> map_fut;

		static auto make (
			const Section::Enum now
			, auto&& change_amcl
			, auto&& change_map
		) noexcept -> PositionBasedBehaviour {
			return PositionBasedBehaviour{
				.now = now
				, .amcl_fut = BufferedFuture<bool>::make(std::forward<decltype(change_amcl)>(change_amcl))
				, .map_fut = BufferedFuture<std::string_view>::make(std::forward<decltype(change_map)>(change_map))
			};
		}

		void update(const Xy& current_pose) noexcept {
			this->amcl_fut.update();
			this->map_fut.update();

			const auto next_section = where_am_i(current_pose, this->now);
			// printlns_to(std::osyncstream{std::cout}, "next_section: ", int(next_section));

			if(next_section != this->now) {
				this->now = next_section;

				if(is_plane(next_section)) {
					printlns_to(std::osyncstream{std::cout}, "change next_section: ", int(next_section));
					this->amcl_fut.set(true);
					this->map_fut.set(Section::to_filepath(next_section));

				}
				else {
					this->amcl_fut.set(false);
				}
			}
		}
	};
}

namespace nhk24_2nd_ws::r2::position_based_bahaviour {
	using impl::PositionBasedBehaviour;
}