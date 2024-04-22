#pragma once

#include <utility>
#include <variant>
#include <stdexcept>
#include <type_traits>
#include <concepts>
#include <chrono>

#include <my_include/debug_print.hpp>

#include "mode.hpp"
#include "modes/pursuit.hpp"
#include "modes/collect_ball.hpp"
#include "modes/watch_silo.hpp"
#include "modes/harvest.hpp"
#include "modes/manual.hpp"

namespace nhk24_2nd_ws::r2::r2_controller::impl {
	using namespace std::string_literals;
	using namespace std::chrono_literals;
	using debug_print::printlns_to;
	using mode::is_mode;
	using mode::ModeOutput;
	using mode::ModeName;
	using mode::ModeArg;
	using modes::pursuit::Area1Start;
	using modes::pursuit::Area2Start;
	using modes::pursuit::GotoCenterStorage;
	using modes::pursuit::GotoSiloWatchPoint;
	using modes::pursuit::GotoSilo;
	using modes::collect_ball::CollectBall;
	using modes::watch_silo::WatchSilo;
	using modes::harvest::Harvest;
	using modes::manual::Manual;

	template<class T_, class Mode_>
	concept is_mode_executor_generator = is_mode<Mode_> && requires(T_ mut, typename Mode_::Out&& out) {
		{mut().input()} -> std::same_as<typename Mode_::In>;
		{mut().output(std::move(out))};
	};

	template <
		is_mode_executor_generator<modes::pursuit::Area1Start> PursuitExecutorGenerator_
		, is_mode_executor_generator<modes::collect_ball::CollectBall> CollectBallExecutorGenerator_
		, is_mode_executor_generator<modes::watch_silo::WatchSilo> WatchSiloExecutorGenerator_
		, is_mode_executor_generator<modes::harvest::Harvest> HarvestExecutorGenerator_
		, is_mode_executor_generator<modes::manual::Manual> ManualExecutorGenerator_
		, class CommonIo_
	>
	requires requires(CommonIo_ mut) {
		{mut.to_manual()} -> std::same_as<bool>;
		{mut.killed()} -> std::same_as<bool>;
		{mut.notify_killed()};
	}
	struct R2Controller final {
		PursuitExecutorGenerator_ pursuit_executor_generator;
		CollectBallExecutorGenerator_ collect_ball_executor_generator;
		WatchSiloExecutorGenerator_ watch_silo_executor_generator;
		HarvestExecutorGenerator_ harvest_executor_generator;
		ManualExecutorGenerator_ manual_executor_generator;
		CommonIo_ common_io;

		std::variant<std::monostate, Area1Start, Area2Start, GotoCenterStorage, CollectBall, GotoSiloWatchPoint, WatchSilo, GotoSilo, Harvest, Manual> mode;
		
		using MVariant = decltype(mode);
		template<ModeName::Enum mname_>
		using Mode = std::remove_cvref_t<decltype(std::get<mname_>(mode))>;

		static auto generate_mode(ModeArg&& marg) -> MVariant {
			auto ret = [idx = marg.arg.index(), marg = std::move(marg)]<u8 ... u8mnames_>(std::integer_sequence<u8, u8mnames_ ...>)
			{
				// printlns_to(std::osyncstream{std::cout},"in generate_mode: ", __LINE__);
				MVariant ret{};
				// printlns_to(std::osyncstream{std::cout},"in generate_mode: ", __LINE__);
				([idx, marg]<ModeName::Enum mname_>(MVariant& ret) {
					// printlns_to(std::osyncstream{std::cout},"in generate_mode: ", __LINE__);
					if(idx == mname_) {
						ret = MVariant{std::in_place_index<mname_>, Mode<mname_>::make(std::get<mname_>(marg.arg))};
					}
				}.template operator()<static_cast<ModeName::Enum>(u8mnames_ + 1)>(ret), ...);
				// printlns_to(std::osyncstream{std::cout},"in generate_mode: ", __LINE__);
				return ret;
			}(std::make_integer_sequence<u8, ModeName::N - 1>{});
			// printlns_to(std::osyncstream{std::cout},"in generate_mode: ", __LINE__);
			return ret;
		}

		static auto make_unique (
			PursuitExecutorGenerator_&& pursuit_executor_generator
			, CollectBallExecutorGenerator_&& collect_ball_executor_generator
			, WatchSiloExecutorGenerator_&& watch_silo_executor_generator
			, HarvestExecutorGenerator_&& harvest_executor_generator
			, ManualExecutorGenerator_&& manual_executor_generator
			, CommonIo_&& common_io
			, ModeArg&& start_arg
		) -> std::unique_ptr<R2Controller> {
			return std::make_unique<R2Controller> (
				std::move(pursuit_executor_generator)
				, std::move(collect_ball_executor_generator)
				, std::move(watch_silo_executor_generator)
				, std::move(harvest_executor_generator)
				, std::move(manual_executor_generator)
				, std::move(common_io)
				, generate_mode(std::move(start_arg))
			);
		}

		template<ModeName::Enum mode_name_>
		auto get_executor() {
			if constexpr (
				mode_name_ == ModeName::area1_start
				|| mode_name_ == ModeName::area2_start
				|| mode_name_ == ModeName::goto_center_storage
				|| mode_name_ == ModeName::goto_silo_watch_point
				|| mode_name_ == ModeName::goto_silo
			) {
				return this->pursuit_executor_generator();
			}
			else if constexpr (mode_name_ == ModeName::collect_ball) {
				return this->collect_ball_executor_generator();
			}
			else if constexpr (mode_name_ == ModeName::watch_silo) {
				return this->watch_silo_executor_generator();
			}
			else if constexpr (mode_name_ == ModeName::harvest) {
				return this->harvest_executor_generator();
			}
			else if constexpr (mode_name_ == ModeName::manual) {
				return this->manual_executor_generator();
			}
			else {
				throw std::runtime_error(__FILE__ "get_executor: invalid mode: "s + std::string{ModeName::to_string(mode_name_)});
			}
		}

		void run() {
			while(this->mode.index() != 0) {
				printlns_to(std::osyncstream{std::cout},"in r2_controller::run2.");
				std::visit([this](auto& m) {
					using Mode = std::remove_cvref_t<decltype(m)>;
					if constexpr(not std::same_as<Mode, std::monostate>) {
						auto executor = this->get_executor<Mode::name>();

						while(true) {
							printlns_to(std::osyncstream{std::cout}, "Mode name: ", ModeName::to_string(Mode::name));
							
							// printlns_to(std::osyncstream{std::cout},"in r2_controller::run: ", __LINE__);
							if(this->common_io.killed()) {
								this->mode = std::monostate{};
								break;
							}

							if(this->common_io.to_manual() && Mode::name != ModeName::manual) {
								this->mode = generate_mode(ModeArg{ModeArg::ArgVs{std::in_place_index<ModeName::manual>, Mode::name}});
								break;
							}

							// printlns_to(std::osyncstream{std::cout},"in r2_controller::run: ", __LINE__);

							auto in = executor.input();
							// printlns_to(std::osyncstream{std::cout},"in r2_controller::run: ", __LINE__);
							auto out = m.update(in);
							// printlns_to(std::osyncstream{std::cout},"in r2_controller::run: ", __LINE__);
							if(out.is_output()) {
								printlns_to(std::osyncstream{std::cout},"in r2_controller::run: ", __LINE__);
								executor.output(std::move(out).get_output());
								// printlns_to(std::osyncstream{std::cout},"in r2_controller::run: ", __LINE__);
							}
							else {
								printlns_to(std::osyncstream{std::cout},"in r2_controller::run: ", __LINE__);
								auto arg = std::move(out).get_arg();
								printlns_to(std::osyncstream{std::cout},"in r2_controller::run: ", __LINE__);
								this->mode = generate_mode(std::move(arg));
								printlns_to(std::osyncstream{std::cout},"in r2_controller::run: ", __LINE__);
								break;
							}

							std::this_thread::sleep_for(10ms);
						}

					}
				}, this->mode);
			}

			this->common_io.notify_killed();
		}
	};

	inline auto make_r2_controller_unique (
		auto&& pursuit_executor_generator
		, auto&& collect_ball_executor_generator
		, auto&& watch_silo_executor_generator
		, auto&& harvest_executor_generator
		, auto&& manual_executor_generator
		, auto&& common_io
		, auto&& start_arg
	) {
		return R2Controller <
			std::remove_cvref_t<decltype(pursuit_executor_generator)>
			, std::remove_cvref_t<decltype(collect_ball_executor_generator)>
			, std::remove_cvref_t<decltype(watch_silo_executor_generator)>
			, std::remove_cvref_t<decltype(harvest_executor_generator)>
			, std::remove_cvref_t<decltype(manual_executor_generator)>
			, std::remove_cvref_t<decltype(common_io)>
		>::make_unique (
			std::forward<decltype(pursuit_executor_generator)>(pursuit_executor_generator)
			, std::forward<decltype(collect_ball_executor_generator)>(collect_ball_executor_generator)
			, std::forward<decltype(watch_silo_executor_generator)>(watch_silo_executor_generator)
			, std::forward<decltype(harvest_executor_generator)>(harvest_executor_generator)
			, std::forward<decltype(manual_executor_generator)>(manual_executor_generator)
			, std::forward<decltype(common_io)>(common_io)
			, std::forward<decltype(start_arg)>(start_arg)
		);
	}
}

namespace nhk24_2nd_ws::r2::r2_controller {
	using impl::R2Controller;
	using impl::make_r2_controller_unique;
}