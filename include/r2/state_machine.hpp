#pragma once

#include <concepts>
#include <tuple>
#include <utility>
#include <optional>
#include <memory>
#include <stop_token>

namespace nhk24_2nd_ws::r2::state_machine::impl {
	template<class T_>
	concept state_info_like = requires {
		typename T_::Content;
		typename T_::Out;
		typename T_::In;
		typename T_::TransitArg;

		requires requires(T_::Content& s, T_::In in) {
			{T_::update(s, std::move(in))} -> std::same_as<std::tuple<typename T_::Out, std::optional<typename T_::TransitArg>>>;
		};
	};

	struct StateBase {
		virtual ~StateBase() = default;
		virtual auto run(const std::stop_token& stoken) -> std::unique_ptr<StateBase> = 0;
	};
	template<class T_>
	concept state_like = std::derived_from<T_, StateBase>;

	template<state_info_like StateInfo_, class Input_, class OutPut_, class Transit_>
	struct State final : StateBase {
		using StateInfo = StateInfo_;
		using Input = Input_;
		using OutPut = OutPut_;
		using Transit = Transit_;

		StateInfo_::Content content;
		Input_ input;
		OutPut_ output;
		Transit_ transit;

		State (
			typename StateInfo_::Content&& content
			, Input_&& input
			, OutPut_&& output
			, Transit_&& transit
		)
			: content(std::move(content))
			, input(std::forward<Input_>(input))
			, output(std::forward<OutPut_>(output))
			, transit(std::forward<Transit_>(transit))
		{}

		auto run(const std::stop_token& stoken) -> std::unique_ptr<StateBase> override {
			while(true) {
				if (stoken.stop_requested()) {
					return nullptr;
				}

				auto [out, opt_targ] = StateInfo::update(content, input());
				output(std::move(out));
				if (opt_targ) {
					return transit(std::move(*opt_targ));
				}
			}
		}
	};

	template<state_info_like StateInfo_>
	inline auto make_state (
		typename StateInfo_::Content&& content
		, auto&& input
		, auto&& output
		, auto&& transit
	) -> State <
		StateInfo_
		, std::remove_cvref_t<decltype(input)>
		, std::remove_cvref_t<decltype(output)>
		, std::remove_cvref_t<decltype(transit)>
	>
	requires requires(typename StateInfo_::Out out, typename StateInfo_::TransitArg targ) {
		{input()} -> std::same_as<typename StateInfo_::In>;
		{output(std::move(out))};
		{transit(std::move(targ))} -> std::same_as<std::unique_ptr<StateBase>>;
	}
	{
		return State <
			StateInfo_
			, std::remove_cvref_t<decltype(input)>
			, std::remove_cvref_t<decltype(output)>
			, std::remove_cvref_t<decltype(transit)>
		> (
			std::move(content),
			std::forward<decltype(input)>(input),
			std::forward<decltype(output)>(output),
			std::forward<decltype(transit)>(transit)
		);
	}

	struct StateMachine final {
		std::unique_ptr<StateBase> state;

		void run(const std::stop_token& stoken) {
			while(state) {
				state = state->run(stoken);
			}
		}

		static auto make(std::unique_ptr<StateBase> start_state) -> StateMachine {
			return {std::move(start_state)};
		}
	};
}

namespace nhk24_2nd_ws::r2::state_machine {
	using impl::state_info_like;
	using impl::StateBase;
	using impl::State;
	using impl::make_state;
	using impl::StateMachine;
}