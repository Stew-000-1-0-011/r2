#pragma once

#include <variant>
#include <optional>
#include <string_view>
#include <type_traits>
#include <concepts>
#include <utility>

#include <my_include/void.hpp>
#include <my_include/std_types.hpp>

#include "field_constant.hpp"

namespace nhk24_2nd_ws::r2::mode::impl {
	using void_::Void;
	using field_constant::SiloIndex;

	struct ModeName {
		enum Enum : u8 {
			terminate
			, area1_start
			, area2_start
			, goto_center_storage
			, collect_ball
			, goto_silo_watch_point
			, watch_silo
			, goto_silo
			, harvest
			, manual

			, N
		};

		static auto to_string(const Enum s) noexcept -> std::string_view {
			switch(s) {
				case Enum::terminate:
					return "terminate";
				case Enum::area1_start:
					return "area1_start";
				case Enum::area2_start:
					return "area2_start";
				case Enum::goto_center_storage:
					return "goto_center_storage";
				case Enum::collect_ball:
					return "collect_ball";
				case Enum::goto_silo_watch_point:
					return "goto_silo_watch_point";
				case Enum::watch_silo:
					return "watch_silo";
				case Enum::goto_silo:
					return "goto_silo";
				case Enum::harvest:
					return "harvest";
				case Enum::manual:
					return "manual";
				default:
					return "unreachable: " __FILE__ ": Mode::to_string";
			}
		}

		static auto from_string(const std::string_view s) noexcept -> std::optional<Enum> {
			if(s == "terminate") {
				return Enum::terminate;
			}
			else if(s == "area1_start") {
				return Enum::area1_start;
			}
			else if(s == "area2_start") {
				return Enum::area2_start;
			}
			else if(s == "goto_center_storage") {
				return Enum::goto_center_storage;
			}
			else if(s == "collect_ball") {
				return Enum::collect_ball;
			}
			else if(s == "goto_silo_watch_point") {
				return Enum::goto_silo_watch_point;
			}
			else if(s == "watch_silo") {
				return Enum::watch_silo;
			}
			else if(s == "goto_silo") {
				return Enum::goto_silo;
			}
			else if(s == "harvest") {
				return Enum::harvest;
			}
			else if(s == "manual") {
				return Enum::manual;
			}
			else {
				return std::nullopt;
			}
		}
	};

	template<ModeName::Enum mname_>
	using Arg = std::conditional_t<
		mname_ == ModeName::goto_silo
		, SiloIndex::Enum
		, std::conditional_t<
			mname_ == ModeName::manual
			, ModeName::Enum
			, Void
		>
	>;

	struct ModeArg {
		using ArgVs = decltype([]<u8 ... u8mnames_>(std::integer_sequence<u8, u8mnames_ ...>)
		-> std::variant<Arg<static_cast<ModeName::Enum>(u8mnames_)> ...>
		{}(std::make_integer_sequence<u8, ModeName::N>{}));

		ArgVs arg;

		template<ModeName::Enum s_>
		static auto make(const Arg<s_>& arg) -> ModeArg {
			return ModeArg{ArgVs{std::in_place_index<s_>, arg}};
		}
	};

	template<class T_>
	requires requires {
		requires std::movable<T_> && !std::same_as<std::remove_cvref_t<T_>, ModeArg>;
	}
	struct ModeOutput final {
		std::variant<T_, ModeArg> value;

		static auto output(T_&& v) -> ModeOutput {
			return ModeOutput{std::move(v)};
		}

		template<ModeName::Enum s_>
		static auto change(const Arg<s_>& arg) -> ModeOutput {
			return ModeOutput{std::variant<T_, ModeArg>(std::in_place_index<1>, ModeArg::make<s_>(arg))};
		}

		auto is_output() const noexcept -> bool {
			return this->value.index() == 0;
		}

		auto get_output() && -> T_ {
			return std::move(std::get<0>(this->value));
		}

		auto get_arg() && -> ModeArg {
			return std::move(std::get<1>(this->value));
		}
	};

	template<class T_>
	concept is_mode = requires {
		requires std::movable<T_>;
		{T_::name} -> std::convertible_to<ModeName::Enum>;
		typename T_::In;
		typename T_::Out;
		{T_::make(std::declval<Arg<T_::name>>())} -> std::same_as<T_>;
		requires requires(T_ mut, typename T_::In&& in) {
			{mut.update(std::move(in))} -> std::same_as<ModeOutput<typename T_::Out>>;
		};
	};
}

namespace nhk24_2nd_ws::r2::mode {
	using impl::ModeName;
	using impl::Arg;
	using impl::ModeArg;
	using impl::ModeOutput;
	using impl::is_mode;
}