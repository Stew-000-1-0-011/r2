#pragma once

#include <vector>
#include <string_view>
#include <string>
#include <optional>
#include <tuple>
#include <expected>

#include <my_include/parser_combinator.hpp>
#include <my_include/basic_parser.hpp>
#include <my_include/xyth.hpp>
#include <my_include/file_loader.hpp>

#include "pacman.hpp"

namespace nhk24_2nd_ws::r2::path_parser::impl {
	using namespace parser_combinator::ops;
	using parser_combinator::literal;
	using basic_parser::floating;
	using basic_parser::identifier;
	using xyth::Xy;
	using xyth::Xyth;
	using xyth::XythScalar;
	using file_loader::load_file;
	using pacman::Milestone;

	inline constexpr auto signed_floating = ~literal<u8"-"> - basic_parser::floating >>= [](auto&& x) -> double {
		return std::get<0>(x) ? -std::get<1>(x) : std::get<1>(x);
	};

	inline constexpr auto xyth = signed_floating - literal<u8" "> - signed_floating - literal<u8" "> - signed_floating >>= [](auto&& x) -> Xyth {
		return Xyth::make(Xy::make(std::get<0>(x), std::get<2>(x)), std::get<4>(x));
	};

	inline constexpr auto xyth_scalar = signed_floating - literal<u8" "> - signed_floating >>= [](auto&& x) -> XythScalar {
		return XythScalar::make(std::get<0>(x), std::get<2>(x));
	};

	inline constexpr auto event = literal<u8"!"> - ~identifier >>= [](auto&& x) -> std::u8string {
		if(auto id = std::get<1>(x)) {
			return std::move(*id);
		}
		return std::u8string{};
	};

	inline constexpr auto milestone = xyth - ~literal<u8" "> - literal<u8"-"> - ~literal<u8" "> - xyth_scalar - ~literal<u8" "> - ~event >>= [](auto&& x) -> Milestone {
		return Milestone::make(std::get<0>(x), std::get<4>(x), std::get<6>(x));
	};

	inline constexpr auto path = *(~milestone - literal<u8"\n">) - milestone >>= [](auto&& x) -> std::vector<Milestone> {
		std::vector<Milestone> ret;
		for(const auto& m : std::get<0>(x)) {
			ret.push_back(*std::get<0>(m));
		}
		ret.push_back(std::get<1>(x));
		return ret;
	};

	inline auto path_load(const std::string_view filepath) -> std::expected<std::vector<Milestone>, std::string> {
		return load_file(filepath).and_then (
			[](const std::u8string_view content) -> std::expected<std::vector<Milestone>, std::string> {
			const auto res = path.parse(content);
			if(res) {
				return {*res.value};
			} else {
				return std::unexpected{"Failed to parse path."};
			}
		});
	}
}

namespace nhk24_2nd_ws::r2::path_parser {
	using impl::Milestone;
	using impl::path_load;
}