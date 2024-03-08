#pragma once

#include <include/parser_combinator.hpp>
#include <include/basic_parser.hpp>
#include <include/xyth.hpp>
#include <include/file_loader.hpp>

namespace nhk24_2nd_ws::r2::path_parser::impl {
	using namespace parser_combinator::ops;
	using parser_combinator::literal;
	using basic_parser::floating;
	using xyth::Xy;
	using xyth::Xyth;
	using file_loader::load_file;

	inline constexpr auto signed_floating = ~literal<u8"-"> - basic_parser::floating >>= [](auto&& x) -> double {
		return std::get<0>(x) ? -std::get<1>(x) : std::get<1>(x);
	};

	inline constexpr auto milestone = signed_floating - literal<u8" "> - signed_floating - literal<u8" "> - signed_floating >>= [](auto&& x) -> Xyth {
		return Xyth::make(Xy::make(std::get<0>(x), std::get<2>(x)), std::get<4>(x));
	};

	inline constexpr auto path = *(~milestone - literal<u8"\n">) - milestone >>= [](auto&& x) -> std::vector<Xyth> {
		std::vector<Xyth> ret;
		for(const auto& m : std::get<0>(x)) {
			if(const auto first = std::get<0>(m); first) ret.push_back(*first);
		}

		ret.push_back(std::get<1>(x));
		return ret;
	};

	inline auto path_load(const std::string_view filepath) -> std::expected<std::vector<Xyth>, std::string> {
		return load_file(filepath).and_then (
			[](const std::u8string_view content) -> std::expected<std::vector<Xyth>, std::string> {
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
	using impl::path_load;
}