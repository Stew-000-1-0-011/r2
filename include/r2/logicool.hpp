#pragma once

#include <include/std_types.hpp>

namespace nhk24_2nd_ws::r2::logicool {
	struct Axes final {
		enum Enum : u8 {
			l_stick_LR = 0,
			l_stick_UD,
			l_trigger,
			r_stick_LR,
			r_stick_UD,
			r_trigger,
			cross_LR,
			cross_UD,

			N
		};
	};

	struct Buttons final {
		enum Enum : u8 {
			a = 0,
			b,
			x,
			y,
			lb,
			rb,
			back,
			start,
			dummmmmyyyyyy,
			l_push,
			r_push,

			N
		};
	};
}