#pragma once

#include <chrono>

namespace nhk24_2nd_ws::r2::lap_timer {
	using Clock = std::chrono::system_clock;

	struct LapTimer final {
		Clock::time_point last;

		static auto make() -> LapTimer {
			return LapTimer{Clock::now()};
		}

		auto watch() const -> std::chrono::duration<double> {
			return std::chrono::duration<double>(Clock::now() - last);
		}

		auto reset() -> void {
			last = Clock::now();
		}

		auto update() -> std::chrono::duration<double> {
			auto ret = this->watch();
			this->reset();
			return ret;
		}
	};
}