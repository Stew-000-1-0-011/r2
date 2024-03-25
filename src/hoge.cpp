#include <cstddef>
#include <cmath>
#include <vector>
#include <iostream>

struct LidarScan final {
	std::vector<float> range;
	float angle_increment;

	static constexpr auto make(std::vector<float>&& range, const float angle_min, const float angle_max) noexcept -> LidarScan {
		std::cout << "in make before ret: " << (angle_max - angle_min) << range.size() << std::endl;
		auto ret = LidarScan{std::move(range), (angle_max - angle_min) / range.size()};
		std::cout << "in make ret: " << ret.angle_increment << std::endl;
		return ret;
	}
};

int main() {
	auto scan = LidarScan::make(std::vector<float>(1000, 0.0f), 0, 3.14 * 1.5);
}