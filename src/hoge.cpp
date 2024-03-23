#include <my_include/debug_print.hpp>

using nhk24_2nd_ws::debug_print::printlns;

int main() {
	//printlns test
	
	printlns("Hello, World!", 42, 3.14);
	printlns(std::string_view{"Hello, World!"});
	printlns(std::string{"Hello, World!"});
	printlns(std::string{"Hello, World!"}.c_str());
	printlns(std::vector<int>{1, 2, 3});
	printlns(std::vector<std::vector<int>>{{1, 2, 3}, {4, 5, 6}});
	printlns(std::variant<int, double>{42});
	printlns(std::variant<int, double>{3.14});
	printlns(std::variant<std::monostate, int, double>{});
	printlns(std::optional<int>{});
	printlns(std::optional<int>{42});
}