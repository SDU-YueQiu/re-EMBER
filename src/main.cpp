#include <iostream>
#include "math/math256.h"
#include "tests/math256_tests.h"

int main()
{
	runMath256Tests();

	std::cout << "math256 tests passed." << std::endl;
	return 0;
}