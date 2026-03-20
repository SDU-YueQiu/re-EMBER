#include <iostream>
#include "math256.h"
#include "math256_tests.h"

int main()
{
	runMath256Tests();

	std::cout << "math256 tests passed." << std::endl;
	return 0;
}