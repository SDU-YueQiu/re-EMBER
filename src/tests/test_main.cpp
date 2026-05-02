#include "bool_problem_tests.h"
#include "io_tests.h"
#include "math256_tests.h"

#include <chrono>
#include <exception>
#include <iostream>
#include <string>

namespace
{
    template <typename Fn>
    void runNamedTest(const std::string &name, Fn &&fn)
    {
        const auto start = std::chrono::steady_clock::now();
        std::cout << "[Test] begin " << name << std::endl;
        fn();
        const auto end = std::chrono::steady_clock::now();
        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "[Test] pass " << name << " elapsed_ms=" << elapsedMs << std::endl;
    }
}

int main()
{
    try
    {
        runNamedTest("math256", runMath256Tests);
        runNamedTest("bool_problem", runBoolProblemTests);
        runNamedTest("io", runIoTests);
        std::cout << "[Test] all passed" << std::endl;
        return 0;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[Test] failure " << ex.what() << std::endl;
        return 1;
    }
}
