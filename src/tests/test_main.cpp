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
    bool runNamedTest(const std::string &name, Fn &&fn)
    {
        const auto start = std::chrono::steady_clock::now();
        std::cout << "[Test] begin " << name << std::endl;
        try
        {
            fn();
            const auto end = std::chrono::steady_clock::now();
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "[Test] pass " << name << " elapsed_ms=" << elapsedMs << std::endl;
            return true;
        }
        catch (const std::exception &ex)
        {
            const auto end = std::chrono::steady_clock::now();
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cerr << "[Test] fail " << name << " elapsed_ms=" << elapsedMs
                      << " error=" << ex.what() << std::endl;
            return false;
        }
    }
}

int main()
{
    int failed = 0;
    failed += runNamedTest("math256", runMath256Tests) ? 0 : 1;
    failed += runNamedTest("bool_problem", runBoolProblemTests) ? 0 : 1;
    failed += runNamedTest("io", runIoTests) ? 0 : 1;

    if (failed != 0)
    {
        std::cerr << "[Test] failed_groups=" << failed << std::endl;
        return 1;
    }

    std::cout << "[Test] all passed" << std::endl;
    return 0;
}
