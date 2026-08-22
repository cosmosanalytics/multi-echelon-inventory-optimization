#pragma once

// A minimal, dependency-free unit test harness. Deliberately tiny (no
// external test framework) so this project builds and runs unit tests on
// any machine with just a C++17 compiler — no network access or package
// manager required to review it.

#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace testfw {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

inline int& failureCount() {
    static int count = 0;
    return count;
}

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

inline void checkImpl(bool condition, const char* expr, const char* file, int line) {
    if (!condition) {
        ++failureCount();
        std::cerr << "  FAILED CHECK: " << expr << "  (" << file << ":" << line << ")\n";
    }
}

inline int runAll() {
    int passedTests = 0;
    for (const TestCase& t : registry()) {
        int before = failureCount();
        std::cout << "[ RUN  ] " << t.name << "\n";
        t.fn();
        if (failureCount() == before) {
            std::cout << "[  OK  ] " << t.name << "\n";
            ++passedTests;
        } else {
            std::cout << "[ FAIL ] " << t.name << "\n";
        }
    }
    std::cout << "\n" << passedTests << "/" << registry().size() << " test cases passed, "
              << failureCount() << " check(s) failed overall.\n";
    return failureCount() == 0 ? 0 : 1;
}

} // namespace testfw

#define TEST(name)                                                            \
    void name();                                                              \
    static testfw::Registrar registrar_##name(#name, name);                   \
    void name()

#define CHECK(cond) testfw::checkImpl((cond), #cond, __FILE__, __LINE__)
