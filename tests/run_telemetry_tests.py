"""Linux-only isolated tests: real driver, Boost.Asio, yaml-cpp and PTYs.
Only application logger/scheduler are stubs; no physical devices are opened.
The scheduler stub records what the driver publishes to the business layer, so the
RX path (runtime_push_source) is exercised instead of being silently dropped.
"""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="awakening-telemetry-") as temp:
    temp = Path(temp)
    (temp / "utils/scheduler").mkdir(parents=True)
    (temp / "utils/logger.hpp").write_text('''#pragma once
namespace isolated_test { template<class... T> void log(const T&...) {} }
#define AWAKENING_INFO(...) isolated_test::log(__VA_ARGS__)
#define AWAKENING_WARN(...) isolated_test::log(__VA_ARGS__)
#define AWAKENING_ERROR(...) isolated_test::log(__VA_ARGS__)
''')
    (temp / "utils/scheduler/scheduler.hpp").write_text('''#pragma once
#include <cstddef>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
namespace awakening {
template<class Tag, class T> struct IOPair { using second_type = T; };
class Scheduler {
public:
    template<class IO> std::size_t register_source(const std::string&) { return ++count_; }
    bool is_running() const { return true; }
    template<class IO, class Fn> void runtime_push_source(std::size_t, Fn&& fn) {
        auto out = fn();
        auto& payload = std::get<0>(out);
        if (payload.has_value()) {
            ++pushed;
            pushed_bytes += payload->size();
        }
    }
    int pushed = 0;
    std::size_t pushed_bytes = 0;
private:
    std::size_t count_ = 0;
};
}
''')
    binary = temp / "test"
    cmd = ["g++", "-std=c++20", "-Wall", "-Wextra", "-Werror", "-g", "-pthread",
           "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
           "-I" + str(temp), "-I" + str(root / "src"), str(root / "tests/telemetry_test.cpp"),
           "-lboost_system", "-lyaml-cpp", "-o", str(binary)]
    subprocess.run(cmd, check=True)
    subprocess.run([str(binary)], check=True, timeout=30,
                   env={**os.environ, "ASAN_OPTIONS": "detect_leaks=1"})
    print("PASS: C++ -Werror build, AddressSanitizer and UndefinedBehaviorSanitizer")
