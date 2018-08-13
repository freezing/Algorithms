cmake_minimum_required(VERSION 3.7)
project(algorithms)

enable_testing()

option(GRAPH_BUILD_STATIC "Build static graph library." ON)
option(GRAPH_BUILD_TESTS "Build graph unit tests." ON)
option(GRAPH_BUILD_BENCHMARKS "Build graph benchmarks." ON)
option(LIBALGORITHM_INCLUDE_GOOGLETEST "Include the vendored thirdparty googletest library." ON)
option(LIBALGORITHM_INCLUDE_GOOGLEBENCHMARK "Include the vendored thirdparty google benchmark library." OFF)
#option(CI_CONFIG "Configure for CI." OFF)

# Disable benchmark.
option(BENCHMARK_ENABLE_TESTING "If you want to run the tests for benchmark library, set to ON." OFF)
# Different versions of CMake require set.
set(BENCHMARK_ENABLE_TESTING OFF)

if (GRAPH_BUILD_BENCHMARKS)
    # It's important to run benchmarks in the release mode.
    set(CMAKE_BUILD_TYPE Release)
    set(LIBALGORITHM_INCLUDE_GOOGLEBENCHMARK ON)
endif()

add_subdirectory(graph)
add_subdirectory(third_party)