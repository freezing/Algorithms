set(GRAPH_LIB_SRC
        include/graph.h
        src/bfs.cpp
        src/dijsktra.cpp
        src/tarjan.cpp)

set(GRAPH_LIB_TEST_SRC
        src/bfs_test.cpp
        src/dijkstra_test.cpp
        src/tarjan_test.cpp
        src/test.cpp)

set(GRAPH_LIB_BENCHMARK_SRC
        benchmark/bfs_benchmark.cpp)

if (GRAPH_BUILD_STATIC)
    add_library(graph_static STATIC ${GRAPH_LIB_SRC})
    set_property(TARGET graph_static PROPERTY CXX_STANDARD 11)
    target_compile_definitions(graph_static PUBLIC LIBGRAPH_STATIC)

    target_include_directories(graph_static
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
            PUBLIC $<INSTALL_INTERFACE:include>
            PRIVATE src)

    if (GRAPH_BUILD_TESTS)
        add_executable(graph_test ${GRAPH_LIB_TEST_SRC})
        target_link_libraries(graph_test graph_static gtest gmock)
        set_property(TARGET graph_test PROPERTY CXX_STANDARD 11)
        add_test(graph_test graph_test)
    endif()

    if (GRAPH_BUILD_BENCHMARKS)
        # BFS Benchmark.
        add_executable(bfs_benchmark ${GRAPH_LIB_BENCHMARK_SRC})
        target_link_libraries(bfs_benchmark graph_static benchmark)
        set_property(TARGET bfs_benchmark PROPERTY CXX_STANDARD 11)
        set_target_properties(bfs_benchmark PROPERTIES benchmark_format json benchmark_out benchmarks/bfs.json)
        # TODO: This should probably not be part of the test suite. Instead we should manually (via CI script) run
        # benchmarks and report them.
#        add_test(bfs_benchmark bfs_benchmark)
    endif()
endif()